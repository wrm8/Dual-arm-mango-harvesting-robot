#include "AiLLMControl.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

AiLLMControl aiLLMControl;

namespace {
constexpr uint8_t TYPE_NULL   = 0x00;
constexpr uint8_t TYPE_BOOL_F = 0x01;
constexpr uint8_t TYPE_BOOL_T = 0x02;
constexpr uint8_t TYPE_INT8   = 0x03;
constexpr uint8_t TYPE_INT16  = 0x04;
constexpr uint8_t TYPE_INT32  = 0x05;
constexpr uint8_t TYPE_UINT8  = 0x06;
constexpr uint8_t TYPE_UINT16 = 0x07;
constexpr uint8_t TYPE_UINT32 = 0x08;
constexpr uint8_t TYPE_STRING = 0x09;
constexpr uint8_t TYPE_ARRAY  = 0x0A;
constexpr uint8_t TYPE_DICT   = 0x0B;

bool parseIntFromString(const char* s, int len, int& out) {
    if (!s || len <= 0) return false;
    int i = 0;
    while (i < len && s[i] != '-' && !isdigit(static_cast<unsigned char>(s[i]))) {
        i++;
    }
    if (i >= len) return false;
    bool neg = false;
    if (s[i] == '-') {
        neg = true;
        i++;
    }
    if (i >= len || !isdigit(static_cast<unsigned char>(s[i]))) return false;
    long val = 0;
    while (i < len && isdigit(static_cast<unsigned char>(s[i]))) {
        val = val * 10 + (s[i] - '0');
        i++;
    }
    out = neg ? -static_cast<int>(val) : static_cast<int>(val);
    return true;
}

int parseNumbersFromString(const char* s, int len, int* out, int max_count) {
    if (!s || len <= 0 || !out || max_count <= 0) return 0;
    int count = 0;
    int i = 0;
    while (i < len && count < max_count) {
        while (i < len && s[i] != '-' && !isdigit(static_cast<unsigned char>(s[i]))) {
            i++;
        }
        if (i >= len) break;
        bool neg = false;
        if (s[i] == '-') {
            neg = true;
            i++;
        }
        if (i >= len || !isdigit(static_cast<unsigned char>(s[i]))) {
            continue;
        }
        long val = 0;
        while (i < len && isdigit(static_cast<unsigned char>(s[i]))) {
            val = val * 10 + (s[i] - '0');
            i++;
        }
        out[count++] = neg ? -static_cast<int>(val) : static_cast<int>(val);
    }
    return count;
}
} // namespace

class DataPacker {
public:
    uint8_t buf[1024];
    uint16_t len = 0;

    void addString(const char* s) {
        buf[len++] = TYPE_STRING;
        uint16_t slen = strlen(s);
        buf[len++] = (slen >> 8) & 0xFF;
        buf[len++] = slen & 0xFF;
        memcpy(&buf[len], s, slen);
        len += slen;
    }

    void addUint8(uint8_t v) {
        buf[len++] = TYPE_UINT8;
        buf[len++] = v;
    }

    void beginArray(uint16_t itemsCount) {
        buf[len++] = TYPE_ARRAY;
        buf[len++] = (itemsCount >> 8) & 0xFF;
        buf[len++] = itemsCount & 0xFF;
    }

    void beginDict(uint16_t pairsCount) {
        buf[len++] = TYPE_DICT;
        buf[len++] = (pairsCount >> 8) & 0xFF;
        buf[len++] = pairsCount & 0xFF;
    }

    void endArray() {}
    void endDict() {}
};

void AiLLMControl::begin() {
    _busy = false;
    _seq = 0;
    Wire1.begin(21, 22);
    Wire1.setClock(400000);
    Wire1.setBufferSize(256);
}

void AiLLMControl::start() {
    if (_busy) return;
    _busy = true;
    _seq = 0;

    ackAndClear();

    _currentState = LLM_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_hb_time = 0;
    _last_hb_status = 0;
    _last_hb_mode = 0;
    _mcp_configured = false;
    _rx_frag_len = 0;
    _rx_frag_func = 0;
    _rx_frag_in_progress = false;
    _rx_frag_time = 0;
    Serial.println(">>>> LLM start");
}

void AiLLMControl::stop() {
    _busy = false;
    _currentState = LLM_STATE_INIT;
    _rx_frag_len = 0;
    _rx_frag_func = 0;
    _rx_frag_in_progress = false;
    _rx_frag_time = 0;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
    Serial.println(">>>> LLM stop");
}

void AiLLMControl::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for (int i = 0; i < 8; i++) Wire1.write(0x00);
    Wire1.endTransmission();
}

bool AiLLMControl::waitRxClear(unsigned long timeout_ms) {
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        Wire1.beginTransmission(K230_ADDR);
        Wire1.write(0x00);
        if (Wire1.endTransmission(false) == 0) {
            if (Wire1.requestFrom(K230_ADDR, static_cast<uint8_t>(2)) == 2) {
                uint8_t b0 = Wire1.read();
                uint8_t b1 = Wire1.read();
                if (b0 == 0x00 && b1 == 0x00) {
                    return true;
                }
            }
        }
        delay(1);
    }
    return false;
}

void AiLLMControl::sendPacket(uint8_t func, const uint8_t* payload, uint16_t len) {
    uint16_t offset = 0;

    while (offset < len || len == 0) {
        uint16_t chunk_size = len - offset;
        if (chunk_size > TX_PAYLOAD_CHUNK_MAX) {
            chunk_size = TX_PAYLOAD_CHUNK_MAX;
        }

        bool is_cont = (offset + chunk_size < len);
        if (!waitRxClear(100)) {
            Serial.println(">>>> TX wait clear timeout");
        }

        Wire1.beginTransmission(K230_ADDR);
        Wire1.write(0x00);
        Wire1.write(K230_H0);
        Wire1.write(K230_H1);
        Wire1.write(chunk_size & 0xFF);

        uint8_t seq_byte = FRAME_TYPE_CMD | (_seq & 0x1F);
        if (is_cont) seq_byte |= CONT_MASK;

        Wire1.write(seq_byte);
        Wire1.write(func);

        uint8_t xor_val = (chunk_size & 0xFF) ^ seq_byte ^ func;
        if (chunk_size > 0) {
            for (uint16_t i = 0; i < chunk_size; i++) {
                Wire1.write(payload[offset + i]);
                xor_val ^= payload[offset + i];
            }
        }
        Wire1.write(xor_val);
        Wire1.endTransmission();

        offset += chunk_size;

        if (is_cont) {
            waitRxClear(200);
            _seq = (_seq + 1) & 0x1F;
        } else {
            break;
        }
    }
    _seq = (_seq + 1) & 0x1F;
}

void AiLLMControl::setupMCPTools() {
    DataPacker p;
    p.beginArray(1);  // 只有1个工具
      p.beginDict(3);
        p.addString("type");
        p.addString("function");
        p.addString("function");
        p.beginDict(3);
          p.addString("name");
          p.addString("run_action");  // 工具名：run_action
          p.addString("description");
          p.addString("用户要求控制舵机时调用此工具。例如：'移动1号舵机到2048'、'id1 position 300'、'servo 2 to 1500'");
          p.addString("parameters");
          p.beginDict(3);
            p.addString("type");
            p.addString("object");
            p.addString("properties");
            p.beginDict(1);  // 只有1个参数
              p.addString("name");
              p.beginDict(2);
                p.addString("type");
                p.addString("string");
                p.addString("description");
                p.addString("用户的舵机控制指令，包含舵机ID和目标位置");
              p.endDict();
            p.endDict();
            p.addString("required");
            p.beginArray(1);
              p.addString("name");
            p.endArray();
          p.endDict();
        p.endDict();
        p.addString("block");
        p.addUint8(5);
      p.endDict();
    p.endArray();

    sendPacket(CMD_SET_MCP_TOOLS, p.buf, p.len);
    Serial.println(">>>> MCP tools sent (run_action)");
}

bool AiLLMControl::readValueAsInt(const uint8_t* payload, uint16_t len, int offset, int& out) {
    if (offset >= len) return false;
    uint8_t type = payload[offset];
    switch (type) {
        case TYPE_UINT8:
            if (offset + 1 >= len) return false;
            out = payload[offset + 1];
            return true;
        case TYPE_INT8:
            if (offset + 1 >= len) return false;
            out = static_cast<int8_t>(payload[offset + 1]);
            return true;
        case TYPE_UINT16:
            if (offset + 2 >= len) return false;
            out = (payload[offset + 1] << 8) | payload[offset + 2];
            return true;
        case TYPE_INT16:
            if (offset + 2 >= len) return false;
            out = static_cast<int16_t>((payload[offset + 1] << 8) | payload[offset + 2]);
            return true;
        case TYPE_UINT32:
            if (offset + 4 >= len) return false;
            out = (payload[offset + 1] << 24) | (payload[offset + 2] << 16) |
                  (payload[offset + 3] << 8) | payload[offset + 4];
            return true;
        case TYPE_INT32:
            if (offset + 4 >= len) return false;
            out = static_cast<int32_t>((payload[offset + 1] << 24) | (payload[offset + 2] << 16) |
                                       (payload[offset + 3] << 8) | payload[offset + 4]);
            return true;
        case TYPE_STRING: {
            if (offset + 2 >= len) return false;
            uint16_t slen = (payload[offset + 1] << 8) | payload[offset + 2];
            int str_offset = offset + 3;
            if (str_offset + slen > len) return false;
            return parseIntFromString(reinterpret_cast<const char*>(&payload[str_offset]), slen, out);
        }
        default:
            break;
    }
    return false;
}

bool AiLLMControl::parseServoArgs(const uint8_t* payload, uint16_t len, int& out_id, int& out_angle, int& out_time_ms) {
    out_id = -1;
    out_angle = -1;
    out_time_ms = SERVO_TIME_DEFAULT_MS;
    int offset = 0;

    Serial.printf(">>>> parseServoArgs: len=%d\n", len);

    while (offset < len) {
        uint8_t type = payload[offset];
        
        if (type == TYPE_STRING) {
            if (offset + 2 >= len) break;
            uint16_t slen = (payload[offset + 1] << 8) | payload[offset + 2];
            int str_offset = offset + 3;
            if (str_offset + slen > len) break;
            const char* str_ptr = reinterpret_cast<const char*>(&payload[str_offset]);
            int val_offset = str_offset + slen;

            // 打印键名
            char key_buf[32] = {0};
            int copy_len = (slen < 31) ? slen : 31;
            memcpy(key_buf, str_ptr, copy_len);
            Serial.printf(">>>> Key: '%s'\n", key_buf);

            // 处理 "name" 字段（run_action的参数）
            if (slen == 4 && memcmp(str_ptr, "name", 4) == 0) {
                if (val_offset < len && payload[val_offset] == TYPE_STRING) {
                    uint16_t vlen = (payload[val_offset + 1] << 8) | payload[val_offset + 2];
                    int str_off = val_offset + 3;
                    if (str_off + vlen <= len) {
                        const char* s = reinterpret_cast<const char*>(&payload[str_off]);
                        
                        // 打印name字段内容
                        char name_buf[128] = {0};
                        int name_len = (vlen < 127) ? vlen : 127;
                        memcpy(name_buf, s, name_len);
                        Serial.printf(">>>> Name value: '%s'\n", name_buf);
                        
                        // 从文本中提取数字
                        int nums[3] = {0};
                        int n = parseNumbersFromString(s, vlen, nums, 3);
                        Serial.printf(">>>> Extracted %d numbers: ", n);
                        for(int i=0; i<n; i++) Serial.printf("%d ", nums[i]);
                        Serial.println();
                        
                        if (n >= 2) {
                            out_id = nums[0];
                            out_angle = nums[1];
                            if (n >= 3) out_time_ms = nums[2];
                        }
                    }
                }
            }
            offset += 3 + slen;
            continue;
        }

        // 跳过其他类型
        if (type == TYPE_UINT8 || type == TYPE_INT8) {
            offset += 2;
        } else if (type == TYPE_UINT16 || type == TYPE_INT16) {
            offset += 3;
        } else if (type == TYPE_UINT32 || type == TYPE_INT32) {
            offset += 5;
        } else if (type == TYPE_ARRAY || type == TYPE_DICT) {
            offset += 3;
        } else if (type == TYPE_NULL || type == TYPE_BOOL_F || type == TYPE_BOOL_T) {
            offset += 1;
        } else {
            offset += 1;
        }
    }

    bool success = (out_id != -1 && out_angle != -1);
    Serial.printf(">>>> Parse result: success=%d, id=%d, angle=%d, time=%d\n",
                  success, out_id, out_angle, out_time_ms);
    return success;
}

bool AiLLMControl::executeServoParams(int target_id, int target_angle, int time_ms) {
    if (target_id < SERVO_ID_MIN || target_id > SERVO_ID_MAX) {
        Serial.println(">>>> Invalid servo id");
        return false;
    }
    if (target_angle < SERVO_POS_MIN || target_angle > SERVO_POS_MAX) {
        Serial.println(">>>> Invalid servo position");
        return false;
    }
    if (time_ms <= 0) {
        time_ms = SERVO_TIME_DEFAULT_MS;
    }
    time_ms = constrain(time_ms, SERVO_TIME_MIN_MS, SERVO_TIME_MAX_MS);

    Serial.printf(">>>> Servo id=%d pos=%d time=%dms\n", target_id, target_angle, time_ms);
    arm.move_servo(target_id, static_cast<int16_t>(target_angle), static_cast<uint16_t>(time_ms));
    return true;
}

bool AiLLMControl::executeAction(const uint8_t* payload, uint16_t len) {
    Serial.printf(">>>> executeAction called, len=%d\n", len);
    
    // 打印原始payload
    Serial.printf(">>>> Raw payload hex: ");
    for(int i=0; i<len && i<100; i++) {
        Serial.printf("%02X ", payload[i]);
        if ((i+1) % 16 == 0) Serial.printf("\n                      ");
    }
    Serial.println();
    
    int target_id = -1;
    int target_angle = -1;
    int time_ms = SERVO_TIME_DEFAULT_MS;

    if (!parseServoArgs(payload, len, target_id, target_angle, time_ms)) {
        Serial.println(">>>> Parse servo args FAILED");
        return false;
    }
    
    Serial.printf(">>>> Calling executeServoParams: id=%d, angle=%d, time=%d\n",
                  target_id, target_angle, time_ms);
    return executeServoParams(target_id, target_angle, time_ms);
}

void AiLLMControl::handleReport(uint8_t func, const uint8_t* payload, int payload_len) {
    if (func != RPT_HEARTBEAT) return;
    if (payload_len < 2) return;

    uint8_t mode = payload[0];
    uint8_t status = payload[1];
    bool status_changed = (mode != _last_hb_mode) || (status != _last_hb_status);
    _last_hb_mode = mode;
    _last_hb_status = status;
    _last_hb_time = millis();

    if (status_changed) {
        Serial.printf(">>>> HB mode=%u status=0x%02X\n", mode, status);
    }

    if ((status & STATUS_RESULT) == 0) return;
    if (payload_len < 4) {
        Serial.println(">>>> HB result too short");
        return;
    }

    uint8_t cmd = payload[2];
    uint8_t code = payload[3];
    Serial.printf(">>>> HB result cmd=0x%02X code=0x%02X len=%d\n", cmd, code, payload_len);

    // 检查是否是MCP工具调用结果
    if (cmd == CMD_RESULT_RETURN && code == ERR_OK) {
        // 检查payload中是否包含 "run_action" 字符串
        bool is_run_action = false;
        const char* search_str = "run_action";
        int search_len = 10;
        
        for (int i = 4; i <= payload_len - search_len; i++) {
            if (memcmp(&payload[i], search_str, search_len) == 0) {
                is_run_action = true;
                Serial.println(">>>> Found 'run_action' in payload!");
                break;
            }
        }
        
        if (is_run_action) {
            Serial.println(">>>> MCP tool call received!");
            bool ok = executeAction(payload + 4, payload_len - 4);
            
            DataPacker p;
            p.addString(ok ? "舵机控制成功！" : "舵机控制失败");
            sendPacket(CMD_RESULT_RETURN, p.buf, p.len);
            
            Serial.printf(">>>> MCP result sent: %s\n", ok ? "ok" : "err");
        } else {
            Serial.println(">>>> Not a run_action call, ignoring");
        }
    } else if (cmd == CMD_RESULT_RETURN) {
        Serial.printf(">>>> MCP error: code=0x%02X\n", code);
    }
}

void AiLLMControl::parseFrame(uint8_t* data, int total_len) {
    if (total_len < 6) return;
    uint8_t func = data[4];
    const uint8_t* payload = &data[5];
    int payload_len = total_len - 6;
    handleReport(func, payload, payload_len);
}

void AiLLMControl::update() {
    if (!_busy) return;

    unsigned long now = millis();
    static unsigned long last_i2c_access = 0;

    if (now - last_i2c_access < 5) return;
    last_i2c_access = now;

    if (_rx_frag_in_progress && (now - _rx_frag_time > 200)) {
        _rx_frag_in_progress = false;
        _rx_frag_len = 0;
        Serial.println(">>>> RX fragment timeout");
    }

    switch (_currentState) {
        case LLM_STATE_SET_MODE:
            if (now - _last_state_time > 100) {
                uint8_t p[] = {APP_MODE_AILLM};
                sendPacket(CMD_SET_MODE, p, 1);
                _currentState = LLM_STATE_WAIT_READY;
                _last_state_time = now;
            }
            break;

        case LLM_STATE_WAIT_READY:
            if (now - _last_state_time > 200) {
                sendPacket(CMD_REQUEST_STATUS, NULL, 0);
                _last_state_time = now;
            }
            if ((_last_hb_status & STATUS_READY) != 0 && _last_hb_mode == APP_MODE_AILLM) {
                _currentState = LLM_STATE_CONFIG_MCP;
                _last_state_time = now;
            }
            break;

        case LLM_STATE_CONFIG_MCP:
            if (!_mcp_configured && now - _last_state_time > 200) {
                setupMCPTools();
                _mcp_configured = true;
                _currentState = LLM_STATE_ENABLE_RUN;
                _last_state_time = now;
            }
            break;

        case LLM_STATE_ENABLE_RUN:
            if (now - _last_state_time > 200) {
                uint8_t p[] = {0x00};
                sendPacket(CMD_DISABLE_RUN, p, 1);
                _currentState = LLM_STATE_ENABLE_ASR;
                _last_state_time = now;
            }
            break;

        case LLM_STATE_ENABLE_ASR:
            if (now - _last_state_time > 200) {
                uint8_t p[] = {0x01};
                sendPacket(CMD_ASR, p, 1);
                _currentState = LLM_STATE_RUNNING;
                _last_state_time = now;
                Serial.println(">>>> LLM running");
            }
            break;

        case LLM_STATE_RUNNING:
            break;

        default: break;
    }

    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) == 0) {
        if (Wire1.requestFrom(K230_ADDR, (uint8_t)5) == 5) {
            uint8_t head[5];
            for (int i = 0; i < 5; i++) head[i] = Wire1.read();

            // 添加调试：打印读取到的头部
            static unsigned long last_debug = 0;
            if (millis() - last_debug > 1000 && (head[0] != 0 || head[1] != 0)) {
                Serial.printf(">>>> I2C head: %02X %02X %02X %02X %02X\n",
                              head[0], head[1], head[2], head[3], head[4]);
                last_debug = millis();
            }

            if (head[0] == K230_H0 && head[1] == K230_H1) {
                uint8_t len = head[2];
                int total_len = len + 6;
                uint8_t seq_byte = head[3];
                uint8_t frame_type = seq_byte & FRAME_TYPE_MASK;

                if (len > (RX_FRAME_MAX_SIZE - 6) || total_len > RX_FRAME_MAX_SIZE) {
                    Serial.printf(">>>> Invalid RX frame len=%u\n", len);
                    ackAndClear();
                    return;
                }

                if (frame_type != FRAME_TYPE_RPT) {
                    return;
                }

                Wire1.beginTransmission(K230_ADDR);
                Wire1.write(0x00);
                Wire1.endTransmission(false);

                if (Wire1.requestFrom(K230_ADDR, static_cast<uint8_t>(total_len)) == total_len) {
                    uint8_t buf[RX_FRAME_MAX_SIZE];
                    for (int i = 0; i < total_len; i++) buf[i] = Wire1.read();

                    uint8_t calc_xor = 0;
                    for (int i = 2; i < total_len - 1; i++) calc_xor ^= buf[i];

                    if (calc_xor == buf[total_len - 1]) {
                        bool cont = (seq_byte & CONT_MASK) != 0;
                        uint8_t func = buf[4];
                        uint8_t payload_len = buf[2];
                        const uint8_t* payload = &buf[5];

                        Serial.printf(">>>> I2C frame: type=0x%02X func=0x%02X, len=%d\n", frame_type, func, payload_len);

                        if (_rx_frag_in_progress && func != _rx_frag_func) {
                            _rx_frag_in_progress = false;
                            _rx_frag_len = 0;
                        }

                        if (!_rx_frag_in_progress) {
                            _rx_frag_func = func;
                            _rx_frag_len = 0;
                        }

                        if (_rx_frag_len + payload_len <= RX_FRAG_MAX) {
                            memcpy(&_rx_frag_buf[_rx_frag_len], payload, payload_len);
                            _rx_frag_len += payload_len;
                        } else {
                            _rx_frag_in_progress = false;
                            _rx_frag_len = 0;
                            Serial.println(">>>> RX fragment overflow");
                        }

                        if (cont) {
                            _rx_frag_in_progress = true;
                            _rx_frag_time = now;
                        } else {
                            handleReport(_rx_frag_func, _rx_frag_buf, _rx_frag_len);
                            _rx_frag_in_progress = false;
                            _rx_frag_len = 0;
                        }
                        ackAndClear();
                    } else {
                        Serial.println(">>>> RX xor mismatch");
                        ackAndClear();
                    }
                }
            } else if (head[0] == 0x00 && head[1] == 0x00) {
                if (_currentState == LLM_STATE_RUNNING && (now - _last_poll_time > 20)) {
                    sendPacket(CMD_REQUEST_STATUS, NULL, 0);
                    _last_poll_time = now;
                }
            }
        }
    }
}


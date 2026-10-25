#include "AprilTagTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"

#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10 
#define CMD_DISABLE_DRAW         0x13 
#define CMD_DISABLE_DRAW_TEXT    0x14 
#define CMD_SET_SIMPLE_RESULT    0x15 
#define CMD_DISABLE_RUN          0x16 

#define RPT_HEARTBEAT         0x70
#define RPT_DETECT_BBOX       0x72
#define APP_APRILTAG          32   

// 移动指令发送间隔
#define MIN_SEND_INTERVAL     15

AprilTagTracker aprilTagTracker;

void AprilTagTracker::begin() {
    _busy = false;
    _seq = 0;
    _last_ex = 0; _last_ey = 0; _last_ew = 0;
    _f_tx = 0; _f_ty = 200; _f_tz = 200; 
    
    // 保持和参考代码一致
    Wire1.setClock(400000);
}

void AprilTagTracker::start() {
    if (_busy) return;
    _busy = true;
    _seq = 0;
    
    ackAndClear();

    _f_tx = 0; _f_ty = 200; _f_tz = 200;
    arm.move(0, 200, 200, 0, 0, 0, 500); 
    
    _ignore_data_until = millis() + 1500;

    _currentState = T_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_config_retry = millis();
    _isBufferEmpty = false;

    Serial.println("[TAG] Start.");
}

void AprilTagTracker::stop() {
    _busy = false;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
    Serial.println("[TAG] Stop.");
}

void AprilTagTracker::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for(int i=0; i<8; i++) Wire1.write(0x00); 
    Wire1.endTransmission();
    _isBufferEmpty = true;
}

void AprilTagTracker::sendPacket(uint8_t func, uint8_t* payload, uint8_t len) {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00); 
    Wire1.write(K230_H0);
    Wire1.write(K230_H1);
    Wire1.write(len);
    
    uint8_t seq = _seq++ & 0x1F;
    Wire1.write(seq);
    Wire1.write(func);
    
    uint8_t xor_val = len ^ seq ^ func;
    if (payload != NULL && len > 0) {
        for (int i = 0; i < len; i++) {
            Wire1.write(payload[i]);
            xor_val ^= payload[i];
        }
    }
    Wire1.write(xor_val);
    Wire1.endTransmission();
    
    _isBufferEmpty = false;
    // Serial.printf("TX: 0x%02X\n", func);
}

// 完全照搬参考代码的 forceConfig，包括 delay(30)
// 这些延时对 K230 接收配置至关重要
void AprilTagTracker::forceConfig() {
    Serial.println("[TAG] Force Config...");
    uint8_t p_on[] = {0x01};
    uint8_t p_off[] = {0x00};
    uint8_t p_thresh[] = {5}; 

    sendPacket(CMD_SET_CONF_THRESH, p_thresh, 1);   delay(30);
    sendPacket(CMD_SET_SIMPLE_RESULT, p_on, 1);     delay(30);
    sendPacket(CMD_DISABLE_DRAW, p_off, 1);         delay(30); 
    sendPacket(CMD_DISABLE_DRAW_TEXT, p_off, 1);    delay(30);
    sendPacket(CMD_DISABLE_RUN, p_off, 1);          delay(30); 
}

void AprilTagTracker::parseFrame(uint8_t* data, int total_len) {
    static unsigned long last_move_cmd_time = 0;
    uint8_t p_len = data[2];
    uint8_t func = data[4];
    uint8_t* payload = &data[5];

    if (func == RPT_DETECT_BBOX) {
        uint8_t count = payload[0];
        
        // 调试打印：确认是否一直收到 BBOX
        // Serial.printf("[TAG] Detect: %d\n", count);

        if (count > 0) {
            int offset = 1;
            uint16_t x = (payload[offset] << 8) | payload[offset+1];
            uint16_t y = (payload[offset+2] << 8) | payload[offset+3];
            uint16_t w = (payload[offset+4] << 8) | payload[offset+5];
            uint16_t h = (payload[offset+6] << 8) | payload[offset+7];
            offset += 8;

            int id = -1;
            if (offset < p_len) {
                uint8_t ext_cnt = payload[offset++];
                for (int e=0; e<ext_cnt; e++) {
                    if (offset >= p_len) break;
                    uint8_t type = payload[offset++];
                    if (type == 3) { 
                        id = (int16_t)((payload[offset]<<8)|payload[offset+1]); 
                        offset+=2; 
                    } else if (type == 1) { 
                        uint8_t slen = payload[offset++]; 
                        offset += slen; 
                    } else { 
                        offset += 2; 
                    }
                }
            }
            
            // Serial.printf("[TAG] ID:%d X:%d Y:%d\n", id, x, y);

            if (millis() < _ignore_data_until) return;

            int cx = x + w / 2;
            int cy = y + h / 2;
            int ex = 160 - cx;          
            int ey = 120 - cy;          
            int ew = target_width - w; 

            float out_x = (float)ex * kp + (float)(ex - _last_ex) * kd;
            float out_z = (float)ey * kp + (float)(ey - _last_ey) * kd;
            float out_y = (float)ew * kp * 1.5f + (float)(ew - _last_ew) * kd;

            _last_ex = ex; _last_ey = ey; _last_ew = ew;
            
            _f_tx -= out_x; 
            _f_tz += out_z; 
            _f_ty += out_y;

            _f_tx = constrain(_f_tx, -400.0f, 400.0f);
            _f_ty = constrain(_f_ty, 100.0f, 350.0f); 
            _f_tz = constrain(_f_tz, 60.0f, 500.0f); 

            unsigned long now = millis();
            if (now - last_move_cmd_time >= MIN_SEND_INTERVAL) {
                if (abs(ex) > 2 || abs(ey) > 2 || abs(ew) > 2) {
                    arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, 0);
                    last_move_cmd_time = now;
                }
            }
        }
    } 
    else if (func == RPT_HEARTBEAT) {
        uint8_t m = payload[0];
        // Serial.printf("[TAG] Heartbeat Mode: %d\n", m);
        if (_currentState == T_STATE_WAIT_READY && m == APP_APRILTAG) {
            _currentState = T_STATE_CONFIG_PARAMS;
            _last_state_time = millis();
        }
    }
}

void AprilTagTracker::update() {
    if (!_busy) return;

    unsigned long now = millis();

    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) == 0) {
        if (Wire1.requestFrom(K230_ADDR, (uint8_t)5) == 5) {
            uint8_t head[5];
            for (int i=0; i<5; i++) head[i] = Wire1.read();

            if (head[0] == K230_H0 && head[1] == K230_H1) {
                _isBufferEmpty = false; // 缓冲区有数据，本轮禁止写入
                if (head[4] >= 0x40) {
                    int total_len = 6 + head[2];
                    Wire1.beginTransmission(K230_ADDR); Wire1.write(0x00); Wire1.endTransmission(false);
                    
                    if (Wire1.requestFrom(K230_ADDR, (uint8_t)total_len) == total_len) {
                        uint8_t buf[128];
                        if(total_len > 128) total_len = 128;
                        
                        for (int i=0; i<total_len; i++) buf[i] = Wire1.read();
                        
                        uint8_t calc_xor = 0;
                        for(int i=2; i<total_len-1; i++) calc_xor ^= buf[i];
                        
                        if(calc_xor == buf[total_len-1]) {
                            parseFrame(buf, total_len);
                            ackAndClear();
                        } else {
                            Serial.println("[TAG] CRC Error");
                            ackAndClear();
                        }
                    }
                }
            } else { 
                _isBufferEmpty = true; 
            }
        }
    }

    if (_isBufferEmpty) {
        switch (_currentState) {
            case T_STATE_SET_MODE:
                if (now - _last_state_time > 1000) {
                    uint8_t p[] = {APP_APRILTAG};
                    sendPacket(CMD_SET_MODE, p, 1);
                    _last_state_time = now;
                    _currentState = T_STATE_WAIT_READY;
                    Serial.println("[TAG] Set Mode -> Wait Ready");
                }
                break;

            case T_STATE_CONFIG_PARAMS:
                forceConfig();
                _currentState = T_STATE_RUNNING;
                _last_state_time = now;
                _last_config_retry = now;
                Serial.println("[TAG] Config -> Running");
                break;

            case T_STATE_RUNNING:
                if (now - _last_config_retry > 5000) {
                    forceConfig();
                    _last_config_retry = now;
                }
                if (now - _last_poll_time > 100) {
                    sendPacket(CMD_REQUEST_STATUS, NULL, 0);
                    _last_poll_time = now;
                }
                break;
                
            default: break;
        }
    }
}
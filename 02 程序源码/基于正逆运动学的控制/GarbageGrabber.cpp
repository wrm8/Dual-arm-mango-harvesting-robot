#include "GarbageGrabber.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include "EspNow_Ctrl.h"
#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10 
#define CMD_DISABLE_DRAW         0x13 
#define CMD_DISABLE_DRAW_TEXT    0x14 
#define CMD_SET_SIMPLE_RESULT    0x15 
#define CMD_DISABLE_RUN          0x16 

#define RPT_HEARTBEAT         0x70
#define RPT_DETECT_BBOX       0x72
#define APP_GARBAGE           17   

#define MIN_SEND_INTERVAL     15

GarbageGrabber garbageGrabber;

void GarbageGrabber::begin() {
    _busy = false;
    _seq = 0;
    _last_ex = 0; _last_ey = 0;
    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z; 
    Wire1.setClock(400000);
}

void GarbageGrabber::start() {
    if (_busy) return;
    _busy = true;
    _seq = 0;
    _stableCount = 0;
    
    ackAndClear();

    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z; 
    arm.move(_f_tx, _f_ty, _f_tz, S_P, 0, 0, 1000); 
    
    delay(1500);
    
    servo.write_pos_ex(6, 0, 2500, (int16_t)C_OPEN);
    delay(100);
    
    _ignore_data_until = millis() + 1500;

    _currentState = GB_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_config_retry = millis();
    _isBufferEmpty = false; 
    _cmd_sent = false;
    Serial.println("[GARBAGE] Start.");
}

void GarbageGrabber::stop() {
    _busy = false;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
    Serial.println("[GARBAGE] Stop.");
}

void GarbageGrabber::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
}

void GarbageGrabber::setThreshold(uint8_t thresh) {
    conf_threshold = thresh;
}

void GarbageGrabber::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for(int i=0; i<8; i++) Wire1.write(0x00); 
    Wire1.endTransmission();
    _isBufferEmpty = true;
}

void GarbageGrabber::sendPacket(uint8_t func, uint8_t* payload, uint8_t len) {
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
}

void GarbageGrabber::forceConfig() {
    Serial.println("[GARBAGE] Config...");
    uint8_t p_on[] = {0x01};
    uint8_t p_off[] = {0x00};
    uint8_t p_thresh[] = {conf_threshold}; 

    sendPacket(CMD_SET_CONF_THRESH, p_thresh, 1);   delay(30);
    sendPacket(CMD_SET_SIMPLE_RESULT, p_on, 1);     delay(30);
    sendPacket(CMD_DISABLE_DRAW, p_off, 1);         delay(30); 
    sendPacket(CMD_DISABLE_DRAW_TEXT, p_off, 1);    delay(30);
    sendPacket(CMD_DISABLE_RUN, p_off, 1);          delay(30); 
}

void GarbageGrabber::parseFrame(uint8_t* data, int total_len) {
    static unsigned long last_move_cmd_time = 0;
    uint8_t p_len = data[2];
    uint8_t func = data[4];
    uint8_t* payload = &data[5];

    if (func == RPT_DETECT_BBOX) {
        if (_currentState != GB_STATE_SEARCH) return;

        uint8_t count = payload[0];
        if (count > 0) {
            int offset = 1;
            uint16_t x = (payload[offset] << 8) | payload[offset+1];
            uint16_t y = (payload[offset+2] << 8) | payload[offset+3];
            uint16_t w = (payload[offset+4] << 8) | payload[offset+5];
            uint16_t h = (payload[offset+6] << 8) | payload[offset+7];
            offset += 8;

            // 过滤全屏误识别框
            if (w > 280 || h > 200) {
                return; 
            }

            int id = -1;
            if (offset < p_len) {
                uint8_t ext_cnt = payload[offset++];
                for (int e=0; e<ext_cnt; e++) {
                    if (offset >= p_len) break;
                    uint8_t type = payload[offset++];
                    if (type == 3) { 
                        id = (int16_t)((payload[offset]<<8)|payload[offset+1]); 
                        offset += 2; 
                    } else if (type == 1) { 
                        uint8_t slen = payload[offset++]; 
                        if (slen > 0) id = payload[offset]; 
                        offset += slen; 
                    } else { 
                        offset += 2; 
                    }
                }
            }
            
            if (millis() < _ignore_data_until) return;

            int cx = x + w / 2;
            int cy = y + h / 2;
            int ex = 160 - cx;          
            int ey = 120 - cy;          

            float out_x = (float)ex * kp + (float)(ex - _last_ex) * kd;
            float out_y = (float)ey * kp + (float)(ey - _last_ey) * kd; 
            
            _last_ex = ex; _last_ey = ey; 
            _f_ty += out_x; 
            _f_tx += out_y; 

            _f_tx = constrain(_f_tx, 100.0f, 350.0f);
            _f_ty = constrain(_f_ty, -180.0f, 180.0f);

            unsigned long now = millis();
            if (now - last_move_cmd_time >= MIN_SEND_INTERVAL) {
                arm.move(_f_tx, _f_ty, _f_tz, S_P, 0, 0, 0);
                last_move_cmd_time = now;
            }

            // 误差必须小于5，且连续稳定 20 帧 (约300ms) 才能抓
            if (abs(ex) < 5 && abs(ey) < 5) {
                _stableCount++;
                if (_stableCount > 20) {
                    Serial.printf("[GARBAGE] Stable! ID: %d -> Grabbing\n", id);
                    _target_id = id; 
                    _currentState = GB_STATE_DOWN;
                    _last_state_time = now;
                }
            } else {
                _stableCount = 0; // 一旦偏离，重新计数
            }
        }
    } 
    else if (func == RPT_HEARTBEAT) {
        uint8_t m = payload[0];
        if (_currentState == GB_STATE_WAIT_READY && m == APP_GARBAGE) {
            _currentState = GB_STATE_CONFIG_PARAMS;
            _cmd_sent = false; 
            _last_state_time = millis();
        }
    }
}

void GarbageGrabber::update() {
    if (!_busy) return;
    unsigned long now = millis();

    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) == 0) {
        if (Wire1.requestFrom(K230_ADDR, (uint8_t)5) == 5) {
            uint8_t head[5];
            for (int i=0; i<5; i++) head[i] = Wire1.read();

            if (head[0] == K230_H0 && head[1] == K230_H1) {
                _isBufferEmpty = false;
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
            case GB_STATE_SET_MODE:
                if (now - _last_state_time > 1000) {
                    uint8_t p[] = {APP_GARBAGE};
                    sendPacket(CMD_SET_MODE, p, 1);
                    _last_state_time = now;
                    _currentState = GB_STATE_WAIT_READY;
                }
                break;

            case GB_STATE_CONFIG_PARAMS:
                forceConfig();
                _currentState = GB_STATE_SEARCH;
                _last_state_time = now;
                _last_config_retry = now;
                break;

            case GB_STATE_SEARCH:
                if (now - _last_config_retry > 5000) { forceConfig(); _last_config_retry = now; }
                if (now - _last_poll_time > 100) { sendPacket(CMD_REQUEST_STATUS, NULL, 0); _last_poll_time = now; }
                break;

            // --- 真正的解耦状态机流 ---
            case GB_STATE_DOWN:
                if (now - _last_state_time > 500) {
                     _f_tx += X_COMP; _f_ty += Y_COMP; _f_tz = GRAB_Z;
                     arm.move(_f_tx, _f_ty, _f_tz, S_P, 0, 0, 2000); // 仅下移
                     _currentState = GB_STATE_GRAB; _last_state_time = now;
                }
                break;

            case GB_STATE_GRAB:
                if (now - _last_state_time > 2000) {
                    // 单独指令：闭合爪子
                    servo.write_pos_ex(6, 0, 2500, (int16_t)C_CLOSE);
                    _currentState = GB_STATE_LIFT; _last_state_time = now;
                }
                break;

            case GB_STATE_LIFT:
                if (now - _last_state_time > 500) { // 等爪子抓紧
                    _f_tz = LIFT_Z;
                    arm.move(_f_tx, _f_ty, _f_tz, S_P, 0, 0, 1500); // 仅抬起
                    _currentState = GB_STATE_PLACE; _last_state_time = now;
                }
                break;

            case GB_STATE_PLACE:
                if (now - _last_state_time > 1500) {
                    float place_x = 0.0f; 
                    int bin = (_target_id > 0) ? (_target_id % 4) : 0;
                    if (bin == 0) place_x = -150.0f;
                    else if (bin == 1) place_x = -50.0f;
                    else if (bin == 2) place_x = 50.0f;
                    else if (bin == 3) place_x = 150.0f;
                    
                    arm.move(place_x, P_Y, P_Z, S_P, 0, 0, 2000); // 仅移动位置
                    _currentState = GB_STATE_RELEASE; _last_state_time = now;
                }
                break;

            case GB_STATE_RELEASE:
                if (now - _last_state_time > 2000) {
                    // 单独指令：松开爪子
                    servo.write_pos_ex(6, 0, 2500, (int16_t)C_OPEN);
                    _currentState = GB_STATE_RESET; _last_state_time = now;
                }
                break;

            case GB_STATE_RESET:
                if (now - _last_state_time > 500) { // 等爪子彻底松开
                    arm.move(S_X, S_Y, S_Z, S_P, 0, 0, 2000); // 机械臂复位
                    stop(); 
                }
                break;
                
            default: break;
        }
    }
}

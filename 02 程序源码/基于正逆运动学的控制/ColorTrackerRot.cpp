#include "ColorTrackerRot.h"
#include <Wire.h>
#include "Robot_Arm.h"

#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10 
#define CMD_DISABLE_DRAW         0x13 
#define CMD_DISABLE_DRAW_TEXT    0x14 
#define CMD_SET_SIMPLE_RESULT    0x15 
#define CMD_DISABLE_RUN          0x16 
#define CMD_COLOR_SET_TARGET     0x30

#define RPT_HEARTBEAT         0x70
#define RPT_DETECT_COLOR      0x75
#define APP_SINGLE_COLOR      21

#define MIN_SEND_INTERVAL     15

ColorTrackerRot colorTrackerRot;

void ColorTrackerRot::begin() {
    _busy = false;
    _seq = 0;
    _last_ex = 0; _last_ey = 0;
    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z; _f_tr = 0;
    Wire1.setClock(400000);
}

void ColorTrackerRot::start(const char* colorName) {
    if (_busy) return;
    _busy = true;
    _seq = 0;
    _stableCount = 0;
    _target_color = String(colorName);
    
    ackAndClear();

    _f_tx = S_X; _f_ty = S_Y; _f_tz = S_Z; _f_tr = 0;
    arm.move(_f_tx, _f_ty, _f_tz, S_P, _f_tr, 0, 1000); 
    
    delay(1500);
    servo.write_pos_ex(6, 0, 2500, (int16_t)C_OPEN);
    delay(100);
    
    _ignore_data_until = millis() + 1500;

    _currentState = CR_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    _isBufferEmpty = false;
    _cmd_sent = false;
}

void ColorTrackerRot::stop() {
    _busy = false;
    _cmd_sent = false;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
}

void ColorTrackerRot::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
}

void ColorTrackerRot::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for(int i=0; i<8; i++) Wire1.write(0x00); 
    Wire1.endTransmission();
    _isBufferEmpty = true;
}

void ColorTrackerRot::sendPacket(uint8_t func, uint8_t* payload, uint8_t len) {
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

void ColorTrackerRot::forceConfig() {
    uint8_t p_off[] = {0x00};
    uint8_t p_thresh[] = {15}; 
    uint8_t p_simple[] = {0x00}; 

    sendPacket(CMD_DISABLE_DRAW, p_off, 1); delay(20);
    sendPacket(CMD_DISABLE_DRAW_TEXT, p_off, 1); delay(20);
    
    uint8_t color_buf[32];
    uint8_t n_len = _target_color.length();
    if(n_len > 30) n_len = 30;
    color_buf[0] = n_len;
    memcpy(&color_buf[1], _target_color.c_str(), n_len);
    sendPacket(CMD_COLOR_SET_TARGET, color_buf, n_len + 1); delay(20);
    
    sendPacket(CMD_SET_CONF_THRESH, p_thresh, 1); delay(20);
    sendPacket(CMD_SET_SIMPLE_RESULT, p_simple, 1); delay(20);
    sendPacket(CMD_DISABLE_RUN, p_off, 1); delay(20); 
}

void ColorTrackerRot::parseFrame(uint8_t* data, int total_len) {
    static unsigned long last_move_cmd_time = 0;
    uint8_t p_len = data[2];
    uint8_t func = data[4];
    uint8_t* payload = &data[5];

    if (func == RPT_DETECT_COLOR) {
        if (_currentState != CR_STATE_SEARCH) return;

        int offset = 0;
        if (p_len < 3) return;
        offset++; 
        uint8_t name_len = payload[offset++];
        offset += name_len; 
        
        if (offset > p_len) return;

        int remaining = p_len - offset;
        uint8_t count = (remaining == 10) ? 1 : payload[offset++];

        if (count > 0 && count < 10) { 
            if (offset + 10 <= p_len) {
                uint16_t x = (payload[offset] << 8) | payload[offset+1];
                uint16_t y = (payload[offset+2] << 8) | payload[offset+3];
                uint16_t w = (payload[offset+4] << 8) | payload[offset+5];
                uint16_t h = (payload[offset+6] << 8) | payload[offset+7];
                int16_t angle_val = (int16_t)((payload[offset+8] << 8) | payload[offset+9]);
                
                if (w > 280 || h > 200) return;
                
                if (millis() < _ignore_data_until) return;

                int cx = x + w / 2;
                int cy = y + h / 2;
                int ex = 160 - cx;
                int ey = 120 - cy;
                float angle = (float)angle_val - 90.0f; 

                Serial.printf("CX:%d CY:%d EX:%d EY:%d\n", cx, cy, ex, ey);

                float ox = (float)ex * kp + (float)(ex - _last_ex) * kd;
                float oz = (float)ey * kp + (float)(ey - _last_ey) * kd;
                _last_ex = ex; _last_ey = ey;
                
                _f_tx -= oz; 
                _f_ty += ox; 
                _f_tr = angle; 
                
                _f_tx = constrain(_f_tx, 100.0f, 350.0f); // X是前后
                _f_ty = constrain(_f_ty, -180.0f, 180.0f); // Y是左右
                _f_tr = constrain(_f_tr, -150.0f, 150.0f);
                
                unsigned long now = millis();
                if (now - last_move_cmd_time >= MIN_SEND_INTERVAL) {
                    arm.move(_f_tx, _f_ty, _f_tz, S_P, _f_tr, 0, 0);
                    last_move_cmd_time = now;
                }

                if (abs(ex) < 5 && abs(ey) < 5) {
                    if (++_stableCount > 20) {
                        _currentState = CR_STATE_DOWN;
                        _last_state_time = now;
                    }
                } else {
                    _stableCount = 0;
                }
            }
        }
    } 
    else if (func == RPT_HEARTBEAT) {
        uint8_t m = payload[0];
        if (_currentState == CR_STATE_WAIT_READY && m == APP_SINGLE_COLOR) {
            _currentState = CR_STATE_CONFIG_PARAMS;
            _cmd_sent = false; 
            _last_state_time = millis();
        }
    }
}

void ColorTrackerRot::update() {
    if (!_busy) return;

    unsigned long now = millis();
    static unsigned long last_i2c_access = 0;
    if (now - last_i2c_access < 5) return; 
    last_i2c_access = now;

    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) == 0) {
        if (Wire1.requestFrom(K230_ADDR, (uint8_t)5) == 5) {
            uint8_t head[5];
            for (int i = 0; i < 5; i++) head[i] = Wire1.read();

            if (head[0] == K230_H0 && head[1] == K230_H1) {
                _isBufferEmpty = false;
                if (head[4] >= 0x40) {
                    int total_len = head[2] + 6;
                    Wire1.beginTransmission(K230_ADDR); Wire1.write(0x00); Wire1.endTransmission(false);
                    if (Wire1.requestFrom(K230_ADDR, (uint8_t)total_len) == total_len) {
                        uint8_t buf[128];
                        if(total_len > 128) total_len = 128;
                        for (int i = 0; i < total_len; i++) buf[i] = Wire1.read();
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
            } 
            else if (head[0] == 0x00 && head[1] == 0x00) {
                _isBufferEmpty = true;
            }
        }
    }

    if (_isBufferEmpty) {
        switch (_currentState) {
            case CR_STATE_SET_MODE:
                if (!_cmd_sent) {
                    uint8_t p[] = {APP_SINGLE_COLOR};
                    sendPacket(CMD_SET_MODE, p, 1);
                    _cmd_sent = true;
                    _last_state_time = now;
                }
                if (now - _last_state_time > 1000) {
                    _cmd_sent = false;
                    _currentState = CR_STATE_WAIT_READY; 
                    _last_state_time = now;
                }
                break;

            case CR_STATE_CONFIG_PARAMS:
                if (!_cmd_sent) {
                    forceConfig();
                    _cmd_sent = true;
                    _last_state_time = now;
                }
                if (now - _last_state_time > 200) {
                    _currentState = CR_STATE_SEARCH;
                    _last_state_time = now;
                    _last_config_retry = now; 
                }
                break;

            case CR_STATE_SEARCH:
                if (now - _last_config_retry > 5000) { 
                    forceConfig(); 
                    _last_config_retry = now; 
                }
                if (now - _last_poll_time > 500) { 
                    sendPacket(CMD_REQUEST_STATUS, NULL, 0); 
                    _last_poll_time = now; 
                }
                break;

            case CR_STATE_DOWN:
                if (now - _last_state_time > 500) {
                     _f_tx += X_COMP; _f_ty += Y_COMP; _f_tz = GRAB_Z;
                     arm.move(_f_tx, _f_ty, _f_tz, S_P, _f_tr, 0, 2000);
                     _currentState = CR_STATE_GRAB; _last_state_time = now;
                }
                break;

            case CR_STATE_GRAB:
                if (now - _last_state_time > 2000) {
                    servo.write_pos_ex(6, 0, 2500, (int16_t)C_CLOSE);
                    _currentState = CR_STATE_LIFT; _last_state_time = now;
                }
                break;

            case CR_STATE_LIFT:
                if (now - _last_state_time > 500) {
                    _f_tz = LIFT_Z;
                    arm.move(_f_tx, _f_ty, _f_tz, S_P, _f_tr, 0, 1500);
                    _currentState = CR_STATE_PLACE; _last_state_time = now;
                }
                break;

            case CR_STATE_PLACE:
                if (now - _last_state_time > 1500) {
                    float place_x = 0.0f;
                    if (_target_color == "red") place_x = -150.0f;
                    else if (_target_color == "green") place_x = 0.0f;
                    else if (_target_color == "blue") place_x = 150.0f;
                    arm.move(place_x, P_Y, P_Z, S_P, 0, 0, 2000);
                    _currentState = CR_STATE_RELEASE; _last_state_time = now;
                }
                break;

            case CR_STATE_RELEASE:
                if (now - _last_state_time > 2000) {
                    servo.write_pos_ex(6, 0, 2500, (int16_t)C_OPEN);
                    _currentState = CR_STATE_RESET; _last_state_time = now;
                }
                break;

            case CR_STATE_RESET:
                if (now - _last_state_time > 500) {
                    arm.move(S_X, S_Y, S_Z, S_P, 0, 0, 2000);
                    stop(); 
                }
                break;
                
            default: break;
        }
    }
}

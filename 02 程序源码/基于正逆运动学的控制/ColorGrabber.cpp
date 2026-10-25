#include "ColorGrabber.h"
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
#define MOVE_INTERVAL         20

ColorGrabber colorGrabber;

const float smooth_factor = 0.4f;

void ColorGrabber::begin() {
    _busy = false;
    _seq = 0;
    _last_ex = 0; _last_ey = 0;
    _f_tx = 0; _f_tz = 200;  
    Wire1.setClock(400000);
}

void ColorGrabber::start(String colorName) {
    if (_busy) return;
    _busy = true;
    _target_color = colorName;
    _seq = 0;
    _last_ex = 0; _last_ey = 0;
    
    ackAndClear();

    _f_tx = 200; 
    float start_y = 0; 
    _f_tz = 200;
    arm.move(_f_tx, start_y, _f_tz, 0, 0, 0, 1000); 
    
    _ignore_data_until = millis() + 1500;

    _currentState = C_STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    _isBufferEmpty = false;
    _cmd_sent = false;
    
    Serial.print("[COLOR] Start: ");
    Serial.println(colorName);
}

void ColorGrabber::stop() {
    _busy = false;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
    Serial.println("[COLOR] Stop.");
}

void ColorGrabber::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for(int i=0; i<8; i++) Wire1.write(0x00); 
    Wire1.endTransmission();
    _isBufferEmpty = true;
}

void ColorGrabber::sendPacket(uint8_t func, uint8_t* payload, uint8_t len) {
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

void ColorGrabber::forceConfig() {
    Serial.printf("[COLOR] Config %s\n", _target_color.c_str());
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

void ColorGrabber::runStateMachine(uint8_t mode, uint8_t status) {
    bool isRun = status & 0x01;
    bool isReady = (status >> 2) & 0x01;

    if (_currentState == C_STATE_WAIT_READY) {
        if (mode == APP_SINGLE_COLOR && isReady) {
            _currentState = C_STATE_CONFIG_PARAMS;
            _last_state_time = millis();
            _cmd_sent = false;
        } else if (mode != APP_SINGLE_COLOR && millis() - _last_state_time > 2000) {
            _currentState = C_STATE_SET_MODE;
            _last_state_time = millis();
            _cmd_sent = false;
        }
    } else if (_currentState == C_STATE_RUNNING) {
        if (!isRun && millis() - _last_state_time > 3000) {
            _currentState = C_STATE_ENABLE_RUN;
            _last_state_time = millis();
            _cmd_sent = false;
        }
    }
}

void ColorGrabber::parseFrame(uint8_t* data, int total_len) {
    static unsigned long last_move_cmd_time = 0;
    uint8_t p_len = data[2];
    uint8_t func = data[4];
    uint8_t* payload = &data[5];

    if (func == RPT_DETECT_COLOR) {
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
                
                if (millis() < _ignore_data_until) return;

                int ex = 160 - (x + w / 2);
                int ey = 120 - (y + h / 2);
                
                float ox = (float)ex * kp + (float)(ex - _last_ex) * kd;
                float oz = (float)ey * kp + (float)(ey - _last_ey) * kd;
                _last_ex = ex; _last_ey = ey;
                static float _f_ty = 0; 
                float target_tx = _f_tx + oz; 
                float target_ty = _f_ty - ox; 
                
                target_tx = constrain(target_tx, 100.0f, 350.0f); // X前后
                target_ty = constrain(target_ty, -180.0f, 180.0f); // Y左右

                 _f_tx = _f_tx * (1.0f - smooth_factor) + target_tx * smooth_factor;
                _f_ty = _f_ty * (1.0f - smooth_factor) + target_ty * smooth_factor;
                
                unsigned long now = millis();
                if (now - last_move_cmd_time >= MIN_SEND_INTERVAL) {
                    if (abs(ex) > 3 || abs(ey) > 3) {
                        arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, 0);
                        last_move_cmd_time = now;
                    }
                }
            }
        }
        _last_state_time = millis();
        if(_currentState != C_STATE_RUNNING) _currentState = C_STATE_RUNNING;
    } 
    else if (func == RPT_HEARTBEAT) {
        runStateMachine(payload[0], payload[1]);
    }
}

void ColorGrabber::update() {
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
                if (now - _last_poll_time > 100) { 
                    sendPacket(CMD_REQUEST_STATUS, NULL, 0); 
                    _last_poll_time = now;
                }
            }
        }
    }

    if (_isBufferEmpty) {
        switch (_currentState) {
            case C_STATE_SET_MODE:
                if (!_cmd_sent) {
                    uint8_t p[] = {APP_SINGLE_COLOR};
                    sendPacket(CMD_SET_MODE, p, 1);
                    _cmd_sent = true;
                    _last_state_time = now;
                }
                if (now - _last_state_time > 1000) {
                    _cmd_sent = false;
                    _currentState = C_STATE_WAIT_READY; 
                    _last_state_time = now;
                }
                break;

            case C_STATE_CONFIG_PARAMS:
                if (!_cmd_sent) {
                    forceConfig();
                    _cmd_sent = true;
                    _last_state_time = now;
                }
                if (now - _last_state_time > 200) {
                    _currentState = C_STATE_ENABLE_RUN;
                    _last_state_time = now;
                    _cmd_sent = false;
                }
                break;

            case C_STATE_ENABLE_RUN:
                if (!_cmd_sent) {
                    uint8_t p[] = {0x00}; 
                    sendPacket(CMD_DISABLE_RUN, p, 1);
                    _cmd_sent = true;
                    _last_state_time = now;
                }
                if (now - _last_state_time > 500) {
                    _currentState = C_STATE_RUNNING; 
                    _last_state_time = now;
                }
                break;
                
            case C_STATE_RUNNING:
                break;
                
            default: break;
        }
    }
}
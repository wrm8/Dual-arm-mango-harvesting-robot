#include "FaceTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"

#define CMD_SET_MODE             0x01
#define CMD_REQUEST_STATUS       0x04
#define CMD_SET_CONF_THRESH      0x10 
#define CMD_DISABLE_DRAW         0x13 
#define CMD_DISABLE_DRAW_TEXT    0x14 
#define CMD_SET_SIMPLE_RESULT    0x15 
#define CMD_DISABLE_RUN          0x16 
#define CMD_FACE_LIVENESS_THRESH 0x25 

#define RPT_HEARTBEAT         0x70
#define RPT_DETECT_BBOX       0x72

#define TARGET_MODE           0x01

#define MIN_SEND_INTERVAL     20

FaceTracker faceTracker;

void FaceTracker::begin() {
    _busy = false; 
    _seq = 0;
    _last_ex = 0; _last_ey = 0;
    _f_tx = 200; _f_ty = 0; _f_tz = 200;  
    Wire1.setClock(400000);
}

void FaceTracker::start() {
    if (_busy) return;
    _busy = true;
    _seq = 0;
    _last_ex = 0; _last_ey = 0;
    
    ackAndClear();

    _f_tx = 0;
    _f_tz = 200;
    arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, 500);  

    _currentState = STATE_SET_MODE;
    _last_state_time = millis();
    _last_poll_time = millis();
    _isBufferEmpty = false;
    _cmd_sent = false;
}

void FaceTracker::stop() {
    _busy = false;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
}

void FaceTracker::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for(int i=0; i<8; i++) Wire1.write(0x00); 
    Wire1.endTransmission();
    _isBufferEmpty = true;
}

void FaceTracker::sendPacket(uint8_t func, uint8_t* payload, uint8_t len) {
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

void FaceTracker::forceConfig() {
    uint8_t p_off[] = {0x00};
    uint8_t p_on[] = {0x01};
    uint8_t p_thresh[] = {40};
    
    sendPacket(CMD_DISABLE_DRAW, p_off, 1); delay(20);
    sendPacket(CMD_DISABLE_DRAW_TEXT, p_off, 1); delay(20);
    sendPacket(CMD_SET_CONF_THRESH, p_thresh, 1); delay(20);
    sendPacket(CMD_SET_SIMPLE_RESULT, p_on, 1); delay(20);
    sendPacket(CMD_DISABLE_RUN, p_off, 1); delay(20);
}

void FaceTracker::parseFrame(uint8_t* data, int total_len) {
    static unsigned long last_move_cmd_time = 0;
    uint8_t func = data[4];
    uint8_t* payload = &data[5];

    if (func < 0x40) return;

    if (func == RPT_DETECT_BBOX) {
        uint8_t count = payload[0];
        if (count > 0) {
            int offset = 1;
            uint16_t x = (payload[offset] << 8) | payload[offset+1];
            uint16_t y = (payload[offset+2] << 8) | payload[offset+3];
            uint16_t w = (payload[offset+4] << 8) | payload[offset+5];
            uint16_t h = (payload[offset+6] << 8) | payload[offset+7];
            
            int ex = 160 - (x + w / 2);
            int ey = 120 - (y + h / 2);
            
            float ox = (float)ex * kp + (float)(ex - _last_ex) * kd;
            float oz = (float)ey * kp + (float)(ey - _last_ey) * kd;
            _last_ex = ex; _last_ey = ey;
                    
            _f_ty -= ox; 
            _f_tz += oz; 
            
            _f_ty = constrain(_f_ty, -180.0f, 180.0f); // Y轴左右摆动范围
            _f_tz = constrain(_f_tz, 60.0f, 500.0f);   // Z轴高度范围
            
            unsigned long now = millis();
            if (now - last_move_cmd_time >= MIN_SEND_INTERVAL) {
                if (abs(ex) > 2 || abs(ey) > 2) {
                    arm.move(_f_tx, _f_ty, _f_tz, 0, 0, 0, 0); 
                    last_move_cmd_time = now;
                }
            }
        } 
        
        _currentState = STATE_RUNNING;
        _last_state_time = millis();
    } 
    else if (func == RPT_HEARTBEAT) {
        uint8_t m = payload[0];
        if (_currentState == STATE_WAIT_READY && m == TARGET_MODE) {
            _currentState = STATE_CONFIG_PARAMS;
            _last_state_time = millis();
        }
    }
}

void FaceTracker::update() {
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
            else {
                _isBufferEmpty = true;
            }
        }
    }

    if (_isBufferEmpty) {
        switch (_currentState) {
            case STATE_SET_MODE:
                if (now - _last_state_time > 1000) {
                    uint8_t p[] = {TARGET_MODE};
                    sendPacket(CMD_SET_MODE, p, 1);
                    _currentState = STATE_WAIT_READY;
                    _last_state_time = now;
                }
                break;

            case STATE_CONFIG_PARAMS:
                forceConfig();
                _currentState = STATE_RUNNING;
                _last_state_time = now;
                _last_config_retry = now;
                break;

            case STATE_RUNNING:
                if (now - _last_config_retry > 5000) {
                    forceConfig();
                    _last_config_retry = now;
                }
                if (now - _last_poll_time > 50) { 
                    sendPacket(CMD_REQUEST_STATUS, NULL, 0); 
                    _last_poll_time = now;
                }
                break;
                
            default: break;
        }
    }
}

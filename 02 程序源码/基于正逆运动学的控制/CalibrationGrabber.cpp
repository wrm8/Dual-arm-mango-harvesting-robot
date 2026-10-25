#include "CalibrationGrabber.h"
#include <Wire.h>
#include "Robot_Arm.h"
#include "usb_ctrl.h"
#include "Espnow_ctrl.h"

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

CalibrationGrabber calibrationGrabber;

void CalibrationGrabber::begin() {
    _busy = false;
    _seq = 0;
    Wire1.setClock(400000);
}

void CalibrationGrabber::start() {
    if (_busy) return;
    _busy = true;
    _seq = 0;
    
    ackAndClear();

    arm.move(S_X, S_Y, S_Z, S_P, 0, 0, 1000); 
    delay(1500);
    servo.write_pos_ex(6, 0, 2500, (int16_t)C_OPEN);
    delay(100);
    
    uint8_t p[] = {APP_APRILTAG};
    sendPacket(CMD_SET_MODE, p, 1);

    _ignore_data_until = millis() + 1500;
    _last_print_time = 0;

    _currentState = CB_STATE_WAIT_READY; 
    _last_state_time = millis();
    _last_poll_time = millis();
    _last_config_retry = millis();
    _isBufferEmpty = false; 
    _cmd_sent = false;
}

void CalibrationGrabber::grab() {
    if (!_busy) return;
    if (_currentState == CB_STATE_SEARCH) {
        _currentState = CB_STATE_DOWN;
        _last_state_time = millis();
    }
}

void CalibrationGrabber::resetArm() {
    if (!_busy) return;
    if (_currentState == CB_STATE_HOLD) {
        _currentState = CB_STATE_RELEASE;
        _last_state_time = millis();
    }
}

void CalibrationGrabber::stop() {
    _busy = false;
    uint8_t run_disable = 1;
    sendPacket(CMD_DISABLE_RUN, &run_disable, 1);
}

void CalibrationGrabber::setOffsets(float x_off, float y_off, float z_grab) {
    X_COMP = x_off; Y_COMP = y_off; GRAB_Z = z_grab;
}

void CalibrationGrabber::ackAndClear() {
    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    for(int i=0; i<8; i++) Wire1.write(0x00); 
    Wire1.endTransmission();
    _isBufferEmpty = true;
}

void CalibrationGrabber::sendPacket(uint8_t func, uint8_t* payload, uint8_t len) {
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

void CalibrationGrabber::forceConfig() {
    uint8_t p_on[] = {0x01};
    uint8_t p_off[] = {0x00};
    uint8_t p_thresh[] = {5}; 

    sendPacket(CMD_SET_CONF_THRESH, p_thresh, 1);   delay(30);
    sendPacket(CMD_SET_SIMPLE_RESULT, p_on, 1);     delay(30);
    sendPacket(CMD_DISABLE_DRAW, p_off, 1);         delay(30); 
    sendPacket(CMD_DISABLE_DRAW_TEXT, p_off, 1);    delay(30);
    sendPacket(CMD_DISABLE_RUN, p_off, 1);          delay(30); 
}

void CalibrationGrabber::parseFrame(uint8_t* data, int total_len) {
    uint8_t p_len = data[2];
    uint8_t func = data[4];
    uint8_t* payload = &data[5];

    if (func == RPT_DETECT_BBOX) {
        _last_config_retry = millis(); 

        if (_currentState != CB_STATE_SEARCH) return;

        uint8_t count = payload[0];
        if (count > 0) {
            int offset = 1;
            uint16_t x = (payload[offset] << 8) | payload[offset+1];
            uint16_t y = (payload[offset+2] << 8) | payload[offset+3];
            uint16_t w = (payload[offset+4] << 8) | payload[offset+5];
            uint16_t h = (payload[offset+6] << 8) | payload[offset+7];
            
            if (w > 280 || h > 200) return;
            if (millis() < _ignore_data_until) return;

            int cx = x + w / 2;
            int cy = y + h / 2;
            int ex = 160 - cx;          
            int ey = 120 - cy;          

            // unsigned long now = millis();
            // if (now - _last_print_time > 500) {
            //     if (abs(ex) < 5 && abs(ey) < 5) {
            //         Serial.printf("[校准] 完美对准中心！(X偏:%d Y偏:%d) -> 发送 02 指令下落测试\n", ex, ey);
            //     } else {
            //         String str_x = (ex > 0) ? "目标偏左" : (ex < 0 ? "目标偏右" : "X已居中");
            //         String str_y = (ey > 0) ? "目标偏前" : (ey < 0 ? "目标偏后" : "Y已居中");
            //         Serial.printf("[校准] %s %d 像素 | %s %d 像素\n", str_x.c_str(), abs(ex), str_y.c_str(), abs(ey));
            //     }
            //     _last_print_time = now;
            // }
            // unsigned long now = millis();
            // if (now - _last_print_time > 500) {
            //     char msg_buf[256];
            //     int msg_len = 0;

            //     if (abs(ex) < 5 && abs(ey) < 5) {
            //         msg_len = snprintf(msg_buf, sizeof(msg_buf), "[校准] 完美对准中心！(X偏:%d Y偏:%d) -> 发送 02 指令下落测试\r\n", ex, ey);
            //     } else {
            //         String str_x = (ex > 0) ? "目标偏左" : (ex < 0 ? "目标偏右" : "X已居中");
            //         String str_y = (ey > 0) ? "目标偏前" : (ey < 0 ? "目标偏后" : "Y已居中");
            //         msg_len = snprintf(msg_buf, sizeof(msg_buf), "[校准] %s %d 像素 | %s %d 像素\r\n", str_x.c_str(), abs(ex), str_y.c_str(), abs(ey));
            //     }

            //     Serial.print(msg_buf);

            //     espnow_ctrl.sendData((uint8_t*)msg_buf, msg_len);

            //     ble_port.sendData((uint8_t*)msg_buf, msg_len);

            //     _last_print_time = now;
            // }
            unsigned long now = millis();
            if (now - _last_print_time > 500) {
                char msg_buf[256];
                int msg_len = 0;

                if (abs(ex) < 5 && abs(ey) < 5) {
                    msg_len = snprintf(msg_buf, sizeof(msg_buf), "[校准] 完美对准中心！(X偏:%d Y偏:%d) -> 发送 02 指令下落测试\r\n", ex, ey);
                } else {
                    String str_x = (ex > 0) ? "目标偏左" : (ex < 0 ? "目标偏右" : "X已居中");
                    String str_y = (ey > 0) ? "目标偏前" : (ey < 0 ? "目标偏后" : "Y已居中");
                    msg_len = snprintf(msg_buf, sizeof(msg_buf), "[校准] %s %d 像素 | %s %d 像素\r\n", str_x.c_str(), abs(ex), str_y.c_str(), abs(ey));
                }
                Serial.print(msg_buf);
                espnow_ctrl.sendData((uint8_t*)msg_buf, msg_len);
                _last_print_time = now;
            }
        }
    } 
    else if (func == RPT_HEARTBEAT) {
        _last_config_retry = millis();
        uint8_t m = payload[0];
        if ((_currentState == CB_STATE_WAIT_READY || _currentState == CB_STATE_SET_MODE) && m == APP_APRILTAG) {
            _currentState = CB_STATE_CONFIG_PARAMS;
            _cmd_sent = false; 
            _last_state_time = millis();
        }
    }
}

void CalibrationGrabber::update() {
    if (!_busy) return;
    unsigned long now = millis();

    Wire1.beginTransmission(K230_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) == 0) {
        if (Wire1.requestFrom(K230_ADDR, (uint8_t)5) == 5) {
            uint8_t head[5];
            for (int i=0; i<5; i++) head[i] = Wire1.read();

            static unsigned long i2c_stuck_timer = 0;

            if (head[0] == K230_H0 && head[1] == K230_H1) {
                _isBufferEmpty = false;
                if (head[4] >= 0x40) {
                    i2c_stuck_timer = 0; 
                    int total_len = 6 + head[2];
                    Wire1.beginTransmission(K230_ADDR); Wire1.write(0x00); Wire1.endTransmission(false);
                    int bytes_read = Wire1.requestFrom(K230_ADDR, (uint8_t)total_len);
                    if (bytes_read == total_len) {
                        uint8_t buf[128];
                        if(total_len > 128) total_len = 128;
                        for (int i=0; i<total_len; i++) buf[i] = Wire1.read();
                        uint8_t calc_xor = 0;
                        for(int i=2; i<total_len-1; i++) calc_xor ^= buf[i];
                        if(calc_xor == buf[total_len-1]) {
                            parseFrame(buf, total_len);
                        }
                    }
                    ackAndClear();
                } else { 
                    if (i2c_stuck_timer == 0) i2c_stuck_timer = now;
                    else if (now - i2c_stuck_timer > 200) {
                        ackAndClear();
                        i2c_stuck_timer = 0;
                    }
                }
            } else { 
                _isBufferEmpty = true;
                i2c_stuck_timer = 0;
            }
        }
    }

    if (_isBufferEmpty) {
        switch (_currentState) {
            case CB_STATE_SET_MODE:
                if (now - _last_state_time > 1000) {
                    uint8_t p[] = {APP_APRILTAG};
                    sendPacket(CMD_SET_MODE, p, 1);
                    _last_state_time = now;
                    _currentState = CB_STATE_WAIT_READY;
                }
                break;
            case CB_STATE_WAIT_READY:
                if (now - _last_state_time > 3000) {
                    _currentState = CB_STATE_SET_MODE;
                    _last_state_time = now;
                }
                break;
            case CB_STATE_CONFIG_PARAMS:
                forceConfig();
                _currentState = CB_STATE_SEARCH;
                _last_state_time = now;
                _last_config_retry = now;
                break;
            case CB_STATE_SEARCH:
                if (now - _last_config_retry > 5000) { forceConfig(); _last_config_retry = now; }
                if (now - _last_poll_time > 100) { sendPacket(CMD_REQUEST_STATUS, NULL, 0); _last_poll_time = now; }
                break;
            case CB_STATE_DOWN:
                if (now - _last_state_time > 100) {
                    // 先读回当前位姿，保持 roll/claw 不变，避免甩
                    arm.update_status();
                    delay(10);
                    float cur_roll = arm.current_pose.roll;
                    float cur_claw = arm.current_pose.claw;
                    Serial.printf("[Calib DOWN] target: x=%.1f y=%.1f z=%.1f(GRAB_Z) p=%.1f r=%.1f c=%.1f\n",
                                  S_X + X_COMP, S_Y + Y_COMP, GRAB_Z, S_P, cur_roll, cur_claw);
                    arm.move(S_X + X_COMP, S_Y + Y_COMP, GRAB_Z, S_P, cur_roll, cur_claw, 2000); 
                     _currentState = CB_STATE_HOLD; 
                     _last_state_time = now;
                }
                break;
            case CB_STATE_HOLD:
                break;
            case CB_STATE_RELEASE:
                if (now - _last_state_time > 100) {
                    arm.move(S_X, S_Y, S_Z, S_P, 0, 0, 2000);
                    _currentState = CB_STATE_RESET;
                    _last_state_time = now;
                }
                break;
            case CB_STATE_RESET:
                if (now - _last_state_time > 2000) {
                    _currentState = CB_STATE_SEARCH; 
                    _last_state_time = now;
                }
                break;
            default: break;
        }
    }
}
#include "SelfLearningTracker.h"
#include <Wire.h>
#include "Robot_Arm.h"

#define K_H0 0xAA
#define K_H1 0x55

SelfLearningTracker selfLearningTracker;

void SelfLearningTracker::begin() {
    _busy = false;
    _is_tracking = false;
    _seq = 0;
    _last_ex = 0;
    _last_ey = 0;
    _f_tx = 0;    
    _f_tz = 200;  
}

// 第一步：开启模式，显示引导框（取色圈/框）
void SelfLearningTracker::start() {
    _busy = true;
    _is_tracking = false; 
    Wire1.setClock(400000); 

    Serial.println("[K230] Step 1: Mode 24 (ColorPicking) Init...");
    
    // 切换模式 24 (ColorPicking)
    uint8_t mode = 24;
    sendK230Cmd(0x01, &mode, 1);
    delay(800);

    // 开启绘图显示
    uint8_t draw = 0;
    sendK230Cmd(0x13, &draw, 1);
    sendK230Cmd(0x14, &draw, 1);
    delay(100);

    // 显示引导矩形框 (红框)
    uint8_t rect_data[8];
    uint16_t rx = 150, ry = 110, rw = 20, rh = 20; // 屏幕中心附近
    rect_data[0] = (rx >> 8) & 0xFF; rect_data[1] = rx & 0xFF;
    rect_data[2] = (ry >> 8) & 0xFF; rect_data[3] = ry & 0xFF;
    rect_data[4] = (rw >> 8) & 0xFF; rect_data[5] = rw & 0xFF;
    rect_data[6] = (rh >> 8) & 0xFF; rect_data[7] = rh & 0xFF;
    sendK230Cmd(0x41, rect_data, 8);
    
    // 开启预览画面运行
    uint8_t run_enable = 0; 
    sendK230Cmd(0x16, &run_enable, 1);

    arm.move(0, 200, 200, 0, 0, 0, 0);
    Serial.println("[K230] Guide box ready. Move object into the box.");
}

// 第二步：执行采样动作 (Mode 24: Set Point)
void SelfLearningTracker::confirm(String targetID, int x, int y) {
    if (!_busy) return;
    _currentID = targetID;

    // 发送 0x37 指令 (CMD_COLOR_TRACK_SET_POINT)
    // 格式: X(2) + Y(2) + Name(Str)
    uint8_t payload[32];
    payload[0] = (x >> 8) & 0xFF;
    payload[1] = x & 0xFF;
    payload[2] = (y >> 8) & 0xFF;
    payload[3] = y & 0xFF;
    
    uint8_t n_len = _currentID.length();
    payload[4] = n_len;
    memcpy(&payload[5], _currentID.c_str(), n_len);
    
    // 先停止之前的追踪
    sendK230Cmd(0x38, NULL, 0);
    delay(50);

    sendK230Cmd(0x37, payload, 5 + n_len);
    delay(50);
    
    // 确保运行开启
    uint8_t run_enable = 0;
    sendK230Cmd(0x16, &run_enable, 1);

    _is_tracking = true;
    Serial.print("[K230] Pick Color at "); Serial.print(x); Serial.print(","); Serial.print(y);
    Serial.print(" ID:"); Serial.println(_currentID);
}

// 必须定义 stop()，否则报错
void SelfLearningTracker::stop() {
    _busy = false;
    _is_tracking = false;
    
    // 发送停止颜色跟踪指令 0x38
    sendK230Cmd(0x38, NULL, 0);
    
    uint8_t run_disable = 1;
    sendK230Cmd(0x16, &run_disable, 1);
    
    arm.move(0, 200, 200, 0, 0, 0, 0);
    Serial.println("[K230] SelfLearning/ColorTrack Stopped.");
}

int SelfLearningTracker::readProtocol(int &x, int &y, int &w, int &h) {
    sendK230Cmd(0x04, NULL, 0); 
    
    Wire1.beginTransmission(0x55);
    Wire1.write(0x00);
    if (Wire1.endTransmission() != 0) return -1;
    if (Wire1.requestFrom((uint8_t)0x55, (uint8_t)48) == 0) return -2;

    uint8_t buf[48];
    int len = 0;
    while(Wire1.available() && len < 48) buf[len++] = Wire1.read();

    Wire1.beginTransmission(0x55);
    Wire1.write(0x00);
    uint8_t zeros[4] = {0, 0, 0, 0};
    Wire1.write(zeros, 4);
    Wire1.endTransmission();

    // 调试打印
    if (len > 0) {
        Serial.print("RX: ");
        for(int i=0; i<len && i<15; i++) Serial.printf("%02X ", buf[i]);
        Serial.println();
    }

    int head = -1;
    for (int i = 0; i < len - 6; i++) {
        if (buf[i] == K_H0 && buf[i+1] == K_H1) { head = i; break; }
    }
    if (head == -1) return -2;
    
    uint8_t f_code = buf[head + 4];
    uint8_t* p = &buf[head + 5];

    // 颜色追踪模式返回 0x75 (RPT_DETECT_COLOR)
    // 格式 (根据 ColorTrackerRot 验证): Count(1) + NameLen(1) + Name + [Box(8) + Angle(2)]
    if (f_code == 0x75) {
        uint8_t count = p[0];
        if (count > 0) {
            uint8_t n_len = p[1];
            // 检查颜色名是否匹配
            if (n_len > 0) {
                char name_buf[32];
                int copy_len = (n_len < 31) ? n_len : 31;
                memcpy(name_buf, &p[2], copy_len);
                name_buf[copy_len] = 0;
                if (_currentID != "1" && String(name_buf) != _currentID) {
                    return 2; // ID 不匹配
                }
            }
            
            int offset = 2 + n_len;
            x = (p[offset] << 8) | p[offset+1];
            y = (p[offset+2] << 8) | p[offset+3];
            w = (p[offset+4] << 8) | p[offset+5];
            h = (p[offset+6] << 8) | p[offset+7];
            return 1;
        }
    }
    return 2;
}

// 必须定义 update()，否则报错
void SelfLearningTracker::update() {
    if (!_busy || !_is_tracking) return;
    
    static uint32_t last_poll = 0;
    if (millis() - last_poll < 30) return;
    last_poll = millis();

    int rx, ry, rw, rh;
    int res = readProtocol(rx, ry, rw, rh);

    if (res == 1) {
        int cx = rx + rw / 2;
        int cy = ry + rh / 2;
        int ex = 160 - cx;
        int ey = 120 - cy;
        
        float ox = (float)ex * kp + (float)(ex - _last_ex) * kd;
        float oz = (float)ey * kp + (float)(ey - _last_ey) * kd;
        _last_ex = ex;
        _last_ey = ey;
        
        _f_tx -= ox; 
        _f_tz -= oz; 
        
        _f_tx = constrain(_f_tx, -160.0f, 160.0f);
        _f_tz = constrain(_f_tz, 60.0f, 320.0f);
        
        if (abs(ex) > 2 || abs(ey) > 2) {
            arm.move(_f_tx, 200, _f_tz, 0, 0, 0, 0);
        }
    }
}

bool SelfLearningTracker::sendK230Cmd(uint8_t func, const uint8_t *p, uint8_t n) {
    uint8_t fr[64];
    int idx = 0;
    fr[idx++] = K_H0; fr[idx++] = K_H1;
    fr[idx++] = n; fr[idx++] = (_seq++ & 0x1F); fr[idx++] = func;
    if (n > 0 && p != NULL) memcpy(&fr[idx], p, n);
    idx += n;
    uint8_t x = 0;
    for (int i = 2; i < idx; i++) x ^= fr[i];
    fr[idx++] = x;

    Wire1.beginTransmission(0x55);
    Wire1.write(0x00);
    Wire1.write(fr, idx);
    return (Wire1.endTransmission() == 0);
}

uint8_t SelfLearningTracker::calcXor(const uint8_t *d, int n) {
    uint8_t x = 0;
    for (int i = 0; i < n; i++) x ^= d[i];
    return x;
}
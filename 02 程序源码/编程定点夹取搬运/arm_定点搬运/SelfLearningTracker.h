#ifndef SELF_LEARNING_TRACKER_H
#define SELF_LEARNING_TRACKER_H

#include <Arduino.h>

class SelfLearningTracker {
public:
    float kp = 0.06f; 
    float kd = 0.003f;

    void begin();
    void start();  // 步骤 1: 进入模式 24，不采样，仅预览
    void confirm(String targetID, int x, int y); // 步骤 2: 发送 0x37 采样特定 ID
    void stop();   
    void update(); 

private:
    const uint8_t K230_ADDR = 0x55;
    const uint8_t K_H0 = 0xAA;
    const uint8_t K_H1 = 0x55;

    bool _busy = false;
    bool _is_tracking = false;
    uint8_t _seq = 0;
    String _currentID = ""; // 记录当前追踪的 ID 名称

    float _last_ex = 0, _last_ey = 0;
    float _f_tx = 0, _f_tz = 200; 

    bool sendK230Cmd(uint8_t func, const uint8_t* p, uint8_t n);
    int readProtocol(int &x, int &y, int &w, int &h);
    uint8_t calcXor(const uint8_t *d, int n);
};

extern SelfLearningTracker selfLearningTracker;

#endif
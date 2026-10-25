#ifndef FACE_TRACKER_H
#define FACE_TRACKER_H
#include <Arduino.h>

// 状态机定义
enum TrackerState {
  STATE_INIT,
  STATE_SET_MODE,
  STATE_WAIT_READY,
  STATE_CONFIG_PARAMS,
  STATE_ENABLE_RUN,
  STATE_RUNNING
};
class FaceTracker {
public:
    float kp = 0.1f; 
    float kd = 0.001f;
    
    void begin();
    void start(); 
    void stop();  
    void update(); 

private:
    const uint8_t K230_ADDR = 0x5F;
    const uint8_t K230_H0 = 0xAA;
    const uint8_t K230_H1 = 0x55;

    bool _busy = false;
    uint8_t _seq = 0;
    
    float _last_ex = 0, _last_ey = 0;
    float _f_tx = 200; 
    float _f_ty = 0;   
    float _f_tz = 200;

    TrackerState _currentState = STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0; 
    bool _isBufferEmpty = false;         
    bool _cmd_sent = false;               

    void ackAndClear();
    void sendPacket(uint8_t func, uint8_t* payload, uint8_t len);
    void forceConfig();                  
    void parseFrame(uint8_t* data, int total_len);
};

extern FaceTracker faceTracker;
#endif

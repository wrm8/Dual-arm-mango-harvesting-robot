#ifndef APRILTAG_TRACKER_H
#define APRILTAG_TRACKER_H

#include <Arduino.h>

enum TagState {
  T_STATE_INIT,
  T_STATE_SET_MODE,
  T_STATE_WAIT_READY,
  T_STATE_CONFIG_PARAMS,
  T_STATE_RUNNING
};

class AprilTagTracker {
public:
    float kp = 0.1f; 
    float kd = 0.003f;
    int target_width = 45; 

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
    
    float _last_ex = 0;
    float _last_ey = 0;
    float _last_ew = 0;

    float _f_tx = 0;   
    float _f_ty = 200; 
    float _f_tz = 200; 

    TagState _currentState = T_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0;
    unsigned long _ignore_data_until = 0; 
    bool _isBufferEmpty = false;

    void ackAndClear();
    void sendPacket(uint8_t func, uint8_t* payload, uint8_t len);
    void forceConfig();
    void parseFrame(uint8_t* data, int total_len);
};

extern AprilTagTracker aprilTagTracker;

#endif
#ifndef GARBAGE_GRABBER_H
#define GARBAGE_GRABBER_H

#include <Arduino.h>

enum GarbageState {
  GB_STATE_INIT,
  GB_STATE_SET_MODE,
  GB_STATE_WAIT_READY,
  GB_STATE_CONFIG_PARAMS,
  GB_STATE_SEARCH,    
  GB_STATE_DOWN,      
  GB_STATE_GRAB,      
  GB_STATE_LIFT,      
  GB_STATE_PLACE,     
  GB_STATE_RELEASE,   
  GB_STATE_RESET      
};

class GarbageGrabber {
public:
    float kp = 0.1f; 
    float kd = 0.01f;
    
    uint8_t conf_threshold = 80; 

    void begin();
    void start(); 
    void stop();  
    void update(); 
    
    void setOffsets(float x_off, float y_off, float z_grab);
    void setThreshold(uint8_t thresh);

private:
    const uint8_t K230_ADDR = 0x5F;
    const uint8_t K230_H0 = 0xAA;
    const uint8_t K230_H1 = 0x55;

    bool _busy = false;
    uint8_t _seq = 0;
    
    float _last_ex = 0, _last_ey = 0;
    float _f_tx = 0, _f_ty = 200, _f_tz = 200; 

    float X_COMP = 15.0f;
    float Y_COMP = 50.0f;
    float GRAB_Z = 60.0f;
    const float LIFT_Z = 220.0f;
    
    const float P_Y = 150.0f; 
    const float P_Z = 150.0f; 
    
    const float S_X = 200.0f;
    const float S_Y = 0.0f; 
    const float S_Z = 200.0f;
    const float S_P = -90.0f;
    
    const float C_OPEN = 1086.0f;  
    const float C_CLOSE = 2418.0f; 

    GarbageState _currentState = GB_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0;
    unsigned long _ignore_data_until = 0; 
    bool _isBufferEmpty = false;
    int _stableCount = 0;
    bool _cmd_sent = false;

    int _target_id = -1; 

    void ackAndClear();
    void sendPacket(uint8_t func, uint8_t* payload, uint8_t len);
    void forceConfig();
    void parseFrame(uint8_t* data, int total_len);
};

extern GarbageGrabber garbageGrabber;

#endif

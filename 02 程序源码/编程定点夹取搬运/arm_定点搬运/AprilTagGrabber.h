#ifndef APRILTAG_GRABBER_H
#define APRILTAG_GRABBER_H

#include <Arduino.h>

enum GrabState {
  G_STATE_INIT,
  G_STATE_SET_MODE,
  G_STATE_WAIT_READY,
  G_STATE_CONFIG_PARAMS,
  G_STATE_SEARCH,    
  G_STATE_DOWN,      
  G_STATE_GRAB,      
  G_STATE_LIFT,      
  G_STATE_PLACE,     
  G_STATE_RELEASE,   
  G_STATE_RESET      
};

class AprilTagGrabber {
public:
    float kp = 0.03f; 
    float kd = 0.001f;

    void begin();
    void start(); 
    void stop();  
    void update(); 
    
    void setOffsets(float x_off, float y_off, float z_grab);

private:
    const uint8_t K230_ADDR = 0x5F;
    const uint8_t K230_H0 = 0xAA;
    const uint8_t K230_H1 = 0x55;

    bool _busy = false;
    uint8_t _seq = 0;
    
    float _last_ex = 0;
    float _last_ey = 0;
    float _f_tx = 0;   
    float _f_ty = 200; 
    float _f_tz = 200; 
    float _f_tr = 0;   

    float X_COMP = 15.0f;
    float Y_COMP = 50.0f;
    float GRAB_Z = 60.0f;
    const float LIFT_Z = 220.0f;
    const float P_Y = 150.0f; 
    const float P_Z = 150.0f; 
    
    const float C_OPEN = 1086.0f;  
    const float C_CLOSE = 2418.0f; 
    
    const float S_P = -90.0f;
    const float S_X = 200.0f;
    const float S_Y = 0.0f; 
    const float S_Z = 200.0f;

    GrabState _currentState = G_STATE_INIT;
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

extern AprilTagGrabber aprilTagGrabber;

#endif

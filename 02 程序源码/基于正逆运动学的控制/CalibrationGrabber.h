#ifndef CALIBRATION_GRABBER_H
#define CALIBRATION_GRABBER_H

#include <Arduino.h>

enum CalibState {
  CB_STATE_INIT,
  CB_STATE_SET_MODE,
  CB_STATE_WAIT_READY,
  CB_STATE_CONFIG_PARAMS,
  CB_STATE_SEARCH,    
  CB_STATE_DOWN,      
  CB_STATE_GRAB,      
  CB_STATE_LIFT,      
  CB_STATE_HOLD,      
  CB_STATE_RELEASE,   
  CB_STATE_RESET      
};

class CalibrationGrabber {
public:
    float kp = 0.03f; 
    float kd = 0.001f;

    void begin();
    void start(); 
    void grab();      
    void resetArm();  
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
    
    float X_COMP = 0.0f;
    float Y_COMP = 0.0f;
    float GRAB_Z = 60.0f;
    const float LIFT_Z = 220.0f;
    
    const float C_OPEN = 1086.0f;  
    const float C_CLOSE = 2418.0f; 
    
    const float S_P = -90.0f;
    const float S_X = 200.0f;
    const float S_Y = 0.0f; 
    const float S_Z = 200.0f;

    CalibState _currentState = CB_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0;
    unsigned long _ignore_data_until = 0; 
    unsigned long _last_print_time = 0;
    bool _isBufferEmpty = false;
    bool _cmd_sent = false;

    void ackAndClear();
    void sendPacket(uint8_t func, uint8_t* payload, uint8_t len);
    void forceConfig();
    void parseFrame(uint8_t* data, int total_len);
};

extern CalibrationGrabber calibrationGrabber;

#endif

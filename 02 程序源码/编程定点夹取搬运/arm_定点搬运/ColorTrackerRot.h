#ifndef COLOR_TRACKER_ROT_H
#define COLOR_TRACKER_ROT_H

#include <Arduino.h>

enum ColorRotState {
  CR_STATE_INIT,
  CR_STATE_SET_MODE,
  CR_STATE_WAIT_READY,
  CR_STATE_CONFIG_PARAMS,
  CR_STATE_SEARCH,    
  CR_STATE_DOWN,      
  CR_STATE_GRAB,      
  CR_STATE_LIFT,      
  CR_STATE_PLACE,     
  CR_STATE_RELEASE,   
  CR_STATE_RESET      
};

class ColorTrackerRot {
public:
    void begin();
    void update();
    void start(const char* colorName); 
    void stop();
    void setOffsets(float x_off, float y_off, float z_grab);

    float kp = 0.06f;
    float kd = 0.003f;

private:
    const uint8_t K230_ADDR = 0x5F;
    const uint8_t K230_H0 = 0xAA;
    const uint8_t K230_H1 = 0x55;

    bool _busy = false;
    uint8_t _seq = 0;
    String _target_color; 
    
    ColorRotState _currentState = CR_STATE_INIT; 
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_config_retry = 0;
    int _stableCount = 0; 
    
    unsigned long _ignore_data_until = 0; 
    bool _isBufferEmpty = false; 
    bool _cmd_sent = false;      

    float _last_ex = 0, _last_ey = 0;
    float _f_tx, _f_ty, _f_tz, _f_tr; 
    
    const float S_X = 0.0f, S_Y = 200.0f, S_Z = 200.0f, S_P = -90.0f; 
    const float P_Y = 150.0f, P_Z = 150.0f;
    const float C_OPEN = 1086.0f, C_CLOSE = 2418.0f;
    
    float X_COMP = 15.0f;
    float Y_COMP = 50.0f;
    float GRAB_Z = 60.0f;
    const float LIFT_Z = 220.0f;

    void ackAndClear();
    void sendPacket(uint8_t func, uint8_t* payload, uint8_t len);
    void forceConfig();
    void parseFrame(uint8_t* data, int total_len);
};

extern ColorTrackerRot colorTrackerRot;
#endif

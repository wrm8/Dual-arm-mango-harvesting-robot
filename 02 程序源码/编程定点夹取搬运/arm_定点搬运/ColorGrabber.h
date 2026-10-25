#ifndef COLOR_GRABBER_H
#define COLOR_GRABBER_H

#include <Arduino.h>

enum ColorState {
  C_STATE_INIT,
  C_STATE_SET_MODE,
  C_STATE_WAIT_READY,
  C_STATE_CONFIG_PARAMS,
  C_STATE_ENABLE_RUN,
  C_STATE_RUNNING
};

class ColorGrabber {
public:
    float kp = 0.01f; 
    float kd = 0.001f;

    void begin();
    void start(String colorName); 
    void stop();  
    void update(); 

private:
    const uint8_t K230_ADDR = 0x5F;
    const uint8_t K230_H0 = 0xAA;
    const uint8_t K230_H1 = 0x55;

    bool _busy = false;
    uint8_t _seq = 0;
    String _target_color;
    
    float _last_ex = 0;
    float _last_ey = 0;
    float _f_tx = 0;  
    float  _f_ty =0; 
    float _f_tz = 200; 

    ColorState _currentState = C_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    
    unsigned long _ignore_data_until = 0; 
    bool _cmd_sent = false;               
    bool _isBufferEmpty = false; 

    void ackAndClear();
    void sendPacket(uint8_t func, uint8_t* payload, uint8_t len);
    void runStateMachine(uint8_t mode, uint8_t status);
    void parseFrame(uint8_t* data, int total_len);
    void forceConfig(); // 补充声明
};

extern ColorGrabber colorGrabber;

#endif
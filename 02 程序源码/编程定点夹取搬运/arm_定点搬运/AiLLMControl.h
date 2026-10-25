#ifndef AILLM_CONTROL_H
#define AILLM_CONTROL_H

#include <Arduino.h>

enum AiLLMState {
  LLM_STATE_INIT,
  LLM_STATE_SET_MODE,
  LLM_STATE_WAIT_READY,
  LLM_STATE_CONFIG_MCP,
  LLM_STATE_ENABLE_RUN,
  LLM_STATE_ENABLE_ASR,
  LLM_STATE_RUNNING
};

class AiLLMControl {
public:
    void begin();
    void start(); 
    void stop();  
    void update(); 

private:
    const uint8_t K230_ADDR = 0x5F;
    const uint8_t K230_H0 = 0xAA;
    const uint8_t K230_H1 = 0x55;

    static constexpr int SERVO_ID_MIN = 1;
    static constexpr int SERVO_ID_MAX = 6;
    static constexpr int SERVO_POS_MIN = 0;
    static constexpr int SERVO_POS_MAX = 4096;
    static constexpr int SERVO_TIME_DEFAULT_MS = 1000;
    static constexpr int SERVO_TIME_MIN_MS = 20;
    static constexpr int SERVO_TIME_MAX_MS = 30000;

    static constexpr uint8_t FRAME_TYPE_CMD = 0x00;
    static constexpr uint8_t FRAME_TYPE_RPT = 0x80;
    static constexpr uint8_t FRAME_TYPE_MASK = 0xC0;
    static constexpr uint8_t CONT_MASK = 0x20;
    static constexpr uint8_t ERR_OK = 0x00;
    static constexpr uint8_t CMD_SET_MODE = 0x01;
    static constexpr uint8_t CMD_REQUEST_STATUS = 0x04;
    static constexpr uint8_t CMD_DISABLE_RUN = 0x16;
    static constexpr uint8_t CMD_ASR = 0x5A;
    static constexpr uint8_t CMD_SET_MCP_TOOLS = 0x5E;
    static constexpr uint8_t CMD_RESULT_RETURN = 0x5F;
    static constexpr uint8_t RPT_HEARTBEAT = 0x70;
    static constexpr uint8_t STATUS_RESULT = 0x02;
    static constexpr uint8_t STATUS_READY = 0x04;
    static constexpr uint8_t APP_MODE_AILLM = 20;
    static constexpr uint16_t TX_PAYLOAD_CHUNK_MAX = 58;
    static constexpr uint16_t RX_FRAME_MAX_SIZE = 64;

    bool _busy = false;
    uint8_t _seq = 0;

    AiLLMState _currentState = LLM_STATE_INIT;
    unsigned long _last_state_time = 0;
    unsigned long _last_poll_time = 0;
    unsigned long _last_hb_time = 0;
    uint8_t _last_hb_status = 0;
    uint8_t _last_hb_mode = 0;
    bool _mcp_configured = false;

    void ackAndClear();
    bool waitRxClear(unsigned long timeout_ms);
    void sendPacket(uint8_t func, const uint8_t* payload, uint16_t len);
    void parseFrame(uint8_t* data, int total_len);
    void handleReport(uint8_t func, const uint8_t* payload, int payload_len);
    void setupMCPTools();
    bool executeAction(const uint8_t* payload, uint16_t len);
    bool executeServoParams(int target_id, int target_angle, int time_ms);
    bool parseServoArgs(const uint8_t* payload, uint16_t len, int& out_id, int& out_angle, int& out_time_ms);
    bool readValueAsInt(const uint8_t* payload, uint16_t len, int offset, int& out);

    static constexpr uint16_t RX_FRAG_MAX = 512;
    uint8_t _rx_frag_buf[RX_FRAG_MAX] = {0};
    uint16_t _rx_frag_len = 0;
    uint8_t _rx_frag_func = 0;
    bool _rx_frag_in_progress = false;
    unsigned long _rx_frag_time = 0;
};

extern AiLLMControl aiLLMControl;

#endif

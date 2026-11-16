#ifndef STRIKER_CONTROL_H
#define STRIKER_CONTROL_H

#include <stdint.h>

typedef struct
{
    uint8_t arm;
    uint32_t counts_per_rev;
    int8_t pwm_sign;
    int8_t encoder_sign;
} StrikerMotorConfig;

typedef enum
{
    STRIKER_IDLE = 0,
    STRIKER_RUNNING,
    STRIKER_SETTLING,
    STRIKER_DONE,
    STRIKER_FAULT
} StrikerState;

typedef enum
{
    STRIKER_ERROR_NONE = 0,
    STRIKER_ERROR_CONFIG,
    STRIKER_ERROR_STALL,
    STRIKER_ERROR_DIRECTION,
    STRIKER_ERROR_TIMEOUT,
    STRIKER_ERROR_NOT_SETTLED
} StrikerError;

/* All callers must be serialized (TIM7 wrapper masks IRQs for main access). */
void StrikerControl_Init(const StrikerMotorConfig motors[4]);
uint8_t StrikerControl_Start(uint8_t arm_mask);
void StrikerControl_Update(const int32_t delta[4], uint32_t elapsed_ms, int32_t pwm[4]);
StrikerState StrikerControl_State(void);
StrikerError StrikerControl_Error(void);

#endif

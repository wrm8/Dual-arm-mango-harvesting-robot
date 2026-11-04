#ifndef USART3_CHASSIS_H
#define USART3_CHASSIS_H

#include <stdint.h>

void uart3_init(uint32_t bound);
void USART3_IRQHandler(void);
void USART3_ProcessCommand(void);
void USART3_CommandWatchdogTick(uint32_t elapsed_ms);
void USART3_GetWheelTargets(int32_t target_mmps[4]);

/* Inputs are normalized: 1 means obstacle detected, regardless of GPIO polarity. */
void USART3_UpdatePhotoelectric(
    uint8_t front_active, uint8_t left_active, uint8_t right_active);
void USART3_GetStageState(uint8_t *stage_id, uint8_t *reached);

/* Call transmit functions from ONE main-loop/task context, never an ISR. */
void USART3_SendStageFeedback(void);
void USART_SendFeedback(
    uint32_t sequence, uint32_t period_ms,
    const int32_t encoder_delta[4], int32_t yaw_mdeg);

#endif

#ifndef STRIKER_BSP_TIM_H
#define STRIKER_BSP_TIM_H

#include "sys.h"
#include "striker_control.h"

void TIM7_Init(u16 psc, u16 arr);
void TIM7_IRQHandler(void);
uint8_t TIM7_StartStrike(uint8_t mask);
StrikerState TIM7_StrikeState(void);
StrikerError TIM7_StrikeError(void);

#endif

#include "stm32f4xx.h"
#include "usart.h"
#include "usart3_striker.h"
#include "delay.h"
#include "pwm.h"
#include "motor.h"
#include "encoder.h"
#include "bsp_tim.h"
#include "striker_config.h"

#include <stdint.h>
#include <stdio.h>

/* Retained for the existing SysTick implementation, if it increments this. */
volatile uint32_t my_time_ms = 0;

int main(void)
{
    uint8_t active = 0;
    uint8_t mask;
    StrikerState state;

    SystemInit();
    delay_init(168);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(1000);
    uart_init(115200);

    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();
    Encoder_Init_TIM5();
    TIM1_PWM_Init();
    TIM9_PWM_Init();
    TIM10_PWM_Init();
    TIM11_PWM_Init();
    motor_control(0, 0, 0, 0);
    TIM7_Init(83, 4999);
    /* Accept commands only after the outputs and controller are initialized. */
    uart3_init(115200);
    printf("STM32 striker ready, UART3 115200, target=%u rev.\r\n",
           (unsigned)STRIKER_TARGET_REVOLUTIONS);

    while (1)
    {
        if (active)
        {
            state = TIM7_StrikeState();
            if (state == STRIKER_DONE)
            {
                active = 0;
                USART3_StrikeFinished();
                printf("Strike finished, ready for next trigger.\r\n");
            }
            else if (state == STRIKER_FAULT)
            {
                active = 0;
                /* Hold busy after a fault. Reset only after fixing the cause. */
                printf("Strike fault=%u, outputs off, reset required.\r\n",
                       (unsigned)TIM7_StrikeError());
            }
        }

        mask = USART3_TakeStrikeCommand();
        if (mask != STRIKE_NONE)
        {
            if (TIM7_StartStrike(mask))
            {
                active = 1;
                printf("Strike start, mask=%u.\r\n", (unsigned)mask);
            }
            else
            {
                printf("Strike rejected, error=%u; check striker_config.h.\r\n",
                       (unsigned)TIM7_StrikeError());
            }
        }
        /* No unconditional motor_control(): TIM7 owns all running outputs. */
        delay_ms(1);
    }
}

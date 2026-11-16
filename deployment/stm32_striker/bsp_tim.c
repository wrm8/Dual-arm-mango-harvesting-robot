#include "bsp_tim.h"
#include "encoder.h"
#include "motor.h"
#include "striker_config.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

/* TIM7=84MHz/(83+1)/(4999+1)=200Hz; main must use TIM7_Init(83,4999). */
#define STRIKER_TICK_MS 5U

static const StrikerMotorConfig motors[4] = {
    {STRIKER_A_ARM, STRIKER_A_COUNTS_PER_REV, STRIKER_A_PWM_SIGN, STRIKER_A_ENCODER_SIGN},
    {STRIKER_B_ARM, STRIKER_B_COUNTS_PER_REV, STRIKER_B_PWM_SIGN, STRIKER_B_ENCODER_SIGN},
    {STRIKER_C_ARM, STRIKER_C_COUNTS_PER_REV, STRIKER_C_PWM_SIGN, STRIKER_C_ENCODER_SIGN},
    {STRIKER_D_ARM, STRIKER_D_COUNTS_PER_REV, STRIKER_D_PWM_SIGN, STRIKER_D_ENCODER_SIGN}
};

void TIM7_Init(u16 psc, u16 arr)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_Cmd(TIM7, DISABLE);
    StrikerControl_Init(motors);
    motor_control(0, 0, 0, 0);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = psc;
    timer.TIM_Period = arr;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &timer);
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

uint8_t TIM7_StartStrike(uint8_t mask)
{
    uint8_t accepted;
    uint8_t i;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    /* Read_Encoder() already reads and clears the counter in the old firmware. */
    if (StrikerControl_State() == STRIKER_IDLE || StrikerControl_State() == STRIKER_DONE)
        for (i = 0; i < 4U; ++i)
            (void)Read_Encoder(i + 2U);
    accepted = StrikerControl_Start(mask);
    __set_PRIMASK(saved);
    return accepted;
}

StrikerState TIM7_StrikeState(void)
{
    StrikerState value;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    value = StrikerControl_State();
    __set_PRIMASK(saved);
    return value;
}

StrikerError TIM7_StrikeError(void)
{
    StrikerError value;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    value = StrikerControl_Error();
    __set_PRIMASK(saved);
    return value;
}

void TIM7_IRQHandler(void)
{
    int32_t delta[4];
    int32_t pwm[4];
    uint8_t i;

    if (TIM_GetITStatus(TIM7, TIM_IT_Update) == RESET)
        return;
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    for (i = 0; i < 4U; ++i)
        delta[i] = Read_Encoder(i + 2U);
    StrikerControl_Update(delta, STRIKER_TICK_MS, pwm);
    /* The only writer of motor outputs after initialization. */
    motor_control(pwm[0], pwm[1], pwm[2], pwm[3]);
}

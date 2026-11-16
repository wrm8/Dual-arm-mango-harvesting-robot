#!/usr/bin/env python3
"""Build the actual STM32 modules with host stubs. No hardware is accessed."""

import importlib.util
from pathlib import Path
import subprocess
import tempfile


MOCK_HEADER = r'''
#ifndef MOCK_STM32_H
#define MOCK_STM32_H
#include <stdint.h>
#include <string.h>
#define SYSTEM_SUPPORT_OS 0
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define GPIOD 0
#define RCC_AHB1Periph_GPIOD 1
#define RCC_APB1Periph_USART3 2
#define RCC_APB1Periph_TIM7 3
#define GPIO_PinSource8 8
#define GPIO_PinSource9 9
#define GPIO_AF_USART3 7
#define GPIO_Pin_8 256
#define GPIO_Pin_9 512
#define GPIO_Mode_AF 2
#define GPIO_Speed_50MHz 2
#define GPIO_OType_PP 0
#define GPIO_PuPd_UP 1
#define USART_WordLength_8b 8
#define USART_StopBits_1 1
#define USART_Parity_No 0
#define USART_HardwareFlowControl_None 0
#define USART_Mode_Rx 1
#define USART_Mode_Tx 2
#define USART_IT_RXNE 1
#define USART3_IRQn 39
#define USART_SR_PE (1U << 0)
#define USART_SR_FE (1U << 1)
#define USART_SR_NE (1U << 2)
#define USART_SR_ORE (1U << 3)
#define USART_SR_RXNE (1U << 5)
#define TIM7 7
#define TIM7_IRQn 55
#define TIM_IT_Update 1
#define TIM_CKD_DIV1 0
#define TIM_CounterMode_Up 0
#define NVIC_PriorityGroup_2 2
typedef uint16_t u16;
typedef struct { volatile uint32_t SR; uint32_t DR; } USART_TypeDef;
extern USART_TypeDef mock_usart;
#define USART3 (&mock_usart)
typedef struct { uint32_t GPIO_Pin, GPIO_Mode, GPIO_Speed, GPIO_OType, GPIO_PuPd; } GPIO_InitTypeDef;
typedef struct {
    uint32_t USART_BaudRate, USART_WordLength, USART_StopBits, USART_Parity;
    uint32_t USART_HardwareFlowControl, USART_Mode;
} USART_InitTypeDef;
typedef struct {
    uint32_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    uint32_t NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
typedef struct {
    uint32_t TIM_Prescaler, TIM_Period, TIM_ClockDivision, TIM_CounterMode;
} TIM_TimeBaseInitTypeDef;
extern uint32_t mock_primask;
extern uint8_t mock_timer_pending;
extern uint32_t mock_baud;
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1; }
static inline void __set_PRIMASK(uint32_t value) { mock_primask = value; }
static inline void RCC_AHB1PeriphClockCmd(int a, int b) { (void)a; (void)b; }
static inline void RCC_APB1PeriphClockCmd(int a, int b) { (void)a; (void)b; }
static inline void GPIO_PinAFConfig(int a, int b, int c) { (void)a; (void)b; (void)c; }
static inline void GPIO_StructInit(GPIO_InitTypeDef *p) { memset(p, 0, sizeof(*p)); }
static inline void GPIO_Init(int a, GPIO_InitTypeDef *p) { (void)a; (void)p; }
static inline void USART_StructInit(USART_InitTypeDef *p) { memset(p, 0, sizeof(*p)); }
static inline void USART_Init(USART_TypeDef *a, USART_InitTypeDef *p) { (void)a; mock_baud = p->USART_BaudRate; }
static inline void USART_ITConfig(USART_TypeDef *a, int b, int c) { (void)a; (void)b; (void)c; }
static inline void NVIC_Init(NVIC_InitTypeDef *p) { (void)p; }
static inline void USART_Cmd(USART_TypeDef *a, int b) { (void)a; (void)b; }
static inline uint16_t USART_ReceiveData(USART_TypeDef *p) { p->SR = 0; return (uint16_t)p->DR; }
static inline void TIM_Cmd(int a, int b) { (void)a; (void)b; }
static inline void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *p) { memset(p, 0, sizeof(*p)); }
static inline void TIM_TimeBaseInit(int a, TIM_TimeBaseInitTypeDef *p) { (void)a; (void)p; }
static inline void TIM_ClearITPendingBit(int a, int b) { (void)a; (void)b; mock_timer_pending = 0; }
static inline void TIM_ITConfig(int a, int b, int c) { (void)a; (void)b; (void)c; }
static inline int TIM_GetITStatus(int a, int b) { (void)a; (void)b; return mock_timer_pending; }
static inline void SystemInit(void) {}
static inline void delay_init(int a) { (void)a; }
static inline void delay_ms(int a) { (void)a; }
static inline void SysTick_Init(int a) { (void)a; }
static inline void NVIC_PriorityGroupConfig(int a) { (void)a; }
static inline void uart_init(uint32_t a) { (void)a; }
static inline void Encoder_Init_TIM2(void) {}
static inline void Encoder_Init_TIM3(void) {}
static inline void Encoder_Init_TIM4(void) {}
static inline void Encoder_Init_TIM5(void) {}
static inline void TIM1_PWM_Init(void) {}
static inline void TIM9_PWM_Init(void) {}
static inline void TIM10_PWM_Init(void) {}
static inline void TIM11_PWM_Init(void) {}
int Read_Encoder(uint8_t timer);
void motor_control(int a, int b, int c, int d);
#endif
'''


TEST_SOURCE = r'''
#include "mock_stm32.h"
#include "usart3_striker.h"
#include "striker_control.h"
#include "striker_config.h"
#include "bsp_tim.h"
#include "host_frames.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

USART_TypeDef mock_usart;
uint32_t mock_primask;
uint8_t mock_timer_pending;
uint32_t mock_baud;
static int encoder[4];
static int output[4];

int Read_Encoder(uint8_t timer)
{
    int value;
    assert(timer >= 2 && timer <= 5);
    value = encoder[timer - 2];
    encoder[timer - 2] = 0;
    return value;
}

void motor_control(int a, int b, int c, int d)
{
    output[0] = a; output[1] = b; output[2] = c; output[3] = d;
}

static void receive(const uint8_t *bytes, unsigned count)
{
    unsigned i;
    for (i = 0; i < count; ++i)
    {
        mock_usart.DR = bytes[i];
        mock_usart.SR = USART_SR_RXNE;
        USART3_IRQHandler();
    }
}

static void frame(const uint8_t bytes[6]) { receive(bytes, 6); }
static void expect_no_event(void) { assert(USART3_TakeStrikeCommand() == STRIKE_NONE); }
static void expect_output(int a, int b, int c, int d)
{
    assert(output[0] == a && output[1] == b && output[2] == c && output[3] == d);
}

static void tick(int a, int b, int c, int d)
{
    encoder[0] = a; encoder[1] = b; encoder[2] = c; encoder[3] = d;
    mock_timer_pending = 1;
    TIM7_IRQHandler();
}

static void test_serial(void)
{
    unsigned i;
    uint8_t corrupt[6];
    const uint8_t overlap[] = {0, 0xAA, 0xAA, 0x55, 1, 2, 0x0D, 0x0A};
    const char *ascii = "AA 55 01 01 0D 0A\r\nCMD_WHEELS 700 700 700 700 1\r\n";
    uart3_init(115200);
    assert(mock_baud == 115200);
    assert(!USART3_IsStrikeBusy());
    expect_no_event();
    for (i = 0; i < 3; ++i)
    {
        receive(host_frames[i], 5);
        expect_no_event();
        receive(host_frames[i] + 5, 1);
        assert(USART3_IsStrikeBusy());
        /* Pending event also prevents overwrite before main has consumed it. */
        frame(host_frames[(i + 1) % 3]);
        assert(USART3_TakeStrikeCommand() == i + 1);
        expect_no_event();
        assert(USART3_IsStrikeBusy());
        frame(host_frames[i]);
        expect_no_event();
        USART3_StrikeFinished();
        assert(!USART3_IsStrikeBusy());
    }
    frame(host_frames[0]);
    assert(USART3_TakeStrikeCommand() == STRIKE_LEFT);
    receive(host_frames[1], 3); /* Partial busy frame cannot finish after rearm. */
    USART3_StrikeFinished();
    receive(host_frames[1] + 3, 3);
    expect_no_event();
    frame(host_frames[0]); /* Identical command after completion is a new strike. */
    assert(USART3_TakeStrikeCommand() == STRIKE_LEFT);
    USART3_StrikeFinished();
    receive((const uint8_t *)ascii, (unsigned)strlen(ascii));
    expect_no_event();
    for (i = 0; i < 6; ++i)
    {
        memcpy(corrupt, host_frames[0], 6);
        corrupt[i] ^= 0x40;
        receive(corrupt, 6);
        expect_no_event();
        frame(host_frames[2]);
        assert(USART3_TakeStrikeCommand() == STRIKE_BOTH);
        USART3_StrikeFinished();
    }
    for (i = 0; i < 256; ++i)
    {
        if (i >= 1 && i <= 3) continue;
        memcpy(corrupt, host_frames[0], 6); corrupt[3] = (uint8_t)i;
        frame(corrupt); expect_no_event();
    }
    receive(overlap, sizeof(overlap));
    assert(USART3_TakeStrikeCommand() == STRIKE_RIGHT);
    USART3_StrikeFinished();
    for (i = 0; i < 4; ++i)
    {
        receive(host_frames[0], 4);
        mock_usart.DR = 0x0D;
        mock_usart.SR = USART_SR_RXNE | (1U << i);
        USART3_IRQHandler();
        receive(host_frames[0] + 5, 1);
        expect_no_event();
        frame(host_frames[1]);
        assert(USART3_TakeStrikeCommand() == STRIKE_RIGHT);
        USART3_StrikeFinished();
    }
    mock_primask = 1;
    uart3_init(115200); assert(mock_primask == 1);
    frame(host_frames[2]);
    assert(USART3_TakeStrikeCommand() == STRIKE_BOTH && mock_primask == 1);
    USART3_StrikeFinished(); assert(mock_primask == 1);
    mock_primask = 0;
    puts("PASS serial: host frames, split input, busy discard, rearm, noise, UART errors, IRQ mask");
}

static void test_timer_integration(void)
{
    unsigned i;
    const int target_a = STRIKER_A_COUNTS_PER_REV * STRIKER_TARGET_REVOLUTIONS;
    const int target_b = STRIKER_B_COUNTS_PER_REV * STRIKER_TARGET_REVOLUTIONS;
    const int remaining_a = STRIKER_A_COUNTS_PER_REV / 4;
    const int first_b = target_b / 4;
    assert(target_a == 16520 && target_b == 16520);
    TIM7_Init(83, 4999);
    uart3_init(115200);
    tick(0, 0, 0, 0); expect_output(0, 0, 0, 0);
    frame(host_frames[2]);
    /* Old, pre-trigger hardware counts must not count towards this strike. */
    encoder[0] = 321; encoder[1] = 654;
    assert(TIM7_StartStrike(USART3_TakeStrikeCommand()));
    tick(0, 0, 0, 0); expect_output(-700, 700, 0, 0);
    tick((target_a - remaining_a) * STRIKER_A_ENCODER_SIGN, 0, 0, 0);
    assert(output[0] == -350 && output[1] == 700); /* Last half rev deceleration. */
    tick(remaining_a * STRIKER_A_ENCODER_SIGN, first_b * STRIKER_B_ENCODER_SIGN, 0, 0);
    expect_output(0, 700, 0, 0);
    assert(TIM7_StrikeState() == STRIKER_RUNNING);
    frame(host_frames[0]); expect_no_event(); /* Busy: no next action queued. */
    tick(0, (target_b - first_b) * STRIKER_B_ENCODER_SIGN, 0, 0);
    expect_output(0, 0, 0, 0);
    assert(TIM7_StrikeState() == STRIKER_SETTLING);
    tick(1, 1, 0, 0); /* Coast delays the settling period. */
    for (i = 0; i < STRIKER_SETTLE_MS / 5; ++i) tick(0, 0, 0, 0);
    assert(TIM7_StrikeState() == STRIKER_DONE && USART3_IsStrikeBusy());
    USART3_StrikeFinished();
    frame(host_frames[0]);
    assert(TIM7_StartStrike(USART3_TakeStrikeCommand()));
    tick(target_a * STRIKER_A_ENCODER_SIGN, 0, 0, 0); expect_output(0, 0, 0, 0);
    assert(TIM7_StrikeState() == STRIKER_SETTLING); /* Relative target resets. */
    for (i = 0; i < STRIKER_SETTLE_MS / 5; ++i) tick(0, 0, 0, 0);
    assert(TIM7_StrikeState() == STRIKER_DONE);
    USART3_StrikeFinished();
    puts("PASS timer integration: idle stop, 10 relative turns, independent stops, settling, repeated strike");
}

static void test_faults_and_mapping(void)
{
    StrikerMotorConfig cfg[4] = {{1, 100, -1, -1}, {2, 100, 1, 1}, {1, 100, 1, 1}, {0, 0, 1, 1}};
    int32_t delta[4] = {0, 0, 0, 0};
    int32_t pwm[4];
    unsigned i;
    StrikerControl_Init(cfg);
    assert(StrikerControl_Start(1));
    StrikerControl_Update(delta, 5, pwm);
    assert(pwm[0] == -700 && pwm[1] == 0 && pwm[2] == 700 && pwm[3] == 0);
    delta[0] = -1000; delta[2] = 500;
    StrikerControl_Update(delta, 5, pwm);
    assert(pwm[0] == 0 && pwm[2] == 700); /* Two channels for the same side. */
    delta[0] = 0; delta[2] = 500;
    StrikerControl_Update(delta, 5, pwm);
    assert(StrikerControl_State() == STRIKER_SETTLING);

    StrikerControl_Init(cfg); assert(StrikerControl_Start(2));
    memset(delta, 0, sizeof(delta));
    for (i = 0; i < STRIKER_STALL_MS / 5; ++i) StrikerControl_Update(delta, 5, pwm);
    assert(StrikerControl_Error() == STRIKER_ERROR_STALL);
    assert(pwm[0] == 0 && pwm[1] == 0 && pwm[2] == 0 && pwm[3] == 0);
    assert(!StrikerControl_Start(1)); /* Fault remains latched until reset. */

    StrikerControl_Init(cfg); assert(StrikerControl_Start(2));
    delta[1] = -9; StrikerControl_Update(delta, 5, pwm);
    assert(StrikerControl_Error() == STRIKER_ERROR_DIRECTION);
    assert(pwm[1] == 0);

    StrikerControl_Init(cfg); assert(StrikerControl_Start(2));
    delta[1] = 0; StrikerControl_Update(delta, UINT_MAX, pwm);
    assert(StrikerControl_Error() == STRIKER_ERROR_TIMEOUT);

    StrikerControl_Init(cfg); assert(StrikerControl_Start(2));
    /* Progress below 10 turns forever must still hit the overall timeout. */
    for (i = 0; i < STRIKER_MAX_RUN_MS / 5; ++i)
    {
        delta[1] = i % 100 == 0 ? 1 : 0;
        StrikerControl_Update(delta, 5, pwm);
    }
    assert(StrikerControl_Error() == STRIKER_ERROR_TIMEOUT && pwm[1] == 0);

    StrikerControl_Init(cfg); assert(StrikerControl_Start(2));
    delta[1] = 1000; StrikerControl_Update(delta, 5, pwm);
    assert(StrikerControl_State() == STRIKER_SETTLING);
    delta[1] = 1;
    for (i = 0; i < STRIKER_MAX_SETTLE_MS / 5; ++i) StrikerControl_Update(delta, 5, pwm);
    assert(StrikerControl_Error() == STRIKER_ERROR_NOT_SETTLED);

    cfg[0].counts_per_rev = 0;
    StrikerControl_Init(cfg); assert(!StrikerControl_Start(1));
    assert(StrikerControl_Error() == STRIKER_ERROR_CONFIG);
    StrikerControl_Update(delta, 5, pwm);
    assert(pwm[0] == 0 && pwm[1] == 0 && pwm[2] == 0 && pwm[3] == 0);
    cfg[0].arm = 0; cfg[2].arm = 0;
    StrikerControl_Init(cfg); assert(!StrikerControl_Start(1));
    cfg[0].arm = 3;
    StrikerControl_Init(cfg); assert(!StrikerControl_Start(3));
    puts("PASS controller faults: mapping, direction, stall, timeout, settling timeout, missing calibration");
}

int main(void)
{
    test_serial();
    test_timer_integration();
    test_faults_and_mapping();
    puts("ALL STRIKER FIRMWARE HOST TESTS PASSED");
    return 0;
}
'''


def main():
    root = Path(__file__).resolve().parent
    protocol = root.parents[1] / 'ros2_ws/src/robot_end_effector/robot_end_effector/serial_striker_io.py'
    spec = importlib.util.spec_from_file_location('striker_host_io', protocol)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.STRIKE_BAUD_RATE == 115200
    expected = {'left': 'AA 55 01 01 0D 0A', 'right': 'AA 55 01 02 0D 0A', 'both': 'AA 55 01 03 0D 0A'}
    assert module.STRIKE_COMMAND_HEX == expected, 'Host protocol changed; review firmware first'
    frames = [module.parse_frame(module.STRIKE_COMMAND_HEX[key]) for key in ('left', 'right', 'both')]
    header = 'static const uint8_t host_frames[3][6] = {\n' + ',\n'.join(
        '    {' + ', '.join(hex(byte) for byte in frame) + '}' for frame in frames) + '\n};\n'
    with tempfile.TemporaryDirectory(prefix='striker-firmware-') as tmp:
        build = Path(tmp)
        (build / 'mock_stm32.h').write_text(MOCK_HEADER)
        for name in ('sys.h', 'stm32f4xx.h', 'stm32f4xx_gpio.h', 'stm32f4xx_rcc.h',
                     'stm32f4xx_usart.h', 'stm32f4xx_tim.h', 'misc.h',
                     'encoder.h', 'motor.h', 'usart.h', 'delay.h', 'pwm.h'):
            (build / name).write_text('#include "mock_stm32.h"\n')
        (build / 'host_frames.h').write_text(header)
        (build / 'test.c').write_text(TEST_SOURCE)
        binary = build / 'test_firmware'
        subprocess.run([
            'gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-g',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-I', str(root), '-I', str(build),
            str(root / 'usart3_striker.c'), str(root / 'striker_control.c'),
            str(root / 'bsp_tim.c'), str(build / 'test.c'), '-o', str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
        subprocess.run([
            'gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
            '-I', str(root), '-I', str(build), '-fsyntax-only', str(root / 'main.c'),
        ], check=True)
        print('PASS main: module API integration compiles with host peripheral stubs')


if __name__ == '__main__':
    main()

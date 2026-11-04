#!/usr/bin/env python3
"""Compile the real USART3 module against host stubs; no board or serial device."""

from pathlib import Path
import subprocess
import tempfile


MOCK_HEADER = r'''
#ifndef MOCK_STM32_H
#define MOCK_STM32_H
#include <stdint.h>
#define ENABLE 1
#define RESET 0
#define GPIOD 0
#define USART3 3
#define RCC_AHB1Periph_GPIOD 1
#define RCC_APB1Periph_USART3 1
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
#define USART_FLAG_TXE 1
#define USART_FLAG_TC 2
typedef struct {
    uint32_t GPIO_Pin, GPIO_Mode, GPIO_Speed, GPIO_OType, GPIO_PuPd;
} GPIO_InitTypeDef;
typedef struct {
    uint32_t USART_BaudRate, USART_WordLength, USART_StopBits, USART_Parity;
    uint32_t USART_HardwareFlowControl, USART_Mode;
} USART_InitTypeDef;
typedef struct {
    uint32_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    uint32_t NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
extern uint32_t mock_primask;
extern uint16_t mock_rx;
void mock_tx(uint16_t byte);
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1; }
static inline void __set_PRIMASK(uint32_t v) { mock_primask = v; }
static inline void RCC_AHB1PeriphClockCmd(int a, int b) { (void)a; (void)b; }
static inline void RCC_APB1PeriphClockCmd(int a, int b) { (void)a; (void)b; }
static inline void GPIO_PinAFConfig(int a, int b, int c) { (void)a; (void)b; (void)c; }
static inline void GPIO_Init(int a, GPIO_InitTypeDef *p) { (void)a; (void)p; }
static inline void USART_Init(int a, USART_InitTypeDef *p) { (void)a; (void)p; }
static inline void USART_ITConfig(int a, int b, int c) { (void)a; (void)b; (void)c; }
static inline void NVIC_Init(NVIC_InitTypeDef *p) { (void)p; }
static inline void USART_Cmd(int a, int b) { (void)a; (void)b; }
static inline int USART_GetITStatus(int a, int b) { (void)a; (void)b; return 1; }
static inline uint16_t USART_ReceiveData(int a) { (void)a; return mock_rx; }
static inline int USART_GetFlagStatus(int a, int b) { (void)a; (void)b; return 1; }
static inline void USART_SendData(int a, uint16_t b) { (void)a; mock_tx(b); }
#endif
'''


TEST_SOURCE = r'''
#include "usart3_chassis.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

uint32_t mock_primask = 0;
uint16_t mock_rx;
static char tx[4096];
static unsigned tx_len;

void mock_tx(uint16_t byte)
{
    assert(tx_len < sizeof(tx) - 1);
    tx[tx_len++] = (char)byte;
    tx[tx_len] = '\0';
}

static void receive(const char *line)
{
    while (*line)
    {
        mock_rx = (uint8_t)*line++;
        USART3_IRQHandler();
    }
}

static void command(const char *line)
{
    receive(line);
    USART3_ProcessCommand();
}

static void expect(int a, int b, int c, int d, unsigned stage, unsigned reached)
{
    int32_t target[4];
    uint8_t actual_stage, actual_reached;
    USART3_GetWheelTargets(target);
    USART3_GetStageState(&actual_stage, &actual_reached);
    assert(target[0] == a && target[1] == b && target[2] == c && target[3] == d);
    assert(actual_stage == stage && actual_reached == reached);
}

int main(void)
{
    char line[80];
    char overflow[400];
    int i;
    const int32_t delta[4] = {1, -2, 3, -4};
    const char *bad[] = {
        "CMD_WHEELS 800 800 800 800 16\n", "CMD_WHEELS 800 800 800 800 -1\n",
        "CMD_WHEELS 800 800 800 800 true\n", "CMD_WHEELS 800 800 800 800 1x\n",
        "CMD_WHEELS 800 800 800 800 1 extra\n", "CMD_WHEELS 800 800 800\n",
        "CMD_WHEELSX 800 800 800 800 1\n", "CMD_WHEELS 800 800 800 800 1 0\n",
        "CMD_WHEELS 999999999999999999999999999999999999 0 0 0 1\n"
    };

    uart3_init(115200);
    command("CMD_WHEELS 200 -200 2000 -2000\r\n");
    expect(200, -200, 1000, -1000, 0, 0);
    USART_SendFeedback(4, 20, delta, 90000);
    assert(strcmp(tx, "FB 4 20 1 -2 3 -4 90000\r\n") == 0);

    command("CMD_WHEELS 200 200 200 200 1\r\n");
    expect(200, 200, 200, 200, 1, 0);
    tx_len = 0;
    USART3_SendStageFeedback();
    assert(strcmp(tx, "STAGE_FB 1 0\r\n") == 0);
    USART3_UpdatePhotoelectric(1, 0, 0);
    expect(0, 0, 0, 0, 1, 1);
    USART3_UpdatePhotoelectric(0, 0, 0);
    command("CMD_WHEELS 200 200 200 200 1\n");
    expect(0, 0, 0, 0, 1, 1);
    command("CMD_WHEELS 200 200 200 200\n");
    expect(0, 0, 0, 0, 1, 1); /* legacy input cannot unlock stage 1 */
    tx_len = 0;
    USART_SendFeedback(5, 20, delta, 90000);
    assert(strcmp(tx, "FB 5 20 1 -2 3 -4 90000\r\nSTAGE_FB 1 1\r\n") == 0);

    USART3_UpdatePhotoelectric(1, 0, 0);
    command("CMD_WHEELS -100 100 -100 100 2\n");
    expect(-100, 100, -100, 100, 2, 0); /* old front sensor is still active */
    USART3_UpdatePhotoelectric(1, 0, 1);
    expect(0, 0, 0, 0, 2, 1);
    USART3_UpdatePhotoelectric(1, 1, 1);
    command("CMD_WHEELS 100 -100 100 -100 4\n");
    expect(0, 0, 0, 0, 4, 1); /* new stage's left sensor is already active */

    /* Exercise every stage's sensor selection independently of wheel signs. */
    for (i = 1; i <= 15; ++i)
    {
        command("CMD_WHEELS 0 0 0 0 0\n");
        USART3_UpdatePhotoelectric(0, 0, 0);
        snprintf(line, sizeof(line), "CMD_WHEELS 100 100 100 100 %d\n", i);
        command(line);
        expect(100, 100, 100, 100, (unsigned)i, 0);
        if (i == 1)
            USART3_UpdatePhotoelectric(1, 0, 0);
        else if (i % 2 == 0 && i % 4 == 2)
            USART3_UpdatePhotoelectric(0, 0, 1);
        else if (i % 4 == 0)
            USART3_UpdatePhotoelectric(0, 1, 0);
        else
            USART3_UpdatePhotoelectric(1, 1, 1); /* reverse stages are unassigned */
        if (i == 1 || i % 2 == 0)
            expect(0, 0, 0, 0, (unsigned)i, 1);
        else
            expect(100, 100, 100, 100, (unsigned)i, 0);
    }

    USART3_UpdatePhotoelectric(0, 0, 0);
    command("CMD_WHEELS 123 123 123 123 3\n");
    for (i = 0; i < (int)(sizeof(bad) / sizeof(bad[0])); ++i)
    {
        command(bad[i]);
        expect(123, 123, 123, 123, 3, 0);
    }
    memset(overflow, 'X', 210);
    strcpy(overflow + 210, "CMD_WHEELS 900 900 900 900 1\n");
    command(overflow);
    expect(123, 123, 123, 123, 3, 0); /* no valid-looking suffix of an overflow */
    command("CMD_WHEELS 321 321");
    expect(123, 123, 123, 123, 3, 0);
    command(" 321 321 3\r\n");
    expect(321, 321, 321, 321, 3, 0);

    USART3_CommandWatchdogTick(299);
    expect(321, 321, 321, 321, 3, 0);
    command(bad[0]); /* malformed input does not feed the watchdog */
    USART3_CommandWatchdogTick(2);
    expect(0, 0, 0, 0, 3, 0);
    USART3_CommandWatchdogTick(UINT32_MAX);
    USART3_CommandWatchdogTick(1000);
    expect(0, 0, 0, 0, 3, 0);

    mock_primask = 1;
    command("CMD_WHEELS 50 50 50 50 1\n");
    assert(mock_primask == 1);
    USART3_UpdatePhotoelectric(1, 0, 0);
    assert(mock_primask == 1);
    USART3_CommandWatchdogTick(1);
    assert(mock_primask == 1);
    expect(0, 0, 0, 0, 1, 1);
    assert(mock_primask == 1);
    mock_primask = 0;

    /* Explicit manual stage zero can release a staged latch; legacy then works. */
    command("CMD_WHEELS 80 80 80 80 0\n");
    expect(80, 80, 80, 80, 0, 0);
    command("CMD_WHEELS 60 60 60 60\n");
    expect(60, 60, 60, 60, 0, 0);
    puts("USART3 host logic tests passed (parser, 15 stages, latch, watchdog, feedback, IRQ mask)");
    return 0;
}
'''


def main():
    root = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix='usart3-test-') as tmp:
        build = Path(tmp)
        (build / 'mock_stm32.h').write_text(MOCK_HEADER)
        for header in ('stm32f4xx_gpio.h', 'stm32f4xx_rcc.h', 'stm32f4xx_usart.h', 'misc.h'):
            (build / header).write_text('#include "mock_stm32.h"\n')
        (build / 'test.c').write_text(TEST_SOURCE)
        binary = build / 'test'
        subprocess.run([
            'gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-I', str(root), '-I', str(build), str(root / 'usart3_chassis.c'),
            str(build / 'test.c'), '-o', str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()

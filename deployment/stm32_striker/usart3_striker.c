/*
 * Independent STM32 striker board, not the chassis controller.
 * Host protocol: AA 55 01 MASK 0D 0A, MASK=01(left),02(right),03(both).
 * One complete frame = one event; discard all input while pending/running.
 * No ACK, periodic speed command, command watchdog, or remote stop in v1.
 */
#include "usart3_striker.h"
#include "sys.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#define STRIKE_FRAME_SIZE 6U
#define USART3_RX_ERRORS (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)

static uint8_t rx_frame[STRIKE_FRAME_SIZE];
static uint8_t rx_length = 0;
static volatile uint8_t strike_pending = STRIKE_NONE;
static volatile uint8_t strike_busy = 0;

static uint32_t save_irq_state(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

/* Called only by the USART3 ISR, with main-loop access protected by PRIMASK. */
static void receive_byte(uint8_t byte)
{
    uint8_t i;

    if (strike_busy)
    {
        /* Do not queue commands or retain a partial frame during a strike. */
        rx_length = 0;
        return;
    }

    rx_frame[rx_length++] = byte;
    if (rx_length < STRIKE_FRAME_SIZE)
    {
        return;
    }

    if (rx_frame[0] == 0xAAU && rx_frame[1] == 0x55U &&
        rx_frame[2] == 0x01U &&
        (rx_frame[3] == STRIKE_LEFT || rx_frame[3] == STRIKE_RIGHT ||
         rx_frame[3] == STRIKE_BOTH) &&
        rx_frame[4] == 0x0DU && rx_frame[5] == 0x0AU)
    {
        /* Latch busy here, before main can consume/start the event. */
        strike_busy = 1;
        strike_pending = rx_frame[3];
        rx_length = 0;
        return;
    }

    /* Bounded sliding window also recovers AA AA 55 ... and damaged frames. */
    for (i = 1; i < STRIKE_FRAME_SIZE; ++i)
    {
        rx_frame[i - 1U] = rx_frame[i];
    }
    rx_length = STRIKE_FRAME_SIZE - 1U;
}

void uart3_init(uint32_t bound)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    uint32_t saved = save_irq_state();

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* Initialize once at boot; do not reinitialize a running motor action. */
    USART_Cmd(USART3, DISABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    rx_length = 0;
    strike_pending = STRIKE_NONE;
    strike_busy = 0;

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = bound;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &uart);

    /* Reading SR followed by DR clears any old RXNE/overrun/error state. */
    (void)USART3->SR;
    (void)USART_ReceiveData(USART3);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
    __set_PRIMASK(saved);
}

void USART3_IRQHandler(void)
{
    uint32_t status;
    uint8_t byte;

#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    status = USART3->SR;
    if ((status & (USART_SR_RXNE | USART3_RX_ERRORS)) != 0U)
    {
        byte = (uint8_t)USART_ReceiveData(USART3);
        if ((status & USART3_RX_ERRORS) != 0U)
        {
            /* A damaged/missing byte invalidates the in-progress frame. */
            rx_length = 0;
        }
        else
        {
            receive_byte(byte);
        }
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}

uint8_t USART3_TakeStrikeCommand(void)
{
    uint8_t command;
    uint32_t saved = save_irq_state();

    command = strike_pending;
    strike_pending = STRIKE_NONE;
    /* Keep busy until the motor controller explicitly reports completion. */
    __set_PRIMASK(saved);
    return command;
}

uint8_t USART3_IsStrikeBusy(void)
{
    return strike_busy;
}

void USART3_StrikeFinished(void)
{
    uint32_t saved = save_irq_state();

    /* A partial command received while busy must not survive rearming. */
    rx_length = 0;
    strike_pending = STRIKE_NONE;
    strike_busy = 0;
    __set_PRIMASK(saved);
}

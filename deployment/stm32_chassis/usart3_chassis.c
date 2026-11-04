/* STM32F4 Standard Peripheral Library; replacement for the supplied USART3 block. */
#include "usart3_chassis.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USART3_REC_LEN 200
#define COMMAND_TIMEOUT_MS 300U
#define PHOTO_FRONT 0x01U
#define PHOTO_LEFT  0x02U
#define PHOTO_RIGHT 0x04U

static volatile uint8_t usart3_command_ready = 0;
static volatile uint16_t usart3_command_length = 0;
static volatile uint8_t usart3_rx_overflow = 0;
static char usart3_command_line[USART3_REC_LEN];
static char usart3_rx_line[USART3_REC_LEN];

static volatile int32_t usart3_wheel_target[4] = {0, 0, 0, 0};
static volatile uint32_t usart3_command_age_ms = COMMAND_TIMEOUT_MS + 1U;
static volatile uint8_t usart3_stage_id = 0;
static volatile uint8_t usart3_stage_reached = 0;
static volatile uint8_t usart3_photo_active = 0;

/* Restore the caller's interrupt mask, including calls from TIM7 or EXTI. */
static uint32_t lock_irq(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

static uint8_t photo_mask_for_stage(uint8_t stage)
{
    switch (stage)
    {
        case 1: return PHOTO_FRONT;
        case 2: case 6: case 10: case 14: return PHOTO_RIGHT;
        case 4: case 8: case 12: return PHOTO_LEFT;
        /* Stage 0 and reverse stages have no assigned photoelectric sensor. */
        default: return 0;
    }
}

/* Caller holds the interrupt mask. Same-stage commands never clear this latch. */
static void apply_stop_locked(void)
{
    int i;
    if ((usart3_photo_active & photo_mask_for_stage(usart3_stage_id)) != 0U)
        usart3_stage_reached = 1;

    if (usart3_stage_reached || usart3_command_age_ms > COMMAND_TIMEOUT_MS)
        for (i = 0; i < 4; ++i) usart3_wheel_target[i] = 0;
}

/* PD8=TX, PD9=RX; settings match the user's original initialization. */
void uart3_init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
    uint8_t byte;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == RESET) return;
    byte = (uint8_t)USART_ReceiveData(USART3);

    if (byte == '\n')
    {
        if (!usart3_rx_overflow && usart3_command_length > 0)
        {
            usart3_rx_line[usart3_command_length] = '\0';
            memcpy(usart3_command_line, usart3_rx_line, usart3_command_length + 1U);
            usart3_command_ready = 1;
        }
        usart3_command_length = 0;
        usart3_rx_overflow = 0;
    }
    else if (byte != '\r' && !usart3_rx_overflow)
    {
        if (usart3_command_length < USART3_REC_LEN - 1)
            usart3_rx_line[usart3_command_length++] = (char)byte;
        else
            usart3_rx_overflow = 1; /* Discard the entire line through its newline. */
    }
}

/* Accept exactly four legacy values or five staged values, with no trailing junk. */
static int parse_command(const char *line, long values[5])
{
    const char *p;
    char *end;
    int count = 0;

    if (strncmp(line, "CMD_WHEELS", 10) != 0) return 0;
    p = line + 10;
    if (*p != ' ' && *p != '\t') return 0;
    while (count < 5)
    {
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0') break;
        errno = 0;
        values[count] = strtol(p, &end, 10);
        if (end == p || errno == ERANGE ||
            (*end != '\0' && *end != ' ' && *end != '\t')) return 0;
        ++count;
        p = end;
    }
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != '\0' || (count != 4 && count != 5)) return 0;
    if (count == 5 && (values[4] < 0 || values[4] > 15)) return 0;
    return count;
}

/* Call from while(1). Parsing and formatting never run in the UART ISR. */
void USART3_ProcessCommand(void)
{
    char line[USART3_REC_LEN];
    long values[5];
    int count;
    int i;
    uint8_t ready;
    uint8_t stage;
    uint32_t saved = lock_irq();

    ready = usart3_command_ready;
    if (ready)
    {
        memcpy(line, usart3_command_line, sizeof(line));
        usart3_command_ready = 0;
    }
    __set_PRIMASK(saved);
    if (!ready) return;

    count = parse_command(line, values);
    if (count == 0) return;
    stage = count == 5 ? (uint8_t)values[4] : 0;

    saved = lock_irq();
    /* Old four-value frames cannot release an active staged stop. */
    if (count == 4 && usart3_stage_id != 0)
    {
        __set_PRIMASK(saved);
        return;
    }
    if (stage != usart3_stage_id)
    {
        usart3_stage_id = stage;
        usart3_stage_reached = 0;
    }
    for (i = 0; i < 4; ++i)
    {
        if (values[i] > 1000) values[i] = 1000;
        if (values[i] < -1000) values[i] = -1000;
        usart3_wheel_target[i] = (int32_t)values[i];
    }
    usart3_command_age_ms = 0;
    apply_stop_locked(); /* Also checks whether the NEW stage's sensor is active. */
    __set_PRIMASK(saved);
}

/* Call in TIM7 before reading the targets for the PI controller. */
void USART3_UpdatePhotoelectric(
    uint8_t front_active, uint8_t left_active, uint8_t right_active)
{
    uint32_t saved = lock_irq();
    usart3_photo_active = (uint8_t)(
        (front_active ? PHOTO_FRONT : 0U) |
        (left_active ? PHOTO_LEFT : 0U) |
        (right_active ? PHOTO_RIGHT : 0U));
    apply_stop_locked();
    __set_PRIMASK(saved);
}

void USART3_CommandWatchdogTick(uint32_t elapsed_ms)
{
    uint32_t saved = lock_irq();
    /* Saturate instead of wrapping back to a fresh command after a long outage. */
    if (elapsed_ms > COMMAND_TIMEOUT_MS + 1U - usart3_command_age_ms)
        usart3_command_age_ms = COMMAND_TIMEOUT_MS + 1U;
    else
        usart3_command_age_ms += elapsed_ms;
    apply_stop_locked();
    __set_PRIMASK(saved);
}

void USART3_GetWheelTargets(int32_t target_mmps[4])
{
    int i;
    uint32_t saved = lock_irq();
    apply_stop_locked();
    for (i = 0; i < 4; ++i) target_mmps[i] = usart3_wheel_target[i];
    __set_PRIMASK(saved);
}

void USART3_GetStageState(uint8_t *stage_id, uint8_t *reached)
{
    uint32_t saved = lock_irq();
    if (stage_id != NULL) *stage_id = usart3_stage_id;
    if (reached != NULL) *reached = usart3_stage_reached;
    __set_PRIMASK(saved);
}

static void send_text(const char *buf, int len)
{
    int i;
    for (i = 0; i < len; ++i)
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART3, (uint8_t)buf[i]);
    }
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {}
}

/* One caller only: use the same main-loop/task as USART_SendFeedback. */
void USART3_SendStageFeedback(void)
{
    char buf[32];
    int len;
    uint8_t stage;
    uint8_t reached;
    USART3_GetStageState(&stage, &reached);
    if (stage == 0) return;
    len = snprintf(buf, sizeof(buf), "STAGE_FB %u %u\r\n",
                   (unsigned)stage, (unsigned)reached);
    if (len > 0 && len < (int)sizeof(buf)) send_text(buf, len);
}

/* Keep the old FB frame, then append a separate STAGE_FB frame every period. */
void USART_SendFeedback(
    uint32_t sequence, uint32_t period_ms,
    const int32_t encoder_delta[4], int32_t yaw_mdeg)
{
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "FB %lu %lu %ld %ld %ld %ld %ld\r\n",
        (unsigned long)sequence, (unsigned long)period_ms,
        (long)encoder_delta[0], (long)encoder_delta[1],
        (long)encoder_delta[2], (long)encoder_delta[3], (long)yaw_mdeg);
    if (len > 0 && len < (int)sizeof(buf)) send_text(buf, len);
    USART3_SendStageFeedback();
}

#ifndef USART3_STRIKER_H
#define USART3_STRIKER_H

#include <stdint.h>

#define STRIKE_NONE   ((uint8_t)0x00U)
#define STRIKE_LEFT   ((uint8_t)0x01U)
#define STRIKE_RIGHT  ((uint8_t)0x02U)
#define STRIKE_BOTH   ((uint8_t)0x03U)

/* STM32F4 SPL: USART3, PD8=TX, PD9=RX. Call once, normally at 115200. */
void uart3_init(uint32_t bound);
void USART3_IRQHandler(void);

/*
 * Main-loop/task only. Returns NONE, LEFT, RIGHT, or BOTH exactly once.
 * Consuming the event does NOT release busy: run and stop the selected motor(s),
 * then call USART3_StrikeFinished(). No motor driver runs in the UART ISR.
 */
uint8_t USART3_TakeStrikeCommand(void);

/* Busy includes a received command that has not yet been consumed. */
uint8_t USART3_IsStrikeBusy(void);

/*
 * Call AFTER every selected motor has actually stopped (or a fault has stopped
 * the outputs). This only rearms reception; it does not stop any motor itself.
 * Never call unconditionally on each main-loop iteration.
 */
void USART3_StrikeFinished(void);

#endif

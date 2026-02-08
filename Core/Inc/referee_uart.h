#ifndef REFEREE_UART_H
#define REFEREE_UART_H

#include <stdint.h>

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REFEREE_UART_RX_DMA_SZ (128U)

void RefereeUart_SetControlTask(void *task_handle);
void RefereeUart_Task(UART_HandleTypeDef *huart);
void RefereeUart_UsartIdleIsr(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* REFEREE_UART_H */

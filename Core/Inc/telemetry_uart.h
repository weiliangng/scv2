#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef struct
{
  uint32_t records_queued;
  uint32_t records_dropped;
  uint32_t bytes_queued;
} TelemetryUartStats;

/*
 * Initialize the always-on, lossy USART telemetry sender. The UART and its TX
 * DMA channel must already have been initialized by CubeMX-generated code.
 */
void TelemetryUart_Init(UART_HandleTypeDef *huart);

/*
 * Copy and start one complete record without blocking. Returns false when a
 * previous DMA transfer is still active or the record cannot be started.
 * Task context only; there must be only one telemetry producer.
 */
bool TelemetryUart_TryWrite(const uint8_t *data, uint16_t len);

/* Copy USART telemetry counters into caller-provided storage. */
void TelemetryUart_GetStats(TelemetryUartStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_UART_H */

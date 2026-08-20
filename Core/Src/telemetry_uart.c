#include "telemetry_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

enum
{
  TELEMETRY_UART_TX_BUF_SIZE = 640U,
};

static UART_HandleTypeDef *s_huart;
static uint8_t s_tx_buffer[TELEMETRY_UART_TX_BUF_SIZE];
static TelemetryUartStats s_stats;

void TelemetryUart_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  memset(&s_stats, 0, sizeof(s_stats));
}

bool TelemetryUart_TryWrite(const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;

  taskENTER_CRITICAL();
  if ((s_huart == NULL) || (data == NULL) || (len == 0U) ||
      (len > (uint16_t)sizeof(s_tx_buffer)) ||
      (HAL_UART_GetState(s_huart) != HAL_UART_STATE_READY))
  {
    s_stats.records_dropped++;
    taskEXIT_CRITICAL();
    return false;
  }

  memcpy(s_tx_buffer, data, len);
  status = HAL_UART_Transmit_DMA(s_huart, s_tx_buffer, len);
  if (status != HAL_OK)
  {
    s_stats.records_dropped++;
    taskEXIT_CRITICAL();
    return false;
  }

  s_stats.records_queued++;
  s_stats.bytes_queued += len;
  taskEXIT_CRITICAL();
  return true;
}

void TelemetryUart_GetStats(TelemetryUartStats *stats)
{
  if (stats == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  *stats = s_stats;
  taskEXIT_CRITICAL();
}

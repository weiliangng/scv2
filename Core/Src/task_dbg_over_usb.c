/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : task_dbg_over_usb.c
  * @brief          : Debug output over USB CDC.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "task_dbg_over_usb.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stream_buffer.h"
#include "task.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "stm32g4xx_hal.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern void Error_Handler(void);

#define DBG_TX_CLI_BUF_SIZE 512U
#define DBG_TX_TELEMETRY_BUF_SIZE 1536U
#define DBG_TX_CHUNK_SIZE 512U

#define EV_TX_DONE (1UL << 0)
#define EV_DATA_READY (1UL << 1)

static StreamBufferHandle_t dbg_cli_stream = NULL;
static StreamBufferHandle_t dbg_telemetry_stream = NULL;
static StaticStreamBuffer_t dbg_cli_stream_struct;
static StaticStreamBuffer_t dbg_telemetry_stream_struct;
static uint8_t dbg_cli_stream_storage[DBG_TX_CLI_BUF_SIZE];
static uint8_t dbg_telemetry_stream_storage[DBG_TX_TELEMETRY_BUF_SIZE];
static uint8_t tx_chunk[DBG_TX_CHUNK_SIZE];
static uint16_t tx_len = 0;
static uint8_t tx_in_flight = 0;
static TaskHandle_t dbg_tx_task_handle = NULL;
static volatile DbgUsbStats dbg_stats;

static uint8_t cdc_tx_ready(void);
static void dbg_notify_tx_task(void);
static void dbg_clear_streams(void);
static uint16_t dbg_receive_next_chunk(void);

void DbgUsb_Init(void)
{
  dbg_cli_stream = xStreamBufferCreateStatic(
      DBG_TX_CLI_BUF_SIZE,
      1U,
      dbg_cli_stream_storage,
      &dbg_cli_stream_struct);
  dbg_telemetry_stream = xStreamBufferCreateStatic(
      DBG_TX_TELEMETRY_BUF_SIZE,
      1U,
      dbg_telemetry_stream_storage,
      &dbg_telemetry_stream_struct);
  if ((dbg_cli_stream == NULL) || (dbg_telemetry_stream == NULL))
  {
    Error_Handler();
  }
}

static uint8_t cdc_tx_ready(void)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return 0U;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) return 0U;
  return (hcdc->TxState == 0U);
}

static void dbg_notify_tx_task(void)
{
  if (dbg_tx_task_handle != NULL)
  {
    (void)xTaskNotify(dbg_tx_task_handle, EV_DATA_READY, eSetBits);
  }
}

static void dbg_clear_streams(void)
{
  while (xStreamBufferReceive(dbg_cli_stream, tx_chunk, sizeof(tx_chunk), 0U) > 0U)
  {
  }
  while (xStreamBufferReceive(dbg_telemetry_stream, tx_chunk, sizeof(tx_chunk), 0U) > 0U)
  {
  }
}

void dbg_write(const uint8_t *data, uint16_t len)
{
  if ((dbg_cli_stream == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }

  while (len > 0U)
  {
    const size_t chunk_len = (len < (DBG_TX_CLI_BUF_SIZE - 1U)) ?
                                 (size_t)len :
                                 (size_t)(DBG_TX_CLI_BUF_SIZE - 1U);
    const size_t sent = xStreamBufferSend(dbg_cli_stream, data, chunk_len, portMAX_DELAY);
    if (sent != chunk_len)
    {
      return;
    }

    taskENTER_CRITICAL();
    dbg_stats.cli_bytes_queued += (uint32_t)sent;
    taskEXIT_CRITICAL();

    data += sent;
    len = (uint16_t)(len - sent);
    dbg_notify_tx_task();
  }
}

bool dbg_try_write(const uint8_t *data, uint16_t len)
{
  size_t sent;

  if ((dbg_telemetry_stream == NULL) || (data == NULL) || (len == 0U) ||
      (len >= DBG_TX_TELEMETRY_BUF_SIZE))
  {
    return false;
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    taskENTER_CRITICAL();
    dbg_stats.telemetry_records_dropped++;
    taskEXIT_CRITICAL();
    return false;
  }

  if (xStreamBufferSpacesAvailable(dbg_telemetry_stream) < (size_t)len)
  {
    taskENTER_CRITICAL();
    dbg_stats.telemetry_records_dropped++;
    taskEXIT_CRITICAL();
    return false;
  }

  sent = xStreamBufferSend(dbg_telemetry_stream, data, (size_t)len, 0U);
  if (sent != (size_t)len)
  {
    taskENTER_CRITICAL();
    dbg_stats.telemetry_records_dropped++;
    taskEXIT_CRITICAL();
    return false;
  }

  taskENTER_CRITICAL();
  dbg_stats.telemetry_records_queued++;
  dbg_stats.telemetry_bytes_queued += (uint32_t)sent;
  taskEXIT_CRITICAL();
  dbg_notify_tx_task();
  return true;
}

void DbgUsb_GetStats(DbgUsbStats *stats)
{
  if (stats == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  *stats = dbg_stats;
  taskEXIT_CRITICAL();
}

void DbgUsb_OnTxCompleteFromISR(void)
{
  BaseType_t hpw = pdFALSE;
  if (dbg_tx_task_handle != NULL)
  {
    (void)xTaskNotifyFromISR(dbg_tx_task_handle, EV_TX_DONE, eSetBits, &hpw);
    if (hpw != pdFALSE)
    {
      portYIELD_FROM_ISR(hpw);
    }
  }
}

void DbgUsb_TxTask(void const * argument)
{
  (void)argument;

  dbg_tx_task_handle = xTaskGetCurrentTaskHandle();
  uint8_t was_configured = 0U;

  for(;;)
  {
    const uint8_t configured = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
    if (configured == 0U)
    {
      if (was_configured != 0U)
      {
        tx_in_flight = 0U;
        tx_len = 0U;
        dbg_clear_streams();
        taskENTER_CRITICAL();
        dbg_stats.usb_disconnect_count++;
        taskEXIT_CRITICAL();
      }
      was_configured = 0U;
      osDelay(20);
      continue;
    }

    was_configured = 1U;

    if (tx_in_flight != 0U)
    {
      uint32_t ev = 0U;
      (void)xTaskNotifyWait(0U, 0xFFFFFFFFUL, &ev, pdMS_TO_TICKS(50));
      if (((ev & EV_TX_DONE) != 0U) || (cdc_tx_ready() != 0U))
      {
        tx_in_flight = 0U;
        tx_len = 0U;
      }
      continue;
    }

    if (tx_len == 0U)
    {
      tx_len = dbg_receive_next_chunk();
      if (tx_len == 0U)
      {
        uint32_t ev = 0U;
        (void)xTaskNotifyWait(0U, 0xFFFFFFFFUL, &ev, pdMS_TO_TICKS(200));
        continue;
      }
    }

    if (CDC_Transmit_FS(tx_chunk, tx_len) == USBD_OK)
    {
      tx_in_flight = 1U;
    }
    else
    {
      /* Busy: wait a bit for the ongoing transfer to complete, then retry. */
      uint32_t ev = 0U;
      (void)xTaskNotifyWait(0U, 0xFFFFFFFFUL, &ev, pdMS_TO_TICKS(5));
    }
  }
}

static uint16_t dbg_receive_next_chunk(void)
{
  size_t received;

  received = xStreamBufferReceive(dbg_cli_stream, tx_chunk, sizeof(tx_chunk), 0U);
  if (received == 0U)
  {
    received = xStreamBufferReceive(dbg_telemetry_stream, tx_chunk, sizeof(tx_chunk), 0U);
  }

  return (uint16_t)received;
}

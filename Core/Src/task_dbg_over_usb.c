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

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern void Error_Handler(void);

#define DBG_TX_BUF_SIZE 2048U
#define DBG_TX_CHUNK_SIZE 512U

#define EV_TX_DONE (1UL << 0)

static StreamBufferHandle_t dbg_stream = NULL;
static StaticStreamBuffer_t dbg_stream_struct;
static uint8_t dbg_stream_storage[DBG_TX_BUF_SIZE];
static uint8_t tx_chunk[DBG_TX_CHUNK_SIZE];
static uint16_t tx_len = 0;
static uint8_t tx_in_flight = 0;
static TaskHandle_t dbg_tx_task_handle = NULL;
static volatile uint32_t dbg_drop_count = 0U;

static uint8_t cdc_tx_ready(void);
static uint8_t dbg_in_isr(void);
static void dbg_write_bytes(const uint8_t *data, uint16_t len);

void DbgUsb_Init(void)
{
  dbg_stream = xStreamBufferCreateStatic(
      DBG_TX_BUF_SIZE,
      1U,
      dbg_stream_storage,
      &dbg_stream_struct);
  if (dbg_stream == NULL)
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

static uint8_t dbg_in_isr(void)
{
  return (__get_IPSR() != 0U) ? 1U : 0U;
}

static void dbg_write_bytes(const uint8_t *data, uint16_t len)
{
  uint8_t in_isr = dbg_in_isr();
  BaseType_t hpw = pdFALSE;
  size_t sent = 0U;

  if ((dbg_stream == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    dbg_drop_count++;
    return;
  }

  if (in_isr != 0U)
  {
    sent = xStreamBufferSendFromISR(dbg_stream, data, len, &hpw);
    if (hpw != pdFALSE)
    {
      portYIELD_FROM_ISR(hpw);
    }
  }
  else
  {
    sent = xStreamBufferSend(dbg_stream, data, len, 0U);
  }
  if (sent < len)
  {
    dbg_drop_count++;
  }
}

void dbg_write(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }
  dbg_write_bytes(data, len);
}

int dbg_printf(const char *fmt, ...)
{
  char tmp[128];
  va_list ap;
  int n;

  if (fmt == NULL)
  {
    return 0;
  }

  va_start(ap, fmt);
  n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return n;
  }

  if (n >= (int)sizeof(tmp))
  {
    n = (int)sizeof(tmp) - 1;
  }

  dbg_write_bytes((const uint8_t *)tmp, (uint16_t)n);
  return n;
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
        uint8_t discard[DBG_TX_CHUNK_SIZE];
        tx_in_flight = 0U;
        tx_len = 0U;
        while (xStreamBufferReceive(dbg_stream, discard, sizeof(discard), 0U) > 0U)
        {
          /* Drain stale buffered output. */
        }
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
      tx_len = (uint16_t)xStreamBufferReceive(dbg_stream, tx_chunk, sizeof(tx_chunk), pdMS_TO_TICKS(200));
      if (tx_len == 0U)
      {
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

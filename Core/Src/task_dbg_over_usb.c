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

#define DBG_TX_BUF_SIZE 256U
#define DBG_TX_CHUNK_SIZE 64U

static StreamBufferHandle_t dbg_stream = NULL;
static StaticStreamBuffer_t dbg_stream_struct;
static uint8_t dbg_stream_storage[DBG_TX_BUF_SIZE];
static uint8_t dbg_tx_chunk[DBG_TX_CHUNK_SIZE];
static uint16_t dbg_tx_pending_len = 0;
static uint8_t dbg_tx_in_flight = 0;
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

void DbgUsb_TxTask(void const * argument)
{
  (void)argument;

  for(;;)
  {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
      dbg_tx_in_flight = 0U;
      dbg_tx_pending_len = 0U;
      osDelay(10);
      continue;
    }

    if (dbg_tx_in_flight != 0U)
    {
      if (cdc_tx_ready())
      {
        dbg_tx_in_flight = 0U;
        dbg_tx_pending_len = 0U;
      }
      else
      {
        osDelay(1);
        continue;
      }
    }

    if (dbg_tx_pending_len == 0U)
    {
      dbg_tx_pending_len = (uint16_t)xStreamBufferReceive(dbg_stream, dbg_tx_chunk, DBG_TX_CHUNK_SIZE, 0U);
      if (dbg_tx_pending_len == 0U)
      {
        osDelay(1);
        continue;
      }
    }

    if (cdc_tx_ready())
    {
      if (CDC_Transmit_FS(dbg_tx_chunk, dbg_tx_pending_len) == USBD_OK)
      {
        dbg_tx_in_flight = 1U;
      }
    }

    osDelay(1);
  }
}

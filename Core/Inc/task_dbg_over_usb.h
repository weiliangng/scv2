/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : task_dbg_over_usb.h
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
#ifndef TASK_DBG_OVER_USB_H
#define TASK_DBG_OVER_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Usage:
 * 1) Call DbgUsb_Init() once after HAL init, before starting the scheduler.
 * 2) Ensure MX_USB_Device_Init() is called from a task after the scheduler starts.
 * 3) Run DbgUsb_TxTask() inside a CubeMX-generated task (e.g., `usbCDCTxTask`).
 * 4) Use dbg_write()/dbg_printf() from tasks or ISRs to enqueue debug output.
 *
 * Notes:
 * - Non-blocking: producers never wait. If the stream buffer is full, newest data
 *   is dropped (drop counter increments internally).
 * - ISR-safe: dbg_write() can be called from ISRs.
 * - dbg_printf() truncates at 127 bytes per call.
 * - If USB is not configured, the TX task waits; queued data may be dropped if
 *   the buffer fills while waiting.
 */

// Create the stream buffer used for debug output. Call once at startup.
void DbgUsb_Init(void);
// Debug USB CDC TX task body. Call from a CubeMX-generated task entry function.
void DbgUsb_TxTask(void const *argument);

// Enqueue raw bytes for USB CDC transmission. ISR-safe, non-blocking.
void dbg_write(const uint8_t *data, uint16_t len);
// Enqueue formatted text (max 127 bytes). ISR-safe, non-blocking.
int dbg_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* TASK_DBG_OVER_USB_H */

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

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t cli_bytes_queued;
  uint32_t telemetry_records_queued;
  uint32_t telemetry_records_dropped;
  uint32_t telemetry_bytes_queued;
  uint32_t usb_disconnect_count;
} DbgUsbStats;

/*
 * Usage:
 * 1) Call DbgUsb_Init() once after HAL init, before starting the scheduler.
 * 2) Ensure MX_USB_Device_Init() is called from a task after the scheduler starts.
 * 3) Run DbgUsb_TxTask() inside a CubeMX-generated task (e.g., `usbCDCTxTask`).
 * 4) Use dbg_write()/dbg_printf() from task context for CLI/debug output.
 *
 * Notes:
 * - CLI output is queued reliably and has priority over telemetry output.
 * - dbg_try_write() is task-context, non-blocking, and all-or-nothing. Use it
 *   for lossy telemetry records that must not delay the application.
 * - dbg_write()/dbg_printf()/dbg_try_write() must not be called from an ISR.
 * - dbg_printf() truncates at 127 bytes per call.
 * - A USB disconnect discards stale buffered output from the previous session.
 */

// Create the stream buffer used for debug output. Call once at startup.
void DbgUsb_Init(void);
// Debug USB CDC TX task body. Call from a CubeMX-generated task entry function.
void DbgUsb_TxTask(void const *argument);

// Called from the USB CDC TX-complete callback (ISR context) to wake the TX task.
void DbgUsb_OnTxCompleteFromISR(void);

// Enqueue raw bytes for USB CDC transmission from task context. CLI output is
// queued completely before this function returns.
void dbg_write(const uint8_t *data, uint16_t len);
// Enqueue formatted text (max 127 bytes) from task context.
int dbg_printf(const char *fmt, ...);
// Attempt to enqueue one complete telemetry record without blocking. Returns
// false when USB is unavailable or the telemetry queue lacks enough space.
bool dbg_try_write(const uint8_t *data, uint16_t len);
// Copy USB-output counters into caller-provided storage.
void DbgUsb_GetStats(DbgUsbStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* TASK_DBG_OVER_USB_H */

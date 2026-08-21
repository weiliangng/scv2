#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef enum
{
  /* These loops remain periodic even when every external cable is absent.
   * USB TX, USB CLI, UART parsing, and CAN traffic are deliberately not
   * watchdog sources because their activity can depend on a connection. */
  APP_WATCHDOG_TASK_TELEMETRY = 0,
  APP_WATCHDOG_TASK_CONTROL,
  APP_WATCHDOG_TASK_COUNT
} app_watchdog_task_t;

/* Read directly by the 50 kHz control ISR so a watchdog failure cannot
 * subsequently re-enable SWEN while reset is pending. */
extern volatile uint8_t g_app_watchdog_failed;

void AppWatchdog_Init(IWDG_HandleTypeDef *hiwdg);
void AppWatchdog_RefreshBeforeScheduler(void);
void AppWatchdog_Heartbeat(app_watchdog_task_t task);
void AppWatchdog_SupervisorRun(void const *argument);
void AppWatchdog_FlashCheckpoint(void);

#endif /* APP_WATCHDOG_H */

#include "app_watchdog.h"

#include <stdbool.h>

#include "cmsis_os.h"
#include "main.h"
#include "shared_state.h"

#define APP_WATCHDOG_SUPERVISOR_PERIOD_MS 10u
#define APP_WATCHDOG_PROGRESS_TIMEOUT_MS  30u
#define APP_WATCHDOG_ISR_SOURCE_INDEX     ((uint32_t)APP_WATCHDOG_TASK_COUNT)
#define APP_WATCHDOG_SOURCE_COUNT         (APP_WATCHDOG_ISR_SOURCE_INDEX + 1u)
#define APP_WATCHDOG_ALL_SOURCES_MASK     ((1u << APP_WATCHDOG_SOURCE_COUNT) - 1u)

volatile uint8_t g_app_watchdog_failed;

static IWDG_HandleTypeDef *s_iwdg;
static volatile uint32_t s_task_heartbeat[APP_WATCHDOG_TASK_COUNT];
static volatile uint8_t s_flash_checkpoint_allowed;
static uint8_t s_initial_refresh_done;

static void AppWatchdog_Fail(void)
{
  s_flash_checkpoint_allowed = 0u;
  g_app_watchdog_failed = 1u;

  /* BSRR is atomic and immediately forces the converter off. */
  GPIO_SWEN_GPIO_Port->BSRR = (uint32_t)GPIO_SWEN_Pin << 16u;
}

void AppWatchdog_Init(IWDG_HandleTypeDef *hiwdg)
{
  s_iwdg = hiwdg;
  g_app_watchdog_failed = 0u;
  s_flash_checkpoint_allowed = 0u;
  s_initial_refresh_done = 0u;
  for (uint32_t task = 0u; task < (uint32_t)APP_WATCHDOG_TASK_COUNT; ++task)
  {
    s_task_heartbeat[task] = 0u;
  }

  /* Debugger halts must not look like application failures. */
  __HAL_DBGMCU_FREEZE_IWDG();
}

void AppWatchdog_RefreshBeforeScheduler(void)
{
  if ((s_iwdg != NULL) && (s_initial_refresh_done == 0u) &&
      (g_app_watchdog_failed == 0u))
  {
    s_initial_refresh_done = 1u;
    if (HAL_IWDG_Refresh(s_iwdg) != HAL_OK)
    {
      AppWatchdog_Fail();
    }
  }
}

void AppWatchdog_Heartbeat(app_watchdog_task_t task)
{
  if ((uint32_t)task < (uint32_t)APP_WATCHDOG_TASK_COUNT)
  {
    s_task_heartbeat[task]++;
  }
}

void AppWatchdog_FlashCheckpoint(void)
{
  if ((s_iwdg != NULL) && (s_flash_checkpoint_allowed != 0u) &&
      (g_app_watchdog_failed == 0u))
  {
    if (HAL_IWDG_Refresh(s_iwdg) != HAL_OK)
    {
      AppWatchdog_Fail();
    }
  }
}

void AppWatchdog_SupervisorRun(void const *argument)
{
  (void)argument;

  uint32_t last_value[APP_WATCHDOG_SOURCE_COUNT];
  uint32_t last_progress_ms[APP_WATCHDOG_SOURCE_COUNT];
  uint32_t seen_mask = 0u;
  const uint32_t start_ms = HAL_GetTick();

  for (uint32_t task = 0u; task < (uint32_t)APP_WATCHDOG_TASK_COUNT; ++task)
  {
    last_value[task] = s_task_heartbeat[task];
    last_progress_ms[task] = start_ms;
  }
  last_value[APP_WATCHDOG_ISR_SOURCE_INDEX] = g_adc_seq_count;
  last_progress_ms[APP_WATCHDOG_ISR_SOURCE_INDEX] = start_ms;

  for (;;)
  {
    osDelay(APP_WATCHDOG_SUPERVISOR_PERIOD_MS);
    const uint32_t now_ms = HAL_GetTick();

    for (uint32_t task = 0u; task < (uint32_t)APP_WATCHDOG_TASK_COUNT; ++task)
    {
      const uint32_t value = s_task_heartbeat[task];
      if (value != last_value[task])
      {
        last_value[task] = value;
        last_progress_ms[task] = now_ms;
        seen_mask |= 1u << task;
      }
    }

    const uint32_t adc_sequence = g_adc_seq_count;
    if (adc_sequence != last_value[APP_WATCHDOG_ISR_SOURCE_INDEX])
    {
      last_value[APP_WATCHDOG_ISR_SOURCE_INDEX] = adc_sequence;
      last_progress_ms[APP_WATCHDOG_ISR_SOURCE_INDEX] = now_ms;
      seen_mask |= 1u << APP_WATCHDOG_ISR_SOURCE_INDEX;
    }

    bool healthy = (seen_mask == APP_WATCHDOG_ALL_SOURCES_MASK);
    for (uint32_t source = 0u; source < APP_WATCHDOG_SOURCE_COUNT; ++source)
    {
      if ((uint32_t)(now_ms - last_progress_ms[source]) >= APP_WATCHDOG_PROGRESS_TIMEOUT_MS)
      {
        healthy = false;
      }
    }

    if (healthy)
    {
      s_flash_checkpoint_allowed = 1u;
      if ((s_iwdg == NULL) || (HAL_IWDG_Refresh(s_iwdg) != HAL_OK))
      {
        AppWatchdog_Fail();
      }
    }
    else if ((uint32_t)(now_ms - start_ms) >= APP_WATCHDOG_PROGRESS_TIMEOUT_MS)
    {
      AppWatchdog_Fail();
    }

    if (g_app_watchdog_failed != 0u)
    {
      for (;;)
      {
        GPIO_SWEN_GPIO_Port->BSRR = (uint32_t)GPIO_SWEN_Pin << 16u;
        osDelay(1u);
      }
    }
  }
}

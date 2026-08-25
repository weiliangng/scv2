/*
 * telemetry_slow_adc_task.c
 *
 * Slow ADC sampling + derived telemetry processing + periodic CAN status frame.
 */

#include "telemetry_slow_adc_task.h"

#include "cmsis_os.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_constants.h"
#include "app_watchdog.h"
#include "can_protocol.h"
#include "capacitor_monitor.h"
#include "shared_state.h"
#include "scap_io_owner.h"

static inline uint16_t clamp_u16(int32_t v)
{
  if (v < 0)
  {
    return 0u;
  }
  if (v > 65535)
  {
    return 65535u;
  }
  return (uint16_t)v;
}

static inline int16_t clamp_i16(int32_t v)
{
  if (v < -32768)
  {
    return (int16_t)-32768;
  }
  if (v > 32767)
  {
    return 32767;
  }
  return (int16_t)v;
}

static inline uint8_t clamp_u8(int32_t v)
{
  if (v < 0)
  {
    return 0u;
  }
  if (v > 255)
  {
    return 255u;
  }
  return (uint8_t)v;
}

void TelemetrySlowAdcTask_Run(void const *argument)
{
  (void)argument;

  static const uint8_t status_code = 0u;
  FDCAN_TxHeaderTypeDef tx_header = {
      .Identifier = DEVC_NODE_ID,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0u,
  };

  uint32_t rate_hz = SCAP_STAT_RATE_HZ;
  if (rate_hz < 10u)
  {
    rate_hz = 10u;
  }
  if (rate_hz > 1000u)
  {
    rate_hz = 1000u;
  }
  uint32_t period_ms = 1000u / rate_hz;
  if (period_ms == 0u)
  {
    period_ms = 1u;
  }

  uint32_t last_tx_ms = HAL_GetTick() - period_ms;
  uint32_t last_can_bus_poll_ms = HAL_GetTick() - CAN_BUS_ACTIVITY_POLL_MS;

  for (;;)
  {
    const float v_cap = g_latest.v_cap;
    const uint32_t now_ms = HAL_GetTick();
    CapacitorMonitor_Update1kHz(now_ms, v_cap);
    ScapIo_Resolve1kHz();
    if ((uint32_t)(now_ms - last_can_bus_poll_ms) >= CAN_BUS_ACTIVITY_POLL_MS)
    {
      last_can_bus_poll_ms = now_ms;
      CanProtocol_PollBusActivity();
    }

    if ((uint32_t)(now_ms - last_tx_ms) >= period_ms)
    {
      last_tx_ms = now_ms;

      const float v_bus = g_latest.v_bus;
      const float i_load = g_latest.i_load;
      const float i_conv = g_latest.i_conv;
      const float v_cap = g_latest.v_cap;

      float p_load = v_bus * i_load;

      const uint16_t v_bus_10mV = clamp_u16((int32_t)((v_bus * 100.0f) + 0.5f));
      const int16_t i_load_10mA = clamp_i16((int32_t)((i_load * 100.0f) + ((i_load >= 0.0f) ? 0.5f : -0.5f)));
      const int16_t i_conv_10mA = clamp_i16((int32_t)((i_conv * 100.0f) + ((i_conv >= 0.0f) ? 0.5f : -0.5f)));

      const float v_cap_max = CapacitorMonitor_GetVcapMaxV();
      const float e_cap_max = 0.5f * C_cap * v_cap_max * v_cap_max;
      const float e_now = 0.5f * C_cap * v_cap * v_cap;
      float capacity_pct_f = 0.0f;
      if (e_cap_max > 0.0f)
      {
        capacity_pct_f = (e_now / e_cap_max) * 100.0f;
      }
      if (capacity_pct_f > 100.0f)
      {
        capacity_pct_f = 100.0f;
      }
      uint8_t capacity_pct = clamp_u8((int32_t)(capacity_pct_f + 0.5f));

      uint8_t data[8];
      data[0] = (uint8_t)(v_bus_10mV & 0xFFu);
      data[1] = (uint8_t)((v_bus_10mV >> 8) & 0xFFu);
      data[2] = (uint8_t)((uint16_t)i_load_10mA & 0xFFu);
      data[3] = (uint8_t)(((uint16_t)i_load_10mA >> 8) & 0xFFu);
      data[4] = (uint8_t)((uint16_t)i_conv_10mA & 0xFFu);
      data[5] = (uint8_t)(((uint16_t)i_conv_10mA >> 8) & 0xFFu);
      data[6] = capacity_pct;
      data[7] = status_code;

      (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
    }

    AppWatchdog_Heartbeat(APP_WATCHDOG_TASK_CONTROL);
    osDelay(1);
  }
}

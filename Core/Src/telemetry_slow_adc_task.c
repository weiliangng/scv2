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

static volatile uint32_t s_can_tx_enqueue_failures;

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

static uint8_t make_status_code(const control_status_t *control_status)
{
  uint8_t status_code = 0u;
  if ((control_status->fault_bits & CONTROL_FAULT_VBUS_OVP) != 0u)
  {
    status_code |= CAN_TELEMETRY_STATUS_VBUS_OVP;
  }
  if ((control_status->fault_bits & CONTROL_FAULT_VCAP_OVP) != 0u)
  {
    status_code |= CAN_TELEMETRY_STATUS_VCAP_OVP;
  }
  return status_code;
}

uint32_t TelemetrySlowAdcTask_GetCanTxEnqueueFailureCount(void)
{
  return s_can_tx_enqueue_failures;
}

void TelemetrySlowAdcTask_Run(void const *argument)
{
  (void)argument;

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
    const uint32_t now_ms = HAL_GetTick();
    CapacitorMonitor_Update1kHz(now_ms, g_latest.v_cap);
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
      const float p_load = v_bus * i_load;
      control_status_t control_status;
      ScapIo_ReadStatus(&control_status);

      const uint16_t p_load_100mW = clamp_u16((int32_t)((p_load * 10.0f) + 0.5f));
      const uint16_t v_cap_100mV = clamp_u16((int32_t)((v_cap * 10.0f) + 0.5f));
      const int16_t i_conv_100mA = clamp_i16((int32_t)((i_conv * 10.0f) + ((i_conv >= 0.0f) ? 0.5f : -0.5f)));
      const uint8_t status_code = make_status_code(&control_status);

      uint8_t data[8];
      data[0] = (uint8_t)(p_load_100mW & 0xFFu);
      data[1] = (uint8_t)((p_load_100mW >> 8) & 0xFFu);
      data[2] = (uint8_t)((uint16_t)v_cap_100mV & 0xFFu);
      data[3] = (uint8_t)(((uint16_t)v_cap_100mV >> 8) & 0xFFu);
      data[4] = (uint8_t)((uint16_t)i_conv_100mA & 0xFFu);
      data[5] = (uint8_t)(((uint16_t)i_conv_100mA >> 8) & 0xFFu);
      data[6] = 0;
      data[7] = status_code;

      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data) != HAL_OK)
      {
        s_can_tx_enqueue_failures++;
      }
    }

    AppWatchdog_Heartbeat(APP_WATCHDOG_TASK_CONTROL);
    osDelay(1);
  }
}

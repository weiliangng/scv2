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
#include "shared_state.h"
#include "scap_io_owner.h"

#define P_SET_DEFAULT_W 50.0f
#define P_SET_MIN_W 0.0f
#define P_SET_MAX_W 240.0f

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

static inline float clamp_p_set_w(float p_w)
{
  if (p_w < P_SET_MIN_W)
  {
    return P_SET_MIN_W;
  }
  if (p_w > P_SET_MAX_W)
  {
    return P_SET_MAX_W;
  }
  return p_w;
}

static void update_rx_connection_status_1khz(void)
{
  const uint32_t now_ms = HAL_GetTick();

  bool can_connected = false;
  if (g_can_rx.can_rx_count != 0u)
  {
    const uint32_t last_ms = g_can_rx.last_can_tick;
    can_connected = ((uint32_t)(now_ms - last_ms) <= CAN_RX_LINK_TIMEOUT_MS);
  }
  g_can_connected = can_connected;

  bool can_cmd_connected = false;
  if (g_can_rx.can_rx_count != 0u)
  {
    const uint32_t last_cmd_ms = g_can_rx.last_cmd_tick;
    can_cmd_connected = ((uint32_t)(now_ms - last_cmd_ms) <= CAN_CMD_TIMEOUT_MS);
  }
  g_can_cmd_connected = can_cmd_connected;

  bool uart_connected = false;
  if (g_uart_rx.uart_rx_count != 0u)
  {
    const TickType_t now_ticks = xTaskGetTickCount();
    const TickType_t last_ticks = (TickType_t)g_uart_rx.last_uart_tick;
    uart_connected = ((TickType_t)(now_ticks - last_ticks) <= pdMS_TO_TICKS(UART_RX_LINK_TIMEOUT_MS));
  }
  g_uart_connected = uart_connected;
}

static void update_p_set_1khz(void)
{
  float p_set_w = P_SET_DEFAULT_W;

  const ctrl_src_t src = g_ctrl_src;
  if (src == SRC_MANUAL)
  {
    p_set_w = g_manual_p_set_w;
  }
  else
  {
    if (g_uart_connected)
    {
      p_set_w = g_uart_rx.chassis_power_limit_w;
    }
    else if (g_can_cmd_connected)
    {
      p_set_w = (float)g_can_rx.can_power;
    }
    else
    {
      p_set_w = P_SET_DEFAULT_W;
    }
  }

  g_latest.p_set = clamp_p_set_w(p_set_w);
}

void TelemetrySlowAdcTask_Run(void const *argument)
{
  (void)argument;

  static const uint8_t status_code = 0u;
  FDCAN_TxHeaderTypeDef tx_header = {
      .Identifier = SCAP_STAT_ID,
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

  for (;;)
  {
    ScapIo_Tick1kHz();
    update_rx_connection_status_1khz();
    update_p_set_1khz();

    const uint16_t n_adc_imonop = g_adc2_dma_buf[1] & 0x0FFFU;
    const uint16_t n_adc_imonon = g_adc2_dma_buf[2] & 0x0FFFU;

    const float v_cap = g_latest.v_cap;

    const float i_out_p = (A_OP * (float)n_adc_imonop) + B_OP;
    const float i_out_n = (A_ON * (float)n_adc_imonon) + B_ON;

    const float i_out = (i_out_p > -i_out_n) ? i_out_p : i_out_n;

    g_latest.i_out_p = i_out_p;
    g_latest.i_out_n = i_out_n;
    g_latest.i_out = i_out;

    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_tx_ms) >= period_ms)
    {
      last_tx_ms = now_ms;

      const float v_bus = g_latest.v_bus;
      const float i_load = g_latest.i_load;
      const float i_conv = g_latest.i_conv;

      const uint16_t v_bus_10mV = clamp_u16((int32_t)((v_bus * 100.0f) + 0.5f));
      const int16_t i_load_10mA = clamp_i16((int32_t)((i_load * 100.0f) + ((i_load >= 0.0f) ? 0.5f : -0.5f)));
      const int16_t i_conv_10mA = clamp_i16((int32_t)((i_conv * 100.0f) + ((i_conv >= 0.0f) ? 0.5f : -0.5f)));

      const float e_cap_max = 0.5f * C_cap * V_cap_max * V_cap_max;
      const float e_now = 0.5f * C_cap * v_cap * v_cap;
      float capacity_pct_f = 0.0f;
      if (e_cap_max > 0.0f)
      {
        capacity_pct_f = (e_now / e_cap_max) * 100.0f;
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

    osDelay(1);
  }
}

#include "usb_cli.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "embedded_cli.h"
#include "shared_state.h"
#include "stream_buffer.h"
#include "task.h"
#include "task_dbg_over_usb.h"
#include "telemetry_uart.h"

#include "main.h"
#include "app_constants.h"
#include "can_protocol.h"
#include "command_inputs.h"
#include "scap_io_owner.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;

#define USBCLI_RX_BUF_SIZE 256U

static StreamBufferHandle_t usbcli_rx_stream = NULL;
static StaticStreamBuffer_t usbcli_rx_stream_struct;
static uint8_t usbcli_rx_stream_storage[USBCLI_RX_BUF_SIZE];

static struct embedded_cli usbcli;

static uint8_t usbcli_in_isr(void)
{
  return (__get_IPSR() != 0U) ? 1U : 0U;
}

static void usbcli_puts(const char *s)
{
  if (s == NULL)
  {
    return;
  }
  dbg_write((const uint8_t *)s, (uint16_t)strlen(s));
}

static void usbcli_put_char(void *data, char ch, bool is_last)
{
  static uint8_t out_buf[64];
  static uint16_t out_len = 0U;

  (void)data;

  out_buf[out_len++] = (uint8_t)ch;
  if ((out_len >= (uint16_t)sizeof(out_buf)) || (is_last != false))
  {
    dbg_write(out_buf, out_len);
    out_len = 0U;
  }
}

static int usbcli_printf(const char *fmt, ...)
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
  dbg_write((const uint8_t *)tmp, (uint16_t)n);
  return n;
}

static int usbcli_streq(const char *a, const char *b)
{
  if ((a == NULL) || (b == NULL))
  {
    return 0;
  }
  return (strcmp(a, b) == 0) ? 1 : 0;
}

static int usbcli_parse_u32(const char *s, uint32_t *out)
{
  char *endp = NULL;
  unsigned long val;

  if ((s == NULL) || (out == NULL))
  {
    return 0;
  }

  errno = 0;
  val = strtoul(s, &endp, 0);
  if ((errno != 0) || (endp == s) || (endp == NULL) || (*endp != '\0'))
  {
    return 0;
  }

  *out = (uint32_t)val;
  return 1;
}

static int usbcli_parse_i32(const char *s, int32_t *out)
{
  char *endp = NULL;
  long val;

  if ((s == NULL) || (out == NULL))
  {
    return 0;
  }

  errno = 0;
  val = strtol(s, &endp, 0);
  if ((errno != 0) || (endp == s) || (endp == NULL) || (*endp != '\0') ||
      (val < INT32_MIN) || (val > INT32_MAX))
  {
    return 0;
  }

  *out = (int32_t)val;
  return 1;
}

static int usbcli_parse_f32(const char *s, float *out)
{
  char *endp = NULL;
  float val;

  if ((s == NULL) || (out == NULL))
  {
    return 0;
  }

  errno = 0;
  val = strtof(s, &endp);
  if ((errno != 0) || (endp == s) || (endp == NULL) || (*endp != '\0') || (!isfinite(val)))
  {
    return 0;
  }

  *out = val;
  return 1;
}

static int usbcli_parse_gpio_pin(const char *s, GPIO_TypeDef **port, uint16_t *pin)
{
  size_t idx = 0U;
  char port_ch;
  uint32_t pin_num;

  if ((s == NULL) || (port == NULL) || (pin == NULL))
  {
    return 0;
  }

  if ((s[0] == 'P') || (s[0] == 'p'))
  {
    idx = 1U;
  }

  port_ch = (char)toupper((unsigned char)s[idx]);
  if ((port_ch < 'A') || (port_ch > 'G'))
  {
    return 0;
  }
  idx++;

  if (!usbcli_parse_u32(&s[idx], &pin_num))
  {
    return 0;
  }
  if (pin_num > 15U)
  {
    return 0;
  }

  switch (port_ch)
  {
#if defined(GPIOA)
  case 'A':
    *port = GPIOA;
    break;
#endif
#if defined(GPIOB)
  case 'B':
    *port = GPIOB;
    break;
#endif
#if defined(GPIOC)
  case 'C':
    *port = GPIOC;
    break;
#endif
#if defined(GPIOD)
  case 'D':
    *port = GPIOD;
    break;
#endif
#if defined(GPIOE)
  case 'E':
    *port = GPIOE;
    break;
#endif
#if defined(GPIOF)
  case 'F':
    *port = GPIOF;
    break;
#endif
#if defined(GPIOG)
  case 'G':
    *port = GPIOG;
    break;
#endif
  default:
    return 0;
  }

  *pin = (uint16_t)(1U << pin_num);
  return 1;
}

static void usbcli_print_f32_6dp(const char *name, float v)
{
  if (name == NULL)
  {
    return;
  }

  if (!isfinite(v))
  {
    usbcli_printf("%s=nan\r\n", name);
    return;
  }

  const bool neg = (v < 0.0f);
  const float av = neg ? -v : v;
  long ip = (long)av;
  unsigned long fp = (unsigned long)(((av - (float)ip) * 1000000.0f) + 0.5f);
  if (fp >= 1000000UL)
  {
    ip += 1L;
    fp -= 1000000UL;
  }

  usbcli_printf("%s=%s%ld.%06lu\r\n", name, neg ? "-" : "", ip, fp);
}

static void usbcli_print_cal_current(void)
{
  usbcli_print_f32_6dp("A_VBUS", A_VBUS);
  usbcli_print_f32_6dp("B_VBUS", B_VBUS);
  usbcli_print_f32_6dp("A_ILOAD", A_ILOAD);
  usbcli_print_f32_6dp("B_ILOAD", B_ILOAD);
  usbcli_print_f32_6dp("MIDPOINT", MIDPOINT);

  usbcli_print_f32_6dp("A_INP", A_INP);
  usbcli_print_f32_6dp("B_INP", B_INP);
  usbcli_print_f32_6dp("A_INN", A_INN);
  usbcli_print_f32_6dp("B_INN", B_INN);

  usbcli_print_f32_6dp("A_VCAP", A_VCAP);
  usbcli_print_f32_6dp("B_VCAP", B_VCAP);

  usbcli_print_f32_6dp("A_OP", A_OP);
  usbcli_print_f32_6dp("B_OP", B_OP);
  usbcli_print_f32_6dp("A_ON", A_ON);
  usbcli_print_f32_6dp("B_ON", B_ON);

  usbcli_printf("DAC3_CH1_BOOT_U12=%lu\r\n", (unsigned long)DAC3_CH1_BOOT_U12);
  usbcli_printf("DAC3_CH2_BOOT_U12=%lu\r\n", (unsigned long)DAC3_CH2_BOOT_U12);
}

static void usbcli_cmd_help(void)
{
  static const char help[] =
      "Commands:\r\n"
      "  help\r\n"
      "  status\r\n"
      "  telemetry on|off|toggle  (USB mirror; USART1 is always on)\r\n"
      "  ctrl <external|manual|measure|direct>\r\n"
      "  pset <-240..240>\r\n"
      "  swen <0|1>\r\n"
      "  gpio write <PA10|PB1|...> <0|1>\r\n"
      "  gpio toggle <PA10|PB1|...>\r\n"
      "  dac set <1|3> <1|2> <0..4095>\r\n"
      "  cal set <NAME> <VALUE>\r\n"
      "  cal load\r\n"
      "  cal save\r\n";
  usbcli_puts(help);
}

static void usbcli_cmd_status(void)
{
  enum
  {
    STATUS_AVG_SAMPLES = 1000,
    STATUS_AVG_DELAY_MS = 1,
  };

  int64_t sum_v_bus_mV = 0;
  int64_t sum_v_cap_mV = 0;
  int64_t sum_i_load_mA = 0;
  int64_t sum_i_out_mA = 0;
  int64_t sum_i_conv_mA = 0;
  DbgUsbStats dbg_stats;
  TelemetryUartStats telemetry_uart_stats;

  for (uint32_t i = 0; i < STATUS_AVG_SAMPLES; i++)
  {
    const float v_bus = g_latest.v_bus;
    const float v_cap = g_latest.v_cap;
    const float i_load = g_latest.i_load;
    const float i_out = g_latest.i_out;
    const float i_conv = g_latest.i_conv;

    sum_v_bus_mV += (int32_t)(v_bus * 1000.0f);
    sum_v_cap_mV += (int32_t)(v_cap * 1000.0f);
    sum_i_load_mA += (int32_t)(i_load * 1000.0f);
    sum_i_out_mA += (int32_t)(i_out * 1000.0f);
    sum_i_conv_mA += (int32_t)(i_conv * 1000.0f);

    osDelay(STATUS_AVG_DELAY_MS);
  }

  const int32_t v_bus_avg_mV = (int32_t)((sum_v_bus_mV + (STATUS_AVG_SAMPLES / 2)) / STATUS_AVG_SAMPLES);
  const int32_t v_cap_avg_mV = (int32_t)((sum_v_cap_mV + (STATUS_AVG_SAMPLES / 2)) / STATUS_AVG_SAMPLES);
  const int32_t i_load_avg_mA = (int32_t)((sum_i_load_mA + (STATUS_AVG_SAMPLES / 2)) / STATUS_AVG_SAMPLES);
  const int32_t i_out_avg_mA = (int32_t)((sum_i_out_mA + (STATUS_AVG_SAMPLES / 2)) / STATUS_AVG_SAMPLES);
  const int32_t i_conv_avg_mA = (int32_t)((sum_i_conv_mA + (STATUS_AVG_SAMPLES / 2)) / STATUS_AVG_SAMPLES);

  const uint32_t telemetry_seq = g_telemetry_seq;
  const uint32_t adc_seq_hz = g_adc_seq_hz;
  control_status_t control_status;
  ScapIo_ReadStatus(&control_status);
  can_command_state_t can_command;
  uart_command_state_t uart_command;
  manual_command_state_t manual_command;
  pushbutton_command_state_t pushbutton_command;
  CanProtocol_ReadCommandSnapshot(&can_command);
  CommandInputs_ReadUartSnapshot(&uart_command);
  CommandInputs_ReadManualSnapshot(&manual_command);
  CommandInputs_ReadPushbuttonSnapshot(&pushbutton_command);
  const uint32_t can_status_now_ms = HAL_GetTick();
  const bool can_command_fresh = CommandInputs_IsFresh(can_command.can_cmd_timestamp,
                                                        can_command.can_cmd_timestamp != 0u,
                                                        can_status_now_ms,
                                                        SCAP_COMMAND_FRESH_TIMEOUT_MS);
  const bool can_bus_up = CanProtocol_IsBusActive(can_status_now_ms);
  DbgUsb_GetStats(&dbg_stats);
  TelemetryUart_GetStats(&telemetry_uart_stats);

  const unsigned swen = (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_SWEN_Pin) != GPIO_PIN_RESET) ? 1u : 0u);
  const unsigned mode_msb = (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_MODEMSB_Pin) != GPIO_PIN_RESET) ? 1u : 0u);
  const unsigned mode_lsb = (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_MODELSB_Pin) != GPIO_PIN_RESET) ? 1u : 0u);
  const unsigned dir = (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_DIR_Pin) != GPIO_PIN_RESET) ? 1u : 0u);
  const unsigned mode_u2 = (unsigned)((mode_msb << 1) | mode_lsb);

  const uint32_t dac1_1_u12 = (uint32_t)(HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1) & 0x0FFFu);
  const uint32_t dac1_2_u12 = (uint32_t)(HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_2) & 0x0FFFu);
  const uint32_t dac3_1_u12 = (uint32_t)(HAL_DAC_GetValue(&hdac3, DAC_CHANNEL_1) & 0x0FFFu);

  const uint32_t dac1_1_mV = (uint32_t)((dac1_1_u12 * 3300u + 2047u) / 4095u);
  const uint32_t dac1_2_mV = (uint32_t)((dac1_2_u12 * 3300u + 2047u) / 4095u);
  const uint32_t dac3_1_mV = (uint32_t)((dac3_1_u12 * 3300u + 2047u) / 4095u);

  usbcli_printf("Control:\r\n");
  usbcli_printf("  USB telemetry mirror enabled: %s\r\n", g_usb_telemetry_enabled ? "yes" : "no");
  usbcli_printf("  USART1 telemetry streaming: always on\r\n");
  usbcli_printf("  Current mode: %s\r\n", ScapIo_DecisionName(control_status.decision));
  usbcli_printf("  Switch enable output (SWEN) pin: %u\r\n", swen);
  usbcli_printf("  Mode selection pins (MODE[1:0]): %u%u (decoded=%u)\r\n", mode_msb, mode_lsb, mode_u2);
  usbcli_printf("  Direction pin (DIR): %u\r\n", dir);
  usbcli_printf("  Safety flag (is_safe): %u\r\n", g_is_safe ? 1u : 0u);
  usbcli_printf("  Fault latched/bits/recovery-ms: %u/0x%02X/%u\r\n",
                control_status.fault_latched ? 1u : 0u,
                (unsigned)control_status.fault_bits,
                (unsigned)control_status.fault_healthy_ms);
  usbcli_printf("  UVLO lockout: %u\r\n", control_status.uvlo_lockout ? 1u : 0u);

  usbcli_printf("Digital-to-analog converter outputs:\r\n");
  usbcli_printf("  DAC1 channel 1 raw: %lu / 4095\r\n", (unsigned long)dac1_1_u12);
  usbcli_printf("  DAC1 channel 1 voltage: %lu.%03lu V\r\n",
                (unsigned long)(dac1_1_mV / 1000u),
                (unsigned long)(dac1_1_mV % 1000u));
  usbcli_printf("  DAC1 channel 2 raw: %lu / 4095\r\n", (unsigned long)dac1_2_u12);
  usbcli_printf("  DAC1 channel 2 voltage: %lu.%03lu V\r\n",
                (unsigned long)(dac1_2_mV / 1000u),
                (unsigned long)(dac1_2_mV % 1000u));
  usbcli_printf("  DAC3 channel 1 raw: %lu / 4095\r\n", (unsigned long)dac3_1_u12);
  usbcli_printf("  DAC3 channel 1 voltage: %lu.%03lu V\r\n",
                (unsigned long)(dac3_1_mV / 1000u),
                (unsigned long)(dac3_1_mV % 1000u));

  usbcli_printf("Measured and computed values:\r\n");
  usbcli_printf("  Averaging window: %lu samples, %lu ms delay\r\n",
                (unsigned long)STATUS_AVG_SAMPLES,
                (unsigned long)STATUS_AVG_DELAY_MS);
  usbcli_printf("  Bus voltage (average): %ld mV\r\n", (long)v_bus_avg_mV);
  usbcli_printf("  Capacitor voltage (average): %ld mV\r\n", (long)v_cap_avg_mV);
  usbcli_printf("  Load current (average): %ld mA\r\n", (long)i_load_avg_mA);
  usbcli_printf("  Output current (average): %ld mA\r\n", (long)i_out_avg_mA);
  usbcli_printf("  Converter current command (I_conv) (average): %ld mA\r\n", (long)i_conv_avg_mA);

  usbcli_printf("  Power setpoint: %ld W\r\n", (long)g_latest.p_set);

  usbcli_printf("Telemetry and link status:\r\n");
  usbcli_printf("  T1 telemetry sequence number: %lu\r\n", (unsigned long)telemetry_seq);
  usbcli_printf("  ADC trigger frequency estimate: %lu Hz\r\n", (unsigned long)adc_seq_hz);
  usbcli_printf("  CAN bus activity up: %u\r\n", can_bus_up ? 1u : 0u);
  usbcli_printf("  Freshness CAN/UART-power/UART-energy: %u/%u/%u\r\n",
                control_status.can_fresh ? 1u : 0u,
                control_status.uart_power_fresh ? 1u : 0u,
                control_status.uart_energy_fresh ? 1u : 0u);
  if (can_command_fresh)
  {
    usbcli_printf("  CAN command timestamp: %lu ms\r\n", (unsigned long)can_command.can_cmd_timestamp);
    usbcli_printf("  CAN command power: %u W\r\n", (unsigned)can_command.can_power);
    if (can_command.can_energy_disabled)
    {
      usbcli_printf("  CAN command energy: disabled (raw %u J)\r\n",
                    (unsigned)can_command.can_energy);
    }
    else
    {
      usbcli_printf("  CAN command energy: %u J\r\n", (unsigned)can_command.can_energy);
    }
    usbcli_printf("  CAN command SWEN: %u\r\n", can_command.can_swen ? 1u : 0u);
  }
  else
  {
    usbcli_puts("no fresh CAN command received\r\n");
  }
  if (uart_command.power_w.present)
  {
    usbcli_printf("  UART power timestamp: %lu ms\r\n", (unsigned long)uart_command.power_w.timestamp_ms);
    usbcli_printf("  UART power: %u W\r\n", (unsigned)uart_command.power_w.value);
  }
  else
  {
    usbcli_printf("  UART power: no valid value\r\n");
  }
  if (uart_command.energy_j.present)
  {
    usbcli_printf("  UART energy timestamp: %lu ms\r\n", (unsigned long)uart_command.energy_j.timestamp_ms);
    usbcli_printf("  UART energy: %u J\r\n", (unsigned)uart_command.energy_j.value);
  }
  else
  {
    usbcli_printf("  UART energy: no valid value\r\n");
  }
  if (manual_command.power_w.present)
  {
    usbcli_printf("  Manual power timestamp: %lu ms\r\n", (unsigned long)manual_command.power_w.timestamp_ms);
    usbcli_printf("  Manual power: %d W\r\n", (int)manual_command.power_w.value);
  }
  else
  {
    usbcli_printf("  Manual power: no valid value\r\n");
  }
  if (manual_command.swen.present)
  {
    usbcli_printf("  Manual SWEN timestamp: %lu ms\r\n", (unsigned long)manual_command.swen.timestamp_ms);
    usbcli_printf("  Manual SWEN: %u\r\n", manual_command.swen.value ? 1u : 0u);
  }
  else
  {
    usbcli_printf("  Manual SWEN: no valid value\r\n");
  }
  if (pushbutton_command.swen.present)
  {
    usbcli_printf("  Pushbutton SWEN timestamp: %lu ms\r\n", (unsigned long)pushbutton_command.swen.timestamp_ms);
    usbcli_printf("  Pushbutton SWEN: %u\r\n", pushbutton_command.swen.value ? 1u : 0u);
  }
  else
  {
    usbcli_printf("  Pushbutton SWEN: no valid value\r\n");
  }
  usbcli_printf("  USB CLI bytes queued: %lu\r\n", (unsigned long)dbg_stats.cli_bytes_queued);
  usbcli_printf("  USB telemetry records queued/dropped: %lu/%lu\r\n",
                (unsigned long)dbg_stats.telemetry_records_queued,
                (unsigned long)dbg_stats.telemetry_records_dropped);
  usbcli_printf("  USB telemetry bytes queued: %lu\r\n", (unsigned long)dbg_stats.telemetry_bytes_queued);
  usbcli_printf("  USB disconnects: %lu\r\n", (unsigned long)dbg_stats.usb_disconnect_count);
  usbcli_printf("  USART1 telemetry records queued/dropped: %lu/%lu\r\n",
                (unsigned long)telemetry_uart_stats.records_queued,
                (unsigned long)telemetry_uart_stats.records_dropped);
  usbcli_printf("  USART1 telemetry bytes queued: %lu\r\n",
                (unsigned long)telemetry_uart_stats.bytes_queued);

  usbcli_printf("DMA interrupt timing:\r\n");
  usbcli_printf("  DMA1 channel 1 ISR cycles (last): %lu\r\n", (unsigned long)g_dma1_ch1_irq_cycles_last);
  usbcli_printf("  DMA1 channel 1 ISR cycles (max): %lu\r\n", (unsigned long)g_dma1_ch1_irq_cycles_max);
}

static void usbcli_cmd_telemetry(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: telemetry on|off|toggle\r\n");
    return;
  }

  if (usbcli_streq(argv[1], "on"))
  {
    g_usb_telemetry_enabled = true;
  }
  else if (usbcli_streq(argv[1], "off"))
  {
    g_usb_telemetry_enabled = false;
  }
  else if (usbcli_streq(argv[1], "toggle"))
  {
    g_usb_telemetry_enabled = !g_usb_telemetry_enabled;
  }
  else
  {
    usbcli_printf("usage: telemetry on|off|toggle\r\n");
    return;
  }

  usbcli_printf("usb_telemetry=%s; usart1_telemetry=on\r\n",
                g_usb_telemetry_enabled ? "on" : "off");
}

static void usbcli_cmd_pset(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: pset <-240..240>\r\n");
    return;
  }

  int32_t w = 0;
  if ((!usbcli_parse_i32(argv[1], &w)) || (w < MANUAL_POWER_MIN_W) || (w > MANUAL_POWER_MAX_W) ||
      !CommandInputs_SetManualPower(&g_manual_command.power_w, (int16_t)w, HAL_GetTick()))
  {
    usbcli_printf("usage: pset <-240..240>\r\n");
    return;
  }

  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_ctrl(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: ctrl <external|manual|measure|direct>\r\n");
    return;
  }

  control_mode_request_t mode;
  if (usbcli_streq(argv[1], "external"))
  {
    mode = CONTROL_MODE_EXTERNAL_SET;
  }
  else if (usbcli_streq(argv[1], "manual"))
  {
    mode = CONTROL_MODE_MANUAL_SET;
  }
  else if (usbcli_streq(argv[1], "measure"))
  {
    mode = CONTROL_MODE_MEASURE;
  }
  else if (usbcli_streq(argv[1], "direct"))
  {
    mode = CONTROL_MODE_DIRECT_GPIO;
  }
  else
  {
    usbcli_printf("usage: ctrl <external|manual|measure|direct>\r\n");
    return;
  }
  ScapIo_SetModeRequest(mode);
  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_swen(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: swen <0|1>\r\n");
    return;
  }

  uint32_t v = 0u;
  if ((!usbcli_parse_u32(argv[1], &v)) || (v > 1u))
  {
    usbcli_printf("usage: swen <0|1>\r\n");
    return;
  }
  if (!CommandInputs_SetSwen(&g_manual_command.swen, v != 0u, HAL_GetTick()))
  {
    usbcli_printf("err\r\n");
    return;
  }
  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_dir(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  usbcli_printf("unavailable: main mode control is staged off\r\n");
}

static void usbcli_cmd_mode(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  usbcli_printf("unavailable: main mode control is staged off\r\n");
}


static void usbcli_cmd_gpio(int argc, char **argv)
{
  GPIO_TypeDef *port = NULL;
  uint16_t pin = 0U;

  if (argc < 3)
  {
    usbcli_printf("usage: gpio write <PA10|...> <0|1> | gpio toggle <PA10|...>\r\n");
    return;
  }

  if (usbcli_streq(argv[1], "write"))
  {
    uint32_t v = 0U;
    if ((argc < 4) || (!usbcli_parse_gpio_pin(argv[2], &port, &pin)) || (!usbcli_parse_u32(argv[3], &v)) || (v > 1U))
    {
      usbcli_printf("usage: gpio write <PA10|...> <0|1>\r\n");
      return;
    }

    const bool stage_pin = ((port == GPIOB) && ((pin & (GPIO_DIR_Pin | GPIO_SWEN_Pin | GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin)) != 0u)) ||
                           ((port == GPIOA) && ((pin & GPIO_LED_Pin) != 0u));
    if (stage_pin && !ScapIo_IsDirectMode())
    {
      usbcli_printf("err: power-stage GPIO writes require ctrl direct\r\n");
      return;
    }

    HAL_GPIO_WritePin(port, pin, (v != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    usbcli_printf("ok\r\n");
    return;
  }

  if (usbcli_streq(argv[1], "toggle"))
  {
    if (!usbcli_parse_gpio_pin(argv[2], &port, &pin))
    {
      usbcli_printf("usage: gpio toggle <PA10|...>\r\n");
      return;
    }
    const bool stage_pin = ((port == GPIOB) && ((pin & (GPIO_DIR_Pin | GPIO_SWEN_Pin | GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin)) != 0u)) ||
                           ((port == GPIOA) && ((pin & GPIO_LED_Pin) != 0u));
    if (stage_pin && !ScapIo_IsDirectMode())
    {
      usbcli_printf("err: power-stage GPIO writes require ctrl direct\r\n");
      return;
    }
    HAL_GPIO_TogglePin(port, pin);
    usbcli_printf("ok\r\n");
    return;
  }

  usbcli_printf("usage: gpio write <PA10|...> <0|1> | gpio toggle <PA10|...>\r\n");
}

static void usbcli_cmd_dac(int argc, char **argv)
{
  uint32_t dac_n = 0U;
  uint32_t ch_n = 0U;
  uint32_t val = 0U;
  DAC_HandleTypeDef *hdac = NULL;
  uint32_t ch = 0U;

  if ((argc < 5) || (!usbcli_streq(argv[1], "set")))
  {
    usbcli_printf("usage: dac set <1|3> <1|2> <0..4095>\r\n");
    return;
  }

  if ((!usbcli_parse_u32(argv[2], &dac_n)) || (!usbcli_parse_u32(argv[3], &ch_n)) || (!usbcli_parse_u32(argv[4], &val)) || (val > 4095U))
  {
    usbcli_printf("usage: dac set <1|3> <1|2> <0..4095>\r\n");
    return;
  }

  if (dac_n == 1U)
  {
    hdac = &hdac1;
  }
  else if (dac_n == 3U)
  {
    hdac = &hdac3;
  }
  else
  {
    usbcli_printf("usage: dac set <1|3> <1|2> <0..4095>\r\n");
    return;
  }

  if (ch_n == 1U)
  {
    ch = DAC_CHANNEL_1;
  }
  else if (ch_n == 2U)
  {
    ch = DAC_CHANNEL_2;
  }
  else
  {
    usbcli_printf("usage: dac set <1|3> <1|2> <0..4095>\r\n");
    return;
  }

  if ((dac_n == 1U) && !ScapIo_IsDirectMode())
  {
    usbcli_printf("err: DAC1 writes require ctrl direct\r\n");
    return;
  }

  (void)HAL_DAC_SetValue(hdac, ch, DAC_ALIGN_12B_R, val);
  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_cal(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: cal set <NAME> <VALUE> | cal load | cal save\r\n");
    return;
  }

  if (usbcli_streq(argv[1], "save"))
  {
    if (AppConstants_SaveToNvm())
    {
      usbcli_printf("ok\r\n");
    }
    else
    {
      usbcli_printf("err: save failed\r\n");
    }
    return;
  }

  if (usbcli_streq(argv[1], "load"))
  {
    if (AppConstants_LoadFromNvm())
    {
      (void)HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC3_CH1_BOOT_U12);
      (void)HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC3_CH2_BOOT_U12);
      usbcli_printf("loaded:\r\n");
      usbcli_print_cal_current();
      usbcli_printf("ok\r\n");
    }
    else
    {
      usbcli_printf("err: no valid saved cal\r\n");
    }
    return;
  }

  if (usbcli_streq(argv[1], "set"))
  {
    if (argc < 4)
    {
      usbcli_printf("usage: cal set <NAME> <VALUE>\r\n");
      return;
    }

    const char *name = argv[2];
    const char *val_s = argv[3];

    float f = 0.0f;
    uint32_t u = 0u;

    if (usbcli_streq(name, "A_VBUS"))
    {
      if ((!usbcli_parse_f32(val_s, &f)) || (f == 0.0f))
      {
        usbcli_printf("err: A_VBUS must be finite and nonzero\r\n");
        return;
      }
      A_VBUS = f;
      AppConstants_RecalcDerived();
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_VBUS"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_VBUS = f;
      AppConstants_RecalcDerived();
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "A_ILOAD"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      A_ILOAD = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_ILOAD"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_ILOAD = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "MIDPOINT"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      MIDPOINT = f;
      usbcli_printf("ok\r\n");
      return;
    }

    if (usbcli_streq(name, "A_INP"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      A_INP = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_INP"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_INP = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "A_INN"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      A_INN = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_INN"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_INN = f;
      usbcli_printf("ok\r\n");
      return;
    }

    if (usbcli_streq(name, "A_VCAP"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      A_VCAP = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_VCAP"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_VCAP = f;
      usbcli_printf("ok\r\n");
      return;
    }

    if (usbcli_streq(name, "A_OP"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      A_OP = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_OP"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_OP = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "A_ON"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      A_ON = f;
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "B_ON"))
    {
      if (!usbcli_parse_f32(val_s, &f))
      {
        usbcli_printf("err: invalid float\r\n");
        return;
      }
      B_ON = f;
      usbcli_printf("ok\r\n");
      return;
    }

    if (usbcli_streq(name, "DAC3_CH1_BOOT_U12"))
    {
      if ((!usbcli_parse_u32(val_s, &u)) || (u > 4095u))
      {
        usbcli_printf("err: DAC3_CH1_BOOT_U12 must be 0..4095\r\n");
        return;
      }
      DAC3_CH1_BOOT_U12 = u;
      (void)HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC3_CH1_BOOT_U12);
      usbcli_printf("ok\r\n");
      return;
    }
    if (usbcli_streq(name, "DAC3_CH2_BOOT_U12"))
    {
      if ((!usbcli_parse_u32(val_s, &u)) || (u > 4095u))
      {
        usbcli_printf("err: DAC3_CH2_BOOT_U12 must be 0..4095\r\n");
        return;
      }
      DAC3_CH2_BOOT_U12 = u;
      (void)HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC3_CH2_BOOT_U12);
      usbcli_printf("ok\r\n");
      return;
    }

    usbcli_printf("err: unknown cal name '%s'\r\n", name);
    usbcli_printf("names: A_VBUS B_VBUS A_ILOAD B_ILOAD MIDPOINT A_INP B_INP A_INN B_INN A_VCAP B_VCAP A_OP B_OP A_ON B_ON DAC3_CH1_BOOT_U12 DAC3_CH2_BOOT_U12\r\n");
    return;
  }

  usbcli_printf("usage: cal set <NAME> <VALUE> | cal load | cal save\r\n");
}

static void usbcli_handle_line(struct embedded_cli *cli)
{
  char **argv = NULL;
  const int argc = embedded_cli_argc(cli, &argv);
  if ((argc <= 0) || (argv == NULL) || (argv[0] == NULL))
  {
    return;
  }

  if (usbcli_streq(argv[0], "help"))
  {
    usbcli_cmd_help();
  }
  else if (usbcli_streq(argv[0], "status"))
  {
    usbcli_cmd_status();
  }
  else if (usbcli_streq(argv[0], "telemetry"))
  {
    usbcli_cmd_telemetry(argc, argv);
  }
  else if (usbcli_streq(argv[0], "ctrl"))
  {
    usbcli_cmd_ctrl(argc, argv);
  }
  else if (usbcli_streq(argv[0], "pset"))
  {
    usbcli_cmd_pset(argc, argv);
  }
  else if (usbcli_streq(argv[0], "swen"))
  {
    usbcli_cmd_swen(argc, argv);
  }
  else if (usbcli_streq(argv[0], "mode"))
  {
    usbcli_cmd_mode(argc, argv);
  }
  else if (usbcli_streq(argv[0], "dir"))
  {
    usbcli_cmd_dir(argc, argv);
  }
  else if (usbcli_streq(argv[0], "gpio"))
  {
    usbcli_cmd_gpio(argc, argv);
  }
  else if (usbcli_streq(argv[0], "dac"))
  {
    usbcli_cmd_dac(argc, argv);
  }
  else if (usbcli_streq(argv[0], "cal"))
  {
    usbcli_cmd_cal(argc, argv);
  }
  else
  {
    usbcli_printf("unknown command: %s\r\n", argv[0]);
  }
}

void UsbCli_Init(void)
{
  usbcli_rx_stream = xStreamBufferCreateStatic(
      USBCLI_RX_BUF_SIZE,
      1U,
      usbcli_rx_stream_storage,
      &usbcli_rx_stream_struct);
  if (usbcli_rx_stream == NULL)
  {
    Error_Handler();
  }

  embedded_cli_init(&usbcli, "scv2> ", usbcli_put_char, NULL);
}

void UsbCli_OnRx(const uint8_t *buf, uint32_t len)
{
  uint8_t in_isr;
  BaseType_t hpw = pdFALSE;

  if ((usbcli_rx_stream == NULL) || (buf == NULL) || (len == 0U))
  {
    return;
  }

  in_isr = usbcli_in_isr();
  if (in_isr != 0U)
  {
    (void)xStreamBufferSendFromISR(usbcli_rx_stream, buf, (size_t)len, &hpw);
    if (hpw != pdFALSE)
    {
      portYIELD_FROM_ISR(hpw);
    }
  }
  else
  {
    (void)xStreamBufferSend(usbcli_rx_stream, buf, (size_t)len, 0U);
  }
}

void UsbCli_Task(void const *argument)
{
  (void)argument;

  usbcli_printf("\r\nCLI ready. Type 'help'.\r\n");
  embedded_cli_prompt(&usbcli);

  for (;;)
  {
    uint8_t ch = 0U;
    const size_t n = xStreamBufferReceive(usbcli_rx_stream, &ch, 1U, portMAX_DELAY);
    if (n != 1U)
    {
      continue;
    }

    if (embedded_cli_insert_char(&usbcli, (char)ch))
    {
      usbcli_handle_line(&usbcli);
      embedded_cli_prompt(&usbcli);
    }
  }
}

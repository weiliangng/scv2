#include "usb_cli.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "embedded_cli.h"
#include "shared_state.h"
#include "stream_buffer.h"
#include "task.h"
#include "task_dbg_over_usb.h"

#include "main.h"

#include <ctype.h>
#include <errno.h>
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

static void usbcli_cmd_help(void)
{
  usbcli_printf(
      "Commands:\r\n"
      "  help\r\n"
      "  status\r\n"
      "  telemetry on|off|toggle\r\n"
      "  control auto|manual\r\n"
      "  gpio write <PA10|PB1|...> <0|1>\r\n"
      "  gpio toggle <PA10|PB1|...>\r\n"
      "  dac set <1|3> <1|2> <0..4095>\r\n");
}

static void usbcli_cmd_status(void)
{
  usbcli_printf("telemetry=%s control=%s\r\n",
                g_telemetry_enabled ? "on" : "off",
                g_control_automatic ? "auto" : "manual");

  const int32_t v_bus_mV = (int32_t)(g_latest.v_bus * 1000.0f);
  const int32_t v_cap_mV = (int32_t)(g_latest.v_cap * 1000.0f);
  const int32_t i_load_mA = (int32_t)(g_latest.i_load * 1000.0f);
  const int32_t i_out_mA = (int32_t)(g_latest.i_out * 1000.0f);
  const int32_t i_conv_mA = (int32_t)(g_latest.i_conv * 1000.0f);

  usbcli_printf("v_bus_mV=%ld v_cap_mV=%ld i_load_mA=%ld i_out_mA=%ld i_conv_mA=%ld\r\n",
                (long)v_bus_mV,
                (long)v_cap_mV,
                (long)i_load_mA,
                (long)i_out_mA,
                (long)i_conv_mA);
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
    g_telemetry_enabled = true;
  }
  else if (usbcli_streq(argv[1], "off"))
  {
    g_telemetry_enabled = false;
  }
  else if (usbcli_streq(argv[1], "toggle"))
  {
    g_telemetry_enabled = !g_telemetry_enabled;
  }
  else
  {
    usbcli_printf("usage: telemetry on|off|toggle\r\n");
    return;
  }

  usbcli_printf("telemetry=%s\r\n", g_telemetry_enabled ? "on" : "off");
}

static void usbcli_cmd_control(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: control auto|manual\r\n");
    return;
  }

  if (usbcli_streq(argv[1], "auto"))
  {
    g_control_automatic = true;
  }
  else if (usbcli_streq(argv[1], "manual"))
  {
    g_control_automatic = false;
  }
  else
  {
    usbcli_printf("usage: control auto|manual\r\n");
    return;
  }

  usbcli_printf("control=%s\r\n", g_control_automatic ? "auto" : "manual");
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

    if (g_control_automatic)
    {
      usbcli_printf("note: control=auto may override some outputs\r\n");
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
    if (g_control_automatic)
    {
      usbcli_printf("note: control=auto may override some outputs\r\n");
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

  if ((dac_n == 1U) && g_control_automatic)
  {
    usbcli_printf("err: control=auto updates DAC1 in ISR; run 'control manual' first\r\n");
    return;
  }

  (void)HAL_DAC_SetValue(hdac, ch, DAC_ALIGN_12B_R, val);
  usbcli_printf("ok\r\n");
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
  else if (usbcli_streq(argv[0], "control"))
  {
    usbcli_cmd_control(argc, argv);
  }
  else if (usbcli_streq(argv[0], "gpio"))
  {
    usbcli_cmd_gpio(argc, argv);
  }
  else if (usbcli_streq(argv[0], "dac"))
  {
    usbcli_cmd_dac(argc, argv);
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

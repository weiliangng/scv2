#include "usb_cli.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "embedded_cli.h"
#include "shared_state.h"
#include "stream_buffer.h"
#include "task.h"
#include "task_dbg_over_usb.h"

#include "main.h"
#include "app_constants.h"
#include "scap_io_owner.h"

#include <ctype.h>
#include <errno.h>
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
      "  telemetry on|off|toggle\r\n"
      "  ctrl <algo|can|manual>\r\n"
      "  pset <0..240>\r\n"
      "  swen <0|1>\r\n"
      "  mode <ccm|hcm|dcm|burst>\r\n"
      "  dir <0|1>\r\n"
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
  const char *src = "unknown";
  switch (g_ctrl_src)
  {
  case SRC_MANUAL:
    src = "manual";
    break;
  case SRC_CAN:
    src = "can";
    break;
  case SRC_ALGO:
  default:
    src = "algo";
    break;
  }

  usbcli_printf("telemetry=%s ctrl=%s swen=%u mode=%u%u dir=%u\r\n",
                g_telemetry_enabled ? "on" : "off",
                src,
                (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_SWEN_Pin) != GPIO_PIN_RESET) ? 1u : 0u),
                (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_MODEMSB_Pin) != GPIO_PIN_RESET) ? 1u : 0u),
                (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_MODELSB_Pin) != GPIO_PIN_RESET) ? 1u : 0u),
                (unsigned)((HAL_GPIO_ReadPin(GPIOB, GPIO_DIR_Pin) != GPIO_PIN_RESET) ? 1u : 0u));

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

  const float p_set = g_latest.p_set;
  const int32_t p_set_w = (int32_t)(p_set + ((p_set >= 0.0f) ? 0.5f : -0.5f));
  usbcli_printf("p_set_W=%ld\r\n", (long)p_set_w);
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

static int usbcli_parse_mode(const char *s, scap_mode_t *out)
{
  if ((s == NULL) || (out == NULL))
  {
    return 0;
  }

  if (usbcli_streq(s, "ccm"))
  {
    *out = SCAP_MODE_CCM;
    return 1;
  }
  if (usbcli_streq(s, "hcm"))
  {
    *out = SCAP_MODE_HCM;
    return 1;
  }
  if (usbcli_streq(s, "dcm"))
  {
    *out = SCAP_MODE_DCM;
    return 1;
  }
  if (usbcli_streq(s, "burst"))
  {
    *out = SCAP_MODE_BURST;
    return 1;
  }

  return 0;
}

static void usbcli_cmd_pset(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: pset <0..240>\r\n");
    return;
  }

  uint32_t w = 0u;
  if ((!usbcli_parse_u32(argv[1], &w)) || (w > 240u))
  {
    usbcli_printf("usage: pset <0..240>\r\n");
    return;
  }

  g_manual_p_set_w = (float)w;
  g_ctrl_src = SRC_MANUAL;
  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_ctrl(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: ctrl <algo|can|manual>\r\n");
    return;
  }

  if (usbcli_streq(argv[1], "algo"))
  {
    g_ctrl_src = SRC_ALGO;
  }
  else if (usbcli_streq(argv[1], "can"))
  {
    g_ctrl_src = SRC_CAN;
  }
  else if (usbcli_streq(argv[1], "manual"))
  {
    g_ctrl_src = SRC_MANUAL;
  }
  else
  {
    usbcli_printf("usage: ctrl <algo|can|manual>\r\n");
    return;
  }

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
  ScapIo_ManualSetSwen(v != 0u);
  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_dir(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: dir <0|1>\r\n");
    return;
  }

  uint32_t v = 0u;
  if ((!usbcli_parse_u32(argv[1], &v)) || (v > 1u))
  {
    usbcli_printf("usage: dir <0|1>\r\n");
    return;
  }
  ScapIo_ManualSetDir(v != 0u);
  usbcli_printf("ok\r\n");
}

static void usbcli_cmd_mode(int argc, char **argv)
{
  if (argc < 2)
  {
    usbcli_printf("usage: mode <ccm|hcm|dcm|burst>\r\n");
    return;
  }

  scap_mode_t m;
  if (!usbcli_parse_mode(argv[1], &m))
  {
    usbcli_printf("usage: mode <ccm|hcm|dcm|burst>\r\n");
    return;
  }
  ScapIo_ManualSetMode(m);
  usbcli_printf("ok\r\n");
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

    if ((port == GPIOB) && ((pin & (GPIO_DIR_Pin | GPIO_SWEN_Pin | GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin)) != 0u))
    {
      usbcli_printf("note: IO owner may override PB1/PB4/PB5/PB6; use 'ctrl/swen/mode/dir'\r\n");
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
    if ((port == GPIOB) && ((pin & (GPIO_DIR_Pin | GPIO_SWEN_Pin | GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin)) != 0u))
    {
      usbcli_printf("note: IO owner may override PB1/PB4/PB5/PB6; use 'ctrl/swen/mode/dir'\r\n");
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

  if ((dac_n == 1U) && (g_ctrl_src != SRC_MANUAL))
  {
    usbcli_printf("err: DAC1 updated in ISR (src=algo/can); run 'ctrl manual' first\r\n");
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

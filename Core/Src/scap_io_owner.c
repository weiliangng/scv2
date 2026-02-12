#include "scap_io_owner.h"

#include "app_constants.h"
#include "main.h"

#define SCAP_IO_BOOT_SETTLE_MS 20u
#define SCAP_IO_DEFAULT_SWEN_PULSE_MS 5u

#define PB_DIR GPIO_DIR_Pin
#define PB_SWEN GPIO_SWEN_Pin
#define PB_MODE_MSB GPIO_MODEMSB_Pin
#define PB_MODE_LSB GPIO_MODELSB_Pin

#define PB_MODE_MASK (PB_MODE_MSB | PB_MODE_LSB)
#define PB_MD_DIR_MASK (PB_MODE_MASK | PB_DIR)

static inline uint16_t pb_pack(scap_mode_t mode, bool dir_high, bool swen_high)
{
  uint16_t v = 0u;

  if ((((uint8_t)mode) & 0x2u) != 0u)
  {
    v |= PB_MODE_MSB;
  }
  if ((((uint8_t)mode) & 0x1u) != 0u)
  {
    v |= PB_MODE_LSB;
  }

  if (dir_high)
  {
    v |= PB_DIR;
  }
  if (swen_high)
  {
    v |= PB_SWEN;
  }

  return v;
}

static inline void gpio_write_masked_bsrr(GPIO_TypeDef *port, uint16_t affect_mask, uint16_t desired)
{
  const uint16_t set_mask = (uint16_t)(desired & affect_mask);
  const uint16_t reset_mask = (uint16_t)((~desired) & affect_mask);
  port->BSRR = ((uint32_t)reset_mask << 16) | (uint32_t)set_mask;
}

static volatile uint16_t g_pb_manual;
static volatile uint16_t g_pb_can;
static volatile uint16_t g_pb_algo;

static volatile uint8_t g_fault_latched;
static volatile uint16_t g_swen_pulse_req_ms;

static uint16_t g_boot_settle_ms;
static uint16_t g_swen_pulse_ms;
static volatile uint16_t g_last_applied_pb;
static uint16_t g_last_ext_md_dir;

void ScapIo_Init(void)
{
  g_pb_manual = pb_pack(UNIDIRECTIONAL, true, false);
  g_pb_can = pb_pack(UNIDIRECTIONAL, true, false);
  g_pb_algo = pb_pack(UNIDIRECTIONAL, true, false);

  g_fault_latched = 0u;
  g_swen_pulse_req_ms = 0u;

  g_boot_settle_ms = SCAP_IO_BOOT_SETTLE_MS;
  g_swen_pulse_ms = 0u;
  g_last_applied_pb = 0xFFFFu;
  g_last_ext_md_dir = 0xFFFFu;
}

void ScapIo_SetFaultLatched(bool fault_latched)
{
  g_fault_latched = fault_latched ? 1u : 0u;
}

void ScapIo_RequestSwenPulseMs(uint16_t pulse_ms)
{
  if (pulse_ms == 0u)
  {
    return;
  }
  if (pulse_ms > g_swen_pulse_req_ms)
  {
    g_swen_pulse_req_ms = pulse_ms;
  }
}

void ScapIo_ManualSetMode(scap_mode_t mode)
{
  uint16_t v = g_pb_manual;
  v &= (uint16_t)~PB_MODE_MASK;

  if ((((uint8_t)mode) & 0x2u) != 0u)
  {
    v |= PB_MODE_MSB;
  }
  if ((((uint8_t)mode) & 0x1u) != 0u)
  {
    v |= PB_MODE_LSB;
  }
  g_pb_manual = v;
  g_ctrl_src = SRC_MANUAL;
}

void ScapIo_ManualSetDir(bool dir_high)
{
  uint16_t v = g_pb_manual;
  if (dir_high)
  {
    v |= PB_DIR;
  }
  else
  {
    v &= (uint16_t)~PB_DIR;
  }
  g_pb_manual = v;
  g_ctrl_src = SRC_MANUAL;
}

void ScapIo_ManualSetSwen(bool swen_high)
{
  uint16_t v = g_pb_manual;
  if (swen_high)
  {
    v |= PB_SWEN;
  }
  else
  {
    v &= (uint16_t)~PB_SWEN;
  }
  g_pb_manual = v;
  g_ctrl_src = SRC_MANUAL;
}

void ScapIo_CanRxUpdateIsr(bool swen_high, bool dir_high, bool can_manual)
{
  const scap_mode_t mode = can_manual ? UNIDIRECTIONAL : BIDIRECTIONAL;
  g_pb_can = pb_pack(mode, dir_high, swen_high);

  /*
   * CAN policy bit selects controller ownership:
   * - 0: UART auto w/ CAN fallback (stay in ALGO)
   * - 1: CAN manual control (CAN owns IO/DIR)
   *
   * Manual always has priority and cannot be preempted by CAN.
   */
  if (g_ctrl_src != SRC_MANUAL)
  {
    g_ctrl_src = can_manual ? SRC_CAN : SRC_ALGO;
  }
}

void ScapIo_Tick1kHz(void)
{
  uint16_t pb = 0u;
  ctrl_src_t src = g_ctrl_src;
  const uint32_t now_ms = HAL_GetTick();
  bool can_cmd_connected = false;

  if (g_can_rx.can_rx_count != 0u)
  {
    const uint32_t last_cmd_ms = g_can_rx.last_cmd_tick;
    can_cmd_connected = ((uint32_t)(now_ms - last_cmd_ms) <= CAN_CMD_TIMEOUT_MS);
  }

  if (src == SRC_CAN)
  {
    if ((!can_cmd_connected) || (g_can_rx.mode == false))
    {
      g_ctrl_src = SRC_ALGO;
      src = SRC_ALGO;
    }
  }

  switch (src)
  {
  case SRC_MANUAL:
    pb = g_pb_manual;
    break;
  case SRC_CAN:
    pb = g_pb_can;
    break;
  case SRC_ALGO:
  default:
    pb = g_pb_algo;
    break;
  }

  if ((src == SRC_MANUAL) || (src == SRC_CAN))
  {
    const uint16_t md_dir = (uint16_t)(pb & PB_MD_DIR_MASK);
    if (md_dir != g_last_ext_md_dir)
    {
      ScapIo_RequestSwenPulseMs(SCAP_IO_DEFAULT_SWEN_PULSE_MS);
      g_last_ext_md_dir = md_dir;
    }
  }

  if ((src != SRC_MANUAL) && can_cmd_connected)
  {
    if (g_can_rx.en)
    {
      pb |= PB_SWEN;
    }
    else
    {
      pb &= (uint16_t)~PB_SWEN;
    }
  }

  const uint16_t req = g_swen_pulse_req_ms;
  if (req != 0u)
  {
    g_swen_pulse_req_ms = 0u;
    if (req > g_swen_pulse_ms)
    {
      g_swen_pulse_ms = req;
    }
  }

  if (g_boot_settle_ms != 0u)
  {
    g_boot_settle_ms--;
  }
  if (g_swen_pulse_ms != 0u)
  {
    g_swen_pulse_ms--;
  }

  if ((g_fault_latched != 0u) || (g_boot_settle_ms != 0u) || (g_swen_pulse_ms != 0u))
  {
    pb &= (uint16_t)~PB_SWEN;
  }

  uint16_t affect = (uint16_t)(PB_SWEN | PB_MODE_MASK);
  if (src != SRC_ALGO)
  {
    affect |= PB_DIR;
  }

  if ((pb & affect) != (g_last_applied_pb & affect))
  {
    gpio_write_masked_bsrr(GPIOB, affect, pb);
    g_last_applied_pb = pb;
  }
}

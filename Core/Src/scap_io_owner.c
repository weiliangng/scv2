#include "scap_io_owner.h"

#include "main.h"

#include "app_constants.h"
#include "can_protocol.h"
#include "command_inputs.h"
#include "shared_state.h"

static control_fast_command_t s_fast_commands[2];
static volatile uint8_t s_fast_command_index;
static volatile control_mode_request_t s_mode_request = CONTROL_MODE_EXTERNAL_SET;
static volatile uint8_t s_fault_latched;
static volatile uint8_t s_fault_bits;
static volatile uint8_t s_fault_clear_request;
static volatile uint8_t s_uvlo_lockout;
static volatile uint8_t s_first_button_consumed;
static control_status_t s_status;
static control_decision_t s_last_decision = CONTROL_DECISION_IDLE;

static void gpio_write_masked_bsrr(GPIO_TypeDef *port, uint16_t affect_mask, uint16_t desired)
{
  const uint16_t set_mask = (uint16_t)(desired & affect_mask);
  const uint16_t reset_mask = (uint16_t)((~desired) & affect_mask);
  port->BSRR = ((uint32_t)reset_mask << 16) | (uint32_t)set_mask;
}

static void publish_fast_command(const control_fast_command_t *command)
{
  const uint8_t next = (uint8_t)(s_fast_command_index ^ 1u);
  s_fast_commands[next] = *command;
  __DMB();
  s_fast_command_index = next;
}

void ScapIo_Init(void)
{
  const control_fast_command_t disabled = {
      .decision = CONTROL_DECISION_IDLE,
      .p_set_w = 0.0f,
      .energy_j = 0u,
      .energy_valid = false,
      .swen_request = false,
      .energy_swen_allowed = false,
  };
  s_fast_commands[0] = disabled;
  s_fast_commands[1] = disabled;
  s_fast_command_index = 0u;
  s_mode_request = CONTROL_MODE_EXTERNAL_SET;
  s_fault_latched = 0u;
  s_fault_bits = 0u;
  s_fault_clear_request = 0u;
  s_uvlo_lockout = 0u;
  s_first_button_consumed = 0u;
  s_status.mode_request = CONTROL_MODE_EXTERNAL_SET;
  s_status.decision = CONTROL_DECISION_IDLE;
  s_last_decision = CONTROL_DECISION_IDLE;

  /* HCM is the normal algorithm mode.  Direct mode leaves these pins alone. */
  gpio_write_masked_bsrr(GPIOB, GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin,
                         GPIO_MODELSB_Pin);
  gpio_write_masked_bsrr(GPIOB, GPIO_DIR_Pin | GPIO_SWEN_Pin, 0u);
  gpio_write_masked_bsrr(GPIO_LED_GPIO_Port, GPIO_LED_Pin, 0u);
}

void ScapIo_SetModeRequest(control_mode_request_t mode)
{
  if (mode > CONTROL_MODE_DIRECT_GPIO)
  {
    return;
  }
  s_mode_request = mode;
  CommandInputs_ResetPushbuttonForModeChange(HAL_GetTick());
}

control_mode_request_t ScapIo_GetModeRequest(void)
{
  return s_mode_request;
}

bool ScapIo_IsDirectMode(void)
{
  return s_mode_request == CONTROL_MODE_DIRECT_GPIO;
}

void ScapIo_ReadStatus(control_status_t *status)
{
  if (status == NULL)
  {
    return;
  }
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *status = s_status;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

void ScapIo_ReadFastCommand(control_fast_command_t *command)
{
  if (command == NULL)
  {
    return;
  }
  const uint8_t index = s_fast_command_index;
  __DMB();
  *command = s_fast_commands[index];
}

const char *ScapIo_DecisionName(control_decision_t decision)
{
  switch (decision)
  {
  case CONTROL_DECISION_DISABLE: return "fault";
  case CONTROL_DECISION_IDLE: return "idle";
  case CONTROL_DECISION_NO_SOURCE: return "no-source";
  case CONTROL_DECISION_MANUAL_SET_ALGO: return "manual";
  case CONTROL_DECISION_CAN_ALGO: return "can";
  case CONTROL_DECISION_UART_ALGO: return "uart";
  case CONTROL_DECISION_MEASURE: return "measure";
  case CONTROL_DECISION_DIRECT_GPIO: return "direct";
  default: return "unknown";
  }
}

bool ScapIo_IsFaultLatched(void)
{
  return s_fault_latched != 0u;
}

bool ScapIo_IsUvloLockout(void)
{
  return s_uvlo_lockout != 0u;
}

uint8_t ScapIo_FastUpdateSafety(uint8_t fault_bits, float v_bus)
{
  if (fault_bits != 0u)
  {
    s_fault_bits |= fault_bits;
    s_fault_latched = 1u;
    s_fault_clear_request = 0u;
  }
  else if ((s_fault_latched != 0u) && (s_fault_clear_request != 0u))
  {
    s_fault_bits = 0u;
    s_fault_latched = 0u;
    s_fault_clear_request = 0u;
  }

  if (s_uvlo_lockout != 0u)
  {
    if (v_bus >= SCAP_UVLO_EXIT_V)
    {
      s_uvlo_lockout = 0u;
    }
  }
  else if (v_bus <= SCAP_UVLO_ENTER_V)
  {
    s_uvlo_lockout = 1u;
  }

  uint8_t state = 0u;
  if (s_fault_latched != 0u)
  {
    state |= SCAP_FAST_SAFETY_FAULT_LATCHED;
  }
  if (s_uvlo_lockout != 0u)
  {
    state |= SCAP_FAST_SAFETY_UVLO_LOCKOUT;
  }
  return state;
}

void ScapIo_HandlePushbuttonFromIsr(uint32_t timestamp_ms)
{
  if (s_first_button_consumed == 0u)
  {
    s_first_button_consumed = 1u;
    s_mode_request = CONTROL_MODE_MANUAL_SET;
    CommandInputs_ResetPushbuttonForModeChange(timestamp_ms);
    return;
  }

  if (s_mode_request == CONTROL_MODE_MANUAL_SET)
  {
    CommandInputs_TogglePushbuttonSwenFromIsr(timestamp_ms);
  }
}

void ScapIo_Resolve1kHz(void)
{
  can_command_state_t can_command;
  uart_command_state_t uart_command;
  manual_command_state_t manual_command;
  pushbutton_command_state_t pushbutton_command;
  control_fast_command_t command = {
      .decision = CONTROL_DECISION_IDLE,
      .p_set_w = 0.0f,
      .energy_j = 0u,
      .energy_valid = false,
      .swen_request = false,
      .energy_swen_allowed = false,
  };
  const uint32_t now_ms = HAL_GetTick();
  const control_mode_request_t mode = s_mode_request;

  CanProtocol_ReadCommandSnapshot(&can_command);
  CommandInputs_ReadUartSnapshot(&uart_command);
  CommandInputs_ReadManualSnapshot(&manual_command);
  CommandInputs_ReadPushbuttonSnapshot(&pushbutton_command);

  const bool can_fresh = CommandInputs_IsFresh(can_command.can_cmd_timestamp != 0u ? can_command.can_cmd_timestamp : 0u,
                                                can_command.can_cmd_timestamp != 0u,
                                                now_ms, SCAP_COMMAND_FRESH_TIMEOUT_MS);
  const bool uart_power_fresh = CommandInputs_IsFresh(uart_command.power_w.timestamp_ms,
                                                       uart_command.power_w.present,
                                                       now_ms, SCAP_COMMAND_FRESH_TIMEOUT_MS);
  const bool uart_energy_fresh = CommandInputs_IsFresh(uart_command.energy_j.timestamp_ms,
                                                        uart_command.energy_j.present,
                                                        now_ms, SCAP_COMMAND_FRESH_TIMEOUT_MS);
  const bool fault_latched = ScapIo_IsFaultLatched();
  const bool uvlo_lockout = ScapIo_IsUvloLockout();
  static uint16_t healthy_ms;
  static uint8_t charge_vcap_lockout;
  static uint8_t discharge_vcap_lockout;

  if (fault_latched)
  {
    if (g_is_safe)
    {
      if (healthy_ms < SCAP_FAULT_RECOVERY_MS)
      {
        healthy_ms++;
      }
    }
    else
    {
      healthy_ms = 0u;
    }
    if (healthy_ms >= SCAP_FAULT_RECOVERY_MS)
    {
      s_fault_clear_request = 1u;
    }
    command.decision = CONTROL_DECISION_DISABLE;
  }
  else if (uvlo_lockout)
  {
    healthy_ms = 0u;
    command.decision = CONTROL_DECISION_IDLE;
  }
  else
  {
    healthy_ms = 0u;
    switch (mode)
    {
    case CONTROL_MODE_DIRECT_GPIO:
      command.decision = CONTROL_DECISION_DIRECT_GPIO;
      break;
    case CONTROL_MODE_MEASURE:
      command.decision = CONTROL_DECISION_MEASURE;
      break;
    case CONTROL_MODE_MANUAL_SET:
      command.decision = CONTROL_DECISION_MANUAL_SET_ALGO;
      command.p_set_w = manual_command.power_w.present ? (float)manual_command.power_w.value : SCAP_MANUAL_DEFAULT_POWER_W;
      if (pushbutton_command.swen.present &&
          (!manual_command.swen.present ||
           ((uint32_t)(pushbutton_command.swen.timestamp_ms - manual_command.swen.timestamp_ms) < 0x80000000u)))
      {
        command.swen_request = pushbutton_command.swen.value;
      }
      else
      {
        command.swen_request = manual_command.swen.present && manual_command.swen.value;
      }
      break;
    case CONTROL_MODE_EXTERNAL_SET:
    default:
      if (uart_power_fresh && uart_energy_fresh)
      {
        command.decision = CONTROL_DECISION_UART_ALGO;
        command.p_set_w = (float)uart_command.power_w.value;
        command.energy_j = uart_command.energy_j.value;
        command.energy_valid = true;
        command.swen_request = true;
      }
      else if (can_fresh)
      {
        command.decision = CONTROL_DECISION_CAN_ALGO;
        command.p_set_w = (float)can_command.can_power;
        command.energy_j = can_command.can_energy;
        command.energy_valid = !can_command.can_energy_disabled;
        command.swen_request = can_command.can_swen;
      }
      else
      {
        command.decision = CONTROL_DECISION_NO_SOURCE;
      }
      break;
    }
  }

  if (((command.decision == CONTROL_DECISION_CAN_ALGO) ||
       (command.decision == CONTROL_DECISION_UART_ALGO)) && command.swen_request)
  {
    if (!command.energy_valid)
    {
      /* CAN's disabled-energy sentinel removes the buffer-energy gate. */
      command.energy_swen_allowed = true;
    }
    else
    {
      const float v_cap = g_latest.v_cap;
      const float i_conv = g_latest.i_conv;

      //add hysteresis around cap UVLO/cap burst mode emulation
      if (charge_vcap_lockout != 0u)
      {
        if (v_cap <= SCAP_CHARGE_LOCKOUT_RESUME_V)
        {
          charge_vcap_lockout = 0u;
        }
      }
      else if (v_cap >= SCAP_VCAP_MAX_V)
      {
        charge_vcap_lockout = 1u;
      }

      if (discharge_vcap_lockout != 0u)
      {
        if (v_cap >= SCAP_DISCHARGE_LOCKOUT_RESUME_V)
        {
          discharge_vcap_lockout = 0u;
        }
      }
      else if (v_cap <= SCAP_VCAP_LOW_V)
      {
        discharge_vcap_lockout = 1u;
      }

      bool can_charge = ((charge_vcap_lockout == 0u) && (i_conv > 0.0f));
      bool can_discharge = ((discharge_vcap_lockout == 0u) && (i_conv < 0.0f));

      command.energy_swen_allowed = can_charge || can_discharge;

      if (can_charge)
      {
        if (command.energy_j > SCAP_ENERGY_CHARGE_J) command.p_set_w = command.p_set_w + SCAP_POWER_HYSTERESIS;
        else command.p_set_w = command.p_set_w - 2*SCAP_POWER_HYSTERESIS;
      }
      else if (can_discharge)
      {
        if (command.energy_j < SCAP_ENERGY_DISCHARGE_J) command.p_set_w = command.p_set_w - SCAP_POWER_HYSTERESIS;
        else command.p_set_w = command.p_set_w + 2*SCAP_POWER_HYSTERESIS;
      }
    }
  }

  if (command.decision != s_last_decision)
  {
    CommandInputs_ResetPushbuttonForModeChange(now_ms);
    s_last_decision = command.decision;
  }

  if (command.decision != CONTROL_DECISION_DIRECT_GPIO)
  {
    gpio_write_masked_bsrr(GPIOB, GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin, GPIO_MODELSB_Pin);
  }

  //manage capacitor derating here (might need to derate capacitor voltage as it fails)



  g_latest.p_set = command.p_set_w;
  publish_fast_command(&command);

  s_status.mode_request = mode;
  s_status.decision = command.decision;
  s_status.can_fresh = can_fresh;
  s_status.uart_power_fresh = uart_power_fresh;
  s_status.uart_energy_fresh = uart_energy_fresh;
  s_status.uvlo_lockout = uvlo_lockout;
  s_status.fault_latched = fault_latched;
  s_status.fault_bits = s_fault_bits;
  s_status.fault_healthy_ms = healthy_ms;
}

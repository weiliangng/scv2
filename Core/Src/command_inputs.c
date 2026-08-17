#include "command_inputs.h"

#include "main.h"

volatile manual_command_state_t g_manual_command;
volatile uart_command_state_t g_uart_command;
volatile pushbutton_command_state_t g_pushbutton_command;

static bool command_inputs_timestamp_is_valid(uint32_t timestamp_ms)
{
  return timestamp_ms != 0u;
}

bool CommandInputs_SetPower(volatile command_u16_field_t *field, uint16_t power_w, uint32_t timestamp_ms)
{
  if ((field == NULL) || !command_inputs_timestamp_is_valid(timestamp_ms) ||
      (power_w < COMMAND_POWER_MIN_W) || (power_w > COMMAND_POWER_MAX_W))
  {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  field->value = power_w;
  field->present = true;
  field->timestamp_ms = timestamp_ms;
  if (primask == 0u)
  {
    __enable_irq();
  }
  return true;
}

bool CommandInputs_SetEnergy(volatile command_u16_field_t *field, uint16_t energy_j, uint32_t timestamp_ms)
{
  if ((field == NULL) || !command_inputs_timestamp_is_valid(timestamp_ms) ||
      (energy_j > COMMAND_ENERGY_MAX_J))
  {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  field->value = energy_j;
  field->present = true;
  field->timestamp_ms = timestamp_ms;
  if (primask == 0u)
  {
    __enable_irq();
  }
  return true;
}

bool CommandInputs_SetSwen(volatile command_bool_field_t *field, bool swen, uint32_t timestamp_ms)
{
  if ((field == NULL) || !command_inputs_timestamp_is_valid(timestamp_ms))
  {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  field->value = swen;
  field->present = true;
  field->timestamp_ms = timestamp_ms;
  if (primask == 0u)
  {
    __enable_irq();
  }
  return true;
}

void CommandInputs_TogglePushbuttonSwenFromIsr(uint32_t timestamp_ms)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool swen = !g_pushbutton_command.swen.present || !g_pushbutton_command.swen.value;
  if (command_inputs_timestamp_is_valid(timestamp_ms))
  {
    g_pushbutton_command.swen.value = swen;
    g_pushbutton_command.swen.present = true;
    g_pushbutton_command.swen.timestamp_ms = timestamp_ms;
  }
  if (primask == 0u)
  {
    __enable_irq();
  }
}

void CommandInputs_ResetPushbuttonForModeChange(uint32_t timestamp_ms)
{
  (void)CommandInputs_SetSwen(&g_pushbutton_command.swen, false, timestamp_ms);
}

void CommandInputs_ReadManualSnapshot(manual_command_state_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot->power_w.value = g_manual_command.power_w.value;
  snapshot->power_w.timestamp_ms = g_manual_command.power_w.timestamp_ms;
  snapshot->power_w.present = g_manual_command.power_w.present;
  snapshot->swen.value = g_manual_command.swen.value;
  snapshot->swen.timestamp_ms = g_manual_command.swen.timestamp_ms;
  snapshot->swen.present = g_manual_command.swen.present;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

void CommandInputs_ReadUartSnapshot(uart_command_state_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot->power_w.value = g_uart_command.power_w.value;
  snapshot->power_w.timestamp_ms = g_uart_command.power_w.timestamp_ms;
  snapshot->power_w.present = g_uart_command.power_w.present;
  snapshot->energy_j.value = g_uart_command.energy_j.value;
  snapshot->energy_j.timestamp_ms = g_uart_command.energy_j.timestamp_ms;
  snapshot->energy_j.present = g_uart_command.energy_j.present;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

void CommandInputs_ReadPushbuttonSnapshot(pushbutton_command_state_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot->swen.value = g_pushbutton_command.swen.value;
  snapshot->swen.timestamp_ms = g_pushbutton_command.swen.timestamp_ms;
  snapshot->swen.present = g_pushbutton_command.swen.present;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

bool CommandInputs_IsFresh(uint32_t timestamp_ms, bool present, uint32_t now_ms, uint32_t timeout_ms)
{
  return present && (timestamp_ms != 0u) && ((uint32_t)(now_ms - timestamp_ms) <= timeout_ms);
}

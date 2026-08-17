#ifndef COMMAND_INPUTS_H
#define COMMAND_INPUTS_H

#include <stdbool.h>
#include <stdint.h>

#define COMMAND_POWER_MIN_W 50u
#define COMMAND_POWER_MAX_W 120u
#define COMMAND_ENERGY_MIN_J 0u
#define COMMAND_ENERGY_MAX_J 60u

#define MANUAL_POWER_MIN_W (-240)
#define MANUAL_POWER_MAX_W 240

typedef struct
{
  uint16_t value;
  uint32_t timestamp_ms;
  bool present;
} command_u16_field_t;

typedef struct
{
  int16_t value;
  uint32_t timestamp_ms;
  bool present;
} command_i16_field_t;

typedef struct
{
  bool value;
  uint32_t timestamp_ms;
  bool present;
} command_bool_field_t;

typedef struct
{
  command_i16_field_t power_w;
  command_bool_field_t swen;
} manual_command_state_t;

typedef struct
{
  command_u16_field_t power_w;
  command_u16_field_t energy_j;
} uart_command_state_t;

typedef struct
{
  command_bool_field_t swen;
} pushbutton_command_state_t;

extern volatile manual_command_state_t g_manual_command;
extern volatile uart_command_state_t g_uart_command;
extern volatile pushbutton_command_state_t g_pushbutton_command;

bool CommandInputs_SetPower(volatile command_u16_field_t *field, uint16_t power_w, uint32_t timestamp_ms);
bool CommandInputs_SetManualPower(volatile command_i16_field_t *field, int16_t power_w, uint32_t timestamp_ms);
bool CommandInputs_SetEnergy(volatile command_u16_field_t *field, uint16_t energy_j, uint32_t timestamp_ms);
bool CommandInputs_SetSwen(volatile command_bool_field_t *field, bool swen, uint32_t timestamp_ms);

void CommandInputs_TogglePushbuttonSwenFromIsr(uint32_t timestamp_ms);

/* Call after every future successful mode transition to publish SWEN = 0. */
void CommandInputs_ResetPushbuttonForModeChange(uint32_t timestamp_ms);

void CommandInputs_ReadManualSnapshot(manual_command_state_t *snapshot);
void CommandInputs_ReadUartSnapshot(uart_command_state_t *snapshot);
void CommandInputs_ReadPushbuttonSnapshot(pushbutton_command_state_t *snapshot);

bool CommandInputs_IsFresh(uint32_t timestamp_ms, bool present, uint32_t now_ms, uint32_t timeout_ms);

#endif /* COMMAND_INPUTS_H */

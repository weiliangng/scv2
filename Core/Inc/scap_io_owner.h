#ifndef SCAP_IO_OWNER_H
#define SCAP_IO_OWNER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  CONTROL_MODE_EXTERNAL_SET = 0,
  CONTROL_MODE_MANUAL_SET,
  CONTROL_MODE_MEASURE,
  CONTROL_MODE_DIRECT_GPIO,
} control_mode_request_t;

typedef enum
{
  CONTROL_DECISION_DISABLE = 0,
  CONTROL_DECISION_IDLE,
  CONTROL_DECISION_NO_SOURCE,
  CONTROL_DECISION_MANUAL_SET_ALGO,
  CONTROL_DECISION_CAN_ALGO,
  CONTROL_DECISION_UART_ALGO,
  CONTROL_DECISION_MEASURE,
  CONTROL_DECISION_DIRECT_GPIO,
} control_decision_t;

enum
{
  CONTROL_FAULT_VBUS_OVP = 1u << 0,
  CONTROL_FAULT_VCAP_OVP = 1u << 1,
};

enum
{
  SCAP_FAST_SAFETY_FAULT_LATCHED = 1u << 0,
  SCAP_FAST_SAFETY_UVLO_LOCKOUT = 1u << 1,
};

typedef struct
{
  control_mode_request_t mode_request;
  control_decision_t decision;
  bool swen_request;
  bool can_fresh;
  bool uart_power_fresh;
  bool uart_energy_fresh;
  bool uvlo_lockout;
  bool fault_latched;
  uint8_t fault_bits;
  uint16_t fault_healthy_ms;
} control_status_t;

typedef struct
{
  control_decision_t decision;
  float p_set_w;
  bool swen_output_request;
} control_fast_command_t;

void ScapIo_Init(void);
void ScapIo_Resolve1kHz(void);
void ScapIo_ReadFastCommand(control_fast_command_t *command);
const char *ScapIo_DecisionName(control_decision_t decision);

void ScapIo_SetModeRequest(control_mode_request_t mode);
control_mode_request_t ScapIo_GetModeRequest(void);
bool ScapIo_IsDirectMode(void);
void ScapIo_ReadStatus(control_status_t *status);

bool ScapIo_IsFaultLatched(void);

bool ScapIo_IsUvloLockout(void);

/* Called from the ADC ISR. Updates fault first, then UVLO, and returns both states. */
uint8_t ScapIo_FastUpdateSafety(uint8_t fault_bits, float v_bus);

/* Called from the debounced pushbutton ISR. */
void ScapIo_HandlePushbuttonFromIsr(uint32_t timestamp_ms);

#endif /* SCAP_IO_OWNER_H */

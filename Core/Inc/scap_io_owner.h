#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "shared_state.h"

typedef enum
{
  SCAP_MODE_CCM = 0,   // 00
  SCAP_MODE_HCM = 1,   // 01
  SCAP_MODE_DCM = 2,   // 10
  SCAP_MODE_BURST = 3, // 11
} scap_mode_t;

/*
 * Logical mode mapping knobs:
 * Adjust these two definitions to remap what "bidirectional" and
 * "unidirectional" mean on the external MODE pins.
 */
#define BIDIRECTIONAL SCAP_MODE_CCM
#define UNIDIRECTIONAL SCAP_MODE_HCM

void ScapIo_Init(void);
void ScapIo_Tick1kHz(void);

void ScapIo_SetFaultLatched(bool fault_latched);
void ScapIo_RequestSwenPulseMs(uint16_t pulse_ms);

void ScapIo_ManualSetMode(scap_mode_t mode);
void ScapIo_ManualSetDir(bool dir_high);
void ScapIo_ManualSetSwen(bool swen_high);

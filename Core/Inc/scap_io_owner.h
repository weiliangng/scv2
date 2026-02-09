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

void ScapIo_Init(void);
void ScapIo_Tick1kHz(void);

void ScapIo_SetFaultLatched(bool fault_latched);
void ScapIo_RequestSwenPulseMs(uint16_t pulse_ms);

void ScapIo_ManualSetMode(scap_mode_t mode);
void ScapIo_ManualSetDir(bool dir_high);
void ScapIo_ManualSetSwen(bool swen_high);

void ScapIo_CanRxUpdateIsr(bool swen_high, bool dir_high, bool mode_bit);


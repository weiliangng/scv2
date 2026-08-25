#pragma once

#include <stdint.h>

void StartSlowAdcTask(void const *argument);
void TelemetrySlowAdcTask_Run(void const *argument);
uint32_t TelemetrySlowAdcTask_GetCanTxEnqueueFailureCount(void);

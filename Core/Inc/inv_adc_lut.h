#ifndef INV_ADC_LUT_H
#define INV_ADC_LUT_H

#include <stdint.h>

/*
 * 1/x lookup for ADC counts.
 * - Stored as float for now (matches other "latest" representations)
 * - inv_adc[0] is unused (kept as 0.0f)
 * - Always clamp LUT index to [1..4095]
 */

#define INV_ADC_LUT_SIZE 4096u

extern float g_inv_adc[INV_ADC_LUT_SIZE];

void InvAdc_Init(void);

static inline uint16_t InvAdc_ClampIndexI32(int32_t idx)
{
  if (idx < 1)
  {
    return 1u;
  }
  if (idx > 4095)
  {
    return 4095u;
  }
  return (uint16_t)idx;
}

static inline float InvAdc_LookupClampedI32(int32_t idx)
{
  return g_inv_adc[InvAdc_ClampIndexI32(idx)];
}

#endif /* INV_ADC_LUT_H */


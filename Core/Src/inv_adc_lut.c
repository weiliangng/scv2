#include "inv_adc_lut.h"

float g_inv_adc[INV_ADC_LUT_SIZE];

void InvAdc_Init(void)
{
  g_inv_adc[0] = 0.0f;
  for (uint16_t i = 1u; i < INV_ADC_LUT_SIZE; i++)
  {
    g_inv_adc[i] = 1.0f / (float)i;
  }
}


#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdbool.h>
#include <stdint.h>

_Static_assert(sizeof(float) == 4, "Expected 32-bit float");

/*
 * Shared "latest values" block:
 * - Many readers, each field has exactly one writer, or multiple writers determined by another state variable (owner)
 * - 32-bit aligned loads/stores are atomic on Cortex-M, so keep these as 32-bit floats
 * - Volatile because some fields are updated in ISRs
 */
typedef struct
{
  float v_bus;
  float v_cap;
  float i_load;
  float i_out;
  float i_out_p;
  float i_out_n;
  float i_conv;
  float p_load;
  float p_set;
} latest_values_t;

extern volatile latest_values_t g_latest;

/*
 * Supercap state/telemetry constants.
 */
extern const float C_cap;
extern const float V_cap_max;
extern const float cap_hi;
extern const float cap_lo;

/*
 * ISR timing metrics (DWT CYCCNT cycles).
 * Written from ISR context; read from telemetry/task context.
 */
extern volatile uint32_t g_dma1_ch1_irq_cycles_last;
extern volatile uint32_t g_dma1_ch1_irq_cycles_max;

/*
 * ADC trigger/sequence counter (increments once per ADC trigger).
 * Implementation detail: incremented from the ADC1 DMA TC ISR (DMA1 CH1),
 * which fires once per TIM2 TRGO -> ADC regular sequence completion.
 */
extern volatile uint32_t g_adc_seq_count;

/*
 * Fast hard-fault health flag computed in the DMA ISR.
 * UVLO is a non-latching idle lockout and does not make this false.
 */
extern volatile bool g_is_safe;

/*
 * Telemetry helpers computed in task context.
 * - g_adc_seq_hz: estimated ADC trigger/sequence rate (Hz).
 * - g_telemetry_seq: sequence counter for USB telemetry frames.
 */
extern volatile uint32_t g_adc_seq_hz;
extern volatile uint32_t g_telemetry_seq;

/*
 * ADC DMA buffers updated by hardware/DMA and consumed in ISR/task contexts.
 */
/*
 * DMA order follows the ADC "Rank" order in `test.ioc`:
 * ADC1 (2 conversions):
 *   g_adc1_dma_buf[0] = Vcap   (PA0 / ADC1_IN1 / ADC_CHANNEL_1)
 *   g_adc1_dma_buf[1] = Vbus   (PA1 / ADC1_IN2 / ADC_CHANNEL_2)
 *
 * ADC2 (3 conversions):
 *   g_adc2_dma_buf[0] = ILOAD  (PA6=IN3+, PA7=IN4- / ADC2_IN3..IN4 differential / ADC_CHANNEL_3)
 *   g_adc2_dma_buf[1] = IMONOP (PB14 -> OPAMP2 PGA -> ADC2 VOPAMP2)
 *   g_adc2_dma_buf[2] = IMONON (PB13 -> OPAMP3 PGA -> ADC2 VOPAMP3_ADC2)
 */
extern volatile uint16_t g_adc1_dma_buf[2];
extern volatile uint16_t g_adc2_dma_buf[3];

/*
 * Telemetry stream enable:
 * - When disabled, the telemetry task stays idle (no USB output spam while using the CLI).
 */
extern volatile bool g_telemetry_enabled;

#endif /* SHARED_STATE_H */

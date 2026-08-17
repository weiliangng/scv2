#include "shared_state.h"

volatile latest_values_t g_latest = {
    .p_set = 50.0f,
};

const float C_cap = 1822.5f;
const float V_cap_max = 26.3f;
const float cap_hi = 0.97f * V_cap_max;
const float cap_lo = 0.20f * V_cap_max;

volatile uint32_t g_dma1_ch1_irq_cycles_last;
volatile uint32_t g_dma1_ch1_irq_cycles_max;
volatile uint32_t g_adc_seq_count;
volatile bool g_is_safe;
volatile uint32_t g_adc_seq_hz;
volatile uint32_t g_telemetry_seq;

/* See `Core/Inc/shared_state.h` for buffer index meanings. */
volatile uint16_t g_adc1_dma_buf[2];
volatile uint16_t g_adc2_dma_buf[3];

volatile bool g_telemetry_enabled = false;

#include "shared_state.h"

volatile latest_values_t g_latest = {
    .p_set = 50.0f,
};

volatile float meter_v = 0.0f;
volatile float meter_i = 0.0f;

const float C_cap = 1822.5f;
const float V_cap_max = 26.3f;

volatile uint32_t g_dma1_ch1_irq_cycles_last;
volatile uint32_t g_dma1_ch1_irq_cycles_max;

/* See `Core/Inc/shared_state.h` for buffer index meanings. */
volatile uint16_t g_adc1_dma_buf[2];
volatile uint16_t g_adc2_dma_buf[3];

volatile can_rx_state_t g_can_rx;
volatile uart_rx_state_t g_uart_rx;

volatile bool g_can_connected;
volatile bool g_uart_connected;

volatile ctrl_src_t g_ctrl_src = SRC_ALGO;
volatile bool g_telemetry_enabled = true;

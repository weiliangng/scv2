#include "shared_state.h"

volatile latest_values_t g_latest;

volatile uint16_t g_adc1_dma_buf[2];
volatile uint16_t g_adc2_dma_buf[3];

volatile can_rx_state_t g_can_rx;
volatile uart_rx_state_t g_uart_rx;


#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdbool.h>
#include <stdint.h>

_Static_assert(sizeof(float) == 4, "Expected 32-bit float");

/*
 * Shared "latest values" block:
 * - Many readers, each field has exactly one writer (owner)
 * - 32-bit aligned loads/stores are atomic on Cortex-M, so keep these as 32-bit floats
 * - Volatile because some fields are updated in ISRs
 */
typedef struct
{
  float v_bus;
  float v_cap;
  float i_load;
  float i_out_p;
  float i_out_n;
  float i_out;
  float p_load;
  float p_cap;
} latest_values_t;

extern volatile latest_values_t g_latest;

/*
 * ADC DMA buffers updated by hardware/DMA and consumed in ISR/task contexts.
 */
extern volatile uint16_t g_adc1_dma_buf[2];
extern volatile uint16_t g_adc2_dma_buf[3];

/*
 * CAN receive ISR state.
 */
typedef struct
{
  float p_set_cmd;
  uint32_t last_can_tick;
  uint32_t can_rx_count;
  uint8_t settings_raw;
  bool mode;
  bool dir;
  bool auto_buffer_tie;
  bool en;
} can_rx_state_t;

extern volatile can_rx_state_t g_can_rx;

/*
 * UART receive ISR state.
 */
typedef struct
{
  float buf_e_j;
  uint32_t last_uart_tick;
  uint32_t uart_rx_count;
} uart_rx_state_t;

extern volatile uart_rx_state_t g_uart_rx;

#endif /* SHARED_STATE_H */


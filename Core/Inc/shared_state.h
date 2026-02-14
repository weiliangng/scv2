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
 * Manual CLI "requested power" mailbox.
 * Resolved into `g_latest.p_set` by the slow 1 kHz task based on priority.
 */
extern volatile float g_manual_p_set_w;

/*
 * Supercap state/telemetry constants.
 */
extern const float C_cap;
extern const float V_cap_max;

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
 * Fast safety flag computed in the DMA ISR.
 * True when the measured voltages are within safe limits.
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
 * CAN receive ISR state.
 */
extern volatile float meter_v; // Volts from wattmeter (ID=METER_ID, decoded from ×100)
extern volatile float meter_i; // Amps  from wattmeter (ID=METER_ID, decoded from ×100)

typedef struct
{
  uint32_t last_can_tick;
  uint32_t last_cmd_tick;
  uint32_t can_rx_count;
  uint8_t settings_raw; // raw settings byte; currently only bit0 (SWEN request) is used
  uint16_t can_power;//alternate power limit source from CAN
  uint8_t can_buf;//alternate buffer energy source from CAN
  bool en;//enable switching
} can_rx_state_t;

extern volatile can_rx_state_t g_can_rx;

typedef enum
{
  SRC_MANUAL = 0, // cli request
  SRC_ALGO = 1,   // default on startup
} ctrl_src_t;

extern volatile ctrl_src_t g_ctrl_src;

/*
 * UART receive ISR state.
 */
typedef struct
{
  float chassis_power_limit_w;
  float buf_e_j;
  uint32_t last_uart_tick;
  uint32_t uart_rx_count;
} uart_rx_state_t;

extern volatile uart_rx_state_t g_uart_rx;

/*
 * Derived connection status flags (RX activity-based).
 * Updated from the 1 kHz slow task.
 */
extern volatile bool g_can_connected;
extern volatile bool g_uart_connected;
extern volatile bool g_can_cmd_connected;

/*
 * Control source for power-stage IO (SWEN/MODE/DIR).
 * When SRC_ALGO is selected, the fast DMA ISR controls DIR.
 */

/*
 * Resolved "current buffer energy" (J) for use in control/telemetry:
 * - When UART link is up, mirrors `g_uart_rx.buf_e_j`.
 * - Else when CAN command packets are fresh, mirrors `g_can_rx.can_buf` (unit/scaling depends on sender).
 * - Else 0.
 *
 * Written from the fast DMA ISR; read from task/CLI contexts.
 */
extern volatile float g_curr_buf_e_j;

/*
 * Telemetry stream enable:
 * - When disabled, the telemetry task stays idle (no USB output spam while using the CLI).
 */
extern volatile bool g_telemetry_enabled;

#endif /* SHARED_STATE_H */

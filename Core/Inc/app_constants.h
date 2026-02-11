#ifndef APP_CONSTANTS_H
#define APP_CONSTANTS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Global calibration/constants.
 *
 * Values in the "calibration" section can be overridden at runtime and persisted
 * to flash via EEPROM emulation.
 */

extern const uint32_t VREF_MV;

// V_bus = A_VBUS * N_adc_vbus + B_VBUS
extern float A_VBUS;     // V/count
extern float B_VBUS;     // V

// I_load = A_ILOAD * N_adc_iload + B_ILOAD
extern float A_ILOAD;    // A/count
extern float B_ILOAD;    // A
extern float MIDPOINT;   // count

// I_conv = P_set * (1/A_VBUS * 1/(N + N_OFFSET)) - I_load
extern float A_VBUS_INV;
extern float N_OFFSET;

// DAC setting (real units -> counts)
// N_dac_p = A_INP + I_conv * B_INP
extern float A_INP;      // counts
extern float B_INP;      // counts/A

// N_dac_n = A_INN + I_conv * B_INN
extern float A_INN;      // counts
extern float B_INN;      // counts/A

// V_cap = A_VCAP * N_adc_vcap + B_VCAP
extern float A_VCAP;     // V/count
extern float B_VCAP;     // V

// i_out_p = A_OP * N_adc_op + B_OP
extern float A_OP;       // A/count
extern float B_OP;       // A

// i_out_n = A_ON * N_adc_on + B_ON
extern float A_ON;       // A/count
extern float B_ON;       // A

// DAC3 default startup values (12-bit, right-aligned).
// These are written once during init and then remain unchanged unless the CLI
// overrides them via `dac set 3 <1|2> <0..4095>`.
extern uint32_t DAC3_CH1_BOOT_U12;
extern uint32_t DAC3_CH2_BOOT_U12;

void AppConstants_RecalcDerived(void);
void AppConstants_ResetToDefaults(void);
void AppConstants_InitFromNvm(void);
bool AppConstants_LoadFromNvm(void);
bool AppConstants_SaveToNvm(void);

// Classic CAN (standard 11-bit IDs)
extern const uint32_t SCAP_STAT_ID; // status/telemetry packets sent out from supercap
extern const uint32_t METER_ID;     // wattmeter -> (V/I scaled x100)
extern const uint32_t SCAP_CMD_ID;  // command/setpoint to supercap
extern const uint32_t SCAP_STAT_RATE_HZ; // 10..1000

// Simple "link up" heuristics based on most recent RX timestamp.
// 0 RX since boot => link down.
extern const uint32_t CAN_RX_LINK_TIMEOUT_MS;
extern const uint32_t UART_RX_LINK_TIMEOUT_MS;

// CAN command freshness timeout (based on `g_can_rx.last_cmd_tick`).
// Used for CAN IO ownership + CAN P_set override eligibility.
extern const uint32_t CAN_CMD_TIMEOUT_MS;

#endif /* APP_CONSTANTS_H */

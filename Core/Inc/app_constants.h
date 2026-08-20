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

// Clamp applied to I_conv in the fast DMA ISR (absolute value, Amps).
#define I_CONV_CLAMP_ABS_A (10.0f)
#define I_CAP_CLAMP_ABS_A (15.0f)

/* Control-policy thresholds (compile-time only; not EEPROM calibration). */
#define SCAP_VBUS_OVP_V                 (30.0f)
#define SCAP_VCAP_OVP_V                 (30.0f)
#define SCAP_UVLO_ENTER_V               (10.0f)
#define SCAP_UVLO_EXIT_V                (11.0f)
#define SCAP_VCAP_MAX_V                 (26.3f)
#define SCAP_VCAP_LOW_V                 (0.20f * SCAP_VCAP_MAX_V)
#define SCAP_VCAP_HYSTERESIS_V          (0.2f)
#define SCAP_ENERGY_CHARGE_J            (55u)
#define SCAP_ENERGY_DISCHARGE_J         (20u)
#define SCAP_CHARGE_LOCKOUT_RESUME_V    (SCAP_VCAP_MAX_V - SCAP_VCAP_HYSTERESIS_V)
#define SCAP_DISCHARGE_LOCKOUT_RESUME_V (SCAP_VCAP_LOW_V + SCAP_VCAP_HYSTERESIS_V)
#define SCAP_MANUAL_DEFAULT_POWER_W     (50.0f)
#define SCAP_COMMAND_FRESH_TIMEOUT_MS   (300u)
#define SCAP_FAULT_RECOVERY_MS          (500u)
#define SCAP_CPU_HZ                     (96000000u)
#define SCAP_SWEN_MIN_ON_MS             (100u)
#define SCAP_SWEN_MIN_OFF_MS            (2u)

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

extern const uint32_t SCAP_STAT_RATE_HZ; // 10..1000

#endif /* APP_CONSTANTS_H */

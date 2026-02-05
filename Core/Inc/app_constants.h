#ifndef APP_CONSTANTS_H
#define APP_CONSTANTS_H

#include <stdint.h>

/*
 * Prefer const over macros for globally shared calibration/constants.
 * Float literals use an explicit 'f' suffix.
 */

extern const uint32_t VREF_MV;

// V_bus = A_VBUS * N_adc_vbus + B_VBUS
extern const float A_VBUS;     // V/count
extern const float B_VBUS;     // V

// I_load = A_ILOAD * N_adc_iload + B_ILOAD
extern const float A_ILOAD;    // A/count
extern const float B_ILOAD;    // A
extern const float MIDPOINT;   // count

// I_conv = P_set * (1/A_VBUS * 1/(N + N_OFFSET)) - I_load
extern const float A_VBUS_INV;
extern const float N_OFFSET;

// DAC setting (real units -> counts)
// N_dac_p = A_INP + I_conv * B_INP
extern const float A_INP;      // counts
extern const float B_INP;      // counts/A

// N_dac_n = A_INN + I_conv * B_INN
extern const float A_INN;      // counts
extern const float B_INN;      // counts/A

// V_cap = A_VCAP * N_adc_vcap + B_VCAP
extern const float A_VCAP;     // V/count
extern const float B_VCAP;     // V

// i_out_p = A_OP * N_adc_op + B_OP
extern const float A_OP;       // A/count
extern const float B_OP;       // A

// i_out_n = A_ON * N_adc_on + B_ON
extern const float A_ON;       // A/count
extern const float B_ON;       // A

#endif /* APP_CONSTANTS_H */


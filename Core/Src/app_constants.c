#include "app_constants.h"

#define A_VBUS_VALUE 0.009654528478f // V/count
#define B_VBUS_VALUE 0.0f            // V

const uint32_t SCAP_STAT_ID = 0x077u;
const uint32_t METER_ID = 0x215u;// to add 214,213
const uint32_t SCAP_CMD_ID = 0x067u;
const uint32_t SCAP_STAT_RATE_HZ = 1000u;

const uint32_t CAN_RX_LINK_TIMEOUT_MS = 500u;
const uint32_t UART_RX_LINK_TIMEOUT_MS = 1000u;

const uint32_t VREF_MV = 3300u; // mV (currently unused)

const float A_VBUS = A_VBUS_VALUE; // V/count
const float B_VBUS = B_VBUS_VALUE; // V

//for single ended mode * 1, for differential mode * 2
const float A_ILOAD = 0.008058608059f * 2; // A/count
const float B_ILOAD = -16.5f * 2;          // A
const float MIDPOINT = 2047.5f;        // count

const float A_VBUS_INV = 1.0f / A_VBUS_VALUE;
const float N_OFFSET = B_VBUS_VALUE / A_VBUS_VALUE;

const float A_INP = 2754.776956f;   // counts
const float B_INP = -124.090909f;   // counts/A

const float A_INN = 2754.776956f;   // counts
const float B_INN = 124.090909f;    // counts/A

const float A_VCAP = 0.009654528478f; // V/count
const float B_VCAP = 0.0f;            // V

const float A_OP = 0.00829074903f;  // A/count
const float B_OP = -10.0f;          // A

const float A_ON = -0.00829074903f; // A/count
const float B_ON = 10.0f;           // A

const uint32_t DAC3_CH1_BOOT_U12 = 2048u;
const uint32_t DAC3_CH2_BOOT_U12 = 3723u;

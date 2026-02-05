#include "app_constants.h"

const uint32_t VREF_MV = 3300u; // mV (currently unused)

const float A_VBUS = 0.009654528478f; // V/count
const float B_VBUS = 0.0f;            // V

const float A_ILOAD = 0.008058608059f; // A/count
const float B_ILOAD = -16.5f;          // A
const float MIDPOINT = 2047.5f;        // count

const float A_VBUS_INV = 1.0f / 0.009654528478f;
const float N_OFFSET = 0.0f / 0.009654528478f;

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


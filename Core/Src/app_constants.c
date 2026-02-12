#include "app_constants.h"

#include <string.h>

#include "nvm_eeprom.h"

const uint32_t SCAP_STAT_ID = 0x077u;
const uint32_t METER_ID = 0x215u;// to add 214,213
const uint32_t SCAP_CMD_ID = 0x067u;
const uint32_t SCAP_STAT_RATE_HZ = 1000u;

const uint32_t CAN_RX_LINK_TIMEOUT_MS = 500u;
const uint32_t UART_RX_LINK_TIMEOUT_MS = 1000u;
const uint32_t CAN_CMD_TIMEOUT_MS = 500u;

const uint32_t VREF_MV = 3300u; // mV (currently unused)

static const float DEFAULT_A_VBUS = 0.009654528478f; // V/count
static const float DEFAULT_B_VBUS = 0.0f;            // V

//for single ended mode * 1, for differential mode * 2
static const float DEFAULT_A_ILOAD = 0.008058608059f * 2.0f; // A/count
static const float DEFAULT_B_ILOAD = -16.5f * 2.0f;          // A
static const float DEFAULT_MIDPOINT = 2047.5f;               // count

static const float DEFAULT_A_INP = 2754.776956f;   // counts
static const float DEFAULT_B_INP = -124.090909f;   // counts/A

static const float DEFAULT_A_INN = 2754.776956f;   // counts
static const float DEFAULT_B_INN = 124.090909f;    // counts/A

static const float DEFAULT_A_VCAP = 0.009654528478f; // V/count
static const float DEFAULT_B_VCAP = 0.0f;            // V

static const float DEFAULT_A_OP = 0.00829074903f;  // A/count
static const float DEFAULT_B_OP = -10.0f;          // A

static const float DEFAULT_A_ON = -0.00829074903f; // A/count
static const float DEFAULT_B_ON = 10.0f;           // A

static const uint32_t DEFAULT_DAC3_CH1_BOOT_U12 = 920u;
static const uint32_t DEFAULT_DAC3_CH2_BOOT_U12 = 3723u;

float A_VBUS = 0.009654528478f; // V/count
float B_VBUS = 0.0f;            // V

float A_ILOAD = 0.008058608059f * 2.0f; // A/count
float B_ILOAD = -16.5f * 2.0f;          // A
float MIDPOINT = 2047.5f;               // count

float A_VBUS_INV = 0.0f;
float N_OFFSET = 0.0f;

float A_INP = 2754.776956f;   // counts
float B_INP = -124.090909f;   // counts/A

float A_INN = 2754.776956f;   // counts
float B_INN = 124.090909f;    // counts/A

float A_VCAP = 0.009654528478f; // V/count
float B_VCAP = 0.0f;            // V

float A_OP = 0.00829074903f;  // A/count
float B_OP = -10.0f;          // A

float A_ON = -0.00829074903f; // A/count
float B_ON = 10.0f;           // A

uint32_t DAC3_CH1_BOOT_U12 = 2048u;
uint32_t DAC3_CH2_BOOT_U12 = 3723u;

enum
{
  EE_VA_MAGIC = 1,
  EE_VA_VERSION = 2,
  EE_VA_SEQ_BEGIN = 3,
  EE_VA_SEQ_END = 4,

  EE_VA_A_VBUS = 5,
  EE_VA_B_VBUS = 6,
  EE_VA_A_ILOAD = 7,
  EE_VA_B_ILOAD = 8,
  EE_VA_MIDPOINT = 9,
  EE_VA_A_INP = 10,
  EE_VA_B_INP = 11,
  EE_VA_A_INN = 12,
  EE_VA_B_INN = 13,
  EE_VA_A_VCAP = 14,
  EE_VA_B_VCAP = 15,
  EE_VA_A_OP = 16,
  EE_VA_B_OP = 17,
  EE_VA_A_ON = 18,
  EE_VA_B_ON = 19,
  EE_VA_DAC3_CH1_BOOT_U12 = 20,
  EE_VA_DAC3_CH2_BOOT_U12 = 21,
};

static const uint32_t APP_CONST_MAGIC = 0x43414C31u; /* 'CAL1' */
static const uint32_t APP_CONST_VERSION = 1u;

static uint32_t u32_from_float(float v)
{
  uint32_t u = 0;
  memcpy(&u, &v, sizeof(u));
  return u;
}

static float float_from_u32(uint32_t u)
{
  float v = 0.0f;
  memcpy(&v, &u, sizeof(v));
  return v;
}

void AppConstants_RecalcDerived(void)
{
  if (A_VBUS != 0.0f)
  {
    A_VBUS_INV = 1.0f / A_VBUS;
    N_OFFSET = B_VBUS / A_VBUS;
  }
  else
  {
    A_VBUS_INV = 0.0f;
    N_OFFSET = 0.0f;
  }
}

void AppConstants_ResetToDefaults(void)
{
  A_VBUS = DEFAULT_A_VBUS;
  B_VBUS = DEFAULT_B_VBUS;
  A_ILOAD = DEFAULT_A_ILOAD;
  B_ILOAD = DEFAULT_B_ILOAD;
  MIDPOINT = DEFAULT_MIDPOINT;

  A_INP = DEFAULT_A_INP;
  B_INP = DEFAULT_B_INP;
  A_INN = DEFAULT_A_INN;
  B_INN = DEFAULT_B_INN;

  A_VCAP = DEFAULT_A_VCAP;
  B_VCAP = DEFAULT_B_VCAP;

  A_OP = DEFAULT_A_OP;
  B_OP = DEFAULT_B_OP;
  A_ON = DEFAULT_A_ON;
  B_ON = DEFAULT_B_ON;

  DAC3_CH1_BOOT_U12 = DEFAULT_DAC3_CH1_BOOT_U12;
  DAC3_CH2_BOOT_U12 = DEFAULT_DAC3_CH2_BOOT_U12;

  AppConstants_RecalcDerived();
}

bool AppConstants_LoadFromNvm(void)
{
  uint32_t magic = 0, version = 0, seq_begin = 0, seq_end = 0;

  if (!NvmEeprom_ReadU32(EE_VA_MAGIC, &magic) || magic != APP_CONST_MAGIC)
  {
    return false;
  }
  if (!NvmEeprom_ReadU32(EE_VA_VERSION, &version) || version != APP_CONST_VERSION)
  {
    return false;
  }
  if (!NvmEeprom_ReadU32(EE_VA_SEQ_BEGIN, &seq_begin) || !NvmEeprom_ReadU32(EE_VA_SEQ_END, &seq_end) ||
      seq_begin == 0u || seq_begin != seq_end)
  {
    return false;
  }

  uint32_t tmp = 0;

  if (!NvmEeprom_ReadU32(EE_VA_A_VBUS, &tmp))
    return false;
  const float a_vbus = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_VBUS, &tmp))
    return false;
  const float b_vbus = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_A_ILOAD, &tmp))
    return false;
  const float a_iload = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_ILOAD, &tmp))
    return false;
  const float b_iload = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_MIDPOINT, &tmp))
    return false;
  const float midpoint = float_from_u32(tmp);

  if (!NvmEeprom_ReadU32(EE_VA_A_INP, &tmp))
    return false;
  const float a_inp = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_INP, &tmp))
    return false;
  const float b_inp = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_A_INN, &tmp))
    return false;
  const float a_inn = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_INN, &tmp))
    return false;
  const float b_inn = float_from_u32(tmp);

  if (!NvmEeprom_ReadU32(EE_VA_A_VCAP, &tmp))
    return false;
  const float a_vcap = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_VCAP, &tmp))
    return false;
  const float b_vcap = float_from_u32(tmp);

  if (!NvmEeprom_ReadU32(EE_VA_A_OP, &tmp))
    return false;
  const float a_op = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_OP, &tmp))
    return false;
  const float b_op = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_A_ON, &tmp))
    return false;
  const float a_on = float_from_u32(tmp);
  if (!NvmEeprom_ReadU32(EE_VA_B_ON, &tmp))
    return false;
  const float b_on = float_from_u32(tmp);

  if (!NvmEeprom_ReadU32(EE_VA_DAC3_CH1_BOOT_U12, &tmp))
    return false;
  const uint32_t dac3_ch1_boot_u12 = tmp;
  if (!NvmEeprom_ReadU32(EE_VA_DAC3_CH2_BOOT_U12, &tmp))
    return false;
  const uint32_t dac3_ch2_boot_u12 = tmp;

  if ((a_vbus == 0.0f) || (dac3_ch1_boot_u12 > 4095u) || (dac3_ch2_boot_u12 > 4095u))
  {
    return false;
  }

  A_VBUS = a_vbus;
  B_VBUS = b_vbus;
  A_ILOAD = a_iload;
  B_ILOAD = b_iload;
  MIDPOINT = midpoint;

  A_INP = a_inp;
  B_INP = b_inp;
  A_INN = a_inn;
  B_INN = b_inn;

  A_VCAP = a_vcap;
  B_VCAP = b_vcap;

  A_OP = a_op;
  B_OP = b_op;
  A_ON = a_on;
  B_ON = b_on;

  DAC3_CH1_BOOT_U12 = dac3_ch1_boot_u12;
  DAC3_CH2_BOOT_U12 = dac3_ch2_boot_u12;

  AppConstants_RecalcDerived();
  return true;
}

void AppConstants_InitFromNvm(void)
{
  AppConstants_ResetToDefaults();
  (void)AppConstants_LoadFromNvm();
}

bool AppConstants_SaveToNvm(void)
{
  uint32_t seq = 0;

  (void)NvmEeprom_ReadU32(EE_VA_SEQ_END, &seq);
  seq += 1u;
  if (seq == 0u)
  {
    seq = 1u;
  }

  if (!NvmEeprom_WriteU32(EE_VA_MAGIC, APP_CONST_MAGIC))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_VERSION, APP_CONST_VERSION))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_SEQ_BEGIN, seq))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_A_VBUS, u32_from_float(A_VBUS)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_VBUS, u32_from_float(B_VBUS)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_A_ILOAD, u32_from_float(A_ILOAD)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_ILOAD, u32_from_float(B_ILOAD)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_MIDPOINT, u32_from_float(MIDPOINT)))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_A_INP, u32_from_float(A_INP)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_INP, u32_from_float(B_INP)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_A_INN, u32_from_float(A_INN)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_INN, u32_from_float(B_INN)))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_A_VCAP, u32_from_float(A_VCAP)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_VCAP, u32_from_float(B_VCAP)))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_A_OP, u32_from_float(A_OP)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_OP, u32_from_float(B_OP)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_A_ON, u32_from_float(A_ON)))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_B_ON, u32_from_float(B_ON)))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_DAC3_CH1_BOOT_U12, DAC3_CH1_BOOT_U12))
    return false;
  if (!NvmEeprom_WriteU32(EE_VA_DAC3_CH2_BOOT_U12, DAC3_CH2_BOOT_U12))
    return false;

  if (!NvmEeprom_WriteU32(EE_VA_SEQ_END, seq))
    return false;

  return true;
}

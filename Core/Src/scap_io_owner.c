#include "scap_io_owner.h"

#include "main.h"

#define STAGING_DIR_HIGH GPIO_DIR_Pin
#define STAGING_MODE_HCM GPIO_MODELSB_Pin

static void ScapIo_ApplyStagingOutputs(void)
{
  const uint16_t affect_mask = GPIO_DIR_Pin | GPIO_SWEN_Pin | GPIO_MODEMSB_Pin | GPIO_MODELSB_Pin;
  const uint16_t desired = STAGING_DIR_HIGH | STAGING_MODE_HCM;
  const uint16_t set_mask = (uint16_t)(desired & affect_mask);
  const uint16_t reset_mask = (uint16_t)((~desired) & affect_mask);
  GPIOB->BSRR = ((uint32_t)reset_mask << 16) | (uint32_t)set_mask;
}

void ScapIo_Init(void)
{
  ScapIo_ApplyStagingOutputs();
}

void ScapIo_Tick1kHz(void)
{
  ScapIo_ApplyStagingOutputs();
}

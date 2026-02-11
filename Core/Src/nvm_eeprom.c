#include "nvm_eeprom.h"

#include "eeprom_emul.h"
#include "stm32g4xx_hal.h"

bool NvmEeprom_Init(void)
{
  HAL_FLASH_Unlock();
  EE_Status status = EE_Init(EE_CONDITIONAL_ERASE);
  HAL_FLASH_Lock();
  return status == EE_OK;
}

bool NvmEeprom_ReadU32(uint16_t virt_addr, uint32_t *out_value)
{
  if (out_value == NULL)
  {
    return false;
  }

  EE_Status status = EE_ReadVariable32bits(virt_addr, out_value);
  return status == EE_OK;
}

bool NvmEeprom_WriteU32(uint16_t virt_addr, uint32_t value)
{
  HAL_FLASH_Unlock();

  EE_Status status = EE_WriteVariable32bits(virt_addr, value);
  if ((status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP)
  {
    status |= EE_CleanUp();
  }

  HAL_FLASH_Lock();

  return (status & EE_STATUSMASK_ERROR) == 0U;
}


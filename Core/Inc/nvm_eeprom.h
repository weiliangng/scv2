#pragma once

#include <stdbool.h>
#include <stdint.h>

bool NvmEeprom_Init(void);
bool NvmEeprom_ReadU32(uint16_t virt_addr, uint32_t *out_value);
bool NvmEeprom_WriteU32(uint16_t virt_addr, uint32_t value);


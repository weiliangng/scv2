/**
  ******************************************************************************
  * @file    eeprom_emul_conf.h
  * @brief   EEPROM emulation configuration file.
  ******************************************************************************
  *
  * NOTE: This project uses the STM32Cube EEPROM Emulation middleware
  *       (`Middlewares/ST/EEPROM_Emul`).
  *
  ******************************************************************************
  */

#ifndef __EEPROM_EMUL_CONF_H
#define __EEPROM_EMUL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Start address of the 1st Flash page used for EEPROM emulation.
 *
 * STM32G431CBTx has 128KB Flash, 2KB pages.
 * This is configured to use the last 8KB (4 pages) of Flash:
 *   0x0801E000 .. 0x0801FFFF
 *
 * Keep this aligned on a Flash page boundary and ensure the linker script
 * does not place application code/data in this region.
 */
#define START_PAGE_ADDRESS 0x0801E000U

/* Number of 10K-cycles requested (minimum 1). Increasing this increases the
 * number of pages used by the emulation algorithm.
 */
#define CYCLES_NUMBER 1U

/* Number of guard pages (must be multiple of 2): 0,2,4.. */
#define GUARD_PAGES_NUMBER 2U

/* CRC configuration for EEPROM emulation. */
#define CRC_POLYNOMIAL_LENGTH LL_CRC_POLYLENGTH_16B
#define CRC_POLYNOMIAL_VALUE  0x8005U

/* Number of variables (virtual addresses) supported by EEPROM emulation. */
#define NB_OF_VARIABLES 128U

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_EMUL_CONF_H */


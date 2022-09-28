/**************************************************************************//**
 * @file     eeprom.h
 * @version  V0.01
 * @brief    eeprom emulation library
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __EEPROM_H__
#define __EEPROM_H__
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define NUEE_PRE_DEFINE 0   /*!< Option for pre-definitions */

#if !NUEE_PRE_DEFINE
/**
 * @brief Initial EEPROM Library
 *
 * @param[in] u32FlashBase   Flash base address for EEPROM emulation
 * @param[in] u32TotalPages  Total pages for EEPROM emulation
 * @param[in] u32PageSize    Flash page size
 * @param[in] u32RomSize     EEPROM size
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details Set flash base, size and necessary parameters to emulate EEPROM.
 *          
 */
int32_t NUEE_Init(uint32_t u32FlashBase, uint32_t u32TotalPages, uint32_t u32PageSize, uint32_t u32RomSize);

#endif
/**
 * @brief Read data from EEPROM.
 *
 * @param[in] u32Addr Address of EEPROM to read.
 *
 * @return The data read from EEPROM.
 *
 * @details To read the data from EEPROM.
 *          
 */
uint8_t NUEE_Read(uint32_t u32Addr);

/**
 * @brief Write data to EEPROM.
 *
 * @param[in] u32Addr Address of EEPROM to write.
 * @param[in] data The dat to write to EEPROM.
 *
 * @details  To write dat to EEPROM.
 *          
 */
void NUEE_Write(uint32_t u32Addr, uint8_t data);


#ifdef __cplusplus
}
#endif

#endif /* EEPROM_H */


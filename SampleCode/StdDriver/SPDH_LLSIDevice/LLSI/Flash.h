/******************************************************************************
 * @file     Flash.h
 * @brief    Flash header file
 *
 * @note
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __Flash_H__
#define __Flash_H__

#include <stdint.h>

extern volatile uint8_t g_u8Strip1_Flash_OneShot;

void ReadStoredSetting(uint8_t *pSetting, uint8_t u8IsCont);

#endif  /* __Flash_H__ */

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

void ReadStoredSetting(uint8_t MODESEL, uint8_t FRESEL, uint8_t LEDFUNSEL, uint16_t PCNTSEL, uint8_t Color_R, uint8_t Color_G, uint8_t Color_B, uint8_t u8Speed, uint8_t u8Brightness);

#endif  /* __Flash_H__ */

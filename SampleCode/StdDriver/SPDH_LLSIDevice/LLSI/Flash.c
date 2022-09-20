/******************************************************************************//**
 * @file     Flash.c
 * @version  V1.00
 * @brief    Flash sample file
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdlib.h>
#include "NuMicro.h"
#include "Global_Variable.h"
#include "Flash.h"

volatile uint8_t g_u8Strip1_Flash_OneShot = 0;

void ReadStoredSetting(uint8_t MODESEL, uint8_t FRESEL, uint8_t LEDFUNSEL, uint16_t PCNTSEL, uint8_t Color_R, uint8_t Color_G, uint8_t Color_B)
{
    Strip1_LEDSetting.AP_Sync = MODESEL;
    Strip1_LEDSetting.LEDNum = PCNTSEL;
    if(FRESEL == 0)
        g_u8Strip1_Flash_OneShot = 1;
    else
        g_u8Strip1_Flash_OneShot = 0;

    Strip1_LEDSetting.Color_R = Color_R;
    Strip1_LEDSetting.Color_G = Color_G;
    Strip1_LEDSetting.Color_B = Color_B;

    Strip1_LEDSetting.LightingMode = LEDFUNSEL;
    Strip1_LEDSetting.Mode_FUNC = (LED_FUNC)Mode_Function[Strip1_LEDSetting.LightingMode];
}

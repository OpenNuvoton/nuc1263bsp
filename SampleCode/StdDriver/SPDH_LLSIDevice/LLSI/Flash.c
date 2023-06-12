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

void ReadStoredSetting(uint8_t *pSetting, uint8_t u8IsCont)
{
    LED_Setting_T *pLEDSetting = (LED_Setting_T *)pSetting;
    
    if (pSetting == NULL)
        return ;
        
    if (u8IsCont == 0)
        g_u8Strip1_Flash_OneShot = 1;
    else
        g_u8Strip1_Flash_OneShot = 0;
    
    Strip1_LEDSetting.AP_Sync   = pLEDSetting->AP_Sync;
    Strip1_LEDSetting.LEDNum    = pLEDSetting->LEDNum;
    Strip1_LEDSetting.Direction = pLEDSetting->Direction;

    Strip1_LEDSetting.LightingMode = pLEDSetting->LightingMode;
    Strip1_LEDSetting.Color_R      = pLEDSetting->Color_R;
    Strip1_LEDSetting.Color_G      = pLEDSetting->Color_G;
    Strip1_LEDSetting.Color_B      = pLEDSetting->Color_B;
    Strip1_LEDSetting.Speed        = pLEDSetting->Speed;
    Strip1_LEDSetting.Brightness   = pLEDSetting->Brightness;
    Strip1_LEDSetting.Mode_FUNC    = (LED_FUNC)Mode_Function[Strip1_LEDSetting.LightingMode];
}

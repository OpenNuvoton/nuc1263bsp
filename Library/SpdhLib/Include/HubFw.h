/**************************************************************************//**
 * @file     HubFw.h
 * @version  V3.00
 * @brief    Hub control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HUBFW_H__
#define __HUBFW_H__
#include "stdio.h"
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ADC_HSA_VALUE_HID0 400
#define ADC_HSA_VALUE_HID1 800
#define ADC_HSA_VALUE_HID2 1200
#define ADC_HSA_VALUE_HID3 1600
#define ADC_HSA_VALUE_HID4 2000
#define ADC_HSA_VALUE_HID5 2400
#define ADC_HSA_VALUE_HID6 2800
#define ADC_HSA_VALUE_HID7 3200
#define ADC_HSA_VALUE_OFFLINE 0
#define ADC_HSA_VALUE_DEVIATION 100

int8_t Hub_Init(void);
int8_t Hub_ResetInit(uint32_t u32I3cAddr);
int8_t Hub_UpdateTempReg(void);
int8_t Temp_Init(void);
int8_t Hub_UpdateInterfaceSel(uint8_t u8InfSel);
int8_t Hub_SetLocalBusPullUp(uint32_t u32LsdaSel, uint32_t u32LsclSel);
int8_t Hub_CheckIBIReg(void);

void Hub_SPDHIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* HUBFW_H */
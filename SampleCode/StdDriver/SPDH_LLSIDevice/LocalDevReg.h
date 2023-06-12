/**************************************************************************//**
 * @file     LocalDevReg.h
 * @version  V3.00
 * @brief    Local device register definition.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __LOCALDEVREG_H__
#define __LOCALDEVREG_H__
#include <stdio.h>
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "LED_Control.h"

#define MAX_DEVREG_LEN          (64)

#define DEV_IBI_INTEN_ERROR     BIT4

#define DEV_ERROR_STATUS_PEC    BIT1
#define DEV_ERROR_STATUS_PAR    BIT0


extern volatile uint8_t g_au8DevReg[MAX_DEVREG_LEN];

int8_t DevReg_EnableI2C(void);
int8_t DevReg_EnableI3C(void);
int8_t DevReg_SetIBIStatus(void);
int8_t DevReg_ClearIBIStatus(void);
int8_t DevReg_GetIBIntEn(void);
int8_t DevReg_IsIBIntEn(uint8_t u8IntMsk);
int8_t DevReg_GetErrorStatus(void);
int8_t DevReg_SetParityErrStatus(void);
int8_t DevReg_IBIEnable(void);
int8_t DevReg_IBIDisable(void);
int8_t DevReg_PECEnable(void);
int8_t DevReg_PECDisable(void);
int8_t DevReg_ParityDisable(void);
int8_t DevReg_ParityEnable(void);
int8_t DevReg_ClearAllEvent(void);

/**
 * @brief Write the specific address register value of device
 *
 * @param[in] u8RegAddr   Register Address
 * @param[in] u8Value     Register Value
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details Write the device register value.
 *
 */
int8_t DevReg_WriteReg(uint8_t u8RegAddr, uint8_t u8Value);
int8_t DevReg_LLSIEnable(uint8_t u8Enable);

#ifdef __cplusplus
}
#endif

#endif /* LOCALDEVREG_H */

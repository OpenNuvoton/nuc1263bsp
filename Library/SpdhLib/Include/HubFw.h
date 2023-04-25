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

//#define printf(...)

//#define DBGLOG_HUB printf
#define DBGLOG_HUB(...)

//#define WRNLOG_HUB printf
#define WRNLOG_HUB(...)

//#define ERRLOG_HUB printf
#define ERRLOG_HUB(...)

/*Configure total pages for EEPROM emulation*/
#define FLASH_PAGE_FOR_EEPROM (8) /* PASS */
//#define FLASH_PAGE_FOR_EEPROM (4) /* FAIL */
//#define FLASH_PAGE_FOR_EEPROM (2) /* FAIL */

#define ADC_HSA_VALUE_HID0 (620)
#define ADC_HSA_VALUE_HID1 (880)
#define ADC_HSA_VALUE_HID2 (1210)
#define ADC_HSA_VALUE_HID3 (1600)
#define ADC_HSA_VALUE_HID4 (2030)
#define ADC_HSA_VALUE_HID5 (2470)
#define ADC_HSA_VALUE_HID6 (2840)
#define ADC_HSA_VALUE_HID7 (3170)
#define ADC_HSA_VALUE_OFFLINE (0)
#define ADC_HSA_VALUE_DEVIATION (125)

#define I3CS_CFG_CMD_QUEUE_EMPTY_THLD    (1UL)
#define I3CS_CFG_RESP_QUEUE_FULL_THLD    (1UL)

#define I3CS_READ_LEN_FLEXBILE  (1)

#define I3CS_SPD_WRITER_PREMATURE_TX_BUF_CNT (2) //Smart SPD Writer

//#define I3CS_PREMATURE_TX_BUF_CNT (64)
//#define I3CS_PREMATURE_TX_BUF_CNT (32)
//#define I3CS_PREMATURE_TX_BUF_CNT (1)
//#define I3CS_PREMATURE_TX_BUF_CNT (2) //Smart SPD Writer
//#define I3CS_PREMATURE_TX_BUF_CNT (4)
//#define I3CS_PREMATURE_TX_BUF_CNT (8)
#define I3CS_PREMATURE_TX_BUF_CNT (16)

#define WRITE_FLASH_AT_IRQ_HANDLER (0)
#define SPDH_DETECT_POWER_DOWN (1)


#define I3CS_RX_BUF_CNT  (64)
#define I3CS_RX_BUF_SIZE (32)//(128)//(32)

#define DEBUG_PIN (1)

int8_t Hub_Init(void);
int8_t Hub_ResetInit(uint32_t u32I3cAddr);
int8_t Hub_UpdateTempReg(void);
int8_t Temp_Init(void);
int8_t Hub_CheckInterfaceSel(void);
int8_t Hub_UpdateInterfaceSel(uint8_t u8InfSel);
int8_t Hub_SetLocalBusPullUp(uint32_t u32LsdaSel, uint32_t u32LsclSel);
int8_t Hub_CheckIBIReg(void);
int8_t Hub_GetBusRstEvt(void);
int8_t Hub_ClrBusRstEvt(void);

void Hub_SPDHIRQHandler(void);

void Hub_SelfTest(void);
int32_t PopFromRxFIFOBuf(void);
int32_t Hub_FIFO_ResetAndResume(I3CS_T *i3cs, uint32_t u32ResetMask, uint32_t u32EnableResume);
int32_t Hub_RespErrorRecovery(I3CS_T *i3cs, uint32_t u32RespStatus);

#ifdef __cplusplus
}
#endif

#endif /* HUBFW_H */
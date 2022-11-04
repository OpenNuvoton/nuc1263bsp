/**************************************************************************//**
 * @file     LocalDevFw.h
 * @version  V3.00
 * @brief    Local device control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __LOCALDEVFW_H__
#define __LOCALDEVFW_H__
#include "stdio.h"
#include "NuMicro.h"

#include "LocalDevReg.h"

#ifdef __cplusplus
extern "C"
{
#endif

//#define DBGLOG printf
#define DBGLOG(...)

//#define WRNLOG printf
#define WRNLOG(...)

//#define ERRLOG printf
#define ERRLOG(...)

#define I3CS_CFG_CMD_QUEUE_EMPTY_THLD    (1UL)
#define I3CS_CFG_RESP_QUEUE_FULL_THLD    (1UL)

#define I3CS_DEVICE_RESP_QUEUE_CNT       (2UL)


#define I3CS_GET_INTSTS(i3cs)                     ((i3cs)->INTSTS)
#define I3CS_GET_RXD(i3cs)                        ((i3cs)->TXRXDAT)


/**
  \brief  Union type to access the Command Queue Port.
 */
typedef union
{
    struct
    {
        uint32_t ATTR:3;            /*!< bit:  0.. 2  Command Attribute */
        uint32_t TID:3;             /*!< bit:  3.. 5  Transmit Transaction ID */
        uint32_t _reserved:10;      /*!< bit:  6..15  Reserved */
        uint32_t LENGTH:16;         /*!< bit: 16..31  Data Length (byte) */
    } b;                            /*!< Structure used for bit  access */
    uint32_t w;                     /*!< Type used for word access */
} CMD_QUEUE_T;

/**
  \brief  Union type to access the Response Queue Port.
 */
typedef union
{
    struct
    {
        uint32_t LENGTH:16;         /*!< bit:  0..15  Data Length (byte) */
        uint32_t HDRCCC:8;          /*!< bit: 16..23  HDR Command Code */
        uint32_t TID:3;             /*!< bit: 24..26  Transmit Transaction ID */
        uint32_t RXRSP:1;           /*!< bit:     27  Transaction Type */
        uint32_t STATUS:4;          /*!< bit: 28..31  Response Status */
    } b;                            /*!< Structure used for bit  access */
    uint32_t w;                     /*!< Type used for word access */
} RESP_QUEUE_T;


int8_t LocalDev_Init(uint8_t u8DevAddr);
int8_t LocalDev_CheckInterfaceSel(void);
int8_t LocalDev_UpdateInterfaceSel(uint8_t u8InfSel);
int8_t LocalDev_CheckIBIReg(void);
void LocalDev_SPDHIRQHandler(void);


#ifdef __cplusplus
}
#endif

#endif /* LOCALDEVFW_H */
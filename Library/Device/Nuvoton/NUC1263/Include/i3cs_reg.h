/**************************************************************************/ /**
* @file     i3cs_reg.h
* @version  V3.0
* @brief    I3C Slave register definition header file
*
* @copyright SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
******************************************************************************/
#ifndef __I3CS_REG_H__
#define __I3CS_REG_H__

#if defined(__CC_ARM)
#pragma anon_unions
#endif

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

    @{
*/

/*---------------------- I3CS Serial Interface Controller -------------------------*/
/**
    @addtogroup I3CS I3CS Serial Interface Controller(I3CS)
    Memory Mapped Structure for I3CS Controller
    @{
*/

typedef struct
{
    __IO uint32_t DEVCTL;      /*!< [0x0000] I3CS Device Control Register */
    __IO uint32_t DEVADDR;     /*!< [0x0004] I3CS Device Address Register */
    __I uint32_t HWCAP;        /*!< [0x0008] I3CS Hardware Capability Register */
    __O uint32_t CMDQUE;       /*!< [0x000c] I3CS Command Queue Port Register */
    __I uint32_t RESPQUE;      /*!< [0x0010] I3CS Response Queue Port Register */
    __IO uint32_t TXRXDAT;     /*!< [0x0014] I3CS Transmit and Receive Data Port Register */
    __I uint32_t RESERVE0;     /*!< [0x0018] Reserved */
    __IO uint32_t QUETHCTL;    /*!< [0x001c] I3CS Queue Threshold Control Register */
    __IO uint32_t DBTHCTL;     /*!< [0x0020] I3CS Data Buffer Threshold Control Register */
    __I uint32_t RESERVE1[4];  /*!< [0x0024 ~ 0x0030] Reserved */
    __IO uint32_t RSTCTL;      /*!< [0x0034] I3CS Reset Control Register */
    __IO uint32_t SLVEVNTS;    /*!< [0x0038] I3CS Slave Event Status Register */
    __IO uint32_t INTSTS;      /*!< [0x003c] I3CS Interrupt Status Register */
    __IO uint32_t INTSTSEN;    /*!< [0x0040] I3CS Interrupt Status Enable Register */
    __IO uint32_t INTEN;       /*!< [0x0044] I3CS Interrupt Signal Enable Register */
    __O uint32_t RESERVE2;     /*!< [0x0048] Reserved */
    __I uint32_t QUESTSLV;     /*!< [0x004c] I3CS Queue Status Level Register */
    __I uint32_t DBSTSLV;      /*!< [0x0050] I3CS Data Buffer Status Level Register */
    __I uint32_t PRESENTS;     /*!< [0x0054] I3CS Present State Register */
    __I uint32_t CCCDEVS;      /*!< [0x0058] I3CS Device Operating Status Register */
    __I uint32_t RESERVE3[5];  /*!< [0x005c ~ 0x006c] Reserved */
    __IO uint32_t SLVMID;      /*!< [0x0070] I3CS MIPI Manufacturer ID Register */
    __IO uint32_t SLVPID;      /*!< [0x0074] I3CS Normal Provisional ID Register */
    __IO uint32_t SLVCHAR;     /*!< [0x0078] I3CS Slave Characteristic Register */
    __I uint32_t SLVMXLEN;     /*!< [0x007c] I3CS Maximum Write/Read Length Register */
    __I uint32_t MXRDTURN;     /*!< [0x0080] I3CS Maximum Read Turnaround Time Register */
    __IO uint32_t MXDS;        /*!< [0x0084] I3CS Maximum Data Speed Register */
    __I uint32_t RESERVE4;     /*!< [0x0088] Reserved */
    __IO uint32_t SIR;         /*!< [0x008c] I3CS Slave Interrupt Request Register */
    __I uint32_t RESERVE5;     /*!< [0x0090 Reserved */
    __IO uint32_t SIRDAT;      /*!< [0x0094] I3CS Slave Interrupt Request Data Register */
    __IO uint32_t SIRRESP;     /*!< [0x0098] I3CS Slave Interrupt Request Response Register */
    __I uint32_t RESERVE6[14]; /*!< [0x009c ~ 0x00d0] Reserved */
    __IO uint32_t BUSFAT;      /*!< [0x00d4] I3CS Bus Free And Available Timing Register */
    __IO uint32_t BUSIDLET;    /*!< [0x00d8] I3CS Bus Idle Timing Register */

} I3CS_T;

/**
    @addtogroup I3CS_CONST I3CS Bit Field Definition
    Constant Definitions for I3CS Controller
    @{
*/

/* DEVCTL -------------------------------------------------------------------*/
#define I3CS_DEVCTL_PENDINT_Pos    3U                                         /*!< I3CS_T::DEVCTL: PENDINT Position */
#define I3CS_DEVCTL_PENDINT_Msk    (0xFUL << I3CS_DEVCTL_PENDINT_Pos)         /*!< I3CS_T::DEVCTL: PENDINT Mask     */

#define I3CS_DEVCTL_IDLECNT_Pos    24U                                        /*!< I3CS_T::DEVCTL: IDLECNT Position */
#define I3CS_DEVCTL_IDLECNT_Msk    (0x3UL << I3CS_DEVCTL_IDLECNT_Pos)         /*!< I3CS_T::DEVCTL: IDLECNT Mask     */

#define I3CS_DEVCTL_SYNC_Pos       26U                                        /*!< I3CS_T::DEVCTL: SYNC Position */
#define I3CS_DEVCTL_SYNC_Msk       (1UL << I3CS_DEVCTL_SYNC_Pos)              /*!< I3CS_T::DEVCTL: SYNC Mask     */

#define I3CS_DEVCTL_ADAPTIVE_Pos   27U                                        /*!< I3CS_T::DEVCTL: ADAPTIVE Position */
#define I3CS_DEVCTL_ADAPTIVE_Msk   (1UL << I3CS_DEVCTL_ADAPTIVE_Pos)          /*!< I3CS_T::DEVCTL: ADAPTIVE Mask     */

#define I3CS_DEVCTL_DMAEN_Pos      28U                                        /*!< I3CS_T::DEVCTL: DMAEN Position */
#define I3CS_DEVCTL_DMAEN_Msk      (1UL << I3CS_DEVCTL_DMAEN_Pos)             /*!< I3CS_T::DEVCTL: DMAEN Mask     */

#define I3CS_DEVCTL_RESUME_Pos     30U                                        /*!< I3CS_T::DEVCTL: RESUME Position */
#define I3CS_DEVCTL_RESUME_Msk     (1UL << I3CS_DEVCTL_RESUME_Pos)            /*!< I3CS_T::DEVCTL: RESUME Mask     */

#define I3CS_DEVCTL_ENABLE_Pos     31U                                        /*!< I3CS_T::DEVCTL: ENABLE Position */
#define I3CS_DEVCTL_ENABLE_Msk     (1UL << I3CS_DEVCTL_ENABLE_Pos)            /*!< I3CS_T::DEVCTL: ENABLE Mask     */

/* DEVADDR ------------------------------------------------------------------*/
#define I3CS_DEVADDR_SA_Pos        0U                                         /*!< I3CS_T::DEVADDR: SA Position */
#define I3CS_DEVADDR_SA_Msk        (0x7FUL << I3CS_DEVADDR_SA_Pos)            /*!< I3CS_T::DEVADDR: SA Mask     */

#define I3CS_DEVADDR_SAVALID_Pos   15U                                        /*!< I3CS_T::DEVADDR: SAVALID Position */
#define I3CS_DEVADDR_SAVALID_Msk   (1UL << I3CS_DEVADDR_SAVALID_Pos)          /*!< I3CS_T::DEVADDR: SAVALID Mask     */

#define I3CS_DEVADDR_DA_Pos        16U                                        /*!< I3CS_T::DEVADDR: DA Position */
#define I3CS_DEVADDR_DA_Msk        (0x7FUL << I3CS_DEVADDR_DA_Pos)            /*!< I3CS_T::DEVADDR: DA Mask     */

#define I3CS_DEVADDR_DAVALID_Pos   31U                                        /*!< I3CS_T::DEVADDR: DAVALID Position */
#define I3CS_DEVADDR_DAVALID_Msk   (1UL << I3CS_DEVADDR_DAVALID_Pos)          /*!< I3CS_T::DEVADDR: DAVALID Mask     */

/* CMDQUE -------------------------------------------------------------------*/
#define I3CS_CMDQUE_ATTR_Pos       0U                                         /*!< I3CS_T::CMDQUE: ATTR Position */
#define I3CS_CMDQUE_ATTR_Msk       (0x7UL << I3CS_CMDQUE_ATTR_Pos)            /*!< I3CS_T::CMDQUE: ATTR Mask     */

#define I3CS_CMDQUE_TID_Pos        3U                                         /*!< I3CS_T::CMDQUE: TID Position */
#define I3CS_CMDQUE_TID_Msk        (0x7UL << I3CS_CMDQUE_TID_Pos)             /*!< I3CS_T::CMDQUE: TID Mask     */

#define I3CS_CMDQUE_DATLEN_Pos     16U                                        /*!< I3CS_T::CMDQUE: DATLEN Position */
#define I3CS_CMDQUE_DATLEN_Msk     (0xFFFFUL << I3CS_CMDQUE_DATLEN_Pos)       /*!< I3CS_T::CMDQUE: DATLEN Mask     */

/* RESPQUE ------------------------------------------------------------------*/
#define I3CS_RESPQUE_DATLEN_Pos    0U                                         /*!< I3CS_T::RESPQUE: DATLEN Position */
#define I3CS_RESPQUE_DATLEN_Msk    (0xFFFFUL << I3CS_RESPQUE_DATLEN_Pos)      /*!< I3CS_T::RESPQUE: DATLEN Mask     */

#define I3CS_RESPQUE_HDRCODE_Pos   16U                                        /*!< I3CS_T::RESPQUE: HDRCODE Position */
#define I3CS_RESPQUE_HDRCODE_Msk   (0xFFUL << I3CS_RESPQUE_HDRCODE_Pos)       /*!< I3CS_T::RESPQUE: HDRCODE Mask     */

#define I3CS_RESPQUE_TID_Pos       24U                                        /*!< I3CS_T::RESPQUE: TID Position */
#define I3CS_RESPQUE_TID_Msk       (0x7UL << I3CS_RESPQUE_TID_Pos)            /*!< I3CS_T::RESPQUE: TID Mask     */

#define I3CS_RESPQUE_RXRESP_Pos    27U                                        /*!< I3CS_T::RESPQUE: RXRESP Position */
#define I3CS_RESPQUE_RXRESP_Msk    (1UL << I3CS_RESPQUE_RXRESP_Pos)           /*!< I3CS_T::RESPQUE: RXRESP Mask     */

#define I3CS_RESPQUE_ERRSTS_Pos    28U                                        /*!< I3CS_T::RESPQUE: ERRSTS Position */
#define I3CS_RESPQUE_ERRSTS_Msk    (0xFUL << I3CS_RESPQUE_ERRSTS_Pos)         /*!< I3CS_T::RESPQUE: ERRSTS Mask     */

/* TXRXDAT ------------------------------------------------------------------*/
#define I3CS_TXRXDAT_DAT_Pos       0U                                         /*!< I3CS_T::TXRXDAT: DAT Position */
#define I3CS_TXRXDAT_DAT_Msk       (0xFFFFFFFFUL << I3CS_TXRXDAT_DAT_Pos)     /*!< I3CS_T::TXRXDAT: DAT Mask     */

/* QUETHCTL -----------------------------------------------------------------*/
#define I3CS_QUETHCTL_CMDETH_Pos   0U                                         /*!< I3CS_T::QUETHCTL: CMDETH Position */
#define I3CS_QUETHCTL_CMDETH_Msk   (0xFFUL << I3CS_QUETHCTL_CMDETH_Pos)       /*!< I3CS_T::QUETHCTL: CMDETH Mask     */

#define I3CS_QUETHCTL_RESPTH_Pos   8U                                         /*!< I3CS_T::QUETHCTL: RESPTH Position */
#define I3CS_QUETHCTL_RESPTH_Msk   (0xFFUL << I3CS_QUETHCTL_RESPTH_Pos)       /*!< I3CS_T::QUETHCTL: RESPTH Mask     */

/* DBTHCTL ------------------------------------------------------------------*/
#define I3CS_DBTHCTL_TXTH_Pos      0U                                         /*!< I3CS_T::DBTHCTL: TXTH Position */
#define I3CS_DBTHCTL_TXTH_Msk      (0x7UL << I3CS_DBTHCTL_TXTH_Pos)           /*!< I3CS_T::DBTHCTL: TXTH Mask     */

#define I3CS_DBTHCTL_RXTH_Pos      8U                                         /*!< I3CS_T::DBTHCTL: RXTH Position */
#define I3CS_DBTHCTL_RXTH_Msk      (0x7UL << I3CS_DBTHCTL_RXTH_Pos)           /*!< I3CS_T::DBTHCTL: RXTH Mask     */

#define I3CS_DBTHCTL_TXSTATH_Pos   16U                                        /*!< I3CS_T::DBTHCTL: TXSTATH Position */
#define I3CS_DBTHCTL_TXSTATH_Msk   (0x7UL << I3CS_DBTHCTL_TXSTATH_Pos)        /*!< I3CS_T::DBTHCTL: TXSTATH Mask     */

#define I3CS_DBTHCTL_RXSTATH_Pos   24U                                        /*!< I3CS_T::DBTHCTL: RXSTATH Position */
#define I3CS_DBTHCTL_RXSTATH_Msk   (0x7UL << I3CS_DBTHCTL_RXSTATH_Pos)        /*!< I3CS_T::DBTHCTL: RXSTATH Mask     */

/* RSTCTL -------------------------------------------------------------------*/
#define I3CS_RSTCTL_SWRST_Pos      0U                                         /*!< I3CS_T::RSTCTL: SWRST Position */
#define I3CS_RSTCTL_SWRST_Msk      (1UL << I3CS_RSTCTL_SWRST_Pos)             /*!< I3CS_T::RSTCTL: SWRST Mask     */

#define I3CS_RSTCTL_CMDRST_Pos     1U                                         /*!< I3CS_T::RSTCTL: CMDRST Position */
#define I3CS_RSTCTL_CMDRST_Msk     (1UL << I3CS_RSTCTL_CMDRST_Pos)            /*!< I3CS_T::RSTCTL: CMDRST Mask     */

#define I3CS_RSTCTL_RESPRST_Pos    2U                                         /*!< I3CS_T::RSTCTL: RESPRST Position */
#define I3CS_RSTCTL_RESPRST_Msk    (1UL << I3CS_RSTCTL_RESPRST_Pos)           /*!< I3CS_T::RSTCTL: RESPRST Mask     */

#define I3CS_RSTCTL_TXRST_Pos      3U                                         /*!< I3CS_T::RSTCTL: TXRST Position */
#define I3CS_RSTCTL_TXRST_Msk      (1UL << I3CS_RSTCTL_TXRST_Pos)             /*!< I3CS_T::RSTCTL: TXRST Mask     */

#define I3CS_RSTCTL_RXRST_Pos      4U                                         /*!< I3CS_T::RSTCTL: RXRST Position */
#define I3CS_RSTCTL_RXRST_Msk      (1UL << I3CS_RSTCTL_RXRST_Pos)             /*!< I3CS_T::RSTCTL: RXRST Mask     */

/* SLVEVNTS -----------------------------------------------------------------*/
#define I3CS_SLVEVNTS_SIREN_Pos    0U                                         /*!< I3CS_T::SLVEVNTS: SIREN Position */
#define I3CS_SLVEVNTS_SIREN_Msk    (1UL << I3CS_SLVEVNTS_SIREN_Pos)           /*!< I3CS_T::SLVEVNTS: SIREN Mask     */

#define I3CS_SLVEVNTS_HJEN_Pos     3U                                         /*!< I3CS_T::SLVEVNTS: HJEN Position */
#define I3CS_SLVEVNTS_HJEN_Msk     (1UL << I3CS_SLVEVNTS_HJEN_Pos)            /*!< I3CS_T::SLVEVNTS: HJEN Mask     */

#define I3CS_SLVEVNTS_ACTSTATE_Pos 4U                                         /*!< I3CS_T::SLVEVNTS: ACTSTATE Position */
#define I3CS_SLVEVNTS_ACTSTATE_Msk (3UL << I3CS_SLVEVNTS_ACTSTATE_Pos)        /*!< I3CS_T::SLVEVNTS: ACTSTATE Mask     */

#define I3CS_SLVEVNTS_MRLUPD_Pos   6U                                         /*!< I3CS_T::SLVEVNTS: MRLUPD Position */
#define I3CS_SLVEVNTS_MRLUPD_Msk   (1UL << I3CS_SLVEVNTS_MRLUPD_Pos)          /*!< I3CS_T::SLVEVNTS: MRLUPD Mask     */

#define I3CS_SLVEVNTS_MWLUPD_Pos   7U                                         /*!< I3CS_T::SLVEVNTS: MWLUPD Position */
#define I3CS_SLVEVNTS_MWLUPD_Msk   (1UL << I3CS_SLVEVNTS_MWLUPD_Pos)          /*!< I3CS_T::SLVEVNTS: MWLUPD Mask     */

/* INTSTS -------------------------------------------------------------------*/
#define I3CS_INTSTS_TXTH_Pos       0U                                         /*!< I3CS_T::INTSTS: TXTH Position */
#define I3CS_INTSTS_TXTH_Msk       (1UL << I3CS_INTSTS_TXTH_Pos)              /*!< I3CS_T::INTSTS: TXTH Mask     */

#define I3CS_INTSTS_RXTH_Pos       1U                                         /*!< I3CS_T::INTSTS: RXTH Position */
#define I3CS_INTSTS_RXTH_Msk       (1UL << I3CS_INTSTS_RXTH_Pos)              /*!< I3CS_T::INTSTS: RXTH Mask     */

#define I3CS_INTSTS_CMDRDY_Pos     3U                                         /*!< I3CS_T::INTSTS: CMDRDY Position */
#define I3CS_INTSTS_CMDRDY_Msk     (1UL << I3CS_INTSTS_CMDRDY_Pos)            /*!< I3CS_T::INTSTS: CMDRDY Mask     */

#define I3CS_INTSTS_RESPRDY_Pos    4U                                         /*!< I3CS_T::INTSTS: RESPRDY Position */
#define I3CS_INTSTS_RESPRDY_Msk    (1UL << I3CS_INTSTS_RESPRDY_Pos)           /*!< I3CS_T::INTSTS: RESPRDY Mask     */

#define I3CS_INTSTS_CCCUPD_Pos     6U                                         /*!< I3CS_T::INTSTS: CCCUPD Position */
#define I3CS_INTSTS_CCCUPD_Msk     (1UL << I3CS_INTSTS_CCCUPD_Pos)            /*!< I3CS_T::INTSTS: CCCUPD Mask     */

#define I3CS_INTSTS_DAA_Pos        8U                                         /*!< I3CS_T::INTSTS: DAA Position */
#define I3CS_INTSTS_DAA_Msk        (1UL << I3CS_INTSTS_DAA_Pos)               /*!< I3CS_T::INTSTS: DAA Mask     */

#define I3CS_INTSTS_TFRERR_Pos     9U                                         /*!< I3CS_T::INTSTS: TFRERR Position */
#define I3CS_INTSTS_TFRERR_Msk     (1UL << I3CS_INTSTS_TFRERR_Pos)            /*!< I3CS_T::INTSTS: TFRERR Mask     */

#define I3CS_INTSTS_READREQ_Pos    11U                                        /*!< I3CS_T::INTSTS: READREQ Position */
#define I3CS_INTSTS_READREQ_Msk    (1UL << I3CS_INTSTS_READREQ_Pos)           /*!< I3CS_T::INTSTS: READREQ Mask     */

#define I3CS_INTSTS_IBIUPD_Pos     12U                                        /*!< I3CS_T::INTSTS: IBIUPD Position */
#define I3CS_INTSTS_IBIUPD_Msk     (1UL << I3CS_INTSTS_IBIUPD_Pos)            /*!< I3CS_T::INTSTS: IBIUPD Mask     */

/* INTSTSEN -----------------------------------------------------------------*/
#define I3CS_INTSTSEN_TXTH_Pos     0U                                         /*!< I3CS_T::INTSTSEN: TXTH Position */
#define I3CS_INTSTSEN_TXTH_Msk     (1UL << I3CS_INTSTSEN_TXTH_Pos)            /*!< I3CS_T::INTSTSEN: TXTH Mask     */

#define I3CS_INTSTSEN_RXTH_Pos     1U                                         /*!< I3CS_T::INTSTSEN: RXTH Position */
#define I3CS_INTSTSEN_RXTH_Msk     (1UL << I3CS_INTSTSEN_RXTH_Pos)            /*!< I3CS_T::INTSTSEN: RXTH Mask     */

#define I3CS_INTSTSEN_CMDRDY_Pos   3U                                         /*!< I3CS_T::INTSTSEN: CMDRDY Position */
#define I3CS_INTSTSEN_CMDRDY_Msk   (1UL << I3CS_INTSTSEN_CMDRDY_Pos)          /*!< I3CS_T::INTSTSEN: CMDRDY Mask     */

#define I3CS_INTSTSEN_RESPRDY_Pos  4U                                         /*!< I3CS_T::INTSTSEN: RESPRDY Position */
#define I3CS_INTSTSEN_RESPRDY_Msk  (1UL << I3CS_INTSTSEN_RESPRDY_Pos)         /*!< I3CS_T::INTSTSEN: RESPRDY Mask     */

#define I3CS_INTSTSEN_CCCUPD_Pos   6U                                         /*!< I3CS_T::INTSTSEN: CCCUPD Position */
#define I3CS_INTSTSEN_CCCUPD_Msk   (1UL << I3CS_INTSTSEN_CCCUPD_Pos)          /*!< I3CS_T::INTSTSEN: CCCUPD Mask     */

#define I3CS_INTSTSEN_DAA_Pos      8U                                         /*!< I3CS_T::INTSTSEN: DAA Position */
#define I3CS_INTSTSEN_DAA_Msk      (1UL << I3CS_INTSTSEN_DAA_Pos)             /*!< I3CS_T::INTSTSEN: DAA Mask     */

#define I3CS_INTSTSEN_TFRERR_Pos   9U                                         /*!< I3CS_T::INTSTSEN: TFRERR Position */
#define I3CS_INTSTSEN_TFRERR_Msk   (1UL << I3CS_INTSTSEN_TFRERR_Pos)          /*!< I3CS_T::INTSTSEN: TFRERR Mask     */

#define I3CS_INTSTSEN_READREQ_Pos  11U                                        /*!< I3CS_T::INTSTSEN: READREQ Position */
#define I3CS_INTSTSEN_READREQ_Msk  (1UL << I3CS_INTSTSEN_READREQ_Pos)         /*!< I3CS_T::INTSTSEN: READREQ Mask     */

#define I3CS_INTSTSEN_IBIUPD_Pos   12U                                        /*!< I3CS_T::INTSTSEN: IBIUPD Position */
#define I3CS_INTSTSEN_IBIUPD_Msk   (1UL << I3CS_INTSTSEN_IBIUPD_Pos)          /*!< I3CS_T::INTSTSEN: IBIUPD Mask     */

/* INTEN --------------------------------------------------------------------*/
#define I3CS_INTEN_TXTH_Pos        0U                                         /*!< I3CS_T::INTEN: TXTH Position */
#define I3CS_INTEN_TXTH_Msk        (1UL << I3CS_INTEN_TXTH_Pos)               /*!< I3CS_T::INTEN: TXTH Mask     */

#define I3CS_INTEN_RXTH_Pos        1U                                         /*!< I3CS_T::INTEN: RXTH Position */
#define I3CS_INTEN_RXTH_Msk        (1UL << I3CS_INTEN_RXTH_Pos)               /*!< I3CS_T::INTEN: RXTH Mask     */

#define I3CS_INTEN_CMDRDY_Pos      3U                                         /*!< I3CS_T::INTEN: CMDRDY Position */
#define I3CS_INTEN_CMDRDY_Msk      (1UL << I3CS_INTEN_CMDRDY_Pos)             /*!< I3CS_T::INTEN: CMDRDY Mask     */

#define I3CS_INTEN_RESPRDY_Pos     4U                                         /*!< I3CS_T::INTEN: RESPRDY Position */
#define I3CS_INTEN_RESPRDY_Msk     (1UL << I3CS_INTEN_RESPRDY_Pos)            /*!< I3CS_T::INTEN: RESPRDY Mask     */

#define I3CS_INTEN_CCCUPD_Pos      6U                                         /*!< I3CS_T::INTEN: CCCUPD Position */
#define I3CS_INTEN_CCCUPD_Msk      (1UL << I3CS_INTEN_CCCUPD_Pos)             /*!< I3CS_T::INTEN: CCCUPD Mask     */

#define I3CS_INTEN_DAA_Pos         8U                                         /*!< I3CS_T::INTEN: DAA Position */
#define I3CS_INTEN_DAA_Msk         (1UL << I3CS_INTEN_DAA_Pos)                /*!< I3CS_T::INTEN: DAA Mask     */

#define I3CS_INTEN_TFRERR_Pos      9U                                         /*!< I3CS_T::INTEN: TFRERR Position */
#define I3CS_INTEN_TFRERR_Msk      (1UL << I3CS_INTEN_TFRERR_Pos)             /*!< I3CS_T::INTEN: TFRERR Mask     */

#define I3CS_INTEN_READREQ_Pos     11U                                        /*!< I3CS_T::INTEN: READREQ Position */
#define I3CS_INTEN_READREQ_Msk     (1UL << I3CS_INTEN_READREQ_Pos)            /*!< I3CS_T::INTEN: READREQ Mask     */

#define I3CS_INTEN_IBIUPD_Pos      12U                                        /*!< I3CS_T::INTEN: IBIUPD Position */
#define I3CS_INTEN_IBIUPD_Msk      (1UL << I3CS_INTEN_IBIUPD_Pos)             /*!< I3CS_T::INTEN: IBIUPD Mask     */

/* QUESTSLV -----------------------------------------------------------------*/
#define I3CS_QUESTSLV_CMDELOC_Pos  0U                                         /*!< I3CS_T::QUESTSLV: CMDELOC Position */
#define I3CS_QUESTSLV_CMDELOC_Msk  (0xFFUL << I3CS_QUESTSLV_CMDELOC_Pos)      /*!< I3CS_T::QUESTSLV: CMDELOC Mask     */

#define I3CS_QUESTSLV_RESPLV_Pos   8U                                         /*!< I3CS_T::QUESTSLV: RESPLV Position */
#define I3CS_QUESTSLV_RESPLV_Msk   (0xFFUL << I3CS_QUESTSLV_RESPLV_Pos)       /*!< I3CS_T::QUESTSLV: RESPLV Mask     */

/* DBSTSLV ------------------------------------------------------------------*/
#define I3CS_DBSTSLV_TXELV_Pos     0U                                         /*!< I3CS_T::DBSTSLV: TXELV Position */
#define I3CS_DBSTSLV_TXELV_Msk     (0xFFUL << I3CS_DBSTSLV_TXELV_Pos)         /*!< I3CS_T::DBSTSLV: TXELV Mask     */

#define I3CS_DBSTSLV_RXLV_Pos      16U                                        /*!< I3CS_T::DBSTSLV: RXLV Position */
#define I3CS_DBSTSLV_RXLV_Msk      (0xFFUL << I3CS_DBSTSLV_RXLV_Pos)          /*!< I3CS_T::DBSTSLV: RXLV Mask     */

/* PRESENTS -----------------------------------------------------------------*/
#define I3CS_PRESENTS_TFRTYPE_Pos  8U                                         /*!< I3CS_T::PRESENTS: TFRTYPE Position */
#define I3CS_PRESENTS_TFRTYPE_Msk  (0x3FUL << I3CS_PRESENTS_TFRTYPE_Pos)      /*!< I3CS_T::PRESENTS: TFRTYPE Mask     */

#define I3CS_PRESENTS_TID_Pos      24U                                        /*!< I3CS_T::PRESENTS: TID Position */
#define I3CS_PRESENTS_TID_Msk      (0xFUL << I3CS_PRESENTS_TID_Pos)           /*!< I3CS_T::PRESENTS: TID Mask     */

/* CCCDEVS ------------------------------------------------------------------*/
#define I3CS_CCCDEVS_PENDINT_Pos   0U                                         /*!< I3CS_T::CCCDEVS: PENDINT Position */
#define I3CS_CCCDEVS_PENDINT_Msk   (0xFUL << I3CS_CCCDEVS_PENDINT_Pos)        /*!< I3CS_T::CCCDEVS: PENDINT Mask     */

#define I3CS_CCCDEVS_PROTERR_Pos   5U                                         /*!< I3CS_T::CCCDEVS: PROTERR Position */
#define I3CS_CCCDEVS_PROTERR_Msk   (1UL << I3CS_CCCDEVS_PROTERR_Pos)          /*!< I3CS_T::CCCDEVS: PROTERR Mask     */

#define I3CS_CCCDEVS_ACTMODE_Pos   6U                                         /*!< I3CS_T::CCCDEVS: ACTMODE Position */
#define I3CS_CCCDEVS_ACTMODE_Msk   (3UL << I3CS_CCCDEVS_ACTMODE_Pos)          /*!< I3CS_T::CCCDEVS: ACTMODE Mask     */

#define I3CS_CCCDEVS_UDFERR_Pos    8U                                         /*!< I3CS_T::CCCDEVS: UDFERR Position */
#define I3CS_CCCDEVS_UDFERR_Msk    (1UL << I3CS_CCCDEVS_UDFERR_Pos)           /*!< I3CS_T::CCCDEVS: UDFERR Mask     */

#define I3CS_CCCDEVS_SLVBUSY_Pos   9U                                         /*!< I3CS_T::CCCDEVS: SLVBUSY Position */
#define I3CS_CCCDEVS_SLVBUSY_Msk   (1UL << I3CS_CCCDEVS_SLVBUSY_Pos)          /*!< I3CS_T::CCCDEVS: SLVBUSY Mask     */

#define I3CS_CCCDEVS_OVFERR_Pos    10U                                        /*!< I3CS_T::CCCDEVS: OVFERR Position */
#define I3CS_CCCDEVS_OVFERR_Msk    (1UL << I3CS_CCCDEVS_OVFERR_Pos)           /*!< I3CS_T::CCCDEVS: OVFERR Mask     */

#define I3CS_CCCDEVS_DATNRDY_Pos   11U                                        /*!< I3CS_T::CCCDEVS: DATNRDY Position */
#define I3CS_CCCDEVS_DATNRDY_Msk   (1UL << I3CS_CCCDEVS_DATNRDY_Pos)          /*!< I3CS_T::CCCDEVS: DATNRDY Mask     */

#define I3CS_CCCDEVS_BFNAVAIL_Pos  12U                                        /*!< I3CS_T::CCCDEVS: BFNAVAIL Position */
#define I3CS_CCCDEVS_BFNAVAIL_Msk  (1UL << I3CS_CCCDEVS_BFNAVAIL_Pos)         /*!< I3CS_T::CCCDEVS: BFNAVAIL Mask     */

#define I3CS_CCCDEVS_FRAMEERR_Pos  13U                                        /*!< I3CS_T::CCCDEVS: FRAMEERR Position */
#define I3CS_CCCDEVS_FRAMEERR_Msk  (1UL << I3CS_CCCDEVS_FRAMEERR_Pos)         /*!< I3CS_T::CCCDEVS: FRAMEERR Mask     */

/* SLVMID -------------------------------------------------------------------*/
#define I3CS_SLVMID_PIDTYPE_Pos    0U                                         /*!< I3CS_T::SLVMID: PIDTYPE Position */
#define I3CS_SLVMID_PIDTYPE_Msk    (1UL << I3CS_SLVMID_PIDTYPE_Pos)           /*!< I3CS_T::SLVMID: PIDTYPE Mask     */

#define I3CS_SLVMID_MID_Pos        1U                                         /*!< I3CS_T::SLVMID: MID Position */
#define I3CS_SLVMID_MID_Msk        (0x7FFFUL << I3CS_SLVMID_MID_Pos)          /*!< I3CS_T::SLVMID: MID Mask     */

/* SLVPID -------------------------------------------------------------------*/
#define I3CS_SLVPID_DCR_Pos        0U                                         /*!< I3CS_T::SLVPID: DCR Position */
#define I3CS_SLVPID_DCR_Msk        (0xFFFUL << I3CS_SLVPID_DCR_Pos)           /*!< I3CS_T::SLVPID: DCR Mask     */

#define I3CS_SVLPID_INSTID_Pos     12U                                        /*!< I3CS_T::SLVPID: INSTID Position */
#define I3CS_SVLPID_INSTID_Msk     (0xFUL << I3CS_SVLPID_INSTID_Pos)          /*!< I3CS_T::SLVPID: INSTID Mask     */

#define I3CS_SLVPID_PARTID_Pos     16U                                        /*!< I3CS_T::SLVPID: PARTID Position */
#define I3CS_SLVPID_PARTID_Msk     (0xFFFFUL << I3CS_SLVPID_PARTID_Pos)       /*!< I3CS_T::SLVPID: PARTID Mask     */

/* SLVCHAR ------------------------------------------------------------------*/
#define I3CS_SLVCHAR_MAXDSLIM_Pos  0U                                         /*!< I3CS_T::SLVCHAR: MAXDSLIM Position */
#define I3CS_SLVCHAR_MAXDSLIM_Msk  (1UL << I3CS_SLVCHAR_MAXDSLIM_Pos)         /*!< I3CS_T::SLVCHAR: MAXDSLIM Mask     */

#define I3CS_SLVCHAR_IBICAP_Pos    1U                                         /*!< I3CS_T::SLVCHAR: IBICAP Position */
#define I3CS_SLVCHAR_IBICAP_Msk    (1UL << I3CS_SLVCHAR_IBICAP_Pos)           /*!< I3CS_T::SLVCHAR: IBICAP Mask     */

#define I3CS_SLVCHAR_IBIPL_Pos     2U                                         /*!< I3CS_T::SLVCHAR: IBIPL Position */
#define I3CS_SLVCHAR_IBIPL_Msk     (1UL << I3CS_SLVCHAR_IBIPL_Pos)            /*!< I3CS_T::SLVCHAR: IBIPL Mask     */

#define I3CS_SLVCHAR_OLCAP_Pos     3U                                         /*!< I3CS_T::SLVCHAR: OLCAP Position */
#define I3CS_SLVCHAR_OLCAP_Msk     (1UL << I3CS_SLVCHAR_OLCAP_Pos)            /*!< I3CS_T::SLVCHAR: OLCAP Mask     */

#define I3CS_SLVCHAR_BRIDGEID_Pos  4U                                         /*!< I3CS_T::SLVCHAR: BRIDGEID Position */
#define I3CS_SLVCHAR_BRIDGEID_Msk  (1UL << I3CS_SLVCHAR_BRIDGEID_Pos)         /*!< I3CS_T::SLVCHAR: BRIDGEID Mask     */

#define I3CS_SLVCHAR_HDRCAP_Pos    5U                                         /*!< I3CS_T::SLVCHAR: HDRCAP Position */
#define I3CS_SLVCHAR_HDRCAP_Msk    (1UL << I3CS_SLVCHAR_HDRCAP_Pos)           /*!< I3CS_T::SLVCHAR: HDRCAP Mask     */

#define I3CS_SLVCHAR_DEVROLE_Pos   6U                                         /*!< I3CS_T::SLVCHAR: DEVROLE Position */
#define I3CS_SLVCHAR_DEVROLE_Msk   (0x3UL << I3CS_SLVCHAR_DEVROLE_Pos)        /*!< I3CS_T::SLVCHAR: DEVROLE Mask     */

#define I3CS_SLVCHAR_DCR_Pos       8U                                         /*!< I3CS_T::SLVCHAR: DCR Position */
#define I3CS_SLVCHAR_DCR_Msk       (0xFFUL << I3CS_SLVCHAR_DCR_Pos)           /*!< I3CS_T::SLVCHAR: DCR Mask     */

#define I3CS_SLVCHAR_HDRCPVAL_Pos  16U                                        /*!< I3CS_T::SLVCHAR: HDRCPVAL Position */
#define I3CS_SLVCHAR_HDRCPVAL_Msk  (0xFFUL << I3CS_SLVCHAR_HDRCPVAL_Pos)      /*!< I3CS_T::SLVCHAR: HDRCPVAL Mask     */

/* SLVMXLEN -----------------------------------------------------------------*/
#define I3CS_SLVMXLEN_MWL_Pos      0U                                         /*!< I3CS_T::SLVMXLEN: MWL Position */
#define I3CS_SLVMXLEN_MWL_Msk      (0xFFFFUL << I3CS_SLVMXLEN_MWL_Pos)        /*!< I3CS_T::SLVMXLEN: MWL Mask     */

#define I3CS_SLVMXLEN_MRL_Pos      16U                                        /*!< I3CS_T::SLVMXLEN: MRL Position */
#define I3CS_SLVMXLEN_MRL_Msk      (0xFFFFUL << I3CS_SLVMXLEN_MRL_Pos)        /*!< I3CS_T::SLVMXLEN: MRL Mask     */

/* MXRDTURN -----------------------------------------------------------------*/
#define I3CS_MXRDTURN_MXRDTURN_Pos 0U                                         /*!< I3CS_T::MXRDTURN: MXRDTURN Position */
#define I3CS_MXRDTURN_MXRDTURN_Msk (0xFFFFFFUL << I3CS_MXRDTURN_MXRDTURN_Pos) /*!< I3CS_T::MXRDTURN: MXRDTURN Mask     */

/* MXDS ---------------------------------------------------------------------*/
#define I3CS_MXDS_MXWR_Pos         0U                                         /*!< I3CS_T::MXDS: MXWR Position */
#define I3CS_MXDS_MXWR_Msk         (0x7UL << I3CS_MXDS_MXWR_Pos)              /*!< I3CS_T::MXDS: MXWR Mask     */

#define I3CS_MXDS_MXRD_Pos         8U                                         /*!< I3CS_T::MXDS: MXRD Position */
#define I3CS_MXDS_MXRD_Msk         (0x7UL << I3CS_MXDS_MXRD_Pos)              /*!< I3CS_T::MXDS: MXRD Mask     */

#define I3CS_MXDS_MXDTURN_Pos      16U                                        /*!< I3CS_T::MXDS: MXDTURN Position */
#define I3CS_MXDS_MXDTURN_Msk      (0x7UL << I3CS_MXDS_MXDTURN_Pos)           /*!< I3CS_T::MXDS: MXDTURN Mask     */

/* SIR ----------------------------------------------------------------------*/
#define I3CS_SIR_EN_Pos            0U                                         /*!< I3CS_T::SIR: EN Position */
#define I3CS_SIR_EN_Msk            (1UL << I3CS_SIR_EN_Pos)                   /*!< I3CS_T::SIR: EN Mask     */

#define I3CS_SIR_CTL_Pos           1U                                         /*!< I3CS_T::SIR: CTL Position */
#define I3CS_SIR_CTL_Msk           (0x3UL << I3CS_SIR_CTL_Pos)                /*!< I3CS_T::SIR: CTL Mask     */

#define I3CS_SIR_MDB_Pos           8U                                         /*!< I3CS_T::SIR: MDB Position */
#define I3CS_SIR_MDB_Msk           (0xFFUL << I3CS_SIR_MDB_Pos)               /*!< I3CS_T::SIR: MDB Mask     */

#define I3CS_SIR_DATLEN_Pos        16U                                        /*!< I3CS_T::SIR: DATLEN Position */
#define I3CS_SIR_DATLEN_Msk        (0xFFUL << I3CS_SIR_DATLEN_Pos)            /*!< I3CS_T::SIR: DATLEN Mask     */

/* SIRDAT -------------------------------------------------------------------*/
#define I3CS_SIRDAT_DAT0_Pos       0U                                         /*!< I3CS_T::SIRDAT: DAT0 Position */
#define I3CS_SIRDAT_DAT0_Msk       (0xFFUL << I3CS_SIRDAT_DAT0_Pos)           /*!< I3CS_T::SIRDAT: DAT0 Mask     */

#define I3CS_SIRDAT_DAT1_Pos       8U                                         /*!< I3CS_T::SIRDAT: DAT1 Position */
#define I3CS_SIRDAT_DAT1_Msk       (0xFFUL << I3CS_SIRDAT_DAT1_Pos)           /*!< I3CS_T::SIRDAT: DAT1 Mask     */

#define I3CS_SIRDAT_DAT2_Pos       16U                                        /*!< I3CS_T::SIRDAT: DAT2 Position */
#define I3CS_SIRDAT_DAT2_Msk       (0xFFUL << I3CS_SIRDAT_DAT2_Pos)           /*!< I3CS_T::SIRDAT: DAT2 Mask     */

#define I3CS_SIRDAT_DAT3_Pos       24U                                        /*!< I3CS_T::SIRDAT: DAT3 Position */
#define I3CS_SIRDAT_DAT3_Msk       (0xFFUL << I3CS_SIRDAT_DAT3_Pos)           /*!< I3CS_T::SIRDAT: DAT3 Mask     */

/* SIRRESP ------------------------------------------------------------------*/
#define I3CS_SIRRESP_IBISTS_Pos    0U                                         /*!< I3CS_T::SIRRESP: IBISTS Position */
#define I3CS_SIRRESP_IBISTS_Msk    (0x7UL << I3CS_SIRRESP_IBISTS_Pos)         /*!< I3CS_T::SIRRESP: IBISTS Mask     */

#define I3CS_SIRRESP_DATLEN_Pos    8U                                         /*!< I3CS_T::SIRRESP: DATLEN Position */
#define I3CS_SIRRESP_DATLEN_Msk    (0xFFFFUL << I3CS_SIRRESP_DATLEN_Pos)      /*!< I3CS_T::SIRRESP: DATLEN Mask     */

/* BUSFAT -------------------------------------------------------------------*/
#define I3CS_BUSFAT_AVAILTC_Pos    16U                                        /*!< I3CS_T::BUSFAT: AVAILTC Position */
#define I3CS_BUSFAT_AVAILTC_Msk    (0xFFFFUL << I3CS_BUSFAT_AVAILTC_Pos)      /*!< I3CS_T::BUSFAT: AVAILTC Mask     */

/* BUSIDLET -----------------------------------------------------------------*/
#define I3CS_BUSIDLET_IDLETC_Pos   0U                                         /*!< I3CS_T::BUSIDLET: IDLETC Position */
#define I3CS_BUSIDLET_IDLETC_Msk   (0xFFFFFUL << I3CS_BUSIDLET_IDLETC_Pos)    /*!< I3CS_T::BUSIDLET: IDLETC Mask     */

/**@}*/ /* I3CS_CONST */
/**@}*/ /* end of I3CS register group */

/**@}*/ /* end of REGISTER group */

#if defined(__CC_ARM)
#pragma no_anon_unions
#endif

#endif /* __I3CS_REG_H__ */

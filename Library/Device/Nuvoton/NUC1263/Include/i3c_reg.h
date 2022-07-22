/**************************************************************************//**
 * @file     i3c_reg.h
 * @version  V3.0
 * @brief    I3C register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __I3C_REG_H__
#define __I3C_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

    @{
*/


/*---------------------- I3C Serial Interface Controller -------------------------*/
/**
    @addtogroup I3C I3C Serial Interface Controller(I3C)
    Memory Mapped Structure for I3C Controller
    @{ 
*/

typedef struct
{
    __IO uint32_t DEVCTL;               /*!< [0x0000] I3C Device Control Register */
    __IO uint32_t DEVADDR;              /*!< [0x0004] I3C Device Address Register */
    __I  uint32_t HWCAP;//RSVD          /*!< [0x0008] I3C Hardware Capability Register */
    __O  uint32_t CMDQUE;               /*!< [0x000c] I3C Command Queue Port Register */
    __I  uint32_t RESPQUE;              /*!< [0x0010] I3C Response Queue Port Register */
    __IO uint32_t TXRXDAT;              /*!< [0x0014] I3C Transmit and Receive Data Port Register */
    __I  uint32_t RESERVE0;             /*!< [0x0018] Reserved */
    __IO uint32_t QUETHCTL;             /*!< [0x001c] I3C Queue Threshold Control Register */
    __IO uint32_t DBTHCTL;              /*!< [0x0020] I3C Data Buffer Threshold Control Register */
    __I  uint32_t RESERVE1[4];          /*!< [0x0024 ~ 0x0030] Reserved */
    __IO uint32_t RSTCTL;               /*!< [0x0034] I3C Reset Control Register */
    __IO uint32_t SLVEVNTS;             /*!< [0x0038] I3C Slave Event Status Register */    
    __IO uint32_t INTSTS;               /*!< [0x003c] I3C Interrupt Status Register */    
    __IO uint32_t INTSTSEN;             /*!< [0x0040] I3C Interrupt Status Enable Register */    
    __IO uint32_t INTEN;                /*!< [0x0044] I3C Interrupt Signal Enable Register */    
    __O  uint32_t INTFORCE;//RSVD       /*!< [0x0048] I3C Interrupt Force Enable Register */    
    __I  uint32_t QUESTSLV;             /*!< [0x004c] I3C Queue Status Level Register */    
    __I  uint32_t DBSTSLV;              /*!< [0x0050] I3C Data Buffer Status Level Register */    
    __I  uint32_t PRESENTS;             /*!< [0x0054] I3C Present State Register */    
    __I  uint32_t CCCDEVS;              /*!< [0x0058] I3C Device Operating Status Register */    
    __I  uint32_t RESERVE2[5];          /*!< [0x005c ~ 0x006c] Reserved */
    __IO uint32_t SLVMID;               /*!< [0x0070] I3C MIPI Manufacturer ID Register */
    __IO uint32_t SLVPID;               /*!< [0x0074] I3C Normal Provisional ID Register */    
    __IO uint32_t SLVCHAR;              /*!< [0x0078] I3C Slave Characteristic Register */    
    __I  uint32_t SLVMXLEN;             /*!< [0x007c] I3C Maximum Write/Read Length Register */    
    __I  uint32_t MXRDTURN;             /*!< [0x0080] I3C Maximum Read Turnaround Time Register */       
    __IO uint32_t MXDS;                 /*!< [0x0084] I3C Maximum Data Speed Register */    
    __I  uint32_t RESERVE3;             /*!< [0x0088] Reserved */
    __IO uint32_t SIR;                  /*!< [0x008c] I3C Slave Interrupt Request Register */        
    __I  uint32_t RESERVE4;             /*!< [0x0090 Reserved */
    __IO uint32_t SIRDAT;               /*!< [0x0094] I3C Slave Interrupt Request Data Register */   
    __IO uint32_t SIRRESP;              /*!< [0x0098] I3C Slave Interrupt Request Response Register */    
    __I  uint32_t RESERVE5[14];         /*!< [0x009c ~ 0x00d0] Reserved */
    __IO uint32_t BUSFAT;               /*!< [0x00d4] I3C Bus Free And Available Timing Register */    
    __IO uint32_t BUSIDLET;             /*!< [0x00d8] I3C Bus Idle Timing Register */
    __I  uint32_t RESERVE6;             /*!< [0x00dc] Reserved */
    __I  uint32_t VERID;//RSVD          /*!< [0x00e0] I3C Version ID Register */
    __I  uint32_t VERTYPE;//RSVD        /*!< [0x00e4] I3C Version Type Register */
    __I  uint32_t QUESIZE;//RSVD        /*!< [0x00e8] I3C Queue Size Capability Register */

} I3C_T;

/**
    @addtogroup I3C_CONST I3C Bit Field Definition
    Constant Definitions for I3C Controller
    @{ 
*/

#define I3C_DEVCTL_PENDINT_Pos          3U
#define I3C_DEVCTL_PENDINT_Msk          (0xFUL << I3C_DEVCTL_PENDINT_Pos)

#define I3C_DEVCTL_IDLECNT_Pos          24U
#define I3C_DEVCTL_IDLECNT_Msk          (0x3UL << I3C_DEVCTL_IDLECNT_Pos)

#define I3C_DEVCTL_SYNC_Pos             26U
#define I3C_DEVCTL_SYNC_Msk             (1UL << I3C_DEVCTL_SYNC_Pos)

#define I3C_DEVCTL_ADAPTIVE_Pos         27U
#define I3C_DEVCTL_ADAPTIVE_Msk         (1UL << I3C_DEVCTL_ADAPTIVE_Pos)

#define I3C_DEVCTL_DMAEN_Pos            28U
#define I3C_DEVCTL_DMAEN_Msk            (1UL << I3C_DEVCTL_DMAEN_Pos)

#define I3C_DEVCTL_RESUME_Pos           30U
#define I3C_DEVCTL_RESUME_Msk           (1UL << I3C_DEVCTL_RESUME_Pos)

#define I3C_DEVCTL_ENABLE_Pos           31U
#define I3C_DEVCTL_ENABLE_Msk           (1UL << I3C_DEVCTL_ENABLE_Pos)

#define I3C_DEVADDR_SA_Pos              0U
#define I3C_DEVADDR_SA_Msk              (0x7FUL << I3C_DEVADDR_SA_Pos)

#define I3C_DEVADDR_SAVALID_Pos         15U
#define I3C_DEVADDR_SAVALID_Msk         (1UL << I3C_DEVADDR_SAVALID_Pos)

#define I3C_DEVADDR_DA_Pos              16U
#define I3C_DEVADDR_DA_Msk              (0x7FUL << I3C_DEVADDR_DA_Pos)

#define I3C_DEVADDR_DAVALID_Pos         31U
#define I3C_DEVADDR_DAVALID_Msk         (1UL << I3C_DEVADDR_DAVALID_Pos)

#define I3C_CMDQUE_ATTR_Pos             0U
#define I3C_CMDQUE_ATTR_Msk             (0x3UL << I3C_CMDQUE_ATTR_Pos)

#define I3C_CMDQUE_TID_Pos              3U
#define I3C_CMDQUE_TID_Msk              (0x3UL << I3C_CMDQUE_TID_Pos)

#define I3C_CMDQUE_DATLEN_Pos           16U
#define I3C_CMDQUE_DATLEN_Msk           (0xFFFFUL << I3C_CMDQUE_DATLEN_Pos)

#define I3C_RESPQUE_DATLEN_Pos          0U
#define I3C_RESPQUE_DATLEN_Msk          (0xFFFFUL << I3C_RESPQUE_DATLEN_Pos)

#define I3C_RESPQUE_HDRCODE_Pos         16U
#define I3C_RESPQUE_HDRCODE_Msk         (0xFFUL << I3C_RESPQUE_HDRCODE_Pos)

#define I3C_RESPQUE_TID_Pos             24U
#define I3C_RESPQUE_TID_Msk             (0x3UL << I3C_RESPQUE_TID_Pos)

#define I3C_RESPQUE_RXRESP_Pos          27U
#define I3C_RESPQUE_RXRESP_Msk          (1UL << I3C_RESPQUE_RXRESP_Pos)

#define I3C_RESPQUE_ERRSTS_Pos          28U
#define I3C_RESPQUE_ERRSTS_Msk          (0xFUL << I3C_RESPQUE_ERRSTS_Pos)

#define I3C_TXRXDAT_DAT_Pos             0U
#define I3C_TXRXDAT_DAT_Msk             (0xFFFFFFFFUL << I3C_TXRXDAT_DAT_Pos)

#define I3C_QUETHCTL_CMDETH_Pos         0U
#define I3C_QUETHCTL_CMDETH_Msk         (0xFFUL << I3C_QUETHCTL_CMDETH_Pos)

#define I3C_QUETHCTL_RESPTH_Pos         8U
#define I3C_QUETHCTL_RESPTH_Msk         (0xFFUL << I3C_QUETHCTL_RESPTH_Pos)

#define I3C_DBTHCTL_TXTH_Pos            0U
#define I3C_DBTHCTL_TXTH_Msk            (0x7UL << I3C_DBTHCTL_TXTH_Pos)

#define I3C_DBTHCTL_RXTH_Pos            8U
#define I3C_DBTHCTL_RXTH_Msk            (0x7UL << I3C_DBTHCTL_RXTH_Pos)

#define I3C_DBTHCTL_TXSTATH_Pos         16U
#define I3C_DBTHCTL_TXSTATH_Msk         (0x7UL << I3C_DBTHCTL_TXSTATH_Pos)

#define I3C_DBTHCTL_RXSTATH_Pos         24U
#define I3C_DBTHCTL_RXSTATH_Msk         (0x7UL << I3C_DBTHCTL_RXSTATH_Pos)

#define I3C_RSTCTL_SWRST_Pos            0U
#define I3C_RSTCTL_SWRST_Msk            (1UL << I3C_RSTCTL_SWRST_Pos)

#define I3C_RSTCTL_CMDRST_Pos           1U
#define I3C_RSTCTL_CMDRST_Msk           (1UL << I3C_RSTCTL_CMDRST_Pos)

#define I3C_RSTCTL_RESPRST_Pos          2U
#define I3C_RSTCTL_RESPRST_Msk          (1UL << I3C_RSTCTL_RESPRST_Pos)

#define I3C_RSTCTL_TXRST_Pos            3U
#define I3C_RSTCTL_TXRST_Msk            (1UL << I3C_RSTCTL_TXRST_Pos)

#define I3C_RSTCTL_RXRST_Pos            4U
#define I3C_RSTCTL_RXRST_Msk            (1UL << I3C_RSTCTL_RXRST_Pos)

#define I3C_SLVEVNTS_SIREN_Pos          0U                            
#define I3C_SLVEVNTS_SIREN_Msk          (1UL << I3C_SLVEVNTS_SIREN_Pos)

#define I3C_SLVEVNTS_HJEN_Pos           3U                            
#define I3C_SLVEVNTS_HJEN_Msk           (1UL << I3C_SLVEVNTS_HJEN_Pos)

#define I3C_SLVEVNTS_ACTSTATE_Pos       4U                            
#define I3C_SLVEVNTS_ACTSTATE_Msk       (3UL << I3C_SLVEVNTS_ACTSTATE_Pos)

#define I3C_SLVEVNTS_MRLUPD_Pos         6U                            
#define I3C_SLVEVNTS_MRLUPD_Msk         (1UL << I3C_SLVEVNTS_MRLUPD_Pos)

#define I3C_SLVEVNTS_MWLUPD_Pos         7U                            
#define I3C_SLVEVNTS_MWLUPD_Msk         (1UL << I3C_SLVEVNTS_MWLUPD_Pos)

#define I3C_INTSTS_TXTH_Pos             0U                            
#define I3C_INTSTS_TXTH_Msk             (1UL << I3C_INTSTS_TXTH_Pos)

#define I3C_INTSTS_RXTH_Pos             1U                            
#define I3C_INTSTS_RXTH_Msk             (1UL << I3C_INTSTS_RXTH_Pos)

#define I3C_INTSTS_CMDRDY_Pos           3U                            
#define I3C_INTSTS_CMDRDY_Msk           (1UL << I3C_INTSTS_CMDRDY_Pos)

#define I3C_INTSTS_RESPRDY_Pos          4U                            
#define I3C_INTSTS_RESPRDY_Msk          (1UL << I3C_INTSTS_RESPRDY_Pos)

#define I3C_INTSTS_CCCUPD_Pos           6U                            
#define I3C_INTSTS_CCCUPD_Msk           (1UL << I3C_INTSTS_CCCUPD_Pos)

#define I3C_INTSTS_DAA_Pos              8U                            
#define I3C_INTSTS_DAA_Msk              (1UL << I3C_INTSTS_DAA_Pos)

#define I3C_INTSTS_TFRERR_Pos           9U                            
#define I3C_INTSTS_TFRERR_Msk           (1UL << I3C_INTSTS_TFRERR_Pos)

#define I3C_INTSTS_READREQ_Pos          11U                            
#define I3C_INTSTS_READREQ_Msk          (1UL << I3C_INTSTS_READREQ_Pos)

#define I3C_INTSTS_IBIUPD_Pos           12U                            
#define I3C_INTSTS_IBIUPD_Msk           (1UL << I3C_INTSTS_IBIUPD_Pos)

#define I3C_INTSTSEN_TXTH_Pos           0U                            
#define I3C_INTSTSEN_TXTH_Msk           (1UL << I3C_INTSTSEN_TXTH_Pos)

#define I3C_INTSTSEN_RXTH_Pos           1U                            
#define I3C_INTSTSEN_RXTH_Msk           (1UL << I3C_INTSTSEN_RXTH_Pos)

#define I3C_INTSTSEN_CMDRDY_Pos         3U                            
#define I3C_INTSTSEN_CMDRDY_Msk         (1UL << I3C_INTSTSEN_CMDRDY_Pos)

#define I3C_INTSTSEN_RESPRDY_Pos        4U                            
#define I3C_INTSTSEN_RESPRDY_Msk        (1UL << I3C_INTSTSEN_RESPRDY_Pos)

#define I3C_INTSTSEN_CCCUPD_Pos         6U                            
#define I3C_INTSTSEN_CCCUPD_Msk         (1UL << I3C_INTSTSEN_CCCUPD_Pos)

#define I3C_INTSTSEN_DAA_Pos            8U                            
#define I3C_INTSTSEN_DAA_Msk            (1UL << I3C_INTSTSEN_DAA_Pos)

#define I3C_INTSTSEN_TFRERR_Pos         9U                            
#define I3C_INTSTSEN_TFRERR_Msk         (1UL << I3C_INTSTSEN_TFRERR_Pos)

#define I3C_INTSTSEN_READREQ_Pos        11U                            
#define I3C_INTSTSEN_READREQ_Msk        (1UL << I3C_INTSTSEN_READREQ_Pos)

#define I3C_INTSTSEN_IBIUPD_Pos         12U                            
#define I3C_INTSTSEN_IBIUPD_Msk         (1UL << I3C_INTSTSEN_IBIUPD_Pos)

#define I3C_INTEN_TXTH_Pos              0U                            
#define I3C_INTEN_TXTH_Msk              (1UL << I3C_INTEN_TXTH_Pos)

#define I3C_INTEN_RXTH_Pos              1U                            
#define I3C_INTEN_RXTH_Msk              (1UL << I3C_INTEN_RXTH_Pos)

#define I3C_INTEN_CMDRDY_Pos            3U                            
#define I3C_INTEN_CMDRDY_Msk            (1UL << I3C_INTEN_CMDRDY_Pos)

#define I3C_INTEN_RESPRDY_Pos           4U                            
#define I3C_INTEN_RESPRDY_Msk           (1UL << I3C_INTEN_RESPRDY_Pos)

#define I3C_INTEN_CCCUPD_Pos            6U                            
#define I3C_INTEN_CCCUPD_Msk            (1UL << I3C_INTEN_CCCUPD_Pos)

#define I3C_INTEN_DAA_Pos               8U                            
#define I3C_INTEN_DAA_Msk               (1UL << I3C_INTEN_DAA_Pos)

#define I3C_INTEN_TFRERR_Pos            9U                            
#define I3C_INTEN_TFRERR_Msk            (1UL << I3C_INTEN_TFRERR_Pos)

#define I3C_INTEN_READREQ_Pos           11U                            
#define I3C_INTEN_READREQ_Msk           (1UL << I3C_INTEN_READREQ_Pos)

#define I3C_INTEN_IBIUPD_Pos            12U                            
#define I3C_INTEN_IBIUPD_Msk            (1UL << I3C_INTEN_IBIUPD_Pos)

#define I3C_QUESTSLV_CMDELOC_Pos        0U                            
#define I3C_QUESTSLV_CMDELOC_Msk        (0xFFUL << I3C_QUESTSLV_CMDELOC_Pos)

#define I3C_QUESTSLV_RESPLV_Pos         8U                            
#define I3C_QUESTSLV_RESPLV_Msk         (0xFFUL << I3C_QUESTSLV_RESPLV_Pos)

#define I3C_DBSTSLV_TXELV_Pos           0U                            
#define I3C_DBSTSLV_TXELV_Msk           (0xFFUL << I3C_DBSTSLV_TXELV_Pos)

#define I3C_DBSTSLV_RXLV_Pos            16U                            
#define I3C_DBSTSLV_RXLV_Msk            (0xFFUL << I3C_DBSTSLV_RXLV_Pos)

#define I3C_PRESENTS_TFRTYPE_Pos        8U                            
#define I3C_PRESENTS_TFRTYPE_Msk        (0x3FUL << I3C_PRESENTS_TFRTYPE_Pos)

#define I3C_PRESENTS_TID_Pos            24U                            
#define I3C_PRESENTS_TID_Msk            (0xFUL << I3C_PRESENTS_TID_Pos)

#define I3C_CCCDEVS_PENDINT_Pos         0U                            
#define I3C_CCCDEVS_PENDINT_Msk         (0xFUL << I3C_CCCDEVS_PENDINT_Pos)

#define I3C_CCCDEVS_PROTERR_Pos         5U                            
#define I3C_CCCDEVS_PROTERR_Msk         (1UL << I3C_CCCDEVS_PROTERR_Pos)

#define I3C_CCCDEVS_ACTMODE_Pos         6U                            
#define I3C_CCCDEVS_ACTMODE_Msk         (3UL << I3C_CCCDEVS_ACTMODE_Pos)

#define I3C_CCCDEVS_UDFERR_Pos          8U                            
#define I3C_CCCDEVS_UDFERR_Msk          (1UL << I3C_CCCDEVS_UDFERR_Pos)

#define I3C_CCCDEVS_SLVBUSY_Pos         9U                            
#define I3C_CCCDEVS_SLVBUSY_Msk         (1UL << I3C_CCCDEVS_SLVBUSY_Pos)

#define I3C_CCCDEVS_OVFERR_Pos          10U                            
#define I3C_CCCDEVS_OVFERR_Msk          (1UL << I3C_CCCDEVS_OVFERR_Pos)

#define I3C_CCCDEVS_DATNRDY_Pos         11U                            
#define I3C_CCCDEVS_DATNRDY_Msk         (1UL << I3C_CCCDEVS_DATNRDY_Pos)

#define I3C_CCCDEVS_BFNAVAIL_Pos        12U                            
#define I3C_CCCDEVS_BFNAVAIL_Msk        (1UL << I3C_CCCDEVS_BFNAVAIL_Pos)

#define I3C_CCCDEVS_FRAMEERR_Pos        13U                            
#define I3C_CCCDEVS_FRAMEERR_Msk        (1UL << I3C_CCCDEVS_FRAMEERR_Pos)

#define I3C_SLVMID_PIDTYPE_Pos          0U                            
#define I3C_SLVMID_PIDTYPE_Msk          (1UL << I3C_SLVMID_PIDTYPE_Pos)

#define I3C_SLVMID_MID_Pos              1U                            
#define I3C_SLVMID_MID_Msk              (0x7FFFUL << I3C_SLVMID_MID_Pos)

#define I3C_SLVPID_DCR_Pos              0U                            
#define I3C_SLVPID_DCR_Msk              (0xFFFUL << I3C_SLVPID_DCR_Pos)

#define I3C_SVLPID_INSTID_Pos           12U                            
#define I3C_SVLPID_INSTID_Msk           (0xFUL << I3C_SVLPID_INSTID_Pos)

#define I3C_SLVPID_PARTID_Pos           16U                            
#define I3C_SLVPID_PARTID_Msk           (0xFFFFUL << I3C_SLVPID_PARTID_Pos)

#define I3C_SLVCHAR_MAXDSLIM_Pos        0U                            
#define I3C_SLVCHAR_MAXDSLIM_Msk        (1UL << I3C_SLVCHAR_MAXDSLIM_Pos)

#define I3C_SLVCHAR_IBICAP_Pos          1U                            
#define I3C_SLVCHAR_IBICAP_Msk          (1UL << I3C_SLVCHAR_IBICAP_Pos)

#define I3C_SLVCHAR_IBIPL_Pos           2U                            
#define I3C_SLVCHAR_IBIPL_Msk           (1UL << I3C_SLVCHAR_IBIPL_Pos)

#define I3C_SLVCHAR_OLCAP_Pos           3U                            
#define I3C_SLVCHAR_OLCAP_Msk           (1UL << I3C_SLVCHAR_OLCAP_Pos)

#define I3C_SLVCHAR_BRIDGEID_Pos        4U                            
#define I3C_SLVCHAR_BRIDGEID_Msk        (1UL << I3C_SLVCHAR_BRIDGEID_Pos)

#define I3C_SLVCHAR_HDRCAP_Pos          5U                            
#define I3C_SLVCHAR_HDRCAP_Msk          (1UL << I3C_SLVCHAR_HDRCAP_Pos)

#define I3C_SLVCHAR_DEVROLE_Pos         6U                            
#define I3C_SLVCHAR_DEVROLE_Msk         (0x3UL << I3C_SLVCHAR_DEVROLE_Pos)

#define I3C_SLVCHAR_DCR_Pos             8U                            
#define I3C_SLVCHAR_DCR_Msk             (0xFFUL << I3C_SLVCHAR_DCR_Pos)

#define I3C_SLVCHAR_HDRCPVAL_Pos        16U                            
#define I3C_SLVCHAR_HDRCPVAL_Msk        (0xFFUL << I3C_SLVCHAR_HDRCPVAL_Pos)

#define I3C_SLVMXLEN_MWL_Pos            0U                            
#define I3C_SLVMXLEN_MWL_Msk            (0xFFFFUL << I3C_SLVMXLEN_MWL_Pos)

#define I3C_SLVMXLEN_MRL_Pos            16U                            
#define I3C_SLVMXLEN_MRL_Msk            (0xFFFFUL << I3C_SLVMXLEN_MRL_Pos)
            
#define I3C_MXRDTURN_MXRDTURN_Pos       0U                            
#define I3C_MXRDTURN_MXRDTURN_Msk       (0xFFFFFFUL << I3C_MXRDTURN_MXRDTURN_Pos)

#define I3C_MXDS_MXWR_Pos               0U                            
#define I3C_MXDS_MXWR_Msk               (0x7UL << I3C_MXDS_MXWR_Pos)

#define I3C_MXDS_MXRD_Pos               8U                            
#define I3C_MXDS_MXRD_Msk               (0x7UL << I3C_MXDS_MXRD_Pos)

#define I3C_MXDS_MXDTURN_Pos            16U                            
#define I3C_MXDS_MXDTURN_Msk            (0x7UL << I3C_MXDS_MXDTURN_Pos)

#define I3C_SIR_EN_Pos                  0U                            
#define I3C_SIR_EN_Msk                  (1UL << I3C_SIR_EN_Pos)

#define I3C_SIR_CTL_Pos                 1U                            
#define I3C_SIR_CTL_Msk                 (0x3UL << I3C_SIR_CTL_Pos)

#define I3C_SIR_MDB_Pos                 8U                            
#define I3C_SIR_MDB_Msk                 (0xFFUL << I3C_SIR_MDB_Pos)

#define I3C_SIR_DATLEN_Pos              16U                            
#define I3C_SIR_DATLEN_Msk              (0xFFUL << I3C_SIR_DATLEN_Pos)

#define I3C_SIRDAT_DAT0_Pos             0U                            
#define I3C_SIRDAT_DAT0_Msk             (0xFFUL << I3C_SIRDAT_DAT0_Pos)

#define I3C_SIRDAT_DAT1_Pos             8U                            
#define I3C_SIRDAT_DAT1_Msk             (0xFFUL << I3C_SIRDAT_DAT1_Pos)

#define I3C_SIRDAT_DAT2_Pos             16U                            
#define I3C_SIRDAT_DAT2_Msk             (0xFFUL << I3C_SIRDAT_DAT2_Pos)

#define I3C_SIRDAT_DAT3_Pos             24U                            
#define I3C_SIRDAT_DAT3_Msk             (0xFFUL << I3C_SIRDAT_DAT3_Pos)

#define I3C_SIRRESP_IBISTS_Pos          0U                            
#define I3C_SIRRESP_IBISTS_Msk          (0x7UL << I3C_SIRRESP_IBISTS_Pos)

#define I3C_SIRRESP_DATLEN_Pos          8U                            
#define I3C_SIRRESP_DATLEN_Msk          (0xFFFFUL << I3C_SIRRESP_DATLEN_Pos)

#define I3C_BUSFAT_AVAILTC_Pos          16U                            
#define I3C_BUSFAT_AVAILTC_Msk          (0xFFFFUL << I3C_BUSFAT_AVAILTC_Pos)

#define I3C_BUSIDLET_IDLETC_Pos         0U                            
#define I3C_BUSIDLET_IDLETC_Msk         (0xFFFFFUL << I3C_BUSIDLET_IDLETC_Pos)

/**@}*/ /* I3C_CONST */
/**@}*/ /* end of I3C register group */

/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __I3C_REG_H__ */

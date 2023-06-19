/**************************************************************************//**
 * @file     LocalDevFw.c
 * @version  V3.00
 * @brief    Local device control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "spdh_device.h"
#include "LocalDevReg.h"
#include "LocalDevFw.h"
#include "Global_Variable.h"
#include "Flash.h"


volatile RESP_QUEUE_T   s_DevRespQue[I3CS_DEVICE_RESP_QUEUE_CNT] __attribute__((aligned(4)));
static uint32_t         s_DevRxBuf[I3CS_DEVICE_RX_BUF_CNT];
static uint8_t          *s_p8DevRx = (uint8_t *)s_DevRxBuf;

static volatile uint8_t s_u8DevMRn;
static volatile uint8_t s_u8DevCMD;
static volatile uint8_t g_u8DevPEC;

#define _BYTE(n)        (s_p8DevRx[(n)])
#define GET_MR_IDX      (_BYTE(0))
#define GET_MR_VAL      (_BYTE(1))


volatile uint32_t g_u32DeviceChangedToI3CMode = 0;

extern volatile uint32_t g_u32FifoClr;


/**
  \brief    For LLSI LED : Definitions of LED Configuration according MRn registers
 */
#define LED_CFG_MODE    (g_au8DevReg[37] & BIT0)
#define LED_CFG_FREQ    (g_au8DevReg[37] & BIT1)
#define LED_CFG_DIR     (g_au8DevReg[37] & BIT2)
#define LED_CFG_NUM     (((g_au8DevReg[40]&0x1)<<8) | (g_au8DevReg[39]&0xFF))
#define LED_CFG_EFFECT  (g_au8DevReg[38] & 0xF)
#define LED_CFG_COLOR_R (g_au8DevReg[44])
#define LED_CFG_COLOR_G (g_au8DevReg[45])
#define LED_CFG_COLOR_B (g_au8DevReg[46])
#define LED_CFG_SPEED   (g_au8DevReg[55])
#define LED_CFG_BRIGHT  (g_au8DevReg[56])
#define LED_SYNCDAT_POS (((g_au8DevReg[42]&0x7)<<8) | g_au8DevReg[41])
__attribute__((aligned (4))) static volatile LED_Setting_T s_LEDSetting;

void ApplyLEDSetting(void)
{    
    uint8_t *pu8LEDSeeting;
    
    pu8LEDSeeting = (uint8_t *)&s_LEDSetting;
      
    s_LEDSetting.AP_Sync    = LED_CFG_MODE;    
    s_LEDSetting.LEDNum     = LED_CFG_NUM;;
    s_LEDSetting.Direction  = LED_CFG_DIR;;
        
    s_LEDSetting.LightingMode = LED_CFG_EFFECT;
    s_LEDSetting.Color_R      = LED_CFG_COLOR_R;
    s_LEDSetting.Color_G      = LED_CFG_COLOR_G;
    s_LEDSetting.Color_B      = LED_CFG_COLOR_B;    
    s_LEDSetting.Speed        = LED_CFG_SPEED;
    s_LEDSetting.Brightness   = LED_CFG_BRIGHT;

    ReadStoredSetting(pu8LEDSeeting, LED_CFG_FREQ); 
}

int32_t LocalDev_FIFOResetAndResume(I3CS_T *i3cs, uint32_t u32ResetMask, uint32_t u32EnableResume)
{
    uint8_t u8InHaltState = 0;
    volatile uint32_t u32Timeout;

    if (I3CS_IS_SLAVE_BUSY(i3cs))
        u8InHaltState = 1;

    if (u32ResetMask)
    {
        if(u8InHaltState == 0)
        {
            /* Disable I3CS controller for reset buffer and queue */
            if (I3CS_Disable(i3cs) != I3CS_STS_NO_ERR)
                return I3CS_TIMEOUT_ERR;
        }

        /* Reset specify source */
        i3cs->RSTCTL = u32ResetMask;
        u32Timeout = (SystemCoreClock / 1000);
        while ((i3cs->RSTCTL != 0) && (--u32Timeout)) {}
        if (u32Timeout == 0)
            return I3CS_TIMEOUT_ERR;

        if (u8InHaltState == 0)
        {
            /* Enable I3CS controller again */
            if (I3CS_Enable(i3cs) != I3CS_STS_NO_ERR)
                return I3CS_TIMEOUT_ERR;
        }
    }

    if (u32EnableResume || u8InHaltState)
    {
        /* The application has to take necessary action to handle the error condition and
            then set RESUME bit to resume the controller. */
        /* Slave will receive GETSTATUS CCC to clear specify status in I3CS_CCCDEVS register. */
        i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
        //while((i3cs->DEVCTL&I3CS_DEVCTL_RESUME_Msk) == I3CS_DEVCTL_RESUME_Msk) {}

        /* RESUME bit is auto-cleared once the controller is ready to accept new transfers. */
    }

    return I3CS_STS_NO_ERR;
}

int32_t LocalDev_RespErrorRecovery(I3CS_T *i3cs, uint32_t u32RespStatus)
{
    if (u32RespStatus != I3CS_RESP_NO_ERR)
    {
        if (I3CS_IS_SLAVE_BUSY(i3cs))
        {
            switch(u32RespStatus)
            {
                case I3CS_RESP_CRC_ERR:
                case I3CS_RESP_PARITY_ERR:
                case I3CS_RESP_FRAME_ERRR:
                case I3CS_RESP_FLOW_ERR:
                    /* Reset RX FIFO -> apply resume */
                    LocalDev_FIFOResetAndResume(i3cs, I3CS_RESET_RX_BUF, TRUE);
                    break;

                case I3CS_RESP_MASTER_TERMINATE_ERR:
                    while ((I3CS_GET_PRESENT_STATUS(i3cs) != 6)) {} // 6 = Slave controller in Halt State waiting for resume from application
                    /* Reset TX FIFO and CMDQ Queue -> apply resume */
                    LocalDev_FIFOResetAndResume(i3cs, (I3CS_RESET_TX_BUF | I3CS_RESET_CMD_QUEUE), TRUE);
                    break;

                default:
                    /* Reset all FIFO and Queue */
                    LocalDev_FIFOResetAndResume(i3cs, I3CS_RESET_ALL_QUEUE_AND_BUF, FALSE);
                    break;
            }
        }
    }

    return I3CS_STS_NO_ERR;
}

int8_t LocalDev_Init(uint8_t u8DevAddr)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable SPD5 Hub clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPDHCKEN_Msk;

    /* Enable I3C1 clock for Hub */
    CLK->APBCLK0 |= CLK_APBCLK0_I3CS1CKEN_Msk;

    I3CS_Open(I3CS1, u8DevAddr, 8);

    /* Disable Hot-join function before I3CS was enabled */
    I3CS1->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;
    
    I3CS1->DBTHCTL = (3 << I3CS_DBTHCTL_TXTH_Pos); // set TX buffer threshold to 16

    /* Configure Bus available time: JESD define is 1 us, 1/0.083 = 12 */
    I3CS1->BUSFAT = (12 << I3CS_BUSFAT_AVAILTC_Pos); // adjust for bus available time for IBI issue

	I3CS1->QUETHCTL = (((I3CS_CFG_CMD_QUEUE_EMPTY_THLD-1) << I3CS_QUETHCTL_CMDETH_Pos) | 
                        ((I3CS_CFG_RESP_QUEUE_FULL_THLD-1) << I3CS_QUETHCTL_RESPTH_Pos));

    /* Enable I3CS1 INT Status */
    I3CS1->INTSTSEN = 0xFFFFFFFFul; // all event was enabled
    /* Enable I3CS1 INT */
    I3CS1->INTEN |= (I3CS_INTEN_RESPQ_READY | I3CS_INTEN_CCC_UPDATED | I3CS_INTEN_DA_ASSIGNED |
                    I3CS_INTEN_TRANSFER_ERR | I3CS_INTEN_READ_REQUEST | I3CS_INTEN_IBI_UPDATED);

    /* Disable SIR */
    SPDH_DisableDEVSIR();

    /* Set LID */
    SPDH_SetDEVLID((((u8DevAddr)&SPDH_DCTL_LID_Msk) >> SPDH_DCTL_LID_Pos));

    /* Enable chip's SPD5 Hub function */
    SYS->SPDHCTL |= SYS_SPDHCTL_SPDHEN_Msk;
    
    /* Enable Hub INT Status for local device */
    SPDH_EnableINT((SPDH_INTEN_BUSRTOEN_Msk | SPDH_INTEN_DDEVCTLEN_Msk | SPDH_INTEN_DDEVCAPEN_Msk | \
                    SPDH_INTEN_DSETHIDEN_Msk | SPDH_INTEN_DEVPCEN_Msk | SPDH_INTEN_DEVIHDEN_Msk));

#if (SPDH_DETECT_POWER_DOWN == 1)
    SPDH_EnableINT(SPDH_INTEN_PWRDTOEN_Msk|SPDH_INTEN_WKUPEN_Msk);
    SPDH_SetPowerDownTimeout(1, 0x7F);
#endif

    /* (221+1) *16384*13.8 = 50194022 ns = 50 ms */
    SPDH_SetBusResetTimeout(TRUE, 221);
                        
    NVIC_EnableIRQ(SPDH_IRQn);
    NVIC_EnableIRQ(I3CS1_IRQn);

    /* Enable I3CS1 controller */
    I3CS_Enable(I3CS1);

    return 0;
}

static int8_t _LocalDev_UpdateInterfaceSel(uint8_t u8InfSel)
{
    if (u8InfSel == 0)
    {
        /* Update MR18[5] NF_SEL bit as I2C basic protocol. */
        DevReg_EnableI2C();
    }
    else if (u8InfSel == 1)
    {
        /* Update MR18[5] NF_SEL bit as I3C basic protocol. */
        DevReg_EnableI3C();
    }
    else
    {
        ERRLOG("[ERR] No support this selection.\n");
        return (-1);
    }
    
    return 0;
}

int8_t LocalDev_ResetInit(uint8_t u8SlaveAddr)
{
    /* Disable Hot-join function before I3CS was enabled */
    I3CS1->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;

    /* Disable SIR */
    SPDH_DisableDEVSIR();

    /* Enable I3CS controller */
    I3CS1->DEVCTL |= (I3CS_DEVCTL_ENABLE_Msk | I3CS_DEVCTL_ADAPTIVE_Msk);

    I3CS1->DEVADDR = (I3CS_DEVADDR_SAVALID_Msk | u8SlaveAddr);

    I3CS1->DBTHCTL = (3 << I3CS_DBTHCTL_TXTH_Pos); // set TX buffer threshold to 16

    /* Enable I3CS1 INT Status */
    I3CS1->INTSTSEN = 0xFFFFFFFFul; // all event was enabled
    /* Enable I3CS1 INT */
    I3CS1->INTEN |= (I3CS_INTEN_RESPQ_READY | I3CS_INTEN_CCC_UPDATED | I3CS_INTEN_DA_ASSIGNED |
                        I3CS_INTEN_TRANSFER_ERR | I3CS_INTEN_READ_REQUEST | I3CS_INTEN_RX_THLD);

    /* Update MRn register while changed back to I2C mode */
    _LocalDev_UpdateInterfaceSel(0);

    return 0;
}

int8_t LocalDev_CheckInterfaceSel(void)
{
    if (g_u32DeviceChangedToI3CMode)
    {
        /* Polling the Hub mode to check Host has sent RSTDAA command.(Because I3C no interrupt for RSADAA command) */
        if (I3CS_IS_DA_VALID(I3CS1) == 0)
        {
            g_u32DeviceChangedToI3CMode = 0;

            if (I3CS_IS_INT_STATUS(I3CS1, I3CS_INTSTS_CCCUPD_Msk))
            {
                DBGLOG("INISTS CCCUPD (I3CS1 SA: 0x%02x)\n", (uint32_t)I3CS_GET_I2C_SA(I3CS1));
                I3CS_CLEAR_CCC_UPDATED_STATUS(I3CS1);
            }

            /* Update MR18[5] INF_SEL bit as I2C basic protocol. */
            _LocalDev_UpdateInterfaceSel(0);
            DBGLOG("I3C slave Back to I2C mode.\n\n");

            /* To check if mode of device behind Hub and I3C slave is matched? */
            if (SPDH_GetDEVMode() == 1)
            {
                WRNLOG("[WARN] Mode of device behide Hub and I3C slave is not matched.\n\n");
            }
        }
    }
    
    return 0;
}

static int8_t _LocalDev_SendIBIReq(void)
{
    uint8_t u8DataLen, u8MDB = 0x0;
    
    DBGLOG("\nI3C device re-sends the IBI request\n");    
        
    if (SPDH_IsDEVPECEnable())
    {
        /* PEC enabled case */
        u8DataLen = 3;
    }
    else
    {
        /* PEC disabled case */
        u8DataLen = 2;
    }
    
    // TODO: Need to re-define the IBI payload
    I3CS1->SIRDAT = (g_au8DevReg[52] | g_au8DevReg[51]); // IBI payload should be MR51 and MR52.
    I3CS1->SIR = (u8DataLen<<I3CS_SIR_DATLEN_Pos) | (u8MDB<<I3CS_SIR_MDB_Pos) | I3CS_SIR_EN_Msk;

    /* Set pending status until IBI request has been accepted or cleared by host. */
    /* Set MR48[7] : IBI_STATUS */
    DevReg_SetIBIStatus();

    return 0;
}

int8_t LocalDev_CheckIBIReg(void)
{
    /* IBI_ERROR_EN */
    if (DevReg_IsIBIntEn(DEV_IBI_INTEN_ERROR))
    {
        if ((DevReg_GetErrorStatus() & (DEV_ERROR_STATUS_PEC|DEV_ERROR_STATUS_PAR)))
        {
            /* MR52[1:0] = 1 generates an IBI to Host .*/
            _LocalDev_SendIBIReq();
        }
    }

    return 0;
}

static void _LocalDev_SPDHIRQHandler(void)
{
    uint8_t  u8StaticAddr, u8LID;
    volatile uint32_t u32SPDHSTS, u32HUBDEVSTS;
    uint8_t  u8StartOffset, u8LLSISycFuncCtrl;

    u32SPDHSTS = SPDH_GetINTStatus();

    if (u32SPDHSTS & SPDH_INTSTS_BUSRTOIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: BUS Reset Time-out Event\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_BUSRTOIF_Msk);

        // TODO: not yet referenced
        // TODO: Do chip reset by software, because Host sent bus reset to all device on the same bus.
        //SYS_ResetChip();
    }

    if (u32SPDHSTS & SPDH_INTSTS_DEVPCIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: PEC Error Occurred\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DEVPCIF_Msk);
    }

    if (u32SPDHSTS & SPDH_INTSTS_DDEVCAPIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received DEVCAP CCC\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DDEVCAPIF_Msk);
    }    
    
    if (u32SPDHSTS & SPDH_INTSTS_DSETHIDIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received SETHID CCC\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DSETHIDIF_Msk);

        u8LID = (I3CS1->DEVADDR & SPDH_DCTL_LID_Msk);
        u8StaticAddr = (uint8_t)(u8LID | SPDH_GetDEVHID());
        DBGLOG("'\tu8StaticAddr: 0x%08X\n", u8StaticAddr);

        /* Change HID of I3C static address after received SETHID CCC command. */
        I3CS1->DEVADDR = (I3CS1->DEVADDR &~I3CS_DEVADDR_SA_Msk) | (I3CS_DEVADDR_SAVALID_Msk | u8StaticAddr);
        DBGLOG("\tI3C1->DEVADDR: 0x%08X\n", I3CS1->DEVADDR);
        
    }

    if (u32SPDHSTS & SPDH_INTSTS_DDEVCTLIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received DEVCTRL CCC\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DDEVCTLIF_Msk);
        
        u32HUBDEVSTS = SPDH_GetDEVStatus();
        
        /* Set PEC enabled/disabled */
        if (u32HUBDEVSTS & SPDH_DSTS_PECSTS_Msk)
        {
            DBGLOG("\tDevice DEVCTRL: PEC enabled.\n");
            /* Enable PEC function in Hub's device. */
            SPDH_EnableDEVCRC();
            /* Update MR18 register. */
            DevReg_PECEnable();
        }
        else
        {
            DBGLOG("\tDevice DEVCTRL: PEC disabled.\n");
            /* Disable PEC function in Hub's device. */
            SPDH_DisableDEVCRC();
            /* Update MR18 register. */
            DevReg_PECDisable();
        }
        
        /* Set Parity enabled/disabled */
        if (u32HUBDEVSTS & SPDH_DSTS_PARDIS_Msk)
        {
            DBGLOG("\tDevice DEVCTRL: Parity disabled.\n");
            /* Update MR18 register. */
            DevReg_ParityDisable();
        }
        else
        {
            DBGLOG("\tDevice DEVCTRL: Parity enabled.\n");
            /* Update MR18 register. */
            DevReg_ParityEnable();
        }
        
        /* Clear all event and pending IBI */
        if (u32HUBDEVSTS & SPDH_DSTS_IBICLR_Msk)
        {
            DBGLOG("\tDevice DEVCTRL: Clear All Event and pending IBI.\n");
            DevReg_ClearAllEvent();
        }

        /* Check if byte-2 data for synchronous function is enabled. */
        /* if RegMod is 0 */
        if ((SPDH_GetDEVCTRL0()&SPDH_HDEVCTRL0_REGMOD_Msk) == 0)
        {
            /* Check StartOffset value */
            u8StartOffset = ((SPDH_GetDEVCTRL0()&SPDH_HDEVCTRL0_STAOFSET_Msk) >> SPDH_HDEVCTRL0_STAOFSET_Pos);
            if (u8StartOffset < 3)
            {
                if (u8StartOffset == 0)
                {
                    u8LLSISycFuncCtrl = _GET_BYTE2(SPDH_GetDEVCTRL1());
                }
                else if (u8StartOffset == 1)
                {
                    u8LLSISycFuncCtrl = _GET_BYTE1(SPDH_GetDEVCTRL1());
                }
                else if (u8StartOffset == 2)
                {
                    u8LLSISycFuncCtrl = _GET_BYTE0(SPDH_GetDEVCTRL1());
                }
                
                /* Check if byte-2 data for synchronous function is enabled. */
                if (u8LLSISycFuncCtrl & BIT0)
                {
                    if (u8LLSISycFuncCtrl & BIT1)
                    {
                        /* Synchronous start to flash LED. */
                        //DevReg_WriteReg(36, 1);
                        
                        ApplyLEDSetting();
                        
                        /* Start to flash LED */
                        LLSI_StartFlashLED(1);
                    }
                    else
                    {
                        /* Synchronous stop to flash LED. */
                        //DevReg_WriteReg(36, 0);
                        
                        /* Stop to flash LED */
                        LLSI_StartFlashLED(0);
                    }
                }
            }
        }
    }
    
    if (u32SPDHSTS & SPDH_INTSTS_DEVIHDIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received IBI Header\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DEVIHDIF_Msk);

        /* If pending status 1, then re-send the IBI request */
        if (SPDH_IsDEVINTStatus(SPDH_DSTS_PENDIBI_Msk))
        {
            DBGLOG("\tRe-send the IBI request\n");
            _LocalDev_SendIBIReq();
        }
    }
    
#if (SPDH_DETECT_POWER_DOWN == 1)
    if (u32SPDHSTS & SPDH_INTSTS_PWRDTOIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Power Down Detect\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_PWRDTOIF_Msk);

        g_DetectedPowerDown = 1;
    }
#endif
}

void SPDH_IRQHandler(void)
{
    /* Call the interrupt handler in LocalDevFw.c */
    _LocalDev_SPDHIRQHandler();
}


// TODO: not yet referenced
/* For I2C mode and I3C mode w/ PEC Disabled
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	A |   |
-----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	A |   |
-----------------------------------------------
|       | Data	                      | A | P |
-----------------------------------------------
*/
static int32_t _SlaveCfgReadHandler(I3CS_T *i3cs)
{
    i3cs->CMDQUE  = ((1<<I3CS_CMDQUE_DATLEN_Pos) | (6<<I3CS_CMDQUE_TID_Pos)); // 1-byte data, and TID is 6
    i3cs->TXRXDAT = g_au8DevReg[GET_MR_IDX];

    return 0;
}

// TODO: not yet referenced
/*
## Command Truth Table (CMD) w/ PEC Enabled
----------------------------------------------------------------
| Command Description	    | Command Name | Command Code | RW |
----------------------------------------------------------------
| Write 1 Byte to Register  | W1R          | 000          | 0  |
----------------------------------------------------------------
| Read 1 Byte from Register | R1R          | 000          | 1  |
----------------------------------------------------------------
| Write 2 Byte to Register  | W2R          | 001          | 0  |
----------------------------------------------------------------
| Read 2 Byte from Register | R2R          | 001          | 1  |
----------------------------------------------------------------
NOTE: Command Code defined at 2nd byte[7:5], RW defined at 2nd byte[4].    
*/
/* For I2C mode and I3C mode w/ PEC Enabled
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	T |   |
-----------------------------------------------	
|       | 3-bits CMD | R = 1 | 0000   | T |   |
-----------------------------------------------
|       | PEC	                      | T |   |
-----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	T |   |
-----------------------------------------------
|       | Data	                      | T |   |
-----------------------------------------------
|       | ...	                      | T |   |
-----------------------------------------------
|       | Data	                      | T |   |
-----------------------------------------------
|       | PEC	                      | T | P |
-----------------------------------------------
*/
static int32_t _SlaveCfgReadHandlerPEC(I3CS_T *i3cs)
{
    uint8_t MRn, dummy_pec = 0x55;
    volatile uint16_t i, r_cnt = 0, tid;
    
    /* Get MRn value (1st byte) */
    MRn = _BYTE(0); 
    
    /* Limited MRn to MAX_DEVREG_LEN */
    if (MRn >= MAX_DEVREG_LEN)
        return -1;    
    
    /* Get MRn value (1st byte) */
    s_u8DevMRn = MRn;
    
    /* Get CMD value (2nd byte) */
    s_u8DevCMD = (_BYTE(1) >> 5);
    
    /* Get PEC  value (3rd byte) */
    g_u8DevPEC = _BYTE(2);
    
    if (s_u8DevCMD == 0)
    {
        /* Command Name : R1R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-1))
        {
            WRNLOG("Read MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -1;
        }
        
        tid = 1;
        r_cnt = 1 + 1; // Reg_cnt + PEC
        
        i3cs->CMDQUE  = ((r_cnt<<I3CS_CMDQUE_DATLEN_Pos) | (tid<<I3CS_CMDQUE_TID_Pos)); // 2-byte data, and TID is 1
        i3cs->TXRXDAT = ((dummy_pec<<8) | (g_au8DevReg[s_u8DevMRn+0]<<0));
    }
    else if (s_u8DevCMD == 1)/* R2R */
    {
        /* Command Name : R2R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-2))
        {
            WRNLOG("Read MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -1;
        }
               
        tid = 2;
        r_cnt = 2 + 1; // Reg_cnt + PEC
        
        i3cs->CMDQUE  = ((r_cnt<<I3CS_CMDQUE_DATLEN_Pos) | (tid<<I3CS_CMDQUE_TID_Pos)); // 3-byte data, and TID is 2
        i3cs->TXRXDAT = ((dummy_pec<<16) | (g_au8DevReg[s_u8DevMRn+1]<<8) | (g_au8DevReg[s_u8DevMRn+0]<<0));
    }
    else if (s_u8DevCMD == 2)/* R4R */
    {
        /* Command Name : R4R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-4))
        {
            WRNLOG("Read MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -1;
        }
                     
        tid = 3;
        r_cnt = 5 + 1; // Reg_cnt + PEC
        
        i3cs->CMDQUE  = ((r_cnt<<I3CS_CMDQUE_DATLEN_Pos) | (tid<<I3CS_CMDQUE_TID_Pos)); // 5-byte data, and TID is 3
        i3cs->TXRXDAT = ((g_au8DevReg[s_u8DevMRn+3]<<24) | (g_au8DevReg[s_u8DevMRn+2]<<16) | (g_au8DevReg[s_u8DevMRn+1]<<8) | (g_au8DevReg[s_u8DevMRn+0]<<0));
        i3cs->TXRXDAT = (dummy_pec<<0);
    }
    else if (s_u8DevCMD == 3)/* R16R */
    {
        /* Command Name : R16R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-16))
        {
            WRNLOG("Read MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -1;
        }
                            
        tid = 4;
        r_cnt = 16 + 1; // Reg_cnt + PEC
        
        i3cs->CMDQUE  = ((r_cnt<<I3CS_CMDQUE_DATLEN_Pos) | (tid<<I3CS_CMDQUE_TID_Pos)); // 17-byte data, and TID is 4
        i3cs->TXRXDAT = ((g_au8DevReg[s_u8DevMRn+3]<<24) | (g_au8DevReg[s_u8DevMRn+2]<<16) | (g_au8DevReg[s_u8DevMRn+1]<<8) | (g_au8DevReg[s_u8DevMRn+0]<<0));
        i3cs->TXRXDAT = ((g_au8DevReg[s_u8DevMRn+7]<<24) | (g_au8DevReg[s_u8DevMRn+6]<<16) | (g_au8DevReg[s_u8DevMRn+5]<<8) | (g_au8DevReg[s_u8DevMRn+4]<<0));
        i3cs->TXRXDAT = ((g_au8DevReg[s_u8DevMRn+11]<<24) | (g_au8DevReg[s_u8DevMRn+10]<<16) | (g_au8DevReg[s_u8DevMRn+9]<<8) | (g_au8DevReg[s_u8DevMRn+8]<<0));
        i3cs->TXRXDAT = ((g_au8DevReg[s_u8DevMRn+15]<<24) | (g_au8DevReg[s_u8DevMRn+14]<<16) | (g_au8DevReg[s_u8DevMRn+13]<<8) | (g_au8DevReg[s_u8DevMRn+12]<<0));
        i3cs->TXRXDAT = (dummy_pec<<0);
    }
    else
    {
        WRNLOG("[WARN] CMD: %d does not support.\n\n", s_u8DevCMD);
    }

    return 0;
}

/* For I2C mode and I3C mode w/ PEC Disabled
-----------------------------------------------	
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------	
|       | MRn register index          |	A |   |
-----------------------------------------------	
|       | Data	                      | A | P |
-----------------------------------------------	
*/
static int32_t _MasterWriteHandler(I3CS_T *i3cs)
{
    uint8_t MRn, Val;
    
    MRn = GET_MR_IDX; // Rx byte-0
    Val = GET_MR_VAL; // Rx byte-1
    
    if (MRn == 36)
    {
        // bit-0: LLSI enable bit
        if (Val & BIT0)
        {
            /* Read setting from MR register */
            ApplyLEDSetting();

            /* Start to flash LED */
            LLSI_StartFlashLED(1);
        }
        else
        {
            /* Stop to flash LED */
            LLSI_StartFlashLED(0);
        }
    }
    else if (MRn == 43)
    {
        /* Write LED number by pixel count */
        LLSI_WriteData(LED_SYNCDAT_POS, Val);
    }
    else
    {        
        DevReg_WriteReg(MRn, Val);
    }
    
    return 0;
}

/* For I2C mode and I3C mode w/ PEC Disabled
-----------------------------------------------	
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------	
|       | MRn register index          |	A |   |
-----------------------------------------------	
|       | Data	                      | A |   |
-----------------------------------------------	
|       | ...	                      | A |   |
-----------------------------------------------	
|       | Data	                      | A | P |
-----------------------------------------------	
*/
static int32_t _MasterBlockWriteHandler(I3CS_T *i3cs, uint16_t uLen)
{
    uint8_t MRn, block_size, block_mode;
    
    MRn        = GET_MR_IDX; // Rx byte-0
    block_size = GET_MR_VAL; // Rx byte-1, defined as block write size according the mainboard's spec.
    block_mode = _BYTE(2); 
    
    /*
        *** LED pixel x 3 ***
        [ Host Write ] 47-12-0x00-0x00-0x03-0xff-0x00-0x00-0x00-0xff-0x00-0x00-0x00-0xff
                          === ~ 2nd byte, block size = 12 
                             ==== ~ block mode, RSVD         
                                  ========= ~ number of LED         
                                            ============================================ ~ 9 bytes RGB ddata (R-G-B x3)
    */
    
    if (block_size == (uLen-2))
    {    
        if (MRn == 47)
        {
            g_au8DevReg[40] = _BYTE(3); // LED_CFG_NUM[8]
            g_au8DevReg[39] = _BYTE(4); // LED_CFG_NUM[7:0]
            
            ApplyLEDSetting();
             
            if (LED_CFG_NUM == 0)
                return -1;
            if (LED_CFG_NUM > cStrip1_LED)
                return -1;
            
            LLSI_WriteBlockData((LED_CFG_NUM * 3), (s_p8DevRx + 5));
        }
        else
        {
            /* Not MR47 */
            return -1;
        }
    }
    else
    {
        /* Block size mismatch */
        return -1;
    }

    return 0;
}

// TODO: not yet referenced
/*
## Command Truth Table (CMD) w/ PEC Enabled
----------------------------------------------------------------
| Command Description	    | Command Name | Command Code | RW |
----------------------------------------------------------------
| Write 1 Byte to Register  | W1R          | 000          | 0  |
----------------------------------------------------------------
| Read 1 Byte from Register | R1R          | 000          | 1  |
----------------------------------------------------------------
| Write 2 Byte to Register  | W2R          | 001          | 0  |
----------------------------------------------------------------
| Read 2 Byte from Register | R2R          | 001          | 1  |
----------------------------------------------------------------
NOTE: Command Code defined at 2nd byte[7:5], RW defined at 2nd byte[4].    
*/
/* For I3C mode with w/ Enabled
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	T |   |
-----------------------------------------------	
|       | 3-bits CMD | W = 0 | 0000   | T |   |
-----------------------------------------------	
|       | Data	                      | T |   |
-----------------------------------------------	
|       | ...	                      | T |   |
-----------------------------------------------	
|       | Data	                      | T |   |
-----------------------------------------------
|       | PEC	                      | T | P |
-----------------------------------------------
*/
static int32_t _MasterWriteHandlerPEC(I3CS_T *i3cs)
{
    uint8_t MRn;
    volatile uint16_t i, start_idx = 2, w_cnt = 0;
    
    /* Get MRn value (1st byte) */
    MRn = _BYTE(0); 
    
    /* Limited MRn to MAX_DEVREG_LEN */
    if (MRn >= MAX_DEVREG_LEN)
        return -1;    
    
    /* Get MRn value (1st byte) */
    s_u8DevMRn = MRn;
    
    /* Get CMD value (2nd byte) */
    s_u8DevCMD = (_BYTE(1) >> 5);
    
    /* Data payload start from 3rd byte */
    start_idx = (3 - 1);
    if (s_u8DevCMD == 0)
    {
        /* Command Name : W1R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-1))
        {
            WRNLOG("Write MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -1;
        }
        
        w_cnt = 1;
    }
    else if (s_u8DevCMD == 1)        
    {
        /* Command Name : W2R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-2))
        {
            WRNLOG("Write MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -2;
        }
        
        w_cnt = 2;
        for(i=0; i<w_cnt; i++)
            DevReg_WriteReg((s_u8DevMRn + i), _BYTE(start_idx + i));
        
        g_u8DevPEC = _BYTE(start_idx + i);
    }
    else if (s_u8DevCMD == 2)
    {
        /* Command Name : W4R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN-4))
        {
            WRNLOG("Write MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -4;
        }
        
        w_cnt = 4;
    }
    else if (s_u8DevCMD == 3)
    {
        /* Command Name : W16R */
        if (s_u8DevMRn > (MAX_DEVREG_LEN - 16))
        {
            WRNLOG("Write MRn has overflow. CMD %d, MRn %d.\n", s_u8DevCMD, s_u8DevMRn);
            return -16;
        }
        
        w_cnt = 16;
    }
    else
    {
        WRNLOG("[WARN] CMD: %d does not support\n\n", s_u8DevCMD);
    }
    
    for(i=0; i<w_cnt; i++)
        DevReg_WriteReg((s_u8DevMRn + i), _BYTE(start_idx + i));
    
    g_u8DevPEC = _BYTE(start_idx + i);
    
    return 0;
}

static int32_t _ProcessI3CSRespQueue(I3CS_T *i3cs)
{
    RESP_QUEUE_T    *pRespQ;
    uint32_t        u32RespSts;
    int32_t         ret = -1;
    volatile uint32_t i;

    pRespQ = (RESP_QUEUE_T *)&s_DevRespQue[0];
    pRespQ->w = I3CS_GET_RESP_DATA(i3cs);
    u32RespSts = (pRespQ->w & I3CS_RESPQUE_ERRSTS_Msk);

    if (u32RespSts == I3CS_RESP_NO_ERR)
    {
        /* I3CS NO Error */
        
        if (pRespQ->b.RXRSP == 1)
        {
            /* I3CS RX_resp and no error */
            
            {
                // Receive all master write data
                volatile uint32_t i;
                for (i=0; i<((pRespQ->b.LENGTH+3)/4); i++)
                    s_DevRxBuf[i] = i3cs->TXRXDAT;
            }
       
            g_u32FifoClr = 0; // not referenced
            
            /*
                I2C mode: 1-byte addressing mode.
                I3C mode: 1-byte addressing mode when PEC mode was disabled,
                          2-bytes addressing mode when PEC mode was enabled and support CMD while PEC mode enabled.
            */
            
            if (I3CS_IS_DA_VALID(i3cs) == 0)
            {
                /* I2C mode using 1-bytes addressing mode */
                
                if (pRespQ->b.LENGTH >= 2)
                {
                    // master write
                    if (pRespQ->b.LENGTH == 2)
                    {
                        /* MRn(!=47) + data */
                        _MasterWriteHandler(i3cs);
                    }
                    else
                    {
                        /* Only for : MR47 + block size + block mode + 2-byte led_cnt + data0~n(led_cntx3) */
                        _MasterBlockWriteHandler(i3cs, pRespQ->b.LENGTH);
                    }
                }
                else
                {
                    // master write address -> read
                    // receive (MRn)
                    // 1-byte addressing mode, only in I2C mode
                    // read: -O0: cannot write data to TXRXD register before send FIFO out. So response NACK.
                    //       -O3: can write data to TXRXD register before send FIFO out. So response correct data.
                    if (pRespQ->b.LENGTH != 0)
                    {
                        _SlaveCfgReadHandler(i3cs);
                    }
                    else
                    {
                        DBGLOG("Default read address pointer\n");
                    }
                }
                ret = 0;
            }
            else
            {
                /* I3C mode */
                
                if (SPDH_IsDEVPECEnable() == 0)
                {
                    // 1-byte addressing mode
                    // PEC was disabled in I3C mode
                    if (pRespQ->b.LENGTH >= 2)
                    {
                        // master write
                        if (pRespQ->b.LENGTH == 2)
                        {
                            /* MRn(!=47) + data */
                            _MasterWriteHandler(i3cs);
                        }
                        else
                        {
                            /* Only for : MR47 + block size + block mode + 2-byte led_cnt + data0~n(led_cntx3) */
                            _MasterBlockWriteHandler(i3cs, pRespQ->b.LENGTH);
                        }
                    }
                    else
                    {
                        // master write address -> read
                        // receive (MRn)
                        // read: -O0: cannot write data to TXRXD register before send FIFO out. So response NACK.
                        //       -O3: can write data to TXRXD register before send FIFO out. So response correct data.
                        if (pRespQ->b.LENGTH != 0)
                        {
                            _SlaveCfgReadHandler(i3cs);
                        }
                        else
                        {
                            DBGLOG("Default read address pointer\n");
                        }
                    }
                }
                else
                {
                    // 2-bytes addressing mode
                    // PEC was enalbed in I3C mode
                    if (pRespQ->b.LENGTH >= 4)
                    {
                        // master write
                        if ((GET_MR_VAL & BIT4) == 0) // 2nd byte[4], check RnW bit = 0
                        {
                            _MasterWriteHandlerPEC(i3cs);
                        }
                        else
                        {
                            ERRLOG("\t[ERR]W bit and package length are not matched.(0x%08x)(L:%d)\n", s_DevRxBuf[0], __LINE__);
                        }
                    }
                    else
                    {
                        // master write address -> read
                        // receive (MRn + CMD + PEC)
                        // read: -O0: cannot write data to TXRXD register before send FIFO out. So response NACK.
                        //       -O3: can write data to TXRXD register before send FIFO out. So response correct data.
                        if (pRespQ->b.LENGTH != 0)
                        {
                            if (GET_MR_VAL & BIT4) // 2nd byte[4], check RnW bit = 1
                            {
                                _SlaveCfgReadHandlerPEC(i3cs);
                            }
                            else
                            {
                                ERRLOG("\t[ERR] R bit and package length are not matched. (0x%08x)(L:%d)\n", s_DevRxBuf[0], __LINE__);
                            }
                        }
                        else
                        {
                            DBGLOG("Default read address pointer\n");
                        }
                    }
                }
                ret = 0;
            }
        }
        else
        {
            /* I3CS TX_resp */
            ret = 1;
        }
    }
    else
    {
        /* I3CS Has Error */
        DBGLOG("\t[I3CS%d] Error status 0x%x\n", ((i3cs==I3CS0)? 0:1), pRespQ->b.STATUS);
        
        /*
            Note:
            The Slave controller NACKs all transfers once it has encountered an error until RESUME bit
            is set in the DEVICE_CTRL register from the Slave application and until the error status is cleared from the
            CCC_DEVICE_STATUS register by GETSTATUS CCC.
            For any other error status like underflow error or Master Early termination,
            the Slave application is expected to reset the TX FIFO and CMD FIFO before applying the resume in DEVICE_CTRL register.
        */
        if (i3cs->CCCDEVS & I3CS_CCCDEVS_PROTERR_Msk) // Protocol error: This bit is set when the slave controller encouters a Parity/CRC error during write data transfer.
        {
            /* Set MR52[0]: PAR_ERROR_STATUS */
            DevReg_SetParityErrStatus();
            WRNLOG("[WARN]Parity error\n");
        }
        
        LocalDev_RespErrorRecovery(i3cs, u32RespSts);
        ret = -1;
    }

    return ret;
}

/*
    judge only write and write then read cases.
    default address pointer mode has enable bit to let firmware knows to changes to normal read/write or default address pointer.
*/
/*
    Note:
    The Slave controller NACKs all transfers once it has encountered an error until RESUME bit
    is set in the DEVICE_CTRL register from the Slave application and until the error status is cleared from the
    CCC_DEVICE_STATUS register by GETSTATUS CCC.
    For any other error status like underflow error or Master Early termination,
    the Slave application is expected to reset the TX FIFO and CMD FIFO before applying the resume in DEVICE_CTRL register.
*/
static void __LocalDev_ParseI3CSIntStatus(I3CS_T *i3cs)
{
    volatile uint32_t u32I3CINTSts, u32IBISts;

    u32I3CINTSts = i3cs->INTSTS;

    // I3CS_INTSTS_RESPRDY_Msk
    if (u32I3CINTSts & I3CS_INTSTS_RESPRDY_Msk) // auto cleared
    {
        /* To process Master data */
        _ProcessI3CSRespQueue(i3cs);
    }

    // I3CS_INTSTS_DAA_Msk
    if (u32I3CINTSts & I3CS_INTSTS_DAA_Msk)
    {
        DBGLOG("INT DYNAASTS (I3CS%d DA: 0x%02x)\n", ((i3cs==I3CS0)? 0:1), (uint32_t)I3CS_GET_I3CS_DA(i3cs));
        I3CS_CLEAR_DA_ASSIGNED_STATUS(i3cs);

        SPDH_EnableDEVSIR();

        /* Update MR18[5] NF_SEL bit as I3C basic protocol. */
        DevReg_EnableI3C();

        g_u32DeviceChangedToI3CMode = 1;

        __NOP();
        /* To check if mode of device behind Hub and I3C slave is matched? */
        if (SPDH_GetDEVMode() == 0)
        {
            WRNLOG("[WARN] Mode of device behind Hub and I3C slave is not matched. (#%d)\n\n", __LINE__);
        }
    }

    // I3CS_INTSTS_READREQ_Msk
    if (u32I3CINTSts & I3CS_INTSTS_READREQ_Msk)
    {
        DBGLOG("INT READ_REQUEST\n");
        I3CS_CLEAR_READ_REQUEST_STATUS(i3cs);
    }

    // I3CS_INTSTS_TFRERR_Msk
    if (u32I3CINTSts & I3CS_INTSTS_TFRERR_Msk)
    {
        DBGLOG("INT TRANSFER_ERR\n");
        I3CS_CLEAR_TRANSFER_ERR_STATUS(i3cs);
    }

    // I3CS_INTSTS_IBIUPD_Msk
    if (u32I3CINTSts & I3CS_INTSTS_IBIUPD_Msk)
    {        
        DBGLOG("INT IBIUPD\n");
        I3CS_CLEAR_IBI_UPDATED_STATUS(i3cs);
        
        u32IBISts = i3cs->SIRRESP;
        DBGLOG("IBI_RESP: 0x%08x\n", u32IBISts);
        if ((u32IBISts & I3CS_SIRRESP_IBISTS_Msk) == I3CS_IBI_ACCEPTED)
        {
            DBGLOG("IBI accepted by the Master (ACK response received)\n");
            /* Clear pending status to 0 */
            i3cs->DEVCTL &= ~I3CS_DEVCTL_PENDINT_Msk;
            
            /* Clear MR48[7] : IBI_STATUS */
            DevReg_ClearIBIStatus();
        }
        else if ((u32IBISts & I3CS_SIRRESP_IBISTS_Msk) == I3CS_IBI_MASTER_TERMINATE)
        {
            DBGLOG("Master Early Terminate (only for SIR with Data)\n");
        }
        else if ((u32IBISts & I3CS_SIRRESP_IBISTS_Msk) == I3CS_IBI_NOT_ATTEMPTED)
        {
            DBGLOG("IBI Not Attempted\n");
        }
    }

    // I3CS_INTSTS_CCCUPD_Msk
    if (u32I3CINTSts & I3CS_INTSTS_CCCUPD_Msk)
    {
        DBGLOG("INT CCCUPD (I3CS%d DA: 0x%02x)\n", ((i3cs==I3CS0)? 0:1), (uint32_t)I3CS_GET_I3CS_DA(i3cs));
        I3CS_CLEAR_CCC_UPDATED_STATUS(i3cs);
        
        /* This interrupt is generated if any of the CCC registers are updated by I3C master through CCC commands. */
        if (i3cs->SLVEVNTS & I3CS_SLVEVNTS_SIREN_Msk)
        {
            /* SIR_EN(SLV_EVENT_STATUS[0]), Slave Interrupt Request Enable, this bit is set by ENEC */
            /* Set MR27[4]: IBI_ERROR_EN, In Band Error Interrupt Enable */
            DevReg_IBIEnable();
        }
        else
        {
            /* SIR_EN(SLV_EVENT_STATUS[0]), Slave Interrupt Request Enable, this bit is clear by DISEC */
            /* Clear MR27[4]: IBI_ERROR_EN, In Band Error Interrupt Enable */
            DevReg_IBIDisable();
        }
    }
}

void I3CS1_IRQHandler(void)
{
    __LocalDev_ParseI3CSIntStatus(I3CS1);
}

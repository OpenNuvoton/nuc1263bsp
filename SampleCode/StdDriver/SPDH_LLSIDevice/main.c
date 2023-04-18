/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    LLSI device firmware.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "HubFw.h"
#include "LocalDevReg.h"
#include "LocalDevFw.h"

#define I3CS1_SA         (0x67) /* LID is 6, default HID is 111 before received SETHID CCC command */

volatile uint32_t g_DetectedPowerDown = 0;

volatile uint32_t g_u32FifoClr = 0;

extern int8_t _ResetTxFifoOnly(I3CS_T * i3c);
extern int8_t _ResetRxFifoOnly(I3CS_T * i3c);
volatile uint32_t   g_u32IntSelMask = 0, g_u32IntOccurredMask = 0;

void I3CSStatusHandler(void)
{
    uint32_t u32IntSts;

    extern volatile RESP_QUEUE_T   g_DevRespQue[I3CS_DEVICE_RESP_QUEUE_CNT] __attribute__((aligned(4)));

    u32IntSts = I3CS_GET_INTSTS(I3CS1);
    if (u32IntSts != 0)
    {
        if (u32IntSts != 0x9)
        {
            WRNLOG("\t[M][I3C1]INT STS 0x%x\n", u32IntSts);
            if (u32IntSts&BIT11)
            {
                WRNLOG("\t[M]I[I3C1]NT status: READ_REQ_RECV_STS, CMDQ is empty\n");
                //I3CS1->INTSTS = BIT11;
            }
            if (u32IntSts & BIT9)
            {
                WRNLOG("\t[M][I3C1]INT status: transfer error (RESP Q :0x%08x)\n", I3CS1->RESPQUE);
                //I3CS1->INTSTS = BIT9;

                //printf("\tSet I3CS0 RESUME ... \n");
                //I3CS0->CTL |= I3CS_CTL_RESUME_Msk;
            }
        }
    }
    if (I3CS1->CCCDEVS != 0)
    {
        WRNLOG("\t[M][I3C1]Dev status 0x%x\n", I3CS1->CCCDEVS);
        if (I3CS1->CCCDEVS & BIT12)
            WRNLOG("\t[M][I3C1]Dev status: Buffer not available \n");
        if (I3CS1->CCCDEVS & BIT5)
        {
            WRNLOG("\t[M][I3C1]Dev status: protocol error(parity error)(RESP Q :0x%08x)\n", I3CS1->RESPQUE);

            //while((I3CS0->CTL&I3CS_CTL_RESUME_Msk) != 0) {}//wait for Host send GETSTATUS CCC, then RESUME bit is became to 0
            //printf("done\n");
            if (u32IntSts & BIT9)
            {
                WRNLOG("\t[M][I3C1]INT status: transfer error (RESP Q :0x%08x)\n", I3CS1->RESPQUE);
                //I3CS1->INTSTS = BIT9;

                //printf("\tSet I3CS0 RESUME ... \n");
                //I3CS0->CTL |= I3CS_CTL_RESUME_Msk;
            }
        }

    }
}

void SPDHIRQHandler(void)
{
    uint32_t u32SpdhSts;
    uint8_t u8StaticAddr, u8LID;

    u32SpdhSts = SPDH->INTSTS;

    if(u32SpdhSts & SPDH_INTSTS_BUSRTOIF_Msk)
    {
        DBGLOG("SPDH_IRQ: BUS Reset Time-out\n");
        SPDH->INTSTS = SPDH_INTSTS_BUSRTOIF_Msk;

        /* Do chip reset by software, because Host sent bus reset to all device on the same bus. */
        SYS_ResetChip();
    }

    if (u32SpdhSts & SPDH_INTSTS_DSETHIDIF_Msk)
    {
        DBGLOG("\nLocal device received SETHID\n");

        u8LID = (I3CS1->DEVADDR & SPDH_DCTL_LID_Msk);
        u8StaticAddr = (uint8_t)(u8LID | (SPDH->DCTL & SPDH_DCTL_HID_Msk));
        DBGLOG("u8StaticAddr: 0x%08X\n", u8StaticAddr);

        /* Change HID of I3C static address after received SETHID CCC command. */
        I3CS1->DEVADDR = (I3CS1->DEVADDR &~I3CS_DEVADDR_SA_Msk) | (I3CS_DEVADDR_SAVALID_Msk | u8StaticAddr);
        DBGLOG("I3C1->DEVADDR: 0x%08X\n", I3CS1->DEVADDR);
        SPDH->INTSTS = SPDH_INTSTS_DSETHIDIF_Msk;
    }

    if (u32SpdhSts & SPDH_INTSTS_DEVPCIF_Msk)
    {
        DBGLOG("\n Local device has PEC error occurred.\n");

        SPDH->INTSTS = SPDH_INTSTS_DEVPCIF_Msk;
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);
    
    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
//    SET_UART0_RXD_PD2();
//    SET_UART0_TXD_PD3();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    /* Initialize local device */
    LocalDev_Init(I3CS1_SA);

    /* Initialize LLSI controller */
    LLSI_Initial();

    printf("+-----------------------------------------------------+\n");
    printf("| The Local LLSI device firmware code behind SPD5 Hub |\n");
    printf("+-----------------------------------------------------+\n\n");
    printf("    - I2C Static Address 0x%x\n", I3CS1_SA);
    while(1)
    {
        if (g_DetectedPowerDown)
        {
            g_DetectedPowerDown = 0;
            printf("system enter power down ...\n\n");
            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            //system enter pwoer down now
            CLK_PowerDown();
        }

        /* Check and handle I3CS status */
        I3CSStatusHandler();

        /* Check if interface has changed from I3C to I2C mode.  */
        LocalDev_CheckInterfaceSel();

        /* Execute the LLSI flash LED routine. */
        if (LLSI_FlashLEDRoutine() == 1)
        {
            /* Update LLSIEN(MR36[0]) as 0 while flash LED has finished. */
            DevReg_LLSIEnable(0);
        }
        
        /* Check if need to send IBI request. */
        LocalDev_CheckIBIReg();
    }
}

void SPDH_IRQHandler(void)
{
    /* Call the interrupt handler of SPDH */
    SPDHIRQHandler();
    /* Call the interrupt handler in LocalDevFw.c */
    LocalDev_SPDHIRQHandler();

}


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

#include "LocalDevReg.h"
#include "LocalDevFw.h"

#define I3CS1_SA        (0x67)  /* 01100111: SA[6:3] LID is 1100b, SA[2:0] HID is default 111b before received SETHID CCC command */


#if (SPDH_DETECT_POWER_DOWN == 1)
volatile uint32_t g_DetectedPowerDown = 0;
#endif

volatile uint32_t g_u32FifoClr = 0;

static void ProcessDeviceStatus(void)
{
    if (I3CS1->CCCDEVS != I3CS_STS_NO_ERR)
    {
        WRNLOG("\t[M][I3C1]Dev status 0x%x\n", I3CS1->CCCDEVS);
        
        if (I3CS1->CCCDEVS & I3CS_CCCDEVS_BFNAVAIL_Msk)
        {
            WRNLOG("\t[M][I3C1]Dev status: Buffer not available \n");
        }

        if (I3CS1->CCCDEVS & I3CS_CCCDEVS_PROTERR_Msk)
        {
            WRNLOG("\t[M][I3C1]Dev status: protocol error(parity error)(RESP Q :0x%08x)\n", I3CS1->RESPQUE);
        }

        if (I3CS1->CCCDEVS & I3CS_CCCDEVS_DATNRDY_Msk)
        {
            WRNLOG("\t[M][I3C1]Dev status: Data not ready \n");
            WRNLOG("[M][I3C1]cmd queue lv: 0x%08x\n", I3CS1->QUESTSLV);
            /*
               This bit is set when private read request from master is NACKED
               because Command FIFO empty/Transmit FIFO threshold is not meet/Response FIFO full
            */
            /* Host needs to send read command again because last read data was lost.
               But if next command is write command, slave cannot write the same data to TX FIFO.
               limitation: The next write command cannot to assign a different MRn.
            */
        }
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
#if 0    
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
#else    
    // NUC1263_DDR5_ARGB_DEMO_BOARD_v1.1
    SET_UART0_RXD_PD2();
    SET_UART0_TXD_PD3();
#endif    
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

    printf("\n\nCPU @ %d Hz new\n", SystemCoreClock);

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
#if (SPDH_DETECT_POWER_DOWN == 1)
        if (g_DetectedPowerDown)
        {
            g_DetectedPowerDown = 0;
            printf("system enter power down ...\n\n");
            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            //system enter pwoer down now
            CLK_PowerDown();
        }
#endif

        /* Check and handle I3CS status */
        ProcessDeviceStatus();

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

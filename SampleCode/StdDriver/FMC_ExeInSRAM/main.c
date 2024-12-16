/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 20/11/27 $
 * @brief
 *           Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK Only)
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if defined (__ICCARM__)
# define __RAMFUNC  __ramfunc
#elif defined (__CC_ARM) || defined (__ARMCC_VERSION)
# define __RAMFUNC
#elif defined (__GNUC__)
# define __RAMFUNC __attribute__((long_call, section(".ramfunc")))
#endif

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

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


__RAMFUNC int32_t Flash_Test(void)
{
    uint32_t u32Addr, u32Data, u32RData;
    int32_t i, i32Timeout;
    
    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;
    FMC_Erase(u32Addr); /* Erase page */
    for(i = 0; i < 0x100; i += 4)
    {

        /* Write Demo */
        u32Data = i + 0x12345678;
        FMC_Write(u32Addr + i, u32Data);

        if((i & 0xf) == 0)
        {
            DEBUG_PORT->DAT = '.';
            i32Timeout = 0x10000;
            while((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0)
            {
                /* timeout */
                if(i32Timeout-- <= 0)
                    return -1;
            }
        }

        /* Read Demo */
        u32RData = FMC_Read(u32Addr + i);

        if(u32Data != u32RData)
        {
            return -1;
        }
    }
    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;
    
    return 0;
}


int main()
{

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Initial UART */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
       For Keil,
       By setting scatter loading file (scatter.scf),
       RO code is placed to SRAM.

       For IAR, GCC
       Compiler key word is used to assign specified function to SRAM.

    */
    
    /* Enable FMC ISP functions */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    
    printf("Will branch to address: 0x%x\n", (uint32_t)Flash_Test);
    
    Flash_Test();

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}


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

int32_t g_FMC_i32ErrCode;

void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as PLL/2 and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL_DIV2;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

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

    /* Enable FMC ISP functions */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk;

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

    printf("Will branch to address: 0x%x\n", (uint32_t)Flash_Test);
    
    Flash_Test();

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}

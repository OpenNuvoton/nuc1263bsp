/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to set SPROM in security mode.
 *           When SPROM is in security mode, user only can see zero data in SPROM area.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC1263.h"




/* check CBS and flash lock before running this sample code,
   BS should be APROM or APROM with IAP, flash lock should be disabled,
   please use target option -> Utilities -> setting -> config to check */

extern void SPROM_Function(void);

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
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void SPROMDump(void)
{
    volatile uint32_t  addr;
    printf("Dump SPROM from 0x200000 to 0x2007FF\n");

    for(addr = FMC_SPROM_BASE; addr <= 0x2007FF; addr += 4)
    {
        if((addr % 16) == 0)
            printf("\n0x%08x: ", addr);

        printf("0x%08x ", inpw(addr));
    }

    printf("\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Enable FMC ISP function and SPROM update */
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_SPUEN_Msk);

    printf("\n\n");

    if(FMC_Read(FMC_SPROM_BASE + 0x7FC) == 0xFFFFFFFF)
    {
        printf("======= SPROM is in non-security mode ========\n");
        printf("User should see valid data on SPROM with non-security mode\n");
        SPROMDump();
        SPROM_Function();
        printf("--------------------------------------------------------\n");
        printf("press any key to enter in SPROM security mode....\n");
        getchar();
        /* the program unit is 4 bytes, hence we program 0x2007FC~0x2007FF to change mode.
           Write 0x0 on 0x2007FF to change mode to security mode,
           user also can write other data on this position except 0xFF(non-security mode) and 0xAA(debug mode) */
        FMC_Write(FMC_SPROM_BASE + 0x7FC, 0x00FFFFFF);
        /* setting SPROM in security mode would take effect after reset chip */
        SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
    }

    printf("======= SPROM is in security mode =======\n");
    printf("User should see all zero data on SPROM with security mode\n");
    SPROMDump();
    SPROM_Function();
    printf("--------------------------------------------------------\n");

    /* Disable FMC ISP function */
    FMC->ISPCTL &= (~FMC_ISPCTL_ISPEN_Msk);

    /* Lock protected registers */
    SYS_LockReg();

    printf("If user want to rerun this test,\nplease do flash again by ICE to erase SPROM area and flash back SPROM.o\n");

    while(1);
}

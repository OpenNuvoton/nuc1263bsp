/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 20/11/27 $
 * @brief
 *           Show how to call LDROM functions from APROM.
 *           The code in APROM will look up the table at 0x100E00 to get the address of function of LDROM and call the function.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define FUNCT_SIGN  0x46554354      /* Signature for function table */
#define KEY_ADDR    0x20000FFC      /* The location of signature */
#define SIGNATURE   0x21557899      /* The signature word is used by AP code to check if simple LD is finished */

extern const uint32_t loaderImageBase;
extern const uint32_t loaderImageLimit;
#define IMG_BASE    (uint32_t)&loaderImageBase
#define IMG_LIMIT   (uint32_t)&loaderImageLimit

uint32_t g_u32ImageSize;
int32_t g_FMC_i32ErrCode;
uint32_t *g_au32funcTable; /* The location of function table */

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

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


void UART0_Init()
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

void FMC_LDROM_Test(void)
{
    int32_t  i32Err;
    uint32_t u32Data, i, *pu32Loader;

    /* Enable LDROM Update */
    FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;

    printf("  Erase LD ROM ............................... ");

    /* Page Erase LDROM */
    for(i = 0; i < FMC_LDROM_SIZE; i += FMC_FLASH_PAGE_SIZE)
        FMC_Erase(FMC_LDROM_BASE + i);

    /* Erase Verify */
    i32Err = 0;

    for(i = FMC_LDROM_BASE; i < (FMC_LDROM_BASE + FMC_LDROM_SIZE); i += 4)
    {
        u32Data = FMC_Read(i);

        if(u32Data != 0xFFFFFFFF)
        {
            i32Err = 1;
        }
    }

    if(i32Err)
        printf("[FAIL]\n");
    else
        printf("[OK]\n");

    printf("  Program Simple LD Code ..................... ");
    pu32Loader = (uint32_t *)IMG_BASE;
    g_u32ImageSize = IMG_LIMIT - IMG_BASE;
    for(i = 0; i < g_u32ImageSize; i += 4)
    {
        FMC_Write(FMC_LDROM_BASE + i, pu32Loader[i / 4]);
    }

    /* Verify loader */
    i32Err = 0;
    for(i = 0; i < g_u32ImageSize; i += 4)
    {
        u32Data = FMC_Read(FMC_LDROM_BASE + i);
        if(u32Data != pu32Loader[i / 4])
            i32Err = 1;
    }

    if(i32Err)
    {
        printf("[FAIL]\n");
    }
    else
    {
        printf("[OK]\n");
    }
}

/*
    Show the config of FMC CONFIG0
*/
void ShowConfig(uint32_t u32Cfg)
{
    if((u32Cfg & 1) == 0)
        printf("Data Flash Eanbled\n");
    if((u32Cfg & 2) == 0)
        printf("Secure Locked\n");
    if((u32Cfg & 0x80000018) != 0x80000018)
        printf("WDT Enabled\n");
    if((u32Cfg & 0xc0) == 0)
        printf("Boot from LDROM with IAP mode\n");
    if((u32Cfg & 0xc0) == 0x40)
        printf("Boot from LDROM without IAP mode\n");
    if((u32Cfg & 0xc0) == 0x80)
        printf("Boot from APROM with IAP mode\n");
    if((u32Cfg & 0xc0) == 0xc0)
        printf("Boot from APROM without IAP mode\n");
    if((u32Cfg & 0x400) == 0)
        printf("GPIO Default to Tri-state\n");
    else
        printf("GPIO Default to Quasi-bidirection\n");
    if((u32Cfg & 0x1000) == 0)
        printf("ICE Locked\n");
    

}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Data, u32Addr;
    uint32_t u32Cfg;
    int32_t (*func)(int32_t n);
    int32_t i;

    /*
        This sample code is used to demo how IAP works.
        In other words, it shows how to call functions of LDROM code in APROM.

        The execution flow of this code is:
        1. Make sure FMC_LD.bin is existed. User can select "FMC_IAP_LD" target to build it.
        2. The code will ask user to update LDROM code. User may press 'y' to update LDROM with "FMC_IAP_LD.bin".
        3. The code will call the functions in LDROM by function table located at 0x100e00.
    */

    /* Unlock protected registers for ISP function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------+\n");
    printf("|                          IAP Sample Code                         |\n");
    printf("+------------------------------------------------------------------+\n");

    printf("\nCPU @ %dHz\n\n", SystemCoreClock);


    /* Enable ISP function */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    /* Check IAP mode */
    u32Cfg = FMC_Read(FMC_CONFIG_BASE);
		
    printf("u32Cfg = 0x%08x\n", (uint32_t)u32Cfg);
    ShowConfig(u32Cfg);
		
    if((u32Cfg & 0xc0) != 0x80)
    {
        printf("Do you want to set to new IAP mode (APROM boot + LDROM) (y/n)?\n");
        if(getchar() == 'y')
        {
            FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk; /* Enable user configuration update */

            /* Set CBS to b'10 */
            u32Cfg &= ~0xc0ul;
            u32Cfg |= 0x80;
            u32Data = FMC_Read(FMC_CONFIG_BASE + 0x4); /* Backup the data of config1 */
            FMC_Erase(FMC_CONFIG_BASE);
            FMC_Write(FMC_CONFIG_BASE, u32Cfg);
            FMC_Write(FMC_CONFIG_BASE + 0x4, u32Data);

            printf("Press any key to reset system to enable new IAP mode ...\n");
            getchar();
            SYS->IPRST0 = 0x1; /* Reset MCU */
            while(1);
        }
        else
        {
            goto lexit;
        }
    }

    printf("Do you want to write LDROM code to 0x100000 [y/n]\n");

    if(getchar() == 'y')
    {
        /* Check LD image size */
        g_u32ImageSize = IMG_LIMIT - IMG_BASE;

        if(g_u32ImageSize == 0)
        {
            printf("  ERROR: Loader Image is 0 bytes!\n");
            goto lexit;
        }

        if(g_u32ImageSize > FMC_LDROM_SIZE)
        {
            printf("  ERROR: Loader Image is larger than 4K Bytes!\n");
            goto lexit;
        }

        /* Erase LDROM, program LD sample code to LDROM, and verify LD sample code */
        FMC_LDROM_Test();
    }

    /* Seek funciton table */
    for(u32Addr = FMC_LDROM_BASE + g_u32ImageSize - 4; u32Addr >= FMC_LDROM_BASE; u32Addr -= 4)
    {
        if(M32(u32Addr) == FUNCT_SIGN)
        {
            g_au32funcTable = (uint32_t *)(u32Addr + 4);
            break;
        }
    }
    
    if(u32Addr == FMC_LDROM_BASE)
    {
        printf("Cannot find function table signature.\n");
        goto lexit;
    }
    
    printf("Function table at 0x%08x\n", (uint32_t)g_au32funcTable);
    
    for(i = 0; i < 4; i++)
    {
        /* Call the function of LDROM */
        func = (int32_t (*)(int32_t))g_au32funcTable[i];
        if(func(i + 1) == i + 1)
        {
            printf("Call LDROM function %d ok!\n", i);
        }
        else
        {
            printf("Call LDROM function %d fail.\n", i);
        }
    }

lexit:

    /* Disable ISP function */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    printf("\nDone\n");
    while(SYS->PDID);
}




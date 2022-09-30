/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to update chip flash data through I2C interface
 *           between chip I2C and ISP Tool.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip I2C and assign update file
 *           of Flash.
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

uint32_t u32Pclk0;
uint32_t u32Pclk1;

void ProcessHardFault(void) {}
void SH_Return(void) {}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        if (--u32TimeOutCnt == 0)
            return -1;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        if (--u32TimeOutCnt == 0)
            return -1;

    /* Select HCLK clock source as PLL/2 and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL_DIV2;

    /* Update System Core Clock */
    PllClock = 144000000;
    SystemCoreClock = 144000000 / 2;
    CyclesPerUs = SystemCoreClock / 1000000; /* For CLK_SysTickDelay() */

    /* Enable I2C1 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC multi-function pins for I2C1 SDA and SCL */
    SET_I2C1_SDA_PC4();
    SET_I2C1_SCL_PC5();
    
    /* I2C clock pin enable schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN5_Msk;

    return 0;
}

int main(void)
{
    uint32_t au8CmdBuff[16];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Enable ISP */
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    if (g_dataFlashAddr < g_apromSize)
    {
        g_dataFlashSize = (g_apromSize - g_dataFlashAddr);
    }
    else
    {
        g_dataFlashSize = 0;
    }

    /* Init I2C */
    I2C_Init();

    /* Set Systick time-out as 300ms */
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (1)
    {
        /* Wait for CMD_CONNECT command */
        if (u8I2cDataReady == 1)
        {
            goto _ISP;
        }
        /* Systick time-out, go to APROM */
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    /* Parse command from master and send response back */
    while (1)
    {
        if (u8I2cDataReady == 1)
        {
            /* Get command from I2C receive buffer */
            memcpy(au8CmdBuff, au8I2cRcvBuf, 64);
            u8I2cDataReady = 0;
            /* Parse the current command */
            ParseCmd((unsigned char *)au8CmdBuff, 64);
        }
    }

_APROM:

    /* Reset system and boot from APROM */
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1)
        ;
}

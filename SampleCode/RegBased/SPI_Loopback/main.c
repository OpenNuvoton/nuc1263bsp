/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate one SPI Master self-loopback transfer and two SPI 4-wire loopback transfer.
 *           SPI1 will be configured as Master mode and SPI0 will be configured as Slave mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <c> Two SPI port loopback transfer
//#define TwoPortLoopback
// </c>
// <o> GPIO Slew Rate Control
// <0=> Basic <1=> Higher <2=> Ultra higher
#define SlewRateMode        0
// *** <<< end of configuration section >>> ***

#define TEST_COUNT          64

/* Global variable declaration */
#ifndef TwoPortLoopback
static uint32_t s_au32SourceData[TEST_COUNT];
static uint32_t s_au32DestinationData[TEST_COUNT];
#else
static uint32_t s_au32MasterToSlaveTestPattern[TEST_COUNT];
static uint32_t s_au32SlaveToMasterTestPattern[TEST_COUNT];
static uint32_t s_au32MasterRxBuffer[TEST_COUNT];
static uint32_t s_au32SlaveRxBuffer[TEST_COUNT];
static volatile uint32_t s_u32MasterTxDataCount, s_u32MasterRxDataCount;
static volatile uint32_t s_u32SlaveTxDataCount, s_u32SlaveRxDataCount;
#endif

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
#ifndef TwoPortLoopback
    uint32_t u32TestCount, u32Err, u32TimeOutCnt;
#endif
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                       SPI Driver Sample Code                       |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
#ifndef TwoPortLoopback
    printf("This sample code demonstrates SPI1 self loop back data transfer.\n");
    printf(" SPI1 configuration:\n");
    printf("     Master mode; data width 32 bits.\n");
    printf(" I/O connection:\n");
    printf("     SPI1_MOSI(PC.2) <--> SPI1_MISO(PC.3) \n");
    printf("\nSPI1 Loopback test ");

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* Set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            s_au32SourceData[u32DataCount] = u32DataCount;
            s_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            /* Write to TX register */
            SPI_WRITE_TX(SPI1, s_au32SourceData[u32DataCount]);

            /* Check SPI1 busy status */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(SPI_IS_BUSY(SPI1))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for SPI busy flag is cleared time-out!\n");
                    u32Err = 1;
                    break;
                }
            }

            if(u32Err)
                break;

            /* Read received data */
            s_au32DestinationData[u32DataCount] = SPI_READ_RX(SPI1);
            u32DataCount++;
            if(u32DataCount >= TEST_COUNT)
                break;
        }

        if(u32Err)
            break;

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if(s_au32DestinationData[u32DataCount] != s_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Disable SPI1 peripheral clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_SPI1CKEN_Msk;
#else
    printf("This sample code demonstrates SPI0/SPI1 loop back transfer.\n\n");
    printf("Configure SPI0 as a slave and SPI1 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("Please connect below I/O connections for SPI0 and SPI1:\n");
    printf("    SPI0_SS  (PA.3)   <->   SPI1_SS  (PC.0)\n");
    printf("    SPI0_CLK (PA.2)   <->   SPI1_CLK (PC.1)\n");
    printf("    SPI0_MISO(PA.1)   <->   SPI1_MISO(PC.3)\n");
    printf("    SPI0_MOSI(PA.0)   <->   SPI1_MOSI(PC.2)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        s_au32MasterToSlaveTestPattern[u32DataCount] = 0x00550000 + u32DataCount;
        s_au32SlaveToMasterTestPattern[u32DataCount] = 0x00AA0000 + u32DataCount;
        /* Clear destination buffer */
        s_au32MasterRxBuffer[u32DataCount] = 0;
        s_au32SlaveRxBuffer[u32DataCount] = 0;
    }

    s_u32MasterTxDataCount = 0;
    s_u32MasterRxDataCount = 0;
    s_u32SlaveTxDataCount = 0;
    s_u32SlaveRxDataCount = 0;
    printf("Press any key to start transmission ...\n");
    getchar();
    printf("\n");

    /* Access TX and RX FIFO */
    while((s_u32MasterRxDataCount < TEST_COUNT) || (s_u32SlaveRxDataCount < TEST_COUNT))
    {
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (s_u32MasterTxDataCount < TEST_COUNT))
            SPI_WRITE_TX(SPI1, s_au32MasterToSlaveTestPattern[s_u32MasterTxDataCount++]); /* Write to TX FIFO */
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (s_u32SlaveTxDataCount < TEST_COUNT))
            SPI_WRITE_TX(SPI0, s_au32SlaveToMasterTestPattern[s_u32SlaveTxDataCount++]); /* Write to TX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
            s_au32MasterRxBuffer[s_u32MasterRxDataCount++] = SPI_READ_RX(SPI1); /* Read RX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
            s_au32SlaveRxBuffer[s_u32SlaveRxDataCount++] = SPI_READ_RX(SPI0); /* Read RX FIFO */
    }

    /* Print the received data */
    printf("\tSPI0 Received data:\tSPI1 Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\t\t0x%X\n", u32DataCount, s_au32SlaveRxBuffer[u32DataCount], s_au32MasterRxBuffer[u32DataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 peripheral clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_SPI0CKEN_Msk;
    /* Disable SPI1 peripheral clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_SPI1CKEN_Msk;
#endif

    while(1);
}

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

    /* Enable SPI0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI0CKEN_Msk;

    /* Select SPI0 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;

    /* Enable SPI1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;

    /* Select SPI1 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

#ifdef TwoPortLoopback
    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS);

    PA->SLEWCTL &= ~((GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN1_Pos) |
                     (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN3_Pos));
#if (SlewRateMode == 0)
    /* Enable SPI0 I/O basic slew rate */
    PA->SLEWCTL |= (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN1_Pos) |
                   (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN3_Pos);
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O higher slew rate */
    PA->SLEWCTL |= (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN1_Pos) |
                   (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN3_Pos);
#elif (SlewRateMode == 2)
    /* Enable SPI0 I/O ultra higher slew rate */
    PA->SLEWCTL |= (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN1_Pos) |
                   (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN3_Pos);
#endif
#endif

    /* Setup SPI1 multi-function pins */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_SPI1_SS | SYS_GPC_MFPL_PC1MFP_SPI1_CLK | SYS_GPC_MFPL_PC2MFP_SPI1_MOSI | SYS_GPC_MFPL_PC3MFP_SPI1_MISO);

    PC->SLEWCTL &= ~((GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN1_Pos) |
                     (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN3_Pos));
#if (SlewRateMode == 0)
    /* Enable SPI1 I/O basic slew rate */
    PC->SLEWCTL |= (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN1_Pos) |
                   (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_NORMAL << GPIO_SLEWCTL_HSREN3_Pos);
#elif (SlewRateMode == 1)
    /* Enable SPI1 I/O higher slew rate */
    PC->SLEWCTL |= (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN1_Pos) |
                   (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_HIGH << GPIO_SLEWCTL_HSREN3_Pos);
#elif (SlewRateMode == 2)
    /* Enable SPI1 I/O ultra higher slew rate */
    PC->SLEWCTL |= (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN0_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN1_Pos) |
                   (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN2_Pos) | (GPIO_SLEWCTL_ULTRA_HIGH << GPIO_SLEWCTL_HSREN3_Pos);
#endif
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI1 */
    /* Configure SPI1 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI1->CTL = SPI_MASTER | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI1->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;
    /* Set IP clock divider. SPI clock rate = f_PCLK0 / (35+1) */
    SPI1->CLKDIV = (SPI1->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | 35;

#ifdef TwoPortLoopback
    /* Configure SPI0 */
    /* Configure SPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI0->CTL = SPI_SLAVE | SPI_MODE_0 | SPI_CTL_SPIEN_Msk;
#endif
}

/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure SPI1 as SPI Master1 and demonstrate how to access SPI
 *           flash through SPI_MUX interface. This sample code needs to work
 *           with SPI_Flash_Master2 sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_NUMBER 3   /* page numbers */
#define TEST_LENGTH 256 /* length */

#define SPI_FLASH_PORT  SPI1

static uint8_t s_au8SrcArray[TEST_LENGTH];
static uint8_t s_au8DestArray[TEST_LENGTH];

uint16_t SpiFlash_ReadMidDid(void);
void SpiFlash_ChipErase(void);
uint8_t SpiFlash_ReadStatusReg(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value);
void SpiFlash_WaitReady(void);
void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

uint16_t SpiFlash_ReadMidDid(void)
{
    uint16_t u16MID_DID;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));
    /* Reset SPI RX */
    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
    while(SPI_FLASH_PORT->FIFOCTL & SPI_FIFOCTL_RXRST_Msk);

    // receive 16-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT));
    u16MID_DID = SPI_FLASH_PORT->RX << 8;
    while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT));
    u16MID_DID |= SPI_FLASH_PORT->RX;

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    return u16MID_DID;
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    SPI_READ_RX(SPI_FLASH_PORT);

    return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_WaitReady(void)
{
    uint8_t u8ReturnValue;

    do
    {
        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    }
    while(u8ReturnValue != 0); // check the BUSY bit
}

void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress       & 0xFF);

    // write data
    while(1)
    {
        if(!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);
            if(u32Cnt > 255) break;
        }
    }

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
}

void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress       & 0xFF);

    while(SPI_IS_BUSY(SPI_FLASH_PORT));
    // clear RX buffer
    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;

    // read data
    for(u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
        while(SPI_IS_BUSY(SPI_FLASH_PORT));
        u8DataBuffer[u32Cnt] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

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

    /* Enable SPI1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;

    /* Select SPI1 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Setup SPI1 multi-function pins */
    SET_SPI1_SS_PA8();
    SET_SPI1_CLK_PA9();
    SET_SPI1_MISO_PA10();
    SET_SPI1_MOSI_PA11();
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
    /* Configure SPI_FLASH_PORT as a master, clock idle low, 8-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI_FLASH_PORT->CTL = SPI_MASTER | (8 << SPI_CTL_DWIDTH_Pos) | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Disable auto SS function, control SS signal manually. */
    SPI_FLASH_PORT->SSCTL &= ~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk);
    /* Set IP clock divider. SPI clock rate = f_PCLK0 / (35+1) */
    SPI_FLASH_PORT->CLKDIV = (SPI_FLASH_PORT->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | 35;
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint16_t u16ID;
    uint32_t u32RWSelect;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART for print message */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("|                    SPI Master1 Sample with SPI Flash                    |\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI1 as a SPI Master1.\n");
    printf("The I/O connection for SPI1:\n");
    printf("    SPI1_SS(PA.8)\n    SPI1_CLK(PA.9)\n");
    printf("    SPI1_MISO(PA.10)\n    SPI1_MOSI(PA.11)\n\n");
    printf("Please connect SPI1 with SPI_MUX and import I/O status to PB0.\n");
    printf("SPI Master1 will write data not expected by SPI Master2 to SPI Flash and read data from SPI Flash.\n");
    printf("Before starting the data transfer, make sure SPI Master2 is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    /* Configure PB.0 as Input mode */
    PB->MODE &= ~GPIO_MODE_MODE0_Msk;

    if((u16ID = SpiFlash_ReadMidDid()) != 0xEF14)
    {
        printf("Wrong ID, 0x%x\n", u16ID);
        while(1);
    }
    else
        printf("Flash found: W25X16 ...\n");

    while(1)
    {
        printf("\n");
        printf("Press 'R/r' to read data or press 'W/w' to write data.\n");
        u32RWSelect = getchar();
        if((u32RWSelect == 'R') || (u32RWSelect == 'r'))
        {
            u32Error = 0;
            while(PB0 != 0)
            {
                printf("SPI Master2 is working. Do not transfer now!\n");
            }

            /* Init expected source data buffer */
            for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
            {
                s_au8SrcArray[u32ByteCount] = (uint8_t)u32ByteCount;
            }

            printf("Normal Read & Compare after SPI Master2 updating expected data to SPI Flash...");

            /* Read SPI flash */
            u32FlashAddress = 0;
            for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
            {
                /* clear destination data buffer */
                for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
                {
                    s_au8DestArray[u32ByteCount] = 0;
                }

                /* page read */
                SpiFlash_NormalRead(u32FlashAddress, s_au8DestArray);
                u32FlashAddress += 0x100;

                for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
                {
                    if(s_au8DestArray[u32ByteCount] != s_au8SrcArray[u32ByteCount])
                        u32Error ++;
                }
            }

            if(u32Error == 0)
                printf("[OK]\n");
            else
                printf("[FAIL]\n");
        }
        else if((u32RWSelect == 'W') || (u32RWSelect == 'w'))
        {
            u32Error = 0;
            while(PB0 != 0)
            {
                printf("SPI Master2 is working. Do not transfer now!\n");
            }

            printf("Erase chip ...");

            /* Erase SPI flash */
            SpiFlash_ChipErase();

            /* Wait ready */
            SpiFlash_WaitReady();

            printf("[OK]\n");

            /* Init unexpected source data buffer */
            for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
            {
                s_au8SrcArray[u32ByteCount] = (uint8_t)u32ByteCount + 1;
            }

            printf("Start to normal write data to Flash ...");
            /* Program SPI flash */
            u32FlashAddress = 0;
            for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
            {
                /* page program */
                SpiFlash_NormalPageProgram(u32FlashAddress, s_au8SrcArray);
                SpiFlash_WaitReady();
                u32FlashAddress += 0x100;
            }

            printf("[OK]\n");

            printf("Normal Read & Compare ...");

            /* Read SPI flash */
            u32FlashAddress = 0;
            for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
            {
                /* clear destination data buffer */
                for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
                {
                    s_au8DestArray[u32ByteCount] = 0;
                }

                /* page read */
                SpiFlash_NormalRead(u32FlashAddress, s_au8DestArray);
                u32FlashAddress += 0x100;

                for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
                {
                    if(s_au8DestArray[u32ByteCount] != s_au8SrcArray[u32ByteCount])
                        u32Error ++;
                }
            }

            if(u32Error == 0)
                printf("[OK]\n");
            else
                printf("[FAIL]\n");
        }
    }
}

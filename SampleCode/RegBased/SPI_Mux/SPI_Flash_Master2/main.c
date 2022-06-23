/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure SPI1 as SPI Master2 and demonstrate how to access SPI
 *           flash through external SPI Master1 or internal SPI Master2. This
 *           sample code needs to work with SPI_Flash_Master1 sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SW_SWITCH   0   /* SPI mux selection. 1 = software switch; 0 = hardware auto */

#define TEST_NUMBER 3   /* page numbers */
#define TEST_LENGTH 256 /* length */

#define SPI_FLASH_PORT  SPI1

volatile uint8_t u8SwitchFlag = 0;

static uint8_t s_au8SrcArray[TEST_LENGTH];
static uint8_t s_au8DestArray[TEST_LENGTH];

void SpiFlash_ChipErase(void);
uint8_t SpiFlash_ReadStatusReg(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value);
int32_t SpiFlash_WaitReady(void);
void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

__STATIC_INLINE void wait_SPI_IS_BUSY(SPI_T *spi)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while(SPI_IS_BUSY(spi))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            break;
        }
    }
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

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
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

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
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

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
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

int32_t SpiFlash_WaitReady(void)
{
    uint8_t u8ReturnValue;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    do
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for QSPI time-out!\n");
            return -1;
        }

        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    }
    while(u8ReturnValue != 0); // check the BUSY bit

    return 0;
}

void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

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
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

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

    wait_SPI_IS_BUSY(SPI_FLASH_PORT);
    // clear RX buffer
    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;

    // read data
    for(u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
        wait_SPI_IS_BUSY(SPI_FLASH_PORT);
        u8DataBuffer[u32Cnt] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

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

    /* Enable TMR0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;

    /* Select TMR0 module clock source as HIRC/2 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HIRC_DIV2;

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

    /* Setup SPI_MUX multi-function pins */
    SET_SPI_SS_MUX_PA3();
    SET_SPI_CLK_MUX_PA2();
    SET_SPI_MISO_MUX_PA1();
    SET_SPI_MOSI_MUX_PA0();
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

void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        if(PA8 == 1)
        {
            TIMER_Stop(TIMER0);
#if SW_SWITCH
            SYS->SPIMUX |= SYS_SPIMUX_MUXSWSEL_Msk;
#endif
            u8SwitchFlag++;
        }
        else
        {
#if SW_SWITCH
            SYS->SPIMUX &= ~SYS_SPIMUX_MUXSWSEL_Msk;
#else
            u8SwitchFlag = 0;
#endif
        }
    }
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART for print message */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER0->CMP = __HXT;
    TIMER0->CTL = TIMER_CTL_INTEN_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    printf("\n\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("|                    SPI Master2 Sample with SPI Flash                    |\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("Configure SPI1 as a SPI Master2.\n");
    printf("The I/O connection for SPI1:\n");
    printf("    SPI1_SS(PA.8)\n    SPI1_CLK(PA.9)\n");
    printf("    SPI1_MISO(PA.10)\n    SPI1_MOSI(PA.11)\n\n");
    printf("The I/O connection for SPI_MUX:\n");
    printf("    SPI0_SS_MUX(PA.3)\n    SPI0_CLK_MUX(PA.2)\n");
    printf("    SPI0_MISO_MUX(PA.1)\n    SPI0_MOSI_MUX(PA.0)\n\n");
    printf("The I/O pin to report status: PB0\n\n");
    printf("SPI Master2 will periodically check if data stored in the SPI Flash is as expected.\n");
    printf("If data is unexpected, SPI Master2 will overwrite it.\n");

    /* Configure PB.0 as Output mode */
    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE0_Msk)) | ((1 << GPIO_MODE_MODE0_Pos));
    PB0 = 0;

    /* Init source data buffer */
    for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8SrcArray[u32ByteCount] = (uint8_t)u32ByteCount;
    }

#if SW_SWITCH
    /* Enable SPI mux software selection mode */
    SYS->SPIMUX |= SYS_SPIMUX_MUXSWEN_Msk;
#endif

    printf("Press any key if SPI Master1 recognizes the SPI Flash.\n");
    getchar();
    printf("\n");

#if (!SW_SWITCH)
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    while(1)
    {
        if(u8SwitchFlag)
        {
            PB0 = 1;

            /* Check data stored in SPI Flash */
            printf("Check data stored in SPI Flash\n");
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

                if(PA8 == 0)
                {
                    printf("\nExternal SPI Master1 is active!\n");
                    break;
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
            {
                u32Error = 0;
                printf("[FAIL]\n");

                printf("\nData is not expected, overwrite it!\n");
                printf("Erase chip ...");

                if(PA8 == 0)
                {
                    printf("\nExternal SPI Master1 is active!\n");
                    break;
                }

                /* Erase SPI flash */
                SpiFlash_ChipErase();
                /* Wait ready */
                if( SpiFlash_WaitReady() < 0 ) return -1;
                printf("[OK]\n");

                printf("Start to normal write data to Flash ...");
                /* Program SPI flash */
                u32FlashAddress = 0;
                for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
                {
                    if(PA8 == 0)
                    {
                        printf("\nExternal SPI Master1 is active!\n");
                        break;
                    }

                    /* page program */
                    SpiFlash_NormalPageProgram(u32FlashAddress, s_au8SrcArray);
                    if( SpiFlash_WaitReady() < 0 ) return -1;
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

                    if(PA8 == 0)
                    {
                        printf("\nExternal SPI Master1 is active!\n");
                        break;
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
            PB0 = 0;
            u32Error = 0;
            u8SwitchFlag = 0;
            TIMER_Start(TIMER0);
        }
    }
#else
    while(1)
    {
        if((PA8 == 1) && (u8SwitchFlag == 5))
        {
            PB0 = 1;
            TIMER_Stop(TIMER0);
            SYS->SPIMUX |= SYS_SPIMUX_MUXSWSEL_Msk;

            /* Check data stored in SPI Flash */
            printf("Check data stored in SPI Flash\n");
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
            {
                u32Error = 0;
                printf("[FAIL]\n");

                printf("\nData is not expected, overwrite it!\n");
                printf("Erase chip ...");
                /* Erase SPI flash */
                SpiFlash_ChipErase();
                /* Wait ready */
                if( SpiFlash_WaitReady() < 0 ) return -1;
                printf("[OK]\n");

                printf("Start to normal write data to Flash ...");
                /* Program SPI flash */
                u32FlashAddress = 0;
                for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
                {
                    /* page program */
                    SpiFlash_NormalPageProgram(u32FlashAddress, s_au8SrcArray);
                    if( SpiFlash_WaitReady() < 0 ) return -1;
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
            u8SwitchFlag = 0;
            SYS->SPIMUX &= ~SYS_SPIMUX_MUXSWSEL_Msk;
            PB0 = 0;
        }
        else
        {
            SYS->SPIMUX &= ~SYS_SPIMUX_MUXSWSEL_Msk;
            /* Start Timer0 counting */
            TIMER_Start(TIMER0);
        }
    }
#endif
}

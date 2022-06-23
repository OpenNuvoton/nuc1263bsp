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

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
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

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
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
    SPI_ClearRxFIFO(SPI_FLASH_PORT);

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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /* Enable SPI1 module clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select SPI1 module clock source as PCLK0 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

    /* Enable TMR0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TMR0 module clock source as HIRC/2 */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC_DIV2, 0);

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

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);

    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_FLASH_PORT);

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER0);

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
    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);
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

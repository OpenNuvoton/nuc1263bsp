/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to PDMA and trigger DAC by Timer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
const uint16_t g_au16Sine[] = {127, 139, 152, 164, 176, 187, 198, 208,
                               217, 225, 233, 239, 244, 249, 252, 253,
                               254, 253, 252, 249, 244, 239, 233, 225,
                               217, 208, 198, 187, 176, 164, 152, 139,
                               127, 115, 102, 90, 78, 67, 56, 46,
                               37, 29, 21, 15, 10, 5, 2, 1,
                               0, 1, 2, 5, 10, 15, 21, 29,
                               37, 46, 56, 67, 78, 90, 102, 115
                              };

static uint32_t g_u32Index = 0;
const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void DAC_FunctionTest(void);


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

    /* Select timer 0 module clock source as HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HIRC_DIV2;

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable DAC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_DACCKEN_Msk;

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Enable Timer 0 module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_TMR0CKEN_Msk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    //SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_TXD_PB13_Msk))) | UART0_TXD_PB13;//UART0_RXD_PB12 pin conflicts with DAC0_OUT pin

    /* Set multi-function pin for DAC voltage output */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | DAC0_OUT_PB12;

    /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));

}

void UART0_Init()
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

void TIMER0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init TIMER0                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set timer0 periodic time-out period is 1000us if timer clock is 24 MHz */
    TIMER0->CMP = 24000;

    /* Start timer counter in periodic mode and enable timer interrupt trigger DAC */
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGDAC_Msk;

}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_FunctionTest(void)
{
    printf("\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          DAC Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("This sample code use PDMA and trigger DAC0 output sine wave by Timer 0.\n");

    /* Reset DAC module */
    SYS->IPRST2 |= SYS_IPRST1_DACRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST1_DACRST_Msk ;

    /* CH0 source request from DAC */
    PDMA->REQSEL0_3 = PDMA_DAC0_TX; //CH0_SEl=DAC

    PDMA->DSCT[0].CTL = ((g_u32ArraySize) << 16) //Transfer Count
                        | (0x1 << 12) //Transfer Width Selection(01=>16bit)
                        | (0x3 << 10) //Destination Address Increment(11=>No increment (fixed address))
                        | (0x0 << 8) //Source Address Increment(00=>Increment)
                        | (0x0 << 7) //Table Interrupt Disable
                        | (0x0 << 4) //Burst Size(128 Transfers)
                        | (0x1 << 2) //Transfer Type(1 = Single transfer type)
                        | (0x1 << 0); //PDMA Operation Mode Selection(01=>Basic Mode)

    /* Set source address */
    PDMA->DSCT[0].SA = (uint32_t)&g_au16Sine[g_u32Index];

    /* Set destination address */
    PDMA->DSCT[0].DA = (uint32_t)&DAC0->DAT;

    /* Enable PDMA channel 0 */
    PDMA->CHCTL = 0x1;

    /* Set the timer 0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC0->CTL = DAC_TIMER0_TRIGGER | DAC_CTL_TRGEN_Msk | DAC_CTL_DMAEN_Msk | DAC_CTL_DACEN_Msk;

    /* When DAC controller APB clock speed is 72MHz and DAC conversion settling time is 1us,
       the selected SETTLET value must be greater than 0x48.  */
    DAC0->TCTL = 0x48;

    /* Set DAC 12-bit holding data */
    DAC0->DAT = g_au16Sine[g_u32Index];

    /* Clear the DAC conversion complete finish flag for safe */
    DAC0->STATUS = DAC_STATUS_FINISH_Msk;

    /* Enable the DAC interrupt.  */
    DAC0->CTL |= DAC_CTL_DACIEN_Msk;
    NVIC_EnableIRQ(DAC_IRQn);

    /* Enable Timer0 counting to start D/A conversion */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk; //Enable timer Counting.

    while(1)
    {
        if(PDMA->TDSTS == 0x1)
        {
            /* Clear CH0 transfer done flag */
            PDMA->TDSTS = 0x01;

            /* Re-Set transfer count and basic operation mode */
            PDMA->DSCT[0].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk);
            PDMA->DSCT[0].CTL |= (PDMA_OP_BASIC | ((g_u32ArraySize - 1) << PDMA_DSCT_CTL_TXCNT_Pos));
        }
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_IRQHandler(void)
{

    if(DAC0->STATUS & DAC_STATUS_FINISH_Msk)
    {
        DAC0->STATUS = DAC_STATUS_FINISH_Msk;
    }
    return;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init TIMER0 for DAC */
    TIMER0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* DAC function test */
    DAC_FunctionTest();

    /* Disable External Interrupt */
    NVIC_DisableIRQ(DAC_IRQn);

    /* Reset PDMA module */
    SYS->IPRST0 |= SYS_IPRST0_PDMARST_Msk ;
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk ;

    /* Reset DAC module */
    SYS->IPRST2 |= SYS_IPRST1_DACRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST1_DACRST_Msk ;

    /* Reset Timer0 module */
    SYS->IPRST1 |= SYS_IPRST1_TMR0RST_Msk ;
    SYS->IPRST1 &= ~SYS_IPRST1_TMR0RST_Msk ;

    /* Disable PDMA module clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;

    /* Disable Timer0 IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_TMR0CKEN_Msk;

    /* Disable DAC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_DACCKEN_Msk;

    printf("Stop DAC output and exit DAC sample code\n");

    while(1);

}

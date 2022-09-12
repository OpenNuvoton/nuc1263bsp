/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 6 $
 * $Date: 15/09/02 10:03a $
 * @brief    Write a data to DAC_DAT register to trigger DAC conversion.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
const uint16_t sine[] = {127, 139, 152, 164, 176, 187, 198, 208,
                                    217, 225, 233, 239, 244, 249, 252, 253,
                                    254, 253, 252, 249, 244, 239, 233, 225,
                                    217, 208, 198, 187, 176, 164, 152, 139,
                                    127, 115, 102, 90, 78, 67, 56, 46,
                                    37, 29, 21, 15, 10, 5, 2, 1,
                                    0, 1, 2, 5, 10, 15, 21, 29,
                                    37, 46, 56, 67, 78, 90, 102, 115};

static uint32_t index = 0;
const uint32_t array_size = sizeof(sine) / sizeof(uint16_t);

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void DAC_FunctionTest(void);


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

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable DAC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_DACCKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    //SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_TXD_PB13_Msk))) | UART0_TXD_PB13;//UART0_RXD_PB12 pin conflicts with DAC0_OUT pin

    /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
    PB->DINOFF |= ((1ul << 12)<<16);

    /* Set multi-function pin for DAC voltage output */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | DAC0_OUT_PB12;

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
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC>>1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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
    printf("Start to trigger DAC by software\n");

    /* Set the software trigger and enable D/A converter */
    DAC0->CTL = DAC_SOFTWARE_TRIGGER | DAC_CTL_DACEN_Msk;

    /* When DAC controller APB clock speed is 72MHz and DAC conversion settling time is 8us,
       the selected SETTLET value must be greater than 0x241 */
    DAC0->TCTL = 0x250;

    /* Set DAC 12-bit holding data */
    DAC0->DAT = sine[index];

    /* Clear the DAC conversion complete finish flag for safe */
    DAC0->STATUS = DAC_STATUS_FINISH_Msk;

    /* Enable the DAC interrupt */
    DAC0->CTL |= DAC_CTL_DACIEN_Msk;
    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC0->SWTRG = DAC_SWTRG_SWTRG_Msk;

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_IRQHandler(void)
{

    if(DAC0->STATUS & DAC_STATUS_FINISH_Msk)
    {
        if(index == array_size)
            index = 0;
        else
        {
            /* Clear the DAC conversion complete finish flag */
            DAC0->STATUS = DAC_STATUS_FINISH_Msk;

            /* Write data */
            DAC0->DAT = sine[index++];

            /* Start A/D conversion */
            DAC0->SWTRG = DAC_SWTRG_SWTRG_Msk;
        }
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Single Mode test */
    DAC_FunctionTest();

    /* Reset DAC module */
    SYS->IPRST2 |= SYS_IPRST1_DACRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST1_DACRST_Msk ;

    /* Disable DAC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_DACCKEN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(DAC_IRQn);

    printf("Exit DAC sample code\n");

    while(1);

}


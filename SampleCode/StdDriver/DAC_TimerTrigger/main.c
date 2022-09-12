/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to trigger DAC by timer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


static const uint16_t g_au16Sine[] = {127, 139, 152, 164, 176, 187, 198, 208,
                                    217, 225, 233, 239, 244, 249, 252, 253,
                                    254, 253, 252, 249, 244, 239, 233, 225,
                                    217, 208, 198, 187, 176, 164, 152, 139,
                                    127, 115, 102, 90, 78, 67, 56, 46,
                                    37, 29, 21, 15, 10, 5, 2, 1,
                                    0, 1, 2, 5, 10, 15, 21, 29,
                                    37, 46, 56, 67, 78, 90, 102, 115};

static const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);
static uint32_t g_u32Index = 0;

void DAC_IRQHandler(void);
void SYS_Init(void);


void DAC_IRQHandler(void)
{
    if(DAC_GET_INT_FLAG(DAC0, 0))
    {

        if(g_u32Index == g_u32ArraySize)
            g_u32Index = 0;
        else
        {
            /* Clear the DAC conversion complete finish flag */
            DAC_CLR_INT_FLAG(DAC0, 0);

            DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index++]);
        }
    }
    return;
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

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select timer 0 module clock source as HXT */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC_DIV2, 0);

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


int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          DAC Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("\nStart to trigger DAC by Timer 0!\n");

    /* Set the timer 0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_TIMER0_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt.  */
    DAC_ENABLE_INT(DAC0, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Configure Timer 0 frequency to 1000Hz and start it. */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_DAC);
    TIMER_Start(TIMER0);

    while(1);

}

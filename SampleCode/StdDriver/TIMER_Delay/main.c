/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use timer0 to create various delay time.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable GPIO Port F module clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32DelayTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------+\n");
    printf("|    Timer Delay API Sample Code    |\n");
    printf("+-----------------------------------+\n\n");

    printf("# This sample code is using Timer1 to check Timer0 TIMER_Delay API delay time is reasonable or not.\n");
    printf("# Delay time includes 100 ms, 200 ms, 300 ms, 400 ms and 500 ms.\n\n");

    /* Start Timer1 to measure delay period of TIMER_Delay API is reasonable or not */
    TIMER1->CTL = TIMER_PERIODIC_MODE | (12 - 1);
    TIMER_ResetCounter(TIMER1);
    TIMER_Start(TIMER1);

    TIMER_ResetCounter(TIMER1);
    TIMER_Delay(TIMER0, 100000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-1 is %d ms.\n", u32DelayTime);

    TIMER_ResetCounter(TIMER1);
    TIMER_Delay(TIMER0, 200000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-2 is %d ms.\n", u32DelayTime);

    TIMER_ResetCounter(TIMER1);
    TIMER_Delay(TIMER0, 300000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-3 is %d ms.\n", u32DelayTime);

    TIMER_ResetCounter(TIMER1);
    TIMER_Delay(TIMER0, 400000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-4 is %d ms.\n", u32DelayTime);

    TIMER_ResetCounter(TIMER1);
    TIMER_Delay(TIMER0, 500000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-5 is %d ms.\n", u32DelayTime);

    printf("\n*** Check TIMER_Delay API delay time done ***\n");

    while(1);
}

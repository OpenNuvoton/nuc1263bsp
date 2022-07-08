/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to reload the WWDT counter value.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u32WWDTINTCounts;


/**
 * @brief       IRQ Handler for WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WWDT.
 */
void WDT_IRQHandler(void)
{
    if(WWDT_GET_INT_FLAG() == 1)
    {
        /* Clear WWDT compare match interrupt flag */
        WWDT_CLEAR_INT_FLAG();

        PA0 ^= 1;

        g_u32WWDTINTCounts++;

        if(g_u32WWDTINTCounts < 10)
        {
            /* To reload the WWDT counter value to 0x3F */
            WWDT_RELOAD_COUNTER();
        }

        printf("WWDT compare match interrupt occurred. (%d)\n", g_u32WWDTINTCounts);
    }
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable peripheral clock */
    CLK_EnableModuleClock(WWDT_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048, 0);
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
    double dPeriodTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    WWDT Compare March Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /* To check if system has been reset by WWDT time-out reset or not */
    if(WWDT_GET_RESET_FLAG() == 1)
    {
        printf("*** System has been reset by WWDT time-out reset event. [WWDT_CTL: 0x%08X] ***\n\n", WWDT->CTL);
        WWDT_CLEAR_RESET_FLAG();
        while(1);
    }

    dPeriodTime = (((double)(1000000 * 2048) / (double)SystemCoreClock) * 1024) * 32;

    printf("# WWDT Settings: \n");
    printf("    - Clock source is PCLK0/2048 (%d Hz)    \n", SystemCoreClock / 2048);
    printf("    - WWDT counter prescale period is 1024, \n");
    printf("        and max WWDT time-out period is 1024 * (64 * WWDT_CLK)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("# System will generate first WWDT compare match interrupt event after %.2f us.\n", dPeriodTime);
    printf("    1.) use PA.0 high/low period to check WWDT compare match interrupt period time\n");
    printf("    2.) reload WWDT counter value to avoid WWDT time-out reset system occurred\n");
    printf("        when interrupt counts less than 11.\n");
    printf("    3.) do not reload WWDT counter value to generate WWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    PA->MODE = 0xFFFFFFFD;
    PA0 = 1;

    /* Enable WWDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    g_u32WWDTINTCounts = 0;

    /*
        Max time-out period is 1024*(64*WWDT_CLK);
        WWDT compare value is 32;
        Enable WWDT compare match interrupt;
    */
    /* Note: WWDT_CTL register can be written only once after chip is powered on or reset */
    WWDT_Open(WWDT_PRESCALER_1024, 32, TRUE);

    printf("[WWDT_CTL: 0x%08X]\n\n", WWDT->CTL);

    while(1);
}

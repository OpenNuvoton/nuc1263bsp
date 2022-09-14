/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:29p $
 * @brief    Measure the current temperature by Temperature Sensor.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t volatile g_u32Done;
uint32_t volatile g_u32tempData;

/* Function prototype declaration */
void SYS_Init(void);
void TS_Init(void);

void TS_IRQHandler(void)
{
    if (TS_GET_INT_FLAG())
    {
        g_u32Done = 1;

        /* get TS data */
        g_u32tempData = TS_GET_DATA();

        /* clear complete flag*/
        TS_CLR_COMPLETE_FLAG();
        while(TS_GET_COMPLETE_FLAG());

        /* clear interrupt flag*/
        TS_CLR_INT_FLAG();
        while(TS_GET_INT_FLAG());
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

    /* Enable BOD for temperature sensor */
    SYS->BODCTL |= SYS_BODCTL_BODEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint8_t u8Option;
    uint32_t u32digiPart, u32deciPart;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    /* Enable TS */
    TS_ENABLE();
    /* Note: User needs to wait for at least 200us to start temperature sensor conversion. */
    CLK_SysTickDelay(200);

    /* Clear TSEOC bit */
    TS_CLR_COMPLETE_FLAG();

    /* Disable TS interrupt */
    NVIC_DisableIRQ(TS_IRQn);

    /* Clear interrupt flag */
    TS_CLR_INT_FLAG();

    /* Enable TS interrupt */
    NVIC_EnableIRQ(TS_IRQn);
    TS_ENABLE_INT();

    while (1)
    {
        printf("\nSelect whether to measure temperature or not: \n");
        printf("[1] Measure temperature  \n");
        printf("[Other] Exit \n");
        u8Option = getchar();
        printf("Select : %d \n", u8Option - 48);

        if (u8Option != '1')
        {
            printf("Exit\n");
            break;
        }
        g_u32Done = 0;

        /* Enable TS transform */
        TS_TRIGGER();

        /* Wait to transform completely */
        u32TimeOutCnt = SystemCoreClock;
        while(!g_u32Done)
        {
            if(u32TimeOutCnt == 0)
            {
                printf("\nTimeout is happened, please check if something is wrong. \n");
                while(1);
            }
            u32TimeOutCnt--;
        }

        /* Read temperature data */
        if (g_u32tempData & 0x800)
        {
            /* negative temperature (2's complement)*/
            g_u32tempData = ~g_u32tempData;
            g_u32tempData++;
            g_u32tempData &= 0x7FF;
            g_u32tempData *= TS_DEGREE_PER_LSB;
            u32digiPart = g_u32tempData / 10000;
            u32deciPart = g_u32tempData % 10000;
            printf("\nMeasured sensor data = -%d.%d !!!\n\n", u32digiPart, u32deciPart);
        }
        else
        {
            /* positive temperature*/
            g_u32tempData &= 0x7FF;
            g_u32tempData *= TS_DEGREE_PER_LSB;
            u32digiPart = g_u32tempData / 10000;
            u32deciPart = g_u32tempData % 10000;
            printf("\nMeasured sensor data = %d.%d !!!\n\n", u32digiPart, u32deciPart);
        }
    }

    /* Close TS */
    NVIC_DisableIRQ(TS_IRQn);
    TS_DISABLE();
    TS_DISABLE_INT();

    while(1);
}

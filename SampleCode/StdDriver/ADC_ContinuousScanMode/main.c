/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 17/05/04 1:14p $
 * @brief    Perform A/D Conversion with ADC continuous scan mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcContScanModeTest(void);


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

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select ADC module clock source as HIRC/2 and ADC module clock divider as 7 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADCSEL_HIRC_DIV2, CLK_CLKDIV0_ADC(7));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, 0xF);

    /* Set multi-function pins for ADC channels */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: ADC_GetConversionRate                                                                         */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   Return the A/D conversion rate (sample/second)                                                        */
/*                                                                                                         */
/* Description:                                                                                            */
/*   The conversion rate depends on the clock source of ADC clock.                                         */
/*   It only needs 21 ADC clocks to complete an A/D conversion.                                            */
/*---------------------------------------------------------------------------------------------------------*/
static __INLINE uint32_t ADC_GetConversionRate()
{
    uint32_t u32AdcClkSrcSel;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, __HIRC>>1};

    /* Set the PLL clock frequency */
    u32ClkTbl[1] = PllClock;
    /* Set the system core clock frequency */
    u32ClkTbl[2] = SystemCoreClock;
    /* Get the clock source setting */
    u32AdcClkSrcSel = ((CLK->CLKSEL1 & CLK_CLKSEL1_ADCSEL_Msk) >> CLK_CLKSEL1_ADCSEL_Pos);
    /* Return the ADC conversion rate */
    return ((u32ClkTbl[u32AdcClkSrcSel]) / (((CLK->CLKDIV0 & CLK_CLKDIV0_ADCDIV_Msk) >> CLK_CLKDIV0_ADCDIV_Pos) + 1) / 21);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcContScanModeTest                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC continuous scan mode test.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AdcContScanModeTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;
    uint32_t u32TimeOutCnt;

    printf("\n\nConversion rate: %d samples/second\n", ADC_GetConversionRate());
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC continuous scan mode sample code                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 2 cycles of conversion result from the specified channels.\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Power on ADC module */
            ADC_POWER_ON(ADC);

			/* Set the ADC operation mode as continuous scan, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, 0xF);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC);

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

        }
        else if(u8Option == '2')
        {
            /* Power on ADC module */
            ADC_POWER_ON(ADC);

			/* Set the ADC operation mode as continuous scan, input mode as differential and
               enable analog input channel 0 and 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_CONTINUOUS, 0x5);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC);

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC);

            for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

        }
        else
            return ;

    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
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

    /* Continuous scan mode test */
    AdcContScanModeTest();

    /* Disable ADC module */
    ADC_Close(ADC);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("\nExit ADC sample code\n");

    while(1);

}


/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up MCU from Power-down mode by ACMP wake-up function.
 * @note
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void PowerDownFunction(void);
int IsDebugFifoEmpty(void);

int32_t main(void)
{
    uint32_t u32DelayCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|             ACMP Sample Code          |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates ACMP0 wake-up function. Using ACMP0_P0 (PB2) as ACMP0\n");
    printf("positive input and using internal CRV as the negative input.\n");
    printf("The compare result reflects on ACMP0_O (PB7).\n");

    printf("When the voltage of the positive input is greater than the voltage of the negative input,\n");
    printf("the analog comparator outputs logical one; otherwise, it outputs logical zero.\n");
    printf("This chip will be waked up from power down mode when detecting a transition of analog comparator's output.\n");
    printf("Press any key to enter power down mode ...");
    getchar();
    printf("\n");

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV0_SRC(ACMP01, ACMP_VREF_CRV0SSEL_VDDA);
    /* Select CRV level: VDDA * 9 / 24 */
    ACMP_CRV0_SEL(ACMP01, 5);
    /* Select CRV as the source of ACMP negative input */
    ACMP_SET_NEG_SRC(ACMP01, 0, ACMP_CTL_NEGSEL_CRV);
    /* Enable ACMP0 */
    ACMP_ENABLE(ACMP01, 0);
    __NOP();
    for(u32DelayCount = 0; u32DelayCount < 100; u32DelayCount++); /* For ACMP setup time */
    __NOP();
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 0);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    PowerDownFunction();
    printf("Wake up by ACMP0!\n");
    while(1);

}

void ACMP01_IRQHandler(void)
{
    printf("\nACMP0 interrupt!\n");
    /* Disable interrupt */
    ACMP_DISABLE_INT(ACMP01, 0);
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    /* Clear wake-up interrupt flag */
    ACMP_CLR_WAKEUP_INT_FLAG(ACMP01, 0);
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LXT clock */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk))
        != (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk));

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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pin for ACMP0 positive input pin */
    SET_ACMP0_P0_PB2();

    /* Set multi-function pin for ACMP0 output pin */
    SET_ACMP0_O_PB7();

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Disable digital input path of analog pin ACMP0_P to prevent leakage */
    PB->DINOFF |= (1 << GPIO_DINOFF_DINOFF2_Pos);
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void PowerDownFunction(void)
{
    printf("\nSystem enter power-down mode ... ");

    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    /* Deep sleep mode is selected */
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;

    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk);
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);

    __WFI();
}



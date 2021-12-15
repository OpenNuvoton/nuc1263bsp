/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate how to implement a USB mouse device with BC1.2 (Battery Charging).
 *           which shows different type of charging port after connected USB port.
 *           The mouse cursor will move automatically when this mouse device connecting to PC by USB.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "hid_mouse.h"
#include <stdio.h>


/* If crystal-less is enabled, system won't use any crystal as clock source
   If using crystal-less, system will be 48MHz, otherwise, system is 72MHz
*/
#define CRYSTAL_LESS        1
#define HIRC_AUTO_TRIM      0x611   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x110)

volatile S_USBD_BC12_PD_STATUS g_sChargeStatus = USBD_BC12_VBUS_OFF;

int IsDebugFifoEmpty(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (!CRYSTAL_LESS)
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Select USB clock source as PLL and USB clock divider as 3 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));
#else
    /* Select HCLK clock source to HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select USB clock source as HIRC and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
#endif

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC/2 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC_DIV2, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

/**
 * @brief Utility for BC1.2 timing use (USBD_BC_Detect)
 * @param[in] us
 * @return none
 */
void SysTick_Delay(TIMER_T *dummy_for_compatible __attribute__((unused)), uint32_t us)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SysTick->CTRL = 0;
    SysTick->VAL = 0x00;

    SysTick->LOAD = us * (SystemCoreClock / 1000000); /* Depend on core clock */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    /* Disable SysTick counter */
    SysTick->CTRL = 0;

    /* Lock protected registers */
    SYS_LockReg();
}

/**
 * @brief BC1.2 Charge Port Detection
 * @param[in] pu32TimerSrc NULL: timing source is SysTick timer;
 *                         TIMER0 ~ TIMER3: timing source is peripheral H/W timer
 * @return status code (S_USBD_BC12_PD_STATUS)
 * @details We recommend that the H/W timer for timing counting if H/W resource is sufficient
 *          because the BSP APIs handle the setting details.
 *          User has to take cares the setting details if SysTick is used for timing counting.
 */
S_USBD_BC12_PD_STATUS USBD_BC_Detect(TIMER_T *pu32TimerSrc)
{
/* TDCD_TIMEOUT (BC1.2 SPEC): 300ms ~ 900ms */
#define DCD_TIMEOUT_PERIOD_US 500000UL

#define ENABLE_BC12_DBG_MSG 0
#if ENABLE_BC12_DBG_MSG
#define DBG_MSG printf
#else
#define DBG_MSG(...)
#endif

#define BC_DELAY(us) pfnBC_Delay(pu32TimerSrc, us)

    void (*pfnBC_Delay)(TIMER_T *dummy, uint32_t us);

    if(pu32TimerSrc == NULL)
    {
        pfnBC_Delay = SysTick_Delay;
    }
    else if((pu32TimerSrc == TIMER0) ||
            (pu32TimerSrc == TIMER1) ||
            (pu32TimerSrc == TIMER2) ||
            (pu32TimerSrc == TIMER3))
    {
        pfnBC_Delay = TIMER_Delay;
    }
    else
    {
        DBG_MSG("Invalid delay timer source.\n");
        return USBD_BC12_ERROR; // Invalid delay timer source
    }

    if(USBD_IS_ATTACHED() == 0)
    {
        USBD->BCDC = 0;
        return USBD_BC12_VBUS_OFF;
    }
    else
    {
        BC_DELAY(1000); // 1ms
        DBG_MSG("VBUS Plug\n");
    }

    DBG_MSG("Check VBUS threshold voltage");
    USBD->BCDC = USBD_BCDC_BCDEN_Msk;

    BC_DELAY(30000); // 30 ms: wait PHY LDO stable
    USBD->BCDC |= USBD_BCDC_DETMOD_VBUS;

    while((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_VBUS_UNREACH) {}

    DBG_MSG("\nCheck VBUS OK\n");
    DBG_MSG("Check data pin contact status\n");
    USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_DCD;

    if(pu32TimerSrc == NULL)   /* Use SysTick timer */
    {
DCD_REPEAT_SYSTICK:
        SYS_UnlockReg();
        SysTick->CTRL = 0;
        SysTick->VAL = (0x00);

        SysTick->LOAD = DCD_TIMEOUT_PERIOD_US * (SystemCoreClock / 1000000);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        while(1)
        {
            /* Using S/W debounce, TDCD_DBNC is 10 ms totally */
            if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
            {
                BC_DELAY(5000);

                if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) != USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(4000);

                if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) != USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(1000);

                if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) != USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                DBG_MSG(" - DCD Data Contact\n");
                SYS_LockReg();
                break;
            }

            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                SysTick->CTRL = 0;
                SYS_LockReg();
                DBG_MSG(" - Timeout\n");
                break;
            }
        }
    }
    else   /* Use hardware timer */
    {
DCD_REPEAT_TIMER:
        {
            uint32_t u32Usec = DCD_TIMEOUT_PERIOD_US;
            uint32_t u32Clk = TIMER_GetModuleClock(pu32TimerSrc);
            uint32_t u32Prescale = 0UL, u32Delay = (SystemCoreClock / u32Clk) + 1UL;
            uint32_t u32Cmpr, u32NsecPerTick;

            /* Clear current timer configuration */
            pu32TimerSrc->CTL = 0UL;
            pu32TimerSrc->EXTCTL = 0UL;

            if(u32Clk <= 1000000UL)  // Minimin delay is 1000 us if timer clock source is <= 1 MHz
            {
                if(u32Usec < 1000UL)
                    u32Usec = 1000UL;

                if(u32Usec > 1000000UL)
                    u32Usec = 1000000UL;
            }
            else
            {
                if(u32Usec < 100UL)
                    u32Usec = 100UL;

                if(u32Usec > 1000000UL)
                    u32Usec = 1000000UL;
            }

            if(u32Clk <= 1000000UL)
            {
                u32Prescale = 0UL;
                u32NsecPerTick = 1000000000UL / u32Clk;
                u32Cmpr = (u32Usec * 1000UL) / u32NsecPerTick;
            }
            else
            {
                u32Cmpr = u32Usec * (u32Clk / 1000000UL);
                u32Prescale = (u32Cmpr >> 24); /* for 24 bits CMPDAT */

                if(u32Prescale > 0UL)
                    u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
            }

            pu32TimerSrc->CMP = u32Cmpr;
            pu32TimerSrc->CTL = TIMER_CTL_CNTEN_Msk | TIMER_ONESHOT_MODE | u32Prescale;

            // When system clock is faster than timer clock, it is possible timer active
            // bit cannot set in time while we check it. And the while loop below return
            // immediately, so put a tiny delay here allowing timer start counting and
            // raise active flag.
            for(; u32Delay > 0UL; u32Delay--)
            {
                __NOP();
            }

            while(1)
            {
                /* Using S/W debounce, TDCD_DBNC is 10 ms totally */
                if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    BC_DELAY(5000);

                    if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) != USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    BC_DELAY(2000);

                    if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) != USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    BC_DELAY(3000);

                    if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) != USBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    DBG_MSG(" - DCD Data Contact\n");
                    break;
                }

                if(!(pu32TimerSrc->CTL & TIMER_CTL_ACTSTS_Msk))
                {
                    DBG_MSG(" - DCD Timeout\n");
                    break;
                }
            }
        }
    }

    USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_PD;
    DBG_MSG("BC1.2 - Primary Detect: ");

    /* Delay 40ms */
    BC_DELAY(40000); // SPEC: TVDPSRC_ON
    BC_DELAY(1000);  // tune

    if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_PD_SDP_NUSP)
    {
        USBD->BCDC = 0; // important: to prevent next loop unexpected pulse
        return USBD_BC12_SDP;
    }
    else
    {
        /* Switch back to IDLE than switch to USBD_BCDC_DETMOD_SD */
        USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_IDLE; /* Hardware limitation */
        BC_DELAY(10000);                                          /* TVDMSRC_DIS         */

        /* Port detect - Secondary detect */
        DBG_MSG("BC1.2 - Secondary Detect: ");
        USBD->BCDC = USBD_BCDC_BCDEN_Msk | USBD_BCDC_DETMOD_SD;
        BC_DELAY(40000); // SPEC: TVDMSRC_ON
        BC_DELAY(5000);  // tune

        if((USBD->BCDC & USBD_BCDC_DETSTS_Msk) == USBD_BCDC_DETSTS_SD_CDP)
        {
            DBG_MSG("* CDP\n");
            USBD->BCDC = 0; // important: to prevent next loop unexpected pulse
            return USBD_BC12_CDP;
        }
        else
        {
            DBG_MSG("* DCP\n");
            return USBD_BC12_DCP;
        }
    }

#undef ENABLE_BC12_DBG_MSG
#undef DBG_MSG
#undef DCD_TIMEOUT_PERIOD_US
#undef BC_DELAY
}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");

    while(!IsDebugFifoEmpty());

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    if(!(USBD->ATTR & USBD_ATTR_USBEN_Msk))
        USBD_ENABLE_USB();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+-----------------------------------------------------+\n");
    printf("|     NuMicro USB HID Mouse Sample Code with BC1.2    |\n");
    printf("+-----------------------------------------------------+\n");

restart:

    while(1)
    {
        g_sChargeStatus = USBD_BC_Detect(TIMER0);

        if(USBD_BC12_SDP == g_sChargeStatus)
        {
            printf("==>is SDP\n");
            break;
        }

        if(USBD_BC12_CDP == g_sChargeStatus)
        {
            printf("==>is CDP\n");
            break;
        }

        if(USBD_BC12_DCP == g_sChargeStatus)
        {
            printf("==>is DCP\n");

            while(USBD_IS_ATTACHED());  // Keep draw current until detach

            printf("==>VBUS detach\n");
            USBD->BCDC = 0; // important: PET shows that D+ must keep whole period during test
            continue; // recognize
        }

        if(USBD_BC12_VBUS_OFF == g_sChargeStatus)
        {
            continue;
        }

        if(USBD_BC12_ERROR == g_sChargeStatus)
        {
            printf("parameter error\n");

            while(1);
        }
    }

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    CLK_EnableCKO(CLK_CLKSEL2_CLKOSEL_HCLK, 1, 0);

    /* Endpoint configuration */
    HID_Init();

    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->IRCTCTL = HIRC_AUTO_TRIM | (8 << SYS_IRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->IRCTCTL = 0;

            /* Clear error flags */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        if((USBD->VBUSDET & USBD_VBUSDET_VBUSDET_Msk) == 0x0)
        {
            printf("VBUS Un-Plug\n");
            USBD_SET_SE0();
            goto restart;
        }

        /* Enter power down when USB suspend */
        if(g_u8Suspend && (USBD->VBUSDET & USBD_VBUSDET_VBUSDET_Msk))
            PowerDown();

        HID_UpdateMouseData();
    }
}

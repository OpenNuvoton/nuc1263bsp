/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    A sample code to show how to debug with semihost message print.
 *  
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#if (defined (__GNUC__) && (!(defined(__ARMCC_VERSION))))
extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main()
{
    char ch;

#if (defined (__GNUC__) && (!(defined(__ARMCC_VERSION))))
    initialise_monitor_handles();
#endif
    /*
        To enable semihost, user must define "DEBUG_ENABLE_SEMIHOST" constant when buildind sample code.
        This sample code is used to show how to print message/getchar on IDE debug environment.
        It will echo all input character back on UART #1 of KEIL IDE.

        In KEIL MDK, user need to open "View->Serial Window->UART #1" windows in debug mode.
        In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

        NOTE1: HardFault_Handler handler is implemented in retarget.c.
        NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
        NOTE3: It does not print any message if Nuvoton NuLink ICE Dongle is not connected.
    */

    printf("\n Start SEMIHOST test: \n");

    while(1)
    {
        /* Get input character */
        ch = (char)getchar();

        /* Print input character back */
        printf("%c\n", ch);
    }
}

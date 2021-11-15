/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 *              
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "isp_user.h"


/* rename for uart_transfer.c */
#define UART_T					UART1
#define UART_T_IRQHandler		UART1_IRQHandler
#define UART_T_IRQn				UART1_IRQn

#define CONFIG_SIZE 8 // in bytes

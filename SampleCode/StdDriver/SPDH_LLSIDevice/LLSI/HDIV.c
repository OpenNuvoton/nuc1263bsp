/******************************************************************************//**
 * @file     HDIV.c
 * @version  V1.00
 * @brief    Hardware Divider sample file
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

 /*!<Includes */
#include <stdlib.h>
#include "NuMicro.h"
#include "HDIV.h"

int32_t HDIV_Div(int32_t x, int16_t y)
{
    return (x/y);
}

int16_t HDIV_Mod(int32_t x, int16_t y)
{
    return (x%y);
}

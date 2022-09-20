/******************************************************************************
 * @file     HDIV.h
 * @brief    Hardware Divider header file
 *
 * @note
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HDIV_H__
#define __HDIV_H__

#include <stdint.h>
#include "NuMicro.h"

int32_t HDIV_Div(int32_t x, int16_t y);
int16_t HDIV_Mod(int32_t x, int16_t y);

#endif  /* __HDIV_H__ */

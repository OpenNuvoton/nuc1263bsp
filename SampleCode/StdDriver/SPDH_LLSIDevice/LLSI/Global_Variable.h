/******************************************************************************
 * @file     Global_Variable.h
 * @brief    Global Variable header file
 *
 * @note
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __Global_Variable_H__
#define __Global_Variable_H__

#include <stdint.h>
#include "NuMicro.h"

#define HCLK_CLK 72000000

enum eLEDType
{
		Type_GRB = 0,
		Type_RGB = 1,
};

enum eDirection
{
		Dir_Forward = 0,
		Dir_Backward = 1,
};

enum eMusicAction
{
		Music_POP = 0x01,
		Music_JAZZ = 0x02,
		Music_Mixed = 0x03,
};

enum eColorTable
{
    eColorRed = 0,
    eColorOrange,
    eColorYellow,
    eColorGreen,
    eColorCyan,
    eColorBlue,
    eColorIndigo,
    eColorPurple
};

struct LED_Setting_Tag;
typedef void (*LED_FUNC)(volatile struct LED_Setting_Tag* LED_Setting);
extern void * const Mode_Function[16];

typedef struct LED_Setting_Tag
{
    uint32_t  TimeCounter;		// 0
    uint8_t   AP_Sync;			// 0
    uint16_t  LEDNum;			// 100
    uint8_t   LightingMode;		// 1
    uint8_t   Color_R;		    // 255
    uint8_t   Color_G;			// 0
    uint8_t   Color_B;			// 0
    uint8_t   Brightness;		// 0xFF
    uint8_t   Speed;			// 0
    uint8_t   Direction;		// Dir_Forward
    uint8_t   LED_Type;			// Type_GRB
    uint8_t   fPDMA_Done;		// 1
    uint8_t   RandomFlag;		// 1
    uint32_t  Random;			// 0
    LED_FUNC  Mode_FUNC;		// FUNC_Static
    uint8_t*  LED_Data;
    uint8_t   Main_Volume;		// 0
    uint8_t   Left_Volume;		// 0
    uint8_t   Right_Volume;		// 0
    uint8_t   Music_Action;		// Music_POP
    uint8_t   LLSI_Num;
    uint32_t  Array_Size;
    uint8_t   LLSI_Trigger;
} LED_Setting_T;

#define cStrip1_LED 300
extern __attribute__((aligned (4))) volatile LED_Setting_T Strip1_LEDSetting;

extern uint8_t g_u8OneShot_Flag;

#endif  /* __Global_Variable__ */

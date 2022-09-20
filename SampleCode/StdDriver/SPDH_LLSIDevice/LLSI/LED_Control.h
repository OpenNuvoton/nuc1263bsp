/******************************************************************************
 * @file     LED_Control.h
 * @brief    Control LED lighting effects header file
 *
 * @note
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __LED_Control_H__
#define __LED_Control_H__

#include <stdint.h>

struct LED_Setting_Tag;

void LLSI_Initial(void);
void Set_Single(uint8_t *LED_DATA, uint32_t TotalLED, uint8_t Data_R, uint8_t Data_G, uint8_t Data_B);
void Set_Array(uint8_t *LED_DATA, uint32_t TotalLED, uint8_t *DisplayData, uint8_t MaxBright);
void Set_InverseArray(uint8_t *LED_DATA, uint32_t TotalLED, uint8_t *DisplayData, uint8_t MaxBright);
void Set_LED_Data(volatile struct LED_Setting_Tag* LED_Setting);
void Clear_LED_Data(volatile struct LED_Setting_Tag* LED_Setting);
void LLSI_StartFlashLED(uint8_t u8LLSIEnable);
uint8_t LLSI_FlashLEDRoutine(void);
void LLSI_WriteData(uint16_t u16ByteSel, uint8_t u8Data);
void LLSI_WriteBlockData(uint16_t u16ByteLen, uint8_t *pu8Data);

/*------Lighting Mode-----------------------------------------*/
void FUNC_Off(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Static(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Breathing(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Cycling(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Random(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Strobe(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Music(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Wave(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Spring(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Water(volatile struct LED_Setting_Tag* LED_Setting);
void FUNC_Rainbow(volatile struct LED_Setting_Tag* LED_Setting);

#endif  /* __LED_Control_H__ */

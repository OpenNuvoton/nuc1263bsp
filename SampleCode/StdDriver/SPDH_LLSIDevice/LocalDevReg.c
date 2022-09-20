/**************************************************************************//**
 * @file     LocalDevReg.c
 * @version  V3.00
 * @brief    To access the volatile register of device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#include "LocalDevReg.h"

#define DEV_REG_ATTR_R   0
#define DEV_REG_ATTR_RW  1
#define DEV_REG_ATTR_W   2
#define DEV_REG_ATTR_RV  3


//MR0 ~ MR63
volatile uint8_t g_au8DevReg[MAX_DEVREG_LEN] = {
    0x51, //MR0,  Device Type; Most Significant Byte
    0x18, //MR1,  Device Type; Least Significant Byte
    0x10,/*0x11,*/ //MR2,  Device Revision
    0xFE, //MR3,  Vendor ID byte 0
    0xFE, //MR4,  Vendor ID byte 1
    0x00, //MR5,  Reserved
    0x00, //MR6,  Reserved
    0x00, //MR7,  Reserved
    0x00, //MR8,  Reserved
    0x00, //MR9,  Reserved
    0x00, //MR10, Reserved
    0x00, //MR11, Reserved
    0x00, //MR12, Reserved
    0x00, //MR13, Reserved
    0x00, //MR14, Reserved
    0x00, //MR15, Reserved
    0x00, //MR16, Reserved
    0x00, //MR17, Reserved
    0x00, //MR18, Device Configuration
    0x00, //MR19, Reserved
    0x00, //MR20, Clear Register Command
    0x00, //MR21, Reserved
    0x00, //MR22, Reserved
    0x00, //MR23, Reserved
    0x00, //MR24, Reserved
    0x00, //MR25, Reserved
    0x00, //MR26, Reserved
    0x00, //MR27, Interrupt Configuration
    0x00, //MR28, Reserved
    0x00, //MR29, Reserved
    0x00, //MR30, Reserved
    0x00, //MR31, Reserved
    0x00, //MR32, Reserved
    0x00, //MR33, Reserved
    0x00, //MR34, Reserved
    0x00, //MR35, Reserved
    0x00, //MR36, LLSI Enable
    0x00, //MR37, LLSI Transfer Mode and Frequency Selection
    0x01, //MR38, LLSI Build-in Mode LED Function Selection, default is static mode
    0x0A, //MR39, LLSI Pixel Count Selection 0, defaule is 10 LEDs
    0x00, //MR40, LLSI Pixel Count Selection 1
    0x00, //MR41, LLSI Data Byte Selection 0
    0x00, //MR42, LLSI Data Byte Selection 1
    0x00, //MR43, LLSI Data
    0x00, //MR44, Color R
    0xFF, //MR45, Color G, default is green color
    0x00, //MR46, Color B
    0x00, //MR47, Write Mode
    0x00, //MR48, Device Status
    0x00, //MR49, Reserved
    0x00, //MR50, Reserved
    0x00, //MR51, Reserved
    0x00, //MR52, Error Status
    0x00, //MR53, Reserved
    0x00, //MR54, Reserved
    0x00, //MR55, Reserved
    0x00, //MR56, Reserved
    0x00, //MR57, Reserved
    0x00, //MR58, Reserved
    0x00, //MR59, Reserved
    0x00, //MR60, Reserved
    0x00, //MR61, Reserved
    0x00, //MR62, Reserved
    0x00 //MR63, Reserved
    };

//MR0 ~ MR63
volatile uint8_t g_au8DevRegAttr[MAX_DEVREG_LEN] = {
    DEV_REG_ATTR_R, //MR0,  Device Type; Most Significant Byte
    DEV_REG_ATTR_R, //MR1,  Device Type; Least Significant Byte
    DEV_REG_ATTR_R, //MR2,  Device Revision
    DEV_REG_ATTR_R, //MR3,  Vendor ID byte 0
    DEV_REG_ATTR_R, //MR4,  Vendor ID byte 1
    DEV_REG_ATTR_R, //MR5,  Reserved
    DEV_REG_ATTR_R, //MR6,  Reserved
    DEV_REG_ATTR_RV, //MR7,  Reserved
    DEV_REG_ATTR_RV, //MR8,  Reserved
    DEV_REG_ATTR_RV, //MR9,  Reserved
    DEV_REG_ATTR_RV, //MR10, Reserved
    DEV_REG_ATTR_RV, //MR11, Reserved
    DEV_REG_ATTR_RV, //MR12, Reserved
    DEV_REG_ATTR_RV, //MR13, Reserved
    DEV_REG_ATTR_RV, //MR14, Reserved
    DEV_REG_ATTR_RV, //MR15, Reserved
    DEV_REG_ATTR_RV, //MR16, Reserved
    DEV_REG_ATTR_RV, //MR17, Reserved
    DEV_REG_ATTR_RW, //MR18, Device Configuration, bit 5 is RO
    DEV_REG_ATTR_RV, //MR19, Reserved
    DEV_REG_ATTR_W, //MR20, Clear Register Command, bit 2 to 4 are RV
    DEV_REG_ATTR_RV, //MR21, Reserved
    DEV_REG_ATTR_RV, //MR22, Reserved
    DEV_REG_ATTR_RV, //MR23, Reserved
    DEV_REG_ATTR_RV, //MR24, Reserved
    DEV_REG_ATTR_RV, //MR25, Reserved
    DEV_REG_ATTR_RV, //MR26, Reserved
    DEV_REG_ATTR_RW, //MR27, Interrupt Configuration, bit 4 is RO, bit 7 is 1O, bit 5, 6 is RV
    DEV_REG_ATTR_RV, //MR28, Reserved
    DEV_REG_ATTR_RV, //MR29, Reserved
    DEV_REG_ATTR_RV, //MR30, Reserved
    DEV_REG_ATTR_RV, //MR31, Reserved
    DEV_REG_ATTR_RV, //MR32, Reserved
    DEV_REG_ATTR_RV, //MR33, Reserved
    DEV_REG_ATTR_RV, //MR34, Reserved
    DEV_REG_ATTR_RV, //MR35, Reserved
    DEV_REG_ATTR_RW, //MR36, LLSI Enable
    DEV_REG_ATTR_RW, //MR37, LLSI Transfer Mode and Frequency Selection
    DEV_REG_ATTR_RW, //MR38, LLSI Build-in Mode LED Function Selection
    DEV_REG_ATTR_RW, //MR39, LLSI Pixel Count Selection 0
    DEV_REG_ATTR_RW, //MR40, LLSI Pixel Count Selection 1
    DEV_REG_ATTR_RW, //MR41, LLSI Data Byte Selection 0
    DEV_REG_ATTR_RW, //MR42, LLSI Data Byte Selection 1
    DEV_REG_ATTR_RW, //MR43, LLSI Data
    DEV_REG_ATTR_RW, //MR44, Color R
    DEV_REG_ATTR_RW, //MR45, Color G
    DEV_REG_ATTR_RW, //MR46, Color B
    DEV_REG_ATTR_RW, //MR47, Write Mode
    DEV_REG_ATTR_R, //MR48, Device Status
    DEV_REG_ATTR_RV, //MR49, Reserved
    DEV_REG_ATTR_RV, //MR50, Reserved
    DEV_REG_ATTR_RV, //MR51, Reserved
    DEV_REG_ATTR_R, //Error Status
    DEV_REG_ATTR_RV, //MR53, Reserved
    DEV_REG_ATTR_RV, //MR54, Reserved
    DEV_REG_ATTR_RV, //MR55, Reserved
    DEV_REG_ATTR_RV, //MR56, Reserved
    DEV_REG_ATTR_RV, //MR57, Reserved
    DEV_REG_ATTR_RV, //MR58, Reserved
    DEV_REG_ATTR_RV, //MR59, Reserved
    DEV_REG_ATTR_RV, //MR60, Reserved
    DEV_REG_ATTR_RV, //MR61, Reserved
    DEV_REG_ATTR_RV, //MR62, Reserved
    DEV_REG_ATTR_RV //MR63, Reserved
};

//MR0 ~ MR63
volatile uint8_t g_au8DevRegWriteMsk[MAX_DEVREG_LEN] = {
    0x0, //MR0,  Device Type; Most Significant Byte
    0x0, //MR1,  Device Type; Least Significant Byte
    0x0, //MR2,  Device Revision
    0x0, //MR3,  Vendor ID byte 0
    0x0, //MR4,  Vendor ID byte 1
    0x0, //MR5,  Reserved
    0x0, //MR6,  Reserved
    0x0, //MR7,  Reserved
    0x0, //MR8,  Reserved
    0x0, //MR9,  Reserved
    0x0, //MR10, Reserved
    0x0, //MR11, Reserved
    0x0, //MR12, Reserved
    0x0, //MR13, Reserved
    0x0, //MR14, Reserved
    0x0, //MR15, Reserved
    0x0, //MR16, Reserved
    0x0, //MR17, Reserved
    0xDE, //MR18, Device Configuration, bit 5 is RO
    0x0, //MR19, Reserved
    0x3, //MR20, Clear Register Command, bit 2 to 4 are RV
    0x0, //MR21, Reserved
    0x0, //MR22, Reserved
    0x0, //MR23, Reserved
    0x0, //MR24, Reserved
    0x0, //MR25, Reserved
    0x0, //MR26, Reserved
    0x8F, //MR27, Interrupt Configuration, bit 4 is RO, bit 7 is 1O, bit 5, 6 is RV
    0x0, //MR28, Reserved
    0x0, //MR29, Reserved
    0x0, //MR30, Reserved
    0x0, //MR31, Reserved
    0x0, //MR32, Reserved
    0x0, //MR33, Reserved
    0x0, //MR34, Reserved
    0x0, //MR35, Reserved
    0x1, //MR36, LLSI Enable
    0x3, //MR37, LLSI Transfer Mode and Frequency Selection
    0x1F, //MR38, LLSI Build-in Mode LED Function Selection
    0xFF, //MR39, LLSI Pixel Count Selection 0
    0x1, //MR40, LLSI Pixel Count Selection 1
    0xFF, //MR41, LLSI Data Byte Selection 0
    0x3, //MR42, LLSI Data Byte Selection 1
    0xFF, //MR43, LLSI Data
    0xFF, //MR44, Color R
    0xFF, //MR45, Color G
    0xFF, //MR46, Color B
    0x1, //MR47, Write Mode
    0x0, //MR48, Device Status
    0x0, //MR49, Reserved
    0x0, //MR50, Reserved
    0x0, //MR51, Reserved
    0x0, //Error Status
    0x0, //MR53, Reserved
    0x0, //MR54, Reserved
    0x0, //MR55, Reserved
    0x0, //MR56, Reserved
    0x0, //MR57, Reserved
    0x0, //MR58, Reserved
    0x0, //MR59, Reserved
    0x0, //MR60, Reserved
    0x0, //MR61, Reserved
    0x0, //MR62, Reserved
    0x0 //MR63, Reserved
};

/**
 * @brief Read the specific address register value of device directly
 *
 * @param[in]  u8RegAddr   Register Address
 *
 * @retval   u8Value
 *
 * @details Read the device register value.
 *
 */
#define READ_DEV_REG_D(u8RegAddr) (g_au8DevReg[(u8RegAddr)])

/**
 * @brief Read the specific address register value of device
 *
 * @param[in]  u8RegAddr   Register Address
 *
 * @retval   u8Value
 *
 * @details Read the device register value.
 *
 */
#define READ_DEV_REG(u8RegAddr) (g_au8DevReg[(u8RegAddr)])

/**
 * @brief Write the specific address register value of device directly
 *
 * @param[in] u8RegAddr   Register Address
 * @param[in] u8Value     Register Value
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details Write the device register value.
 *
 */
#define WRITE_DEV_REG_D(u8RegAddr, u8Value) (g_au8DevReg[(u8RegAddr)] = (u8Value))

/**
 * @brief Clear the specific address register bit value of device
 *
 * @param[in] u8RegAddr   Register Address
 * @param[in] u8Bit       Register Bit
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details Clear the device register value.
 *
 */
#define CLR_DEV_REG_BIT_D(u8RegAddr, u8Bit) (g_au8DevReg[(u8RegAddr)] &= ~(1 << (u8Bit)))

/**
 * @brief Set the specific address register bit value of device
 *
 * @param[in] u8RegAddr   Register Address
 * @param[in] u8Bit       Register Bit
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details Clear the device register value.
 *
 */
#define SET_DEV_REG_BIT_D(u8RegAddr, u8Bit) (g_au8DevReg[(u8RegAddr)] |= (1 << (u8Bit)))

__INLINE int8_t DevReg_EnableI2C(void)
{
    /* Update MR18[5] NF_SEL bit as I2C basic protocol. */
    CLR_DEV_REG_BIT_D(18, 5);
    /* Disable IBI&PEC function */
    CLR_DEV_REG_BIT_D(27, 4);
    CLR_DEV_REG_BIT_D(18, 7);
    /* Clear parity function */
    CLR_DEV_REG_BIT_D(18, 6);

    return 0;
}

__INLINE int8_t DevReg_EnableI3C(void)
{
    /* Update MR18[5] NF_SEL bit as I3C basic protocol. */
    SET_DEV_REG_BIT_D(18, 5);

    return 0;
}

__INLINE int8_t DevReg_SetIBIStatus(void)
{
    /* Set MR48[7] : IBI_STATUS */
    SET_DEV_REG_BIT_D(48, 7);

    return 0;
}

__INLINE int8_t DevReg_ClearIBIStatus(void)
{
    /* Clear MR48[7] : IBI_STATUS */
    CLR_DEV_REG_BIT_D(48, 7);

    return 0;
}

__INLINE int8_t DevReg_GetIBIntEn(void)
{
    /* Get MR27[4:0] : IBI_STATUS */
    
    return (READ_DEV_REG_D(27)&0xF);
}

__INLINE int8_t DevReg_IsIBIntEn(uint8_t u8IntMsk)
{
    /* Get MR27[4:0] : IBI_STATUS */
    
    return ((READ_DEV_REG_D(27)&u8IntMsk)?1:0);
}

__INLINE int8_t DevReg_GetErrorStatus(void)
{
    /* Get MR52 : ERROR_STATUS */
    
    return (READ_DEV_REG_D(52));
}

__INLINE int8_t DevReg_SetParityErrStatus(void)
{
    /* MR52[0]: PAR_ERROR_STATUS */
    SET_DEV_REG_BIT_D(52, 0);

    return 0;
}

__INLINE int8_t DevReg_IBIEnable(void)
{
    /* Set MR27[4]: IBI_ERROR_EN, In Band Error Interrupt Enable */
    SET_DEV_REG_BIT_D(27, 4);

    return 0;
}

__INLINE int8_t DevReg_IBIDisable(void)
{
    /* Clear MR27[4]: IBI_ERROR_EN, In Band Error Interrupt Enable */
    CLR_DEV_REG_BIT_D(27, 4);

    return 0;
}

__INLINE int8_t DevReg_PECEnable(void)
{
    /* Update MR18 register. */
    SET_DEV_REG_BIT_D(18, 7);

    return 0;
}

__INLINE int8_t DevReg_PECDisable(void)
{
    /* Clear MR18 register. */
    CLR_DEV_REG_BIT_D(18, 7);

    return 0;
}

__INLINE int8_t DevReg_ParityDisable(void)
{
    /* Set MR18 register. */
    SET_DEV_REG_BIT_D(18, 6);

    return 0;
}

__INLINE int8_t DevReg_ParityEnable(void)
{
    /* Clear MR18 register. */
    CLR_DEV_REG_BIT_D(18, 6);

    return 0;
}

__INLINE int8_t DevReg_ClearAllEvent(void)
{
    /* MR27[7] CLR_GLOBAL is 1O.
        Global Clear Event Status and In Band Interrupt Status1,2
        1 = MR48[7], MR51[3:0] & MR52[7:5,3,1:0] Register (But no BIT3 definition in JESD300-5, this is a typo error.)

        [NOTE 1]  This register is a self clearing register after corresponding registers are cleared. Writing '0' in this
                    register has no effect.
        [NOTE 2]  After this command is issued, the device does not generate an IBI for any pending event. But if
                    new event occurs, the device does generate an IBI.
    */

    /* Set Bit 7 to 1 first before to clear corresponding registers. */
    g_au8DevReg[27] |= BIT7;

    g_au8DevReg[48]&= (~BIT7);

    g_au8DevReg[51]&= (~(BIT3|BIT2|BIT1|BIT0));

    g_au8DevReg[52]&= (~(BIT7|BIT6|BIT5|BIT3|BIT1|BIT0));/* No BIT3 definition in JESD300-5. */

    /* Set Bit 7 to 0 After cleared corresponding registers. */
    g_au8DevReg[27] &= (~BIT7);

    return 0;
}

/**
 * @brief Write the specific address register value of device
 *
 * @param[in] u8RegAddr   Register Address
 * @param[in] u8Value     Register Value
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details Write the device register value.
 *
 */
__INLINE int8_t DevReg_WriteReg(uint8_t u8RegAddr, uint8_t u8Value)
{
    uint32_t i;

    if (g_au8DevRegAttr[u8RegAddr] == DEV_REG_ATTR_RW)
    {
        //write value to MRn register
        g_au8DevReg[u8RegAddr] = u8Value & g_au8DevRegWriteMsk[u8RegAddr];

        if (u8RegAddr == 27)
        {
            //bit 7 is 1O
            if (u8Value & BIT7)
            {
                // clear MR48[7], MR51[3:0, MR52[7:5, 3, 1:0] register
                g_au8DevReg[48]&= (~BIT7);
                g_au8DevReg[51]&= (~(BIT3|BIT2|BIT1|BIT0));
                g_au8DevReg[52]&= (~(BIT7|BIT6|BIT5|BIT3|BIT1|BIT0));/* No BIT3 definition in JESD300-5. */
                // clear BIT0
                u8Value &= ~BIT7;
            }
            //update MRn register
            g_au8DevReg[u8RegAddr] = u8Value & g_au8DevRegWriteMsk[u8RegAddr];
            return 0;
        }
        else if (u8RegAddr == 36)
        {
            //bit 0: LLSI enable bit
            if (u8Value & BIT0)
            {
                /* Read setting from MR register */
                //void ReadStoredSetting(uint8_t MODESEL, uint8_t FRESEL, uint8_t LEDFUNSEL, uint16_t PCNTSEL)
                ReadStoredSetting(g_au8DevReg[37]&BIT0, g_au8DevReg[37]&BIT1, g_au8DevReg[38]&0xF, ((g_au8DevReg[40]&0x1)<<8)|(g_au8DevReg[39]&0xFF), g_au8DevReg[44], g_au8DevReg[45], g_au8DevReg[46]);
                /* Trigger LLSI to flash LED */
                LLSI_StartFlashLED(1);
            }
            else
            {
                /* Stop to flash LED */
                LLSI_StartFlashLED(0);
            }
        }
        else if (u8RegAddr == 43)
        {
            /* Write LED number by pixel count. */
            ReadStoredSetting(g_au8DevReg[37]&BIT0, g_au8DevReg[37]&BIT1, g_au8DevReg[38]&0xF, ((g_au8DevReg[40]&0x1)<<8)|(g_au8DevReg[39]&0xFF), g_au8DevReg[44], g_au8DevReg[45], g_au8DevReg[46]);
            LLSI_WriteData(((g_au8DevReg[42]&0x3)<<8)|g_au8DevReg[41], u8Value);
        }
    }
    else if (g_au8DevRegAttr[u8RegAddr] == DEV_REG_ATTR_W)
    {
        if (u8RegAddr == 19)
        {
            #if 0
            //bit 0 to 3 is 1O
            if (u8Value & BIT0)
            {
                // clear MR51[0] register
                g_au8HubReg[51] &= ~BIT0;
                // clear BIT0
                u8Value &= ~BIT0;
            }
            if (u8Value & BIT1)
            {
                // clear MR51[1] register
                g_au8HubReg[51] &= ~BIT1;
                // clear BIT1
                u8Value &= ~BIT1;
            }
            if (u8Value & BIT2)
            {
                // clear MR51[2] register
                g_au8HubReg[51] &= ~BIT2;
                // clear BIT2
                u8Value &= ~BIT2;
            }
            if (u8Value & BIT3)
            {
                // clear MR51[3] register
                g_au8HubReg[51] &= ~BIT3;
                // clear BIT3
                u8Value &= ~BIT3;
            }
            //bit 4 to 7 are RV
            g_au8HubReg[u8RegAddr] = u8Value & 0xF;
            #endif
            return 0;
        }
        else if (u8RegAddr == 20)
        {
            //bit 0, 1 and 5 to 7 is 1O
            if (u8Value & BIT0)
            {
                // clear MR52[0] register
                g_au8DevReg[52] &= ~BIT0;
                // clear BIT0
                u8Value &= ~BIT0;
            }
            if (u8Value & BIT1)
            {
                // clear MR52[1] register
                g_au8DevReg[52] &= ~BIT1;
                // clear BIT1
                u8Value &= ~BIT1;
            }

            //write value to MRn register
            g_au8DevReg[u8RegAddr] = u8Value & g_au8DevRegWriteMsk[u8RegAddr];
            return 0;
        }
    }

    return 0;
}

__INLINE int8_t DevReg_LLSIEnable(uint8_t u8Enable)
{
    /* Update LLSIEN(MR36[0]) bit */
    if (u8Enable)
    {
        SET_DEV_REG_BIT_D(36, 0);
    }
    else
    {
        CLR_DEV_REG_BIT_D(36, 0);
    }

    return 0;
}



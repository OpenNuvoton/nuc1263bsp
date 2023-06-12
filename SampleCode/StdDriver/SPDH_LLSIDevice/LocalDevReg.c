/**************************************************************************//**
 * @file     LocalDevReg.c
 * @version  V3.00
 * @brief    To access the volatile register of device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#include "spdh_device.h"
#include "LocalDevReg.h"
#include "MR_Register_LLSI.h"

#define DEV_REG_ATTR_R   0
#define DEV_REG_ATTR_RW  1
#define DEV_REG_ATTR_W   2
#define DEV_REG_ATTR_RV  3


//MR0 ~ MR63
volatile uint8_t g_au8DevReg[MAX_DEVREG_LEN] = {
    MR0_REG,    //MR0,  Device Type; Most Significant Byte
    MR1_REG,    //MR1,  Device Type; Least Significant Byte
    MR2_REG,    //MR2,  Device Revision
    MR3_REG,    //MR3,  Vendor ID byte 0
    MR4_REG,    //MR4,  Vendor ID byte 1
    MR5_REG,    //MR5,  Reserved
    MR6_REG,    //MR6,  Reserved
    MR7_REG,    //MR7,  Reserved
    MR8_REG,    //MR8,  Reserved
    MR9_REG,    //MR9,  Reserved
    MR10_REG,   //MR10, Reserved
    MR11_REG,   //MR11, Reserved
    MR12_REG,   //MR12, Reserved
    MR13_REG,   //MR13, Reserved
    MR14_REG,   //MR14, Reserved
    MR15_REG,   //MR15, Reserved
    MR16_REG,   //MR16, Reserved
    MR17_REG,   //MR17, Reserved
    MR18_REG,   //MR18, Device Configuration
    MR19_REG,   //MR19, Reserved
    MR20_REG,   //MR20, Clear Register Command
    MR21_REG,   //MR21, Reserved
    MR22_REG,   //MR22, Reserved
    MR23_REG,   //MR23, Reserved
    MR24_REG,   //MR24, Reserved
    MR25_REG,   //MR25, Reserved
    MR26_REG,   //MR26, Reserved
    MR27_REG,   //MR27, Interrupt Configuration
    MR28_REG,   //MR28, Reserved
    MR29_REG,   //MR29, Reserved
    MR30_REG,   //MR30, Reserved
    MR31_REG,   //MR31, Reserved
    MR32_REG,   //MR32, Reserved
    MR33_REG,   //MR33, Reserved
    MR34_REG,   //MR34, Reserved
    MR35_REG,   //MR35, Reserved
    MR36_REG,   //MR36, LLSI Enable
    MR37_REG,   //MR37, LLSI Transfer Mode and Frequency Selection
    MR38_REG,   //MR38, LLSI Build-in Mode LED Function Selection, default is static mode
    MR39_REG,   //MR39, LLSI Pixel Count Selection 0, defaule is 10 LEDs
    MR40_REG,   //MR40, LLSI Pixel Count Selection 1
    MR41_REG,   //MR41, LLSI Data Byte Selection 0
    MR42_REG,   //MR42, LLSI Data Byte Selection 1
    MR43_REG,   //MR43, LLSI Data
    MR44_REG,   //MR44, Color R
    MR45_REG,   //MR45, Color G, default is green color
    MR46_REG,   //MR46, Color B
    MR47_REG,   //MR47, Block Write Size
    MR48_REG,   //MR48, Device Status
    MR49_REG,   //MR49, Reserved
    MR50_REG,   //MR50, Reserved
    MR51_REG,   //MR51, Reserved
    MR52_REG,   //MR52, Error Status
    MR53_REG,   //MR53, Reserved
    MR54_REG,   //MR54, Reserved
    MR55_REG,   //MR55, LED Speed
    MR56_REG,   //MR56, LED Brightness
    MR57_REG,   //MR57, Reserved
    MR58_REG,   //MR58, Reserved
    MR59_REG,   //MR59, Reserved
    MR60_REG,   //MR60, Reserved
    MR61_REG,   //MR61, Reserved
    MR62_REG,   //MR62, Reserved
    MR63_REG,   //MR63, Reserved
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
    DEV_REG_ATTR_RW, //MR55, LED Speed
    DEV_REG_ATTR_RW, //MR56, LED Brightness
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
    0x7, //MR42, LLSI Data Byte Selection 1
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
    0xFF, //MR55, LED Speed
    0xFF, //MR56, LED Brightness
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
#define READ_DEV_REG_D(u8RegAddr)   (g_au8DevReg[(u8RegAddr)])

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
#define READ_DEV_REG(u8RegAddr)     (g_au8DevReg[(u8RegAddr)])

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

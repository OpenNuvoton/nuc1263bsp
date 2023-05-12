/**************************************************************************//**
 * @file     MR_Register_LLSI.h
 * @version  V3.00
 * @brief    MR registers configuration for LLSI device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef MR_REG_LLSI_CFG
#define MR_REG_LLSI_CFG

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------


// <o> MR0 - Device Type; Most Significant Byte, DTYP[7:0]
// <i> The code in this register is used in conjunction with any device type in MR1 register. 0xFF is invalid.
//      <0x00-0xFE:0x1>
#define MR0_REG     0x51


// <o> MR1 - Device Type; Least Significant Byte, DTYP[15:8]
// <i> The code in this register is used in conjunction with any device type in MR0 register. 0xFF is invalid.
//      <0x00-0xFE:0x1>
#define MR1_REG     0x08


// <o> MR2 - Device Revision
//      <0x00-0xFF:0x1>
#define MR2_REG     0x01


// <o> MR3 - Vendor ID Byte 0
//      <0x00-0xFF:0x1>
#define MR3_REG     0xFF


// <o> MR4 - Vendor ID Byte 1
//      <0x00-0xFF:0x1>
#define MR4_REG     0xFF


// <n> MR5 to MR17 - Reserved
#define MR5_REG     0x00
#define MR6_REG     0x00
#define MR7_REG     0x00
#define MR8_REG     0x00
#define MR9_REG     0x00
#define MR10_REG    0x00
#define MR11_REG    0x00
#define MR12_REG    0x00
#define MR13_REG    0x00
#define MR14_REG    0x00
#define MR15_REG    0x00
#define MR16_REG    0x00
#define MR17_REG    0x00


// <h> MR18 - Device Configuration
// <i> The write (or update) transaction to this register must be followed by STOP operation to allow the SPD5 Hub device to update the setting.
//      <n> [5] Interface Selection (RO)
//          <i> 0 = I2C Protocol (Max speed of 1 MHz) and 1 = I3C Basic Protocol. This register is automatically updated when SETAASA CCC or RSTDAA CCC is registered by the SPD5 Hub device or when SPD5 Hub device goes through bus reset as described in Bus Reset. This register can be read by the Host through normal Read operation but it cannot be written with normal write operation either in I2C mode or I3C Basic mode of operation. When this register is updated, it takes in effect when there is a next START operation (i.e., after STOP operation).
//      <o.6> [6] Parity (T bit) Disable
//      <i> 1. This register is updated when RSTDAA CCC is registered by SPD5 Hub device or when SPD5 Hub device goes through bus reset as described in Bus Reset.
//      <i> 2. This register is only applicable if MR18[5] = 1. When Parity function is disabled, the SPD5 Hub device simply ignores the T bit information from the Host. The host may actually choose to compute the parity and send that information in T bit or simply drive static low or high in T bit.
//          <0=> Enable
//          <1=> Disable
//      <o.7> [7] PEC Enable
//      <i> 1. This register is only applicable if MR18[5] = 1.
//      <i> 2. This register is updated when RSTDAA CCC is registered by SPD5 Hub device or when SPD5 Hub device goes through bus reset as described in Bus Reset.
//          <0=> Disable
//          <1=> Enable
// </h>
#define MR18_REG    0x00


// <n> MR19 - Reserved
#define MR19_REG    0x00


// <h> MR20 - Clear Register MR52 Status Command (1O)
// <i> This entire register is self clearing register after corresponding register is cleared.
//      <n> [0] Write 1 to clear MR52[0] Parity Error Status
//      <n> [1] Write 1 to clear MR52[1] Packet Error Status
// </h>
#define MR20_REG    0x00


// <n> MR21 to MR26 - Reserved
#define MR21_REG    0x00
#define MR22_REG    0x00
#define MR23_REG    0x00
#define MR24_REG    0x00
#define MR25_REG    0x00
#define MR26_REG    0x00


// <h> MR27 - Interrupt Configurations
//      <n> [4] In Band Error Interrupt Enable for MR52 Error Log (RO)
//          <i> 1. This register is automatically updated when ENEC CCC or DISEC CCC or RSTDAA CCC is registered by the SPD5 Hub device or when SPD5 Hub device goes through bus reset as described in Bus Reset. This register can be read by the Host through normal read operation but cannot be written with normal write operation either in I2C mode or I3C Basic mode. When this register is updated, it takes effect when there is a next START operation (i.e, after STOP operation).
//          <i> 2. 0 = Disable; Errors logged in MR52[7:5,1:0] registers do not generate an IBI to Host.
//          <i> 3. 1 = Enable; Errors logged in MR52[7:5,1:0] registers generates an IBI to Host.
//      <n> [7] Global Clear Event Status and In Band Interrupt Status (1O)
//          <i> 1. This register is a self clearing register after corresponding registers are cleared. Writing 0 in thisregister has no effect.
//          <i> 2. After this command is issued, the device does not generate an IBI for any pending event. But if new event occurs, the device does generate an IBI.
//          <i> 3. Write 1 to clear MR48[7], MR51[3:0] and MR52[7:5,3,1:0] Register,
// </h>
#define MR27_REG    0x00


// <n> MR28 to MR35 - Reserved
#define MR28_REG    0x00
#define MR29_REG    0x00
#define MR30_REG    0x00
#define MR31_REG    0x00
#define MR32_REG    0x00
#define MR33_REG    0x00
#define MR34_REG    0x00
#define MR35_REG    0x00


// <h> MR36 - LLSI Enable
//      <o.0> [0] Flash Start Enable  
//          <0=> Stop to flash LED light strip
//          <1=> Start to flash LED light strip
// </h>
#define MR36_REG    0x00


// <h> MR37 - LLSI Transfer Mode and Frequency Selection
//      <o.0> [0] Transfer Mode Selection  
//          <0=> Build in mode
//          <1=> Synchronous mode
//      <o.1> [1] Flash Frequency Selection  
//          <0=> One Shot
//          <1=> Continuous
// </h>
#define MR37_REG    0x00


// <h> MR38 - LLSI Build-in Mode LED Function Selection
// <i> Default is Static mode.
//      <o.0..3> [3:0] Display Type Selection for Build in Mode  
//          <0=> Off
//          <1=> Static
//          <2=> Breathing
//          <3=> Strobe
//          <4=> Cycling
//          <5=> Random
//          <6=> Off
//          <7=> Wave
//          <8=> Spring
//          <9=> Off
//          <10=> Off
//          <11=> Off
//          <12=> Off
//          <13=> Water
//          <14=> Rainbow
//          <15=> Off
// </h>
#define MR38_REG    0x01


// <o> MR39 - LLSI Pixel Count Selection 0, PCNTSEL[7:0]
// <i> LED Pixels Selection Bit[0:7]. Default is 10 LEDs.
//      <0x00-0xFF:0x1>
#define MR39_REG    0x0A


// <o> MR40 - LLSI Pixel Count Selection 1, PCNTSEL[8]
// <i> LED Pixels Selection Bit[8].
//      <0x00-0x1:0x1>
#define MR40_REG    0x00


// <o> MR41 - LLSI Data Byte Selection 0, BYTESEL[7:0]
// <i> LLSI Data Byte Selection Bit[0:7].
//      <0x00-0xFF:0x1>
#define MR41_REG    0x00


// <o> MR42 - LLSI Data Byte Selection 1, BYTESEL[8]
// <i> LLSI Data Byte Selection Bit[8].
//      <0x00-0x1:0x1>
#define MR42_REG    0x00


// <o> MR43 - LLSI Data
// <i> 10 LED need (10*3) bytes(RGB) = 30 bytes data, max 300 LED need (300*3) bytes = 900 bytes.
//      <0x00-0xFF:0x1>
#define MR43_REG    0x00


// <o> MR44 - Color R
//      <0x00-0xFF:0x1>
#define MR44_REG    0x00


// <o> MR45 - Color G
// <i> Default is green color.
//      <0x00-0xFF:0x1>
#define MR45_REG    0xFF


// <o> MR46 - Color B
//      <0x00-0xFF:0x1>
#define MR46_REG    0x00


// <h> MR47 - Write Mode
//      <o.0> [0] Block Write Enable  
//          <0=> Block write mode disabled 
//          <1=> Block write mode enabled
// </h>
#define MR47_REG    0x00


// <h> MR48 - Device Status (RO)
//      <n> [7] In Band Interrupt Status
//          <i> 1. 0 = No pending IBI.
//          <i> 2. 1 = Pending IBI.
// </h>
#define MR48_REG    0x00


// <n> MR49 to MR51 - Reserved
#define MR49_REG    0x00
#define MR50_REG    0x00
#define MR51_REG    0x00


// <h> MR52 - Error Status (RO)
//      <n> [0] Parity Check Error
//          <i> 1. Only applicable in MR18[5] = 1 and if Parity function is not disabled or for supported CCC in I2C mode.
//          <i> 2. 0 = No Parity Error.
//          <i> 3. 1 = Parity Error in one or more bytes.
//      <n> [1] Packet Error
//          <i> 1. Only applicable in MR18[5] = 1 and if PEC function is enabled.
//          <i> 2. 0 = No Parity Error.
//          <i> 3. 1 = Parity Error in one or more bytes.
// </h>
#define MR52_REG    0x00


// <n> MR53 to MR54 - Reserved
#define MR53_REG    0x00
#define MR54_REG    0x00


// <o> MR55 - LED Speed
//      <0x00-0xFF:0x1>
#define MR55_REG    0x80


// <o> MR56 - LED Brightness
//      <0x00-0xFF:0x1>
#define MR56_REG    0xFF


// <n> MR57 to MR63 - Reserved
#define MR57_REG    0x00
#define MR58_REG    0x00
#define MR59_REG    0x00
#define MR60_REG    0x00
#define MR61_REG    0x00
#define MR62_REG    0x00
#define MR63_REG    0x00


//------------- <<< end of configuration section >>> -----------------------

#endif  /* MR_REG_LLSI_CFG */

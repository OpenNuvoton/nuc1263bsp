/**************************************************************************//**
 * @file     NUC1263.h
 * @version  V3.0
 * @brief    NUC1263 Series Peripheral Access Layer Header File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of NUC1263 MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Copyright Notice</b>
  *
  * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
  */

#ifndef __NUC1263_H__
#define __NUC1263_H__


/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

/**
 * @details  Interrupt Number Definition.
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn       = -14,      /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,      /*!< 3 Cortex-M23 Hard Fault Interrupt                    */
    SVCall_IRQn               = -5,       /*!< 11 Cortex-M23 SV Call Interrupt                      */
    PendSV_IRQn               = -2,       /*!< 14 Cortex-M23 Pend SV Interrupt                      */
    SysTick_IRQn              = -1,       /*!< 15 Cortex-M23 System Tick Interrupt                  */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                  = 0,        /*!< Brown-Out Low Voltage Detected Interrupt             */
    WDT_IRQn                  = 1,        /*!< Watch Dog Timer Interrupt                            */
    EINT024_IRQn              = 2,        /*!< EINT0, EINT2 and EINT4 Interrupt                     */
    EINT135_IRQn              = 3,        /*!< EINT1, EINT3 and EINT5 Interrupt                     */
    GPAB_IRQn                 = 4,        /*!< GPIO_PA/PB Interrupt                                 */
    GPCDF_IRQn                = 5,        /*!< GPIO_PC/PD/PF Interrupt                              */
    BPWM0_IRQn                = 6,        /*!< BPWM0 Interrupt                                      */
    BPWM1_IRQn                = 7,        /*!< BPWM1 Interrupt                                      */
    TMR0_IRQn                 = 8,        /*!< TIMER0 Interrupt                                     */
    TMR1_IRQn                 = 9,        /*!< TIMER1 Interrupt                                     */
    TMR2_IRQn                 = 10,       /*!< TIMER2 Interrupt                                     */
    TMR3_IRQn                 = 11,       /*!< TIMER3 Interrupt                                     */
    UART0_IRQn                = 12,       /*!< UART0 Interrupt                                      */
    UART1_IRQn                = 13,       /*!< UART1 Interrupt                                      */
    SPI0_IRQn                 = 14,       /*!< SPI0 Interrupt                                       */
    SPI1_IRQn                 = 15,       /*!< SPI1 Interrupt                                       */
    BPWM2_IRQn                = 16,       /*!< BPWM2 Interrupt                                      */
    BPWM3_IRQn                = 17,       /*!< BPWM3 Interrupt                                      */
    I2C0_IRQn                 = 18,       /*!< I2C0 Interrupt                                       */
    I2C1_IRQn                 = 19,       /*!< I2C1 Interrupt                                       */
    I2C2_IRQn                 = 20,       /*!< I2C2 Interrupt                                       */
    USBD_IRQn                 = 23,       /*!< USB Device Interrupt                                 */
    ACMP01_IRQn               = 25,       /*!< ACMP01 Interrupt                                     */
    PDMA_IRQn                 = 26,       /*!< PDMA Interrupt                                       */
    PWRWU_IRQn                = 28,       /*!< Power Down Wake Up Interrupt                         */
    ADC_IRQn                  = 29,       /*!< ADC Interrupt                                        */
    CLKDIRC_IRQn              = 30,       /*!< Clock fail detect and IRC TRIM Interrupt             */
    LLSI0_IRQn                = 32,       /*!< LLSI0 Interrupt                                      */
    LLSI1_IRQn                = 33,       /*!< LLSI1 Interrupt                                      */
    LLSI2_IRQn                = 34,       /*!< LLSI2 Interrupt                                      */
    LLSI3_IRQn                = 35,       /*!< LLSI3 Interrupt                                      */
    LLSI4_IRQn                = 36,       /*!< LLSI4 Interrupt                                      */
    LLSI5_IRQn                = 37,       /*!< LLSI5 Interrupt                                      */
    SPI2_IRQn                 = 42,       /*!< SPI2 Interrupt                                       */
    UART2_IRQn                = 43,       /*!< UART2 Interrupt                                      */
    I3C0_IRQn                 = 44,       /*!< I3C0 Interrupt                                       */
    I3C1_IRQn                 = 45,       /*!< I3C1 Interrupt                                       */
    DAC_IRQn                  = 46,       /*!< DAC Interrupt                                        */
    ACMP23_IRQn               = 47,       /*!< ACMP23 Interrupt                                     */
    TS_IRQn                   = 48,       /*!< Temperature Sensor Interrupt                         */
    SPDH_IRQn                 = 49,       /*!< SPDH Interrupt                                       */

} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT           0       /*!< armikcmu does not provide a MPU present or not       */
#define __NVIC_PRIO_BITS        2       /*!< armikcmu Supports 2 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used         */


#include "core_cm0.h"                   /*!< Cortex-M0 processor and core peripherals             */
#include "system_NUC1263.h"             /*!< NUC1263 System                                       */


#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/**
 * Initialize the system clock
 *
 * @param  None
 * @return None
 *
 * @brief  Setup the microcontroller system
 *         Initialize the PLL and update the SystemFrequency variable
 */
extern void SystemInit(void);



/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

    @{
*/



/*---------------------- Cyclic Redundancy Check Controller -------------------------*/
/**
    @addtogroup CRC Cyclic Redundancy Check Controller(CRC)
    Memory Mapped Structure for CRC Controller
    @{ 
*/

typedef struct
{


    /**
     * @var CRC_T::CTL
     * Offset: 0x00  CRC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRCEN     |CRC Generator Enable Bit
     * |        |          |Set this bit 1 to enable CRC generator for CRC operation.
     * |        |          |0 = No effect.
     * |        |          |1 = CRC generator is active.
     * |[1]     |CHKSINIT  |Checksum Initialization
     * |        |          |Set this bit will auto reolad SEED (CRC_SEED [31:0]) to CHECKSUM (CRC_CHECKSUM[31:0]) as CRC operation initial value.
     * |        |          |0 = No effect.
     * |        |          |1 = Reload SEED value to CHECKSUM as CRC operation initial value.
     * |        |          |The others contents of CRC_CTL register will not be cleared.
     * |        |          |Note: This bit will be cleared automatically.
     * |[24]    |DATREV    |Write Data Bit Order Reverse Enable Bit
     * |        |          |This bit is used to enable the bit order reverse function per byte for write data value DATA (CRC_DATA[31:0]).
     * |        |          |0 = Bit order reversed for CRC DATA Disabled.
     * |        |          |1 = Bit order reversed for CRC DATA Enabled (per byte).
     * |        |          |Note: If the write data is 0xAABBCCDD, the bit order reverse for CRC write data in is 0x55DD33BB.
     * |[25]    |CHKSREV   |Checksum Bit Order Reverse Enable Bit
     * |        |          |This bit is used to enable the bit order reverse function for checksum result CHECKSUM (CRC_CHECKSUM[31:0]).
     * |        |          |0 = Bit order reverse for CRC CHECKSUMCRC Disabled.
     * |        |          |1 = Bit order reverse for CRC CHECKSUMCRC Enabled.
     * |        |          |Note: If the checksum result is 0xDD7B0F2E, the bit order reverse result for CRC checksum is 0x74F0DEBB.
     * |[26]    |DATFMT    |Write Data 1's Complement Enable Bit
     * |        |          |This bit is used to enable the 1's complement function for write data value DATA (CRC_DATA[31:0]).
     * |        |          |0 = 1's complement for CRC DATA Disabled.
     * |        |          |1 = 1's complement for CRC DATA Enabled.
     * |[27]    |CHKSFMT   |Checksum 1's Complement Enable Bit
     * |        |          |This bit is used to enable the 1's complement function for checksum result in CHECKSUM (CRC_CHECKSUM[31:0]).
     * |        |          |0 = 1's complement for CRC CHECKSUM Disabled.
     * |        |          |1 = 1's complement for CRC CHECKSUM Enabled.
     * |[29:28] |DATLEN    |CPU Write Data Length
     * |        |          |This field indicates the valid write data length of DATA (CRC_DAT[31:0]).
     * |        |          |00 = Data length is 8-bit mode.
     * |        |          |01 = Data length is 16-bit mode.
     * |        |          |1x = Data length is 32-bit mode.
     * |        |          |Note: When the write data length is 8-bit mode, the valid data in CRC_DAT register is only DATA[7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_DAT register is only DATA[15:0]
     * |[31:30] |CRCMODE   |CRC Polynomial Mode
     * |        |          |This field indicates the CRC operation polynomial mode.
     * |        |          |00 = CRC-CCITT Polynomial mode.
     * |        |          |01 = CRC-8 Polynomial mode.
     * |        |          |10 = CRC-16 Polynomial mode.
     * |        |          |11 = CRC-32 Polynomial mode.
     * @var CRC_T::DAT
     * Offset: 0x04  CRC Write Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DATA      |CRC Write Data Bits
     * |        |          |User can write data directly by CPU mode or use PDMA function to write data to this field to perform CRC operation.
     * |        |          |Note: When the write data length is 8-bit mode, the valid data in CRC_DAT register is only DATA[7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_DAT register is only DATA[15:0].
     * @var CRC_T::SEED
     * Offset: 0x08  CRC Seed Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |SEED      |CRC Seed Value
     * |        |          |This field indicates the CRC seed value.
     * |        |          |Note1: This SEED value will be loaded to checksum initial value CHECKSUM (CRC_CHECKSUM[31:0]) after set CHKSINIT (CRC_CTL[1]) to 1.
     * |        |          |Note2: The valid bits of CRC_SEED[31:0] is correlated to CRCMODE (CRC_CTL[31:30]).
     * @var CRC_T::CHECKSUM
     * Offset: 0x0C  CRC Checksum Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CHECKSUM  |CRC Checksum Results
     * |        |          |This field indicates the CRC checksum result.
     * |        |          |Note: The valid bits of CRC_CHECKSUM[31:0] is correlated to CRCMODE (CRC_CTL[31:30]).
     */
    __IO uint32_t CTL;                   /*!< [0x0000] CRC Control Register                                             */
    __IO uint32_t DAT;                   /*!< [0x0004] CRC Write Data Register                                          */
    __IO uint32_t SEED;                  /*!< [0x0008] CRC Seed Register                                                */
    __I  uint32_t CHECKSUM;              /*!< [0x000c] CRC Checksum Register                                            */

} CRC_T;

/**
    @addtogroup CRC_CONST CRC Bit Field Definition
    Constant Definitions for CRC Controller
    @{ 
*/

#define CRC_CTL_CRCEN_Pos                (0)                                               /*!< CRC_T::CTL: CRCEN Position             */
#define CRC_CTL_CRCEN_Msk                (0x1ul << CRC_CTL_CRCEN_Pos)                      /*!< CRC_T::CTL: CRCEN Mask                 */

#define CRC_CTL_CHKSINIT_Pos             (1)                                               /*!< CRC_T::CTL: CHKSINIT Position          */
#define CRC_CTL_CHKSINIT_Msk             (0x1ul << CRC_CTL_CHKSINIT_Pos)                   /*!< CRC_T::CTL: CHKSINIT Mask              */

#define CRC_CTL_DATREV_Pos               (24)                                              /*!< CRC_T::CTL: DATREV Position            */
#define CRC_CTL_DATREV_Msk               (0x1ul << CRC_CTL_DATREV_Pos)                     /*!< CRC_T::CTL: DATREV Mask                */

#define CRC_CTL_CHKSREV_Pos              (25)                                              /*!< CRC_T::CTL: CHKSREV Position           */
#define CRC_CTL_CHKSREV_Msk              (0x1ul << CRC_CTL_CHKSREV_Pos)                    /*!< CRC_T::CTL: CHKSREV Mask               */

#define CRC_CTL_DATFMT_Pos               (26)                                              /*!< CRC_T::CTL: DATFMT Position            */
#define CRC_CTL_DATFMT_Msk               (0x1ul << CRC_CTL_DATFMT_Pos)                     /*!< CRC_T::CTL: DATFMT Mask                */

#define CRC_CTL_CHKSFMT_Pos              (27)                                              /*!< CRC_T::CTL: CHKSFMT Position           */
#define CRC_CTL_CHKSFMT_Msk              (0x1ul << CRC_CTL_CHKSFMT_Pos)                    /*!< CRC_T::CTL: CHKSFMT Mask               */

#define CRC_CTL_DATLEN_Pos               (28)                                              /*!< CRC_T::CTL: DATLEN Position            */
#define CRC_CTL_DATLEN_Msk               (0x3ul << CRC_CTL_DATLEN_Pos)                     /*!< CRC_T::CTL: DATLEN Mask                */

#define CRC_CTL_CRCMODE_Pos              (30)                                              /*!< CRC_T::CTL: CRCMODE Position           */
#define CRC_CTL_CRCMODE_Msk              (0x3ul << CRC_CTL_CRCMODE_Pos)                    /*!< CRC_T::CTL: CRCMODE Mask               */

#define CRC_DAT_DATA_Pos                 (0)                                               /*!< CRC_T::DAT: DATA Position              */
#define CRC_DAT_DATA_Msk                 (0xfffffffful << CRC_DAT_DATA_Pos)                /*!< CRC_T::DAT: DATA Mask                  */

#define CRC_SEED_SEED_Pos                (0)                                               /*!< CRC_T::SEED: SEED Position             */
#define CRC_SEED_SEED_Msk                (0xfffffffful << CRC_SEED_SEED_Pos)               /*!< CRC_T::SEED: SEED Mask                 */

#define CRC_CHECKSUM_CHECKSUM_Pos        (0)                                               /*!< CRC_T::CHECKSUM: CHECKSUM Position     */
#define CRC_CHECKSUM_CHECKSUM_Msk        (0xfffffffful << CRC_CHECKSUM_CHECKSUM_Pos)       /*!< CRC_T::CHECKSUM: CHECKSUM Mask         */

/**@}*/ /* CRC_CONST */
/**@}*/ /* end of CRC register group */


/*---------------------- External Bus Interface Controller -------------------------*/
/**
    @addtogroup EBI External Bus Interface Controller(EBI)
    Memory Mapped Structure for EBI Controller
    @{ 
*/

typedef struct
{


    /**
     * @var EBI_T::CTL0
     * Offset: 0x00  External Bus Interface Bank0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EN        |EBI Enable Bit
     * |        |          |This bit is the functional enable bit for EBI.
     * |        |          |0 = EBI function Disabled.
     * |        |          |1 = EBI function Enabled.
     * |[1]     |DW16      |EBI Data Width 16-bit Select
     * |        |          |This bit defines if the EBI data width is 8-bit or 16-bit.
     * |        |          |0 = EBI data width is 8-bit.
     * |        |          |1 = EBI data width is 16-bit.
     * |[2]     |CSPOLINV  |Chip Select Pin Polar Inverse
     * |        |          |This bit defines the active level of EBI chip select pin (EBI_nCSx), x = 0 or 1..
     * |        |          |0 = Chip select pin (EBI_nCSx) is active low.
     * |        |          |1 = Chip select pin (EBI_nCSx) is active high.
     * |        |          |x = 0, 1
     * |[4]     |CACCESS   |Continuous Data Access Mode
     * |        |          |When con ttinuousenuous access mode enabled, the tASU, tALE and tLHD cycles are bypass for continuous data transfer request.
     * |        |          |0 = Continuous data access mode Disabled.
     * |        |          |1 = Continuous data access mode Enabled.
     * |[10:8]  |MCLKDIV   |External Output Clock Divider
     * |        |          |The frequency of EBI output clock (MCLK) is controlled by MCLKDIV as follow:
     * |        |          |000 = HCLK/1.
     * |        |          |001 = HCLK/2.
     * |        |          |010 = HCLK/4.
     * |        |          |011 = HCLK/8.
     * |        |          |100 = HCLK/16.
     * |        |          |101 = HCLK/32.
     * |        |          |110 = HCLK/64.
     * |        |          |111 = HCLK/128.
     * |[18:16] |TALE      |Extend Time Of of ALE
     * |        |          |The EBI_ALE high pulse period (tALE) to latch the address can be controlled by TALE.
     * |        |          |tALE = (TALE + 1)*EBI_MCLK.
     * |        |          |Note: This field only available in EBI_CTL0 register
     * |[24]    |WBUFEN    |EBI Write Buffer Enable Bit
     * |        |          |0 = EBI write buffer Disabled.
     * |        |          |1 = EBI write buffer Enabled.
     * |        |          |Note: This bit only available in EBI_CTL0 register
     * @var EBI_T::TCTL0
     * Offset: 0x04  External Bus Interface Bank0 Timing Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:3]   |TACC      |EBI Data Access Time
     * |        |          |TACC define data access time (tACC).
     * |        |          |tACC = (TACC + 1) * EBI_MCLK.
     * |[10:8]  |TAHD      |EBI Data Access Hold Time
     * |        |          |TAHD define data access hold time (tAHD).
     * |        |          |tAHD = (TAHD + 1) * EBI_MCLK.
     * |[15:12] |W2X       |Idle Cycle After Write
     * |        |          |This field defines the number of W2X idle cycle.
     * |        |          |When write action is finish, W2X idle cycle is inserted and EBI_nCSx return to idle state, x = 0 or 1.
     * |        |          |W2X idle cycle = (W2X * EBI_MCLK).
     * |        |          |When write action is finish, W2X idle cycle is inserted and EBI_nCSx return to idle state. (x = 0, 1)
     * |[22]    |RAHDOFF   |Access Hold Time Disable Control When Read
     * |        |          |0 = The Data Access Hold Time (tAHD) during EBI reading is Enabled.
     * |        |          |1 = The Data Access Hold Time (tAHD) during EBI reading is Disabled.
     * |[23]    |WAHDOFF   |Access Hold Time Disable Control When Write
     * |        |          |0 = The Data Access Hold Time (tAHD) during EBI writing is Enabled.
     * |        |          |1 = The Data Access Hold Time (tAHD) during EBI writing is Disabled.
     * |[27:24] |R2R       |Idle Cycle Between Read-to-read
     * |        |          |This field defines the number of R2R idle cycle.
     * |        |          |When read action is finish and next action is going to read, R2R idle cycle is inserted and EBI_nCSx return to idle state, x = 0 or 1.
     * |        |          |R2R idle cycle = (R2R * EBI_MCLK).
     * |        |          |When read action is finish and next action is going to read, R2R idle cycle is inserted and EBI_nCSx return to idle state
     * |        |          |(x = 0, 1)
     * @var EBI_T::CTL1
     * Offset: 0x10  External Bus Interface Bank1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EN        |EBI Enable Bit
     * |        |          |This bit is the functional enable bit for EBI.
     * |        |          |0 = EBI function Disabled.
     * |        |          |1 = EBI function Enabled.
     * |[1]     |DW16      |EBI Data Width 16-bit Select
     * |        |          |This bit defines if the EBI data width is 8-bit or 16-bit.
     * |        |          |0 = EBI data width is 8-bit.
     * |        |          |1 = EBI data width is 16-bit.
     * |[2]     |CSPOLINV  |Chip Select Pin Polar Inverse
     * |        |          |This bit defines the active level of EBI chip select pin (EBI_nCSx), x = 0 or 1..
     * |        |          |0 = Chip select pin (EBI_nCSx) is active low.
     * |        |          |1 = Chip select pin (EBI_nCSx) is active high.
     * |        |          |x = 0, 1
     * |[4]     |CACCESS   |Continuous Data Access Mode
     * |        |          |When con ttinuousenuous access mode enabled, the tASU, tALE and tLHD cycles are bypass for continuous data transfer request.
     * |        |          |0 = Continuous data access mode Disabled.
     * |        |          |1 = Continuous data access mode Enabled.
     * |[10:8]  |MCLKDIV   |External Output Clock Divider
     * |        |          |The frequency of EBI output clock (MCLK) is controlled by MCLKDIV as follow:
     * |        |          |000 = HCLK/1.
     * |        |          |001 = HCLK/2.
     * |        |          |010 = HCLK/4.
     * |        |          |011 = HCLK/8.
     * |        |          |100 = HCLK/16.
     * |        |          |101 = HCLK/32.
     * |        |          |110 = HCLK/64.
     * |        |          |111 = HCLK/128.
     * |[18:16] |TALE      |Extend Time Of of ALE
     * |        |          |The EBI_ALE high pulse period (tALE) to latch the address can be controlled by TALE.
     * |        |          |tALE = (TALE + 1)*EBI_MCLK.
     * |        |          |Note: This field only available in EBI_CTL0 register
     * |[24]    |WBUFEN    |EBI Write Buffer Enable Bit
     * |        |          |0 = EBI write buffer Disabled.
     * |        |          |1 = EBI write buffer Enabled.
     * |        |          |Note: This bit only available in EBI_CTL0 register
     * @var EBI_T::TCTL1
     * Offset: 0x14  External Bus Interface Bank1 Timing Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:3]   |TACC      |EBI Data Access Time
     * |        |          |TACC define data access time (tACC).
     * |        |          |tACC = (TACC + 1) * EBI_MCLK.
     * |[10:8]  |TAHD      |EBI Data Access Hold Time
     * |        |          |TAHD define data access hold time (tAHD).
     * |        |          |tAHD = (TAHD + 1) * EBI_MCLK.
     * |[15:12] |W2X       |Idle Cycle After Write
     * |        |          |This field defines the number of W2X idle cycle.
     * |        |          |When write action is finish, W2X idle cycle is inserted and EBI_nCSx return to idle state, x = 0 or 1.
     * |        |          |W2X idle cycle = (W2X * EBI_MCLK).
     * |        |          |When write action is finish, W2X idle cycle is inserted and EBI_nCSx return to idle state. (x = 0, 1)
     * |[22]    |RAHDOFF   |Access Hold Time Disable Control When Read
     * |        |          |0 = The Data Access Hold Time (tAHD) during EBI reading is Enabled.
     * |        |          |1 = The Data Access Hold Time (tAHD) during EBI reading is Disabled.
     * |[23]    |WAHDOFF   |Access Hold Time Disable Control When Write
     * |        |          |0 = The Data Access Hold Time (tAHD) during EBI writing is Enabled.
     * |        |          |1 = The Data Access Hold Time (tAHD) during EBI writing is Disabled.
     * |[27:24] |R2R       |Idle Cycle Between Read-to-read
     * |        |          |This field defines the number of R2R idle cycle.
     * |        |          |When read action is finish and next action is going to read, R2R idle cycle is inserted and EBI_nCSx return to idle state, x = 0 or 1.
     * |        |          |R2R idle cycle = (R2R * EBI_MCLK).
     * |        |          |When read action is finish and next action is going to read, R2R idle cycle is inserted and EBI_nCSx return to idle state
     * |        |          |(x = 0, 1)
     */
    __IO uint32_t CTL0;                  /*!< [0x0000] External Bus Interface Bank0 Control Register                    */
    __IO uint32_t TCTL0;                 /*!< [0x0004] External Bus Interface Bank0 Timing Control Register             */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t CTL1;                  /*!< [0x0010] External Bus Interface Bank1 Control Register                    */
    __IO uint32_t TCTL1;                 /*!< [0x0014] External Bus Interface Bank1 Timing Control Register             */

} EBI_T;

/**
    @addtogroup EBI_CONST EBI Bit Field Definition
    Constant Definitions for EBI Controller
    @{ 
*/

#define EBI_CTL_EN_Pos                   (0)                                               /*!< EBI_T::CTL: EN Position                  */
#define EBI_CTL_EN_Msk                   (0x1ul << EBI_CTL_EN_Pos)                         /*!< EBI_T::CTL: EN Mask                      */

#define EBI_CTL_DW16_Pos                 (1)                                               /*!< EBI_T::CTL: DW16 Position                */
#define EBI_CTL_DW16_Msk                 (0x1ul << EBI_CTL_DW16_Pos)                       /*!< EBI_T::CTL: DW16 Mask                    */

#define EBI_CTL_CSPOLINV_Pos             (2)                                               /*!< EBI_T::CTL: CSPOLINV Position            */
#define EBI_CTL_CSPOLINV_Msk             (0x1ul << EBI_CTL_CSPOLINV_Pos)                   /*!< EBI_T::CTL: CSPOLINV Mask                */

#define EBI_CTL_CACCESS_Pos              (4)                                               /*!< EBI_T::CTL: CACCESS Position             */
#define EBI_CTL_CACCESS_Msk              (0x1ul << EBI_CTL_CACCESS_Pos)                    /*!< EBI_T::CTL: CACCESS Mask                 */

#define EBI_CTL_MCLKDIV_Pos              (8)                                               /*!< EBI_T::CTL: MCLKDIV Position             */
#define EBI_CTL_MCLKDIV_Msk              (0x7ul << EBI_CTL_MCLKDIV_Pos)                    /*!< EBI_T::CTL: MCLKDIV Mask                 */

#define EBI_CTL_TALE_Pos                 (16)                                              /*!< EBI_T::CTL: TALE Position                */
#define EBI_CTL_TALE_Msk                 (0x7ul << EBI_CTL_TALE_Pos)                       /*!< EBI_T::CTL: TALE Mask                    */

#define EBI_CTL_WBUFEN_Pos               (24)                                              /*!< EBI_T::CTL: WBUFEN Position              */
#define EBI_CTL_WBUFEN_Msk               (0x1ul << EBI_CTL_WBUFEN_Pos)                     /*!< EBI_T::CTL: WBUFEN Mask                  */

#define EBI_TCTL_TACC_Pos                (3)                                               /*!< EBI_T::TCTL: TACC Position               */
#define EBI_TCTL_TACC_Msk                (0x1ful << EBI_TCTL_TACC_Pos)                     /*!< EBI_T::TCTL: TACC Mask                   */

#define EBI_TCTL_TAHD_Pos                (8)                                               /*!< EBI_T::TCTL: TAHD Position               */
#define EBI_TCTL_TAHD_Msk                (0x7ul << EBI_TCTL_TAHD_Pos)                      /*!< EBI_T::TCTL: TAHD Mask                   */

#define EBI_TCTL_W2X_Pos                 (12)                                              /*!< EBI_T::TCTL: W2X Position                */
#define EBI_TCTL_W2X_Msk                 (0xful << EBI_TCTL_W2X_Pos)                       /*!< EBI_T::TCTL: W2X Mask                    */

#define EBI_TCTL_RAHDOFF_Pos             (22)                                              /*!< EBI_T::TCTL: RAHDOFF Position            */
#define EBI_TCTL_RAHDOFF_Msk             (0x1ul << EBI_TCTL_RAHDOFF_Pos)                   /*!< EBI_T::TCTL: RAHDOFF Mask                */

#define EBI_TCTL_WAHDOFF_Pos             (23)                                              /*!< EBI_T::TCTL: WAHDOFF Position            */
#define EBI_TCTL_WAHDOFF_Msk             (0x1ul << EBI_TCTL_WAHDOFF_Pos)                   /*!< EBI_T::TCTL: WAHDOFF Mask                */

#define EBI_TCTL_R2R_Pos                 (24)                                              /*!< EBI_T::TCTL: R2R Position                */
#define EBI_TCTL_R2R_Msk                 (0xful << EBI_TCTL_R2R_Pos)                       /*!< EBI_T::TCTL: R2R Mask                    */

/**@}*/ /* EBI_CONST */
/**@}*/ /* end of EBI register group */



/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TIMER Timer Controller(TIMER)
    Memory Mapped Structure for TIMER Controller
    @{ 
*/

typedef struct
{


    /**
     * @var TIMER_T::CTL
     * Offset: 0x00  Timer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Timer input clock or event source is divided by (PSC+1) before it is fed to the timer up counter.
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |        |          |Note: Update prescale counter value will reset internal 8-bit prescale counter and 24-bit up counter value.
     * |[8]     |TRGPDMA   |Trigger PDMA Enable Bit  
     * |        |          |If this bit is set to 1, timer time-out interrupt or capture interrupt can trigger PDMA. 
     * |        |          |0 = Timer interrupt trigger PDMA Disabled.
     * |        |          |1 = Timer interrupt trigger PDMA Enabled.
     * |        |          |Note: If TRGSSEL (TIMERx_CTL[18]) = 0, time-out interrupt signal will trigger PDMA. If TRGSSEL (TIMERx_CTL[18]) = 1, capture interrupt signal will trigger PDMA.
     * |[9]     |TRGBPWM23 |Trigger BPWM23 Enable Bit  
     * |        |          |If this bit is set to 1, timer time-out interrupt or capture interrupt can trigger BPWM23. 
     * |        |          |0 = Timer interrupt trigger BPWM23 Disabled.
     * |        |          |1 = Timer interrupt trigger BPWM23 Enabled.
     * |        |          |Note: If TRGSSEL (TIMERx_CTL[18]) = 0, time-out interrupt signal will trigger BPWM23. If TRGSSEL (TIMERx_CTL[18]) = 1, capture interrupt signal will trigger BPWM23.
     * |[10]    |INTRGEN   |Inter-Timer Trigger Mode Enable Bit
     * |        |          |Setting this bit will enable the inter-timer trigger capture function.
     * |        |          |The Timer0/2 will be in event counter mode and counting with external clock source or event.
     * |        |          |Also, Timer1/3 will be in trigger-counting mode of capture function.
     * |        |          |0 = Inter-Timer Trigger mode Disabled.
     * |        |          |1 = Inter-Timer Trigger mode Enabled.
     * |        |          |Note: For Timer1/3, this bit is ignored and the read back value is always 0.
     * |[16]    |CAPSRC    |Capture Pin Source Selection
     * |        |          |0 = Capture Function source is from TMx_EXT (x= 0~3) pin.
     * |        |          |1 = Capture Function source is from LIRC.
     * |[18]    |TRGSSEL   |Trigger Source Select Bit  
     * |        |          |This bit is used to select trigger source is from Timer time-out interrupt signal or capture interrupt signal.
     * |        |          |0 = Timer time-out interrupt signal is used to trigger BPWM, ADC and PDMA.
     * |        |          |1 = Capture interrupt signal is used to trigger BPWM, ADC and PDMA.
     * |[19]    |TRGBPWM01 |Trigger BPWM01 Enable Bit  
     * |        |          |If this bit is set to 1, timer time-out interrupt or capture interrupt can trigger BPWM01. 
     * |        |          |0 = Timer interrupt trigger BPWM01 Disabled.
     * |        |          |1 = Timer interrupt trigger BPWM01 Enabled.
     * |        |          |Note: If TRGSSEL (TIMERx_CTL[18]) = 0, time-out interrupt signal will trigger BPWM01. If TRGSSEL (TIMERx_CTL[18]) = 1, capture interrupt signal will trigger BPWM01.
     * |[20]    |TRGDAC    |Trigger DAC Enable Bit  
     * |        |          |If this bit is set to 1, timer time-out interrupt or capture interrupt can trigger DAC. 
     * |        |          |0 = Timer interrupt trigger DAC Disabled.
     * |        |          |1 = Timer interrupt trigger DAC Enabled.
     * |        |          |Note: If TRGSSEL (TIMERx_CTL[18]) = 0, time-out interrupt signal will trigger DAC. If TRGSSEL (TIMERx_CTL[18]) = 1, capture interrupt signal will trigger DAC.
     * |[21]    |TRGADC    |Trigger ADC Enable Bit 
     * |        |          |If this bit is set to 1, timer time-out interrupt or capture interrupt can trigger ADC.
     * |        |          |0 = Timer interrupt trigger ADC Disabled.
     * |        |          |1 = Timer interrupt trigger ADC Enabled.
     * |        |          |Note: If TRGSSEL (TIMERx_CTL[18]) = 0, time-out interrupt signal will trigger ADC. If TRGSSEL (TIMERx_CTL[18]) = 1, capture interrupt signal will trigger ADC.
     * |[22]    |TGLPINSEL |Toggle-output Pin Select
     * |        |          |0 = Toggle mode output to TMx (Timer Event Counter Pin).
     * |        |          |1 = Toggle mode output to TMx_EXT (Timer External Capture Pin).
     * |[23]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while timer interrupt flag TIF (TIMERx_INTSTS[0]) is 1 and INTEN (TIMERx_CTL[29]) is enabled, the timer interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if timer interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if timer interrupt signal generated.
     * |[24]    |EXTCNTEN  |Event Counter Mode Enable Bit
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |0 = Event counter mode Disabled.
     * |        |          |1 = Event counter mode Enabled.
     * |        |          |Note1: When timer is used as an event counter, this bit should be set to 1 and select PCLKx (x=0~1) as timer clock source.
     * |        |          |Note2: When Timer0/Timer2 INTRGEN is set to 1, this bit is forced to 1 in Timer0/Timer2, and this bit is forced to 0 in Timer1/Timer3.
     * |[25]    |ACTSTS    |Timer Active Status Bit (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |RSTCNT    |Timer Counter Reset Bit
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset internal 8-bit prescale counter, 24-bit up counter value and CNTEN bit.
     * |        |          |Note: This bit will be auto cleared.
     * |[28:27] |OPMODE    |Timer Counting Mode Select
     * |        |          |00 = The timer controller is operated in One-shot mode.
     * |        |          |01 = The timer controller is operated in Periodic mode.
     * |        |          |10 = The timer controller is operated in Toggle-output mode.
     * |        |          |11 = The timer controller is operated in Continuous Counting mode.
     * |[29]    |INTEN     |Timer Interrupt Enable Bit
     * |        |          |0 = Timer time-out interrupt Disabled.
     * |        |          |1 = Timer time-out interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the timer time-out interrupt flag TIF is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CNTEN     |Timer Counting Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CNTEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TIMER_CTL[28:27] = 00) when the timer time-out interrupt flag TIF (TIMERx_INTSTS[0]) is generated.
     * |        |          |Note3: Setting this bit enable/disable needs 2 * TMR_CLK period to become active. User can read ACTSTS (TIMERx_CTL[25]) to check enable/disable command is completed or not.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var TIMER_T::CMP
     * Offset: 0x04  Timer Comparator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Timer Comparedator Value
     * |        |          |CMPDAT is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF (TIMERx_INTSTS[0] Timer Interrupt Flag) will set to 1.
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into CMPDAT field.
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest CMPDAT value to be the timer compared value while user writes a new value into CMPDAT field.
     * @var TIMER_T::INTSTS
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while 24-bit timer up counter CNT (TIMERx_CNT[23:0]) value reaches CMPDAT (TIMERx_CMP[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Timer Wake-Up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of timer.
     * |        |          |0 = Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var TIMER_T::CNT
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Timer Data Register
     * |        |          |Read this register to get CNT value. For example:
     * |        |          |If EXTCNTEN (TIMERx_CTL[24]) is 0, user can read CNT value for getting current 24-bit counter value.
     * |        |          |If EXTCNTEN (TIMERx_CTL[24]) is 1, user can read CNT value for getting current 24-bit event input counter value.
     * @var TIMER_T::CAP
     * Offset: 0x10  Timer Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Capture Data Register
     * |        |          |When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~3) pin or LIRC matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, CAPIF (TIMERx_EINTSTS[0]) will set to 1 and the current timer counter value CNT (TIMERx_CNT[23:0]) will be auto-loaded into this CAPDAT field.
     * @var TIMER_T::EXTCTL
     * Offset: 0x14  Timer External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTPHASE  |Timer External Count Phase
     * |        |          |This bit indicates the detection phase of external counting pin TMx (x= 0~3).
     * |        |          |0 = A falling edge of external counting pin will be counted.
     * |        |          |1 = A rising edge of external counting pin will be counted.
     * |[2:1]   |CAPEDGE   |Timer External Capture Pin Edge Detect
     * |        |          |00 = A Falling edge on TMx_EXT (x= 0~3) pin or LIRC will be detected.
     * |        |          |01 = A Rising edge on TMx_EXT (x= 0~3) pin or LIRC will be detected.
     * |        |          |10 = Either Rising or Falling edge on TMx_EXT (x= 0~3) pin or LIRC will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |CAPEN     |Timer Capture Enable Bit
     * |        |          |This bit enables the capture input function.
     * |        |          |0 = Capture source Disabled.
     * |        |          |1 = Capture source Enabled.
     * |[4]     |CAPFUNCS  |Capture Function Selection
     * |        |          |0 = Capture Mode Enabled.
     * |        |          |1 = Capture and Reset Mode Enabled.
     * |        |          |Note1: When CAPFUNCS is 0, transition on TMx_EXT (x= 0~3) pin or LIRC is using to save current 24-bit timer counter value (CNT value) to CAPDAT field.
     * |        |          |Note2: When CAPFUNCS is 1, transition on TMx_EXT (x= 0~3) pin or LIRC is using to save current 24-bit timer counter value (CNT value) to CAPDAT field then CNT value will be reset immediately.
     * |[5]     |CAPIEN    |Timer External Capture Interrupt Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~3) pin or LIRC detection Interrupt Disabled.
     * |        |          |1 = TMx_EXT (x= 0~3) pin or LIRC detection Interrupt Enabled.
     * |        |          |Note: CAPIEN is used to enable timer capture interrupt.
     * |        |          |If CAPIEN enabled, timer will rise an interrupt when CAPIF (TIMERx_EINTSTS[0]) is 1.
     * |        |          |For example, while CAPIEN = 1, CAPEN = 1, and CAPEDGE = 00, a 1 to 0 transition on the TMx_EXT pin (x= 0~3) pin or LIRC will cause the CAPIF to be set then the interrupt signal is generated and sent to NVIC to inform CPU.
     * |[6]     |CAPDBEN   |Timer External Capture Pin De-bounce Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~3) pin de-bounce Disabled.
     * |        |          |1 = TMx_EXT (x= 0~3) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx_EXT pin output is detected with de-bounce circuit.
     * |[7]     |CNTDBEN   |Timer Counter Pin De-bounce Enable Bit
     * |        |          |0 = TMx (x= 0~3) pin de-bounce Disabled.
     * |        |          |1 = TMx (x= 0~3) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     * |[8]     |ICAPSEL   |Internal Capture Source Select
     * |        |          |000 = Capture Function source is from internal ACMP0 output signal.
     * |        |          |001 = Capture Function source is from internal ACMP1 output signal.
     * |        |          |010 = Capture Function source is from internal ACMP2 output signal.
     * |        |          |011 = Capture Function source is from internal ACMP3 output signal.
     * |        |          |100 = Reserved.
     * |        |          |101 = Capture Function source is from LIRC.
     * |        |          |110, 111 = Reserved.
     * |        |          |Note: These bits only available when CAPSRC (TIMERx_CTL[16]) is 1.
     * @var TIMER_T::EINTSTS
     * Offset: 0x18  Timer0 External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPIF     |Timer Capture Interrupt Flag
     * |        |          |This bit indicates the timer capture interrupt flag status.
     * |        |          |0 = TMx_EXT (x= 0~3) pin or LIRC capture interrupt did not occur.
     * |        |          |1 = TMx_EXT (x= 0~3) pin or LIRC capture interrupt occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~3) pin LIRC matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPIF status.
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
     */

    __IO uint32_t CTL;                   /*!< [0x0000] Timer Control Register                                           */
    __IO uint32_t CMP;                   /*!< [0x0004] Timer Comparator Register                                        */
    __IO uint32_t INTSTS;                /*!< [0x0008] Timer Interrupt Status Register                                  */
    __IO uint32_t CNT;                   /*!< [0x000c] Timer Data Register                                              */
    __I  uint32_t CAP;                   /*!< [0x0010] Timer Capture Data Register                                      */
    __IO uint32_t EXTCTL;                /*!< [0x0014] Timer External Control Register                                  */
    __IO uint32_t EINTSTS;               /*!< [0x0018] Timer External Interrupt Status Register                         */

} TIMER_T;

/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
    @{ 
*/

#define TIMER_CTL_PSC_Pos                (0)                                               /*!< TIMER_T::CTL: PSC Position             */
#define TIMER_CTL_PSC_Msk                (0xfful << TIMER_CTL_PSC_Pos)                     /*!< TIMER_T::CTL: PSC Mask                 */

#define TIMER_CTL_TRGPDMA_Pos            (8)                                               /*!< TIMER_T::CTL: TRGPDMA Position          */
#define TIMER_CTL_TRGPDMA_Msk            (0x1ul << TIMER_CTL_TRGPDMA_Pos)                  /*!< TIMER_T::CTL: TRGPDMA Mask              */

#define TIMER_CTL_TRGBPWM23_Pos          (9)                                               /*!< TIMER_T::CTL: TRGBPWM23 Position        */
#define TIMER_CTL_TRGBPWM23_Msk          (0x1ul << TIMER_CTL_TRGBPWM23_Pos)                /*!< TIMER_T::CTL: TRGBPWM23 Mask            */

#define TIMER_CTL_INTRGEN_Pos            (10)                                              /*!< TIMER_T::CTL: INTRGEN Position         */
#define TIMER_CTL_INTRGEN_Msk            (0x1ul << TIMER_CTL_INTRGEN_Pos)                  /*!< TIMER_T::CTL: INTRGEN Mask             */

#define TIMER_CTL_CAPSRC_Pos             (16)                                              /*!< TIMER_T::CTL: CAPSRC Position          */
#define TIMER_CTL_CAPSRC_Msk             (0x1ul << TIMER_CTL_CAPSRC_Pos)                   /*!< TIMER_T::CTL: CAPSRC Mask              */

#define TIMER_CTL_TRGSSEL_Pos            (18)                                              /*!< TIMER_T::CTL: TRGSSEL Position          */
#define TIMER_CTL_TRGSSEL_Msk            (0x1ul << TIMER_CTL_TRGSSEL_Pos)                  /*!< TIMER_T::CTL: TRGSSEL Mask              */

#define TIMER_CTL_TRGBPWM01_Pos          (19)                                              /*!< TIMER_T::CTL: TRGBPWM01 Position        */
#define TIMER_CTL_TRGBPWM01_Msk          (0x1ul << TIMER_CTL_TRGBPWM01_Pos)                /*!< TIMER_T::CTL: TRGBPWM01 Mask            */

#define TIMER_CTL_TRGDAC_Pos             (20)                                              /*!< TIMER_T::CTL: TRGDAC Position           */
#define TIMER_CTL_TRGDAC_Msk             (0x1ul << TIMER_CTL_TRGDAC_Pos)                   /*!< TIMER_T::CTL: TRGDAC Mask               */

#define TIMER_CTL_TRGADC_Pos             (21)                                              /*!< TIMER_T::CTL: TRGADC Position           */
#define TIMER_CTL_TRGADC_Msk             (0x1ul << TIMER_CTL_TRGADC_Pos)                   /*!< TIMER_T::CTL: TRGADC Mask               */

#define TIMER_CTL_TGLPINSEL_Pos          (22)                                              /*!< TIMER_T::CTL: TGLPINSEL Position       */
#define TIMER_CTL_TGLPINSEL_Msk          (0x1ul << TIMER_CTL_TGLPINSEL_Pos)                /*!< TIMER_T::CTL: TGLPINSEL Mask           */

#define TIMER_CTL_WKEN_Pos               (23)                                              /*!< TIMER_T::CTL: WKEN Position            */
#define TIMER_CTL_WKEN_Msk               (0x1ul << TIMER_CTL_WKEN_Pos)                     /*!< TIMER_T::CTL: WKEN Mask                */

#define TIMER_CTL_EXTCNTEN_Pos           (24)                                              /*!< TIMER_T::CTL: EXTCNTEN Position        */
#define TIMER_CTL_EXTCNTEN_Msk           (0x1ul << TIMER_CTL_EXTCNTEN_Pos)                 /*!< TIMER_T::CTL: EXTCNTEN Mask            */

#define TIMER_CTL_ACTSTS_Pos             (25)                                              /*!< TIMER_T::CTL: ACTSTS Position          */
#define TIMER_CTL_ACTSTS_Msk             (0x1ul << TIMER_CTL_ACTSTS_Pos)                   /*!< TIMER_T::CTL: ACTSTS Mask              */

#define TIMER_CTL_RSTCNT_Pos             (26)                                              /*!< TIMER_T::CTL: RSTCNT Position          */
#define TIMER_CTL_RSTCNT_Msk             (0x1ul << TIMER_CTL_RSTCNT_Pos)                   /*!< TIMER_T::CTL: RSTCNT Mask              */

#define TIMER_CTL_OPMODE_Pos             (27)                                              /*!< TIMER_T::CTL: OPMODE Position          */
#define TIMER_CTL_OPMODE_Msk             (0x3ul << TIMER_CTL_OPMODE_Pos)                   /*!< TIMER_T::CTL: OPMODE Mask              */

#define TIMER_CTL_INTEN_Pos              (29)                                              /*!< TIMER_T::CTL: INTEN Position           */
#define TIMER_CTL_INTEN_Msk              (0x1ul << TIMER_CTL_INTEN_Pos)                    /*!< TIMER_T::CTL: INTEN Mask               */

#define TIMER_CTL_CNTEN_Pos              (30)                                              /*!< TIMER_T::CTL: CNTEN Position           */
#define TIMER_CTL_CNTEN_Msk              (0x1ul << TIMER_CTL_CNTEN_Pos)                    /*!< TIMER_T::CTL: CNTEN Mask               */

#define TIMER_CTL_ICEDEBUG_Pos           (31)                                              /*!< TIMER_T::CTL: ICEDEBUG Position        */
#define TIMER_CTL_ICEDEBUG_Msk           (0x1ul << TIMER_CTL_ICEDEBUG_Pos)                 /*!< TIMER_T::CTL: ICEDEBUG Mask            */

#define TIMER_CMP_CMPDAT_Pos             (0)                                               /*!< TIMER_T::CMP: CMPDAT Position          */
#define TIMER_CMP_CMPDAT_Msk             (0xfffffful << TIMER_CMP_CMPDAT_Pos)              /*!< TIMER_T::CMP: CMPDAT Mask              */

#define TIMER_INTSTS_TIF_Pos             (0)                                               /*!< TIMER_T::INTSTS: TIF Position          */
#define TIMER_INTSTS_TIF_Msk             (0x1ul << TIMER_INTSTS_TIF_Pos)                   /*!< TIMER_T::INTSTS: TIF Mask              */

#define TIMER_INTSTS_TWKF_Pos            (1)                                               /*!< TIMER_T::INTSTS: TWKF Position         */
#define TIMER_INTSTS_TWKF_Msk            (0x1ul << TIMER_INTSTS_TWKF_Pos)                  /*!< TIMER_T::INTSTS: TWKF Mask             */

#define TIMER_CNT_CNT_Pos                (0)                                               /*!< TIMER_T::CNT: CNT Position             */
#define TIMER_CNT_CNT_Msk                (0xfffffful << TIMER_CNT_CNT_Pos)                 /*!< TIMER_T::CNT: CNT Mask                 */

#define TIMER_CAP_CAPDAT_Pos             (0)                                               /*!< TIMER_T::CAP: CAPDAT Position          */
#define TIMER_CAP_CAPDAT_Msk             (0xfffffful << TIMER_CAP_CAPDAT_Pos)              /*!< TIMER_T::CAP: CAPDAT Mask              */

#define TIMER_EXTCTL_CNTPHASE_Pos        (0)                                               /*!< TIMER_T::EXTCTL: CNTPHASE Position     */
#define TIMER_EXTCTL_CNTPHASE_Msk        (0x1ul << TIMER_EXTCTL_CNTPHASE_Pos)              /*!< TIMER_T::EXTCTL: CNTPHASE Mask         */

#define TIMER_EXTCTL_CAPEDGE_Pos         (1)                                               /*!< TIMER_T::EXTCTL: CAPEDGE Position      */
#define TIMER_EXTCTL_CAPEDGE_Msk         (0x3ul << TIMER_EXTCTL_CAPEDGE_Pos)               /*!< TIMER_T::EXTCTL: CAPEDGE Mask          */

#define TIMER_EXTCTL_CAPEN_Pos           (3)                                               /*!< TIMER_T::EXTCTL: CAPEN Position        */
#define TIMER_EXTCTL_CAPEN_Msk           (0x1ul << TIMER_EXTCTL_CAPEN_Pos)                 /*!< TIMER_T::EXTCTL: CAPEN Mask            */

#define TIMER_EXTCTL_CAPFUNCS_Pos        (4)                                               /*!< TIMER_T::EXTCTL: CAPFUNCS Position     */
#define TIMER_EXTCTL_CAPFUNCS_Msk        (0x1ul << TIMER_EXTCTL_CAPFUNCS_Pos)              /*!< TIMER_T::EXTCTL: CAPFUNCS Mask         */

#define TIMER_EXTCTL_CAPIEN_Pos          (5)                                               /*!< TIMER_T::EXTCTL: CAPIEN Position       */
#define TIMER_EXTCTL_CAPIEN_Msk          (0x1ul << TIMER_EXTCTL_CAPIEN_Pos)                /*!< TIMER_T::EXTCTL: CAPIEN Mask           */

#define TIMER_EXTCTL_CAPDBEN_Pos         (6)                                               /*!< TIMER_T::EXTCTL: CAPDBEN Position      */
#define TIMER_EXTCTL_CAPDBEN_Msk         (0x1ul << TIMER_EXTCTL_CAPDBEN_Pos)               /*!< TIMER_T::EXTCTL: CAPDBEN Mask          */

#define TIMER_EXTCTL_CNTDBEN_Pos         (7)                                               /*!< TIMER_T::EXTCTL: CNTDBEN Position      */
#define TIMER_EXTCTL_CNTDBEN_Msk         (0x1ul << TIMER_EXTCTL_CNTDBEN_Pos)               /*!< TIMER_T::EXTCTL: CNTDBEN Mask          */

#define TIMER_EXTCTL_ICAPSEL_Pos         (8)                                               /*!< TIMER_T::EXTCTL: ICAPSEL Position      */
#define TIMER_EXTCTL_ICAPSEL_Msk         (0x7ul << TIMER_EXTCTL_ICAPSEL_Pos)               /*!< TIMER_T::EXTCTL: ICAPSEL Mask          */

#define TIMER_EINTSTS_CAPIF_Pos          (0)                                               /*!< TIMER_T::EINTSTS: CAPIF Position       */
#define TIMER_EINTSTS_CAPIF_Msk          (0x1ul << TIMER_EINTSTS_CAPIF_Pos)                /*!< TIMER_T::EINTSTS: CAPIF Mask           */

/**@}*/ /* TIMER_CONST */
/**@}*/ /* end of TIMER register group */




/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
    @{ 
*/

typedef struct
{


    /**
     * @var WDT_T::CTL
     * Offset: 0x00  WDT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RSTEN     |WDT Time-out Reset Enable Bit (Write Protect)
     * |        |          |Setting this bit will enable the WDT time-out reset system function if the WDT up counter value has not been cleared after the specific WDT reset delay period expires.
     * |        |          |0 = WDT time-out reset system function Disabled.
     * |        |          |1 = WDT time-out reset system function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |RSTF      |WDT Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WDT time-out reset system event or not.
     * |        |          |0 = WDT time-out reset system event did not occur.
     * |        |          |1 = WDT time-out reset system event has been occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |IF        |WDT Time-out Interrupt Flag
     * |        |          |This bit will set to 1 while WDT up counter value reaches the selected WDT time-out interval.
     * |        |          |0 = WDT time-out interrupt did not occur.
     * |        |          |1 = WDT time-out interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |WKEN      |WDT Time-out Wake-up Function Control (Write Protect)
     * |        |          |If this bit is set to 1, while WDT time-out interrupt flag IF (WDT_CTL[3]) is generated and interrupt enable bit INTEN (WDT_CTL[6]) is enabled, the WDT time-out interrupt signal will generate a event to trigger CPU wake-up trigger event to chip.
     * |        |          |0 = Trigger wake-up event function Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Trigger wake-up event function Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: Chip can be woken-up by while WDT time-out interrupt signal generated only if WDT clock source is selected to LIRC (10 kHz) or LXT (32 kHz).
     * |[5]     |WKF       |WDT Time-out Wake-up Flag (Write Protect)
     * |        |          |This bit indicates the WDT time-out event has triggered chip wake-up or not.
     * |        |          |0 = WDT does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode when WDT time-out interrupt signal is generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[6]     |INTEN     |WDT Time-out Interrupt Enable Bit (Write Protect)
     * |        |          |If this bit is enabled, when WDT time-out event occurs, the IF (WDT_CTL[3]) will be set to 1 and WDT time-out interrupt signal is generated and inform to CPU.
     * |        |          |0 = WDT time-out interrupt Disabled.
     * |        |          |1 = WDT time-out interrupt Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |WDTEN     |WDT Enable Bit (Write Protect)
     * |        |          |0 = Set WDT counter stop, and internal up counter value will be reset also.
     * |        |          |1 = Set WDT counter start.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: Perform enable or disable WDTEN bit needs 2 * WDT_CLK period to become active, user can read SYNC (WDT_CTL[30]) to check enabe/disable command is completed or not.
     * |        |          |Note3: If CWDTEN[2:0] (combined by with Config0[31] and Config0[4:3]) bits is not configure to 0x111, this bit is forced as 1 and user cannot change this bit to 0.
     * |[10:8]  |TOUTSEL   |WDT Time-out Interval Selection (Write Protect)
     * |        |          |These three bits select the time-out interval period after WDT starts counting.
     * |        |          |000 = 2^4 * WDT_CLK.
     * |        |          |001 = 2^6 * WDT_CLK.
     * |        |          |010 = 2^8 * WDT_CLK.
     * |        |          |011 = 2^10 * WDT_CLK.
     * |        |          |100 = 2^12 * WDT_CLK.
     * |        |          |101 = 2^14 * WDT_CLK.
     * |        |          |110 = 2^16 * WDT_CLK.
     * |        |          |111 = 2^18 * WDT_CLK.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[30]    |SYNC      |WDT Enable Control SYNC Flag Indicator (Read Only)
     * |        |          |If user execute enable/disable WDTEN (WDT_CTL[7]), this flag can be indicated enable/disable WDTEN function is completed or not.
     * |        |          |0 = Set WDTEN bit is completed.
     * |        |          |1 = Set WDTEN bit is synchronizing and not become active yet.
     * |        |          |Note: Perform enable or disable WDTEN bit needs 2 * WDT_CLK period to become active.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement affects WDT counting.
     * |        |          |WDT up counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var WDT_T::ALTCTL
     * Offset: 0x04  WDT Alternative Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |RSTDSEL   |WDT Reset Delay Period Selection (Write Protect)
     * |        |          |When WDT time-out event happened, user has a time named WDT Reset Delay Period to clear execute WDT counter by setting RSTCNT (WDT_CTL[0]) reset to prevent WDT time-out reset system occurredhappened
     * |        |          |User can select a suitable setting of RSTDSEL for different application programWDT Reset Delay Period.
     * |        |          |00 = WDT Reset Delay Period is 1026 * WDT_CLK.
     * |        |          |01 = WDT Reset Delay Period is 130 * WDT_CLK.
     * |        |          |10 = WDT Reset Delay Period is 18 * WDT_CLK.
     * |        |          |11 = WDT Reset Delay Period is 3 * WDT_CLK.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This register will be reset to 0 if WDT time-out reset system event occurred.
     * @var WDT_T::RSTCNT
     * Offset: 0x08  WDT Reset Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RSTCNT    |WDT Reset Counter Register
     * |        |          |Writing 0x00005AA5 to this field will reset the internal 18-bit WDT up counter value to 0.
     * |        |          |Note: Perform RSTCNT to reset counter needs 2 * WDT_CLK period to become active.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] WDT Control Register                                             */
    __IO uint32_t ALTCTL;                /*!< [0x0004] WDT Alternative Control Register                                 */
    __O  uint32_t RSTCNT;                /*!< [0x0008] WDT Reset Counter Register                                       */

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
    @{ 
*/

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT_T::CTL: RSTEN Position             */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT_T::CTL: RSTEN Mask                 */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT_T::CTL: RSTF Position              */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT_T::CTL: RSTF Mask                  */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT_T::CTL: IF Position                */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT_T::CTL: IF Mask                    */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT_T::CTL: WKEN Position              */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT_T::CTL: WKEN Mask                  */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT_T::CTL: WKF Position               */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT_T::CTL: WKF Mask                   */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position             */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                 */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position             */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                 */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position           */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask               */

#define WDT_CTL_SYNC_Pos                 (30)                                              /*!< WDT_T::CTL: SYNC Position              */
#define WDT_CTL_SYNC_Msk                 (0x1ul << WDT_CTL_SYNC_Pos)                       /*!< WDT_T::CTL: SYNC Mask                  */

#define WDT_CTL_ICEDEBUG_Pos             (31)                                              /*!< WDT_T::CTL: ICEDEBUG Position          */
#define WDT_CTL_ICEDEBUG_Msk             (0x1ul << WDT_CTL_ICEDEBUG_Pos)                   /*!< WDT_T::CTL: ICEDEBUG Mask              */

#define WDT_ALTCTL_RSTDSEL_Pos           (0)                                               /*!< WDT_T::ALTCTL: RSTDSEL Position        */
#define WDT_ALTCTL_RSTDSEL_Msk           (0x3ul << WDT_ALTCTL_RSTDSEL_Pos)                 /*!< WDT_T::ALTCTL: RSTDSEL Mask            */

#define WDT_RSTCNT_RSTCNT_Pos            (0)                                               /*!< WDT_T::RSTCNT: RSTCNT Position         */
#define WDT_RSTCNT_RSTCNT_Msk            (0xfffffffful << WDT_RSTCNT_RSTCNT_Pos)           /*!< WDT_T::RSTCNT: RSTCNT Mask             */


/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


/*---------------------- Window Watchdog Timer -------------------------*/
/**
    @addtogroup WWDT Window Watchdog Timer(WWDT)
    Memory Mapped Structure for WWDT Controller
    @{ 
*/

typedef struct
{


    /**
     * @var WWDT_T::RLDCNT
     * Offset: 0x00  WWDT Reload Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RLDCNT    |WWDT Reload Counter Register
     * |        |          |Writing only 0x00005AA5 to this register will reload the WWDT counter value to 0x3F.
     * |        |          |Note1: User can only execute the reload WWDT counter value command when current CNTDAT (WWDT_CNT[5:0]) is between 1 and CMPDAT (WWDT_CTL[21:16]).
     * |        |          |If user writes 0x00005AA5 in WWDT_RLDCNT register when current CNTDAT is larger than CMPDAT, WWDT reset signal system event will be generated immediately.
     * |        |          |Note2: Execute WWDT counter relaod always needs (WWDT_CLK *3) period to reload CNTDAT to 0x3F and intrenal prescale counter will be reset also.
     * @var WWDT_T::CTL
     * Offset: 0x04  WWDT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTEN    |WWDT Enable Bit
     * |        |          |Set this bit to start WWDT counter counting.
     * |        |          |0 = WWDT counter is stopped.
     * |        |          |1 = WWDT counter is starting counting.
     * |[1]     |INTEN     |WWDT Interrupt Enable Bit
     * |        |          |If this bit is enabled, when WWDTIF (WWDT_STATUS[0]) is set to 1, the WWDT counter compare match interrupt signal is generated and inform to CPU.
     * |        |          |0 = WWDT counter compare match interrupt Disabled.
     * |        |          |1 = WWDT counter compare match interrupt Enabled.
     * |[11:8]  |PSCSEL    |WWDT Counter Prescale Period Selection
     * |        |          |0000 = Pre-scale is 1; Max time-out period is 1 * 64 * WWDT_CLK.
     * |        |          |0001 = Pre-scale is 2; Max time-out period is 2 * 64 * WWDT_CLK.
     * |        |          |0010 = Pre-scale is 4; Max time-out period is 4 * 64 * WWDT_CLK.
     * |        |          |0011 = Pre-scale is 8; Max time-out period is 8 * 64 * WWDT_CLK.
     * |        |          |0100 = Pre-scale is 16; Max time-out period is 16 * 64 * WWDT_CLK.
     * |        |          |0101 = Pre-scale is 32; Max time-out period is 32 * 64 * WWDT_CLK.
     * |        |          |0110 = Pre-scale is 64; Max time-out period is 64 * 64 * WWDT_CLK.
     * |        |          |0111 = Pre-scale is 128; Max time-out period is 128 * 64 * WWDT_CLK.
     * |        |          |1000 = Pre-scale is 192; Max time-out period is 192 * 64 * WWDT_CLK.
     * |        |          |1001 = Pre-scale is 256; Max time-out period is 256 * 64 * WWDT_CLK.
     * |        |          |1010 = Pre-scale is 384; Max time-out period is 384 * 64 * WWDT_CLK.
     * |        |          |1011 = Pre-scale is 512; Max time-out period is 512 * 64 * WWDT_CLK.
     * |        |          |1100 = Pre-scale is 768; Max time-out period is 768 * 64 * WWDT_CLK.
     * |        |          |1101 = Pre-scale is 1024; Max time-out period is 1024 * 64 * WWDT_CLK.
     * |        |          |1110 = Pre-scale is 1536; Max time-out period is 1536 * 64 * WWDT_CLK.
     * |        |          |1111 = Pre-scale is 2048; Max time-out period is 2048 * 64 * WWDT_CLK.
     * |[21:16] |CMPDAT    |WWDT Window Compare Value
     * |        |          |Set this field to adjust the valid reload window interval when WWDTIF (WWDT_STATUS[0]) is generated..
     * |        |          |Note: User can only write WWDT_RLDCNT register to reload WWDT counter value when current WWDT CNTDAT (WWDT_CNT[5:0]) is between 1 and CMPDAT.
     * |        |          |If user writes 0x00005AA5 in WWDT_RLDCNT register when current CNTDAT is larger than CMPDAT, WWDT reset system event will be generated immediately.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit
     * |        |          |0 = ICE debug mode acknowledgement effects WWDT counter counting.
     * |        |          |The WWDT down counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |The WWDT down counter will keep going counting no matter CPU is held by ICE or not.
     * @var WWDT_T::STATUS
     * Offset: 0x08  WWDT Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTIF    |WWDT Compare Match Interrupt Flag
     * |        |          |This bit indicates that current CNTDAT (WWDT_CNT[5:0]) matches the CMPDAT (WWDT_CTL[21:16]).
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT CNTDAT matches the CMPDAT.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |WWDTRF    |WWDT Timer-out Reset System Flag
     * |        |          |If this bit is set to 1, it indicates that system has been reset by WWDT counter time-out reset system event.
     * |        |          |0 = WWDT time-out reset system event did not occur.
     * |        |          |1 = WWDT time-out reset system event occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var WWDT_T::CNT
     * Offset: 0x0C  WWDT Counter Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CNTDAT    |WWDT Counter Value
     * |        |          |CNTDAT will be updated continuously.
     */
    __O  uint32_t RLDCNT;                /*!< [0x0000] WWDT Reload Counter Register                                     */
    __IO uint32_t CTL;                   /*!< [0x0004] WWDT Control Register                                            */
    __IO uint32_t STATUS;                /*!< [0x0008] WWDT Status Register                                             */
    __I  uint32_t CNT;                   /*!< [0x000c] WWDT Counter Value Register                                      */

} WWDT_T;

/**
    @addtogroup WWDT_CONST WWDT Bit Field Definition
    Constant Definitions for WWDT Controller
    @{ 
*/

#define WWDT_RLDCNT_RLDCNT_Pos           (0)                                               /*!< WWDT_T::RLDCNT: RLDCNT Position        */
#define WWDT_RLDCNT_RLDCNT_Msk           (0xfffffffful << WWDT_RLDCNT_RLDCNT_Pos)          /*!< WWDT_T::RLDCNT: RLDCNT Mask            */

#define WWDT_CTL_WWDTEN_Pos              (0)                                               /*!< WWDT_T::CTL: WWDTEN Position           */
#define WWDT_CTL_WWDTEN_Msk              (0x1ul << WWDT_CTL_WWDTEN_Pos)                    /*!< WWDT_T::CTL: WWDTEN Mask               */

#define WWDT_CTL_INTEN_Pos               (1)                                               /*!< WWDT_T::CTL: INTEN Position            */
#define WWDT_CTL_INTEN_Msk               (0x1ul << WWDT_CTL_INTEN_Pos)                     /*!< WWDT_T::CTL: INTEN Mask                */

#define WWDT_CTL_PSCSEL_Pos              (8)                                               /*!< WWDT_T::CTL: PSCSEL Position           */
#define WWDT_CTL_PSCSEL_Msk              (0xful << WWDT_CTL_PSCSEL_Pos)                    /*!< WWDT_T::CTL: PSCSEL Mask               */

#define WWDT_CTL_CMPDAT_Pos              (16)                                              /*!< WWDT_T::CTL: CMPDAT Position           */
#define WWDT_CTL_CMPDAT_Msk              (0x3ful << WWDT_CTL_CMPDAT_Pos)                   /*!< WWDT_T::CTL: CMPDAT Mask               */

#define WWDT_CTL_ICEDEBUG_Pos            (31)                                              /*!< WWDT_T::CTL: ICEDEBUG Position         */
#define WWDT_CTL_ICEDEBUG_Msk            (0x1ul << WWDT_CTL_ICEDEBUG_Pos)                  /*!< WWDT_T::CTL: ICEDEBUG Mask             */

#define WWDT_STATUS_WWDTIF_Pos           (0)                                               /*!< WWDT_T::STATUS: WWDTIF Position        */
#define WWDT_STATUS_WWDTIF_Msk           (0x1ul << WWDT_STATUS_WWDTIF_Pos)                 /*!< WWDT_T::STATUS: WWDTIF Mask            */

#define WWDT_STATUS_WWDTRF_Pos           (1)                                               /*!< WWDT_T::STATUS: WWDTRF Position        */
#define WWDT_STATUS_WWDTRF_Msk           (0x1ul << WWDT_STATUS_WWDTRF_Pos)                 /*!< WWDT_T::STATUS: WWDTRF Mask            */

#define WWDT_CNT_CNTDAT_Pos              (0)                                               /*!< WWDT_T::CNT: CNTDAT Position           */
#define WWDT_CNT_CNTDAT_Msk              (0x3ful << WWDT_CNT_CNTDAT_Pos)                   /*!< WWDT_T::CNT: CNTDAT Mask               */

/**@}*/ /* WWDT_CONST */
/**@}*/ /* end of WWDT register group */

/**@}*/ /* end of REGISTER group */

#include "fmc_reg.h"
#include "i2c_reg.h"
#include "acmp_reg.h"
#include "clk_reg.h"
#include "gpio_reg.h"
#include "sys_reg.h"
#include "uart_reg.h"
#include "llsi_reg.h"
#include "spi_reg.h"
#include "usbd_reg.h"
#include "adc_reg.h"
#include "bpwm_reg.h"
#include "dac_reg.h"
#include "pdma_reg.h"


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup PERIPHERAL_BASE Peripheral Memory Base
  Memory Mapped Structure for Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)
#define APB2_BASE           ((     uint32_t)0x40100000)

/* Peripheral memory map */
#define GPIO_BASE           (AHB_BASE       + 0x4000)                   /*!< GPIO Base Address                                   */
#define PA_BASE             (GPIO_BASE              )                   /*!< GPIO PA Base Address                                */
#define PB_BASE             (GPIO_BASE      + 0x0040)                   /*!< GPIO PB Base Address                                */
#define PC_BASE             (GPIO_BASE      + 0x0080)                   /*!< GPIO PC Base Address                                */
#define PD_BASE             (GPIO_BASE      + 0x00C0)                   /*!< GPIO PD Base Address                                */
#define PF_BASE             (GPIO_BASE      + 0x0140)                   /*!< GPIO PF Base Address                                */
#define GPIO_DBCTL_BASE     (GPIO_BASE      + 0x0180)                   /*!< GPIO De-bounce Cycle Control Base Address           */
#define GPIO_PIN_DATA_BASE  (GPIO_BASE      + 0x0200)                   /*!< GPIO Pin Data Input/Output Control Base Address     */

#define UART0_BASE          (APB1_BASE      + 0x50000)                  /*!< UART0 Base Address                               */
#define UART1_BASE          (APB2_BASE      + 0x50000)                  /*!< UART1 Base Address                               */
#define UART2_BASE          (APB1_BASE      + 0x80000)                  /*!< UART2 Base Address                               */

#define TIMER0_BASE         (APB1_BASE      + 0x10000)                  /*!< Timer0 Base Address                              */
#define TIMER1_BASE         (APB1_BASE      + 0x10020)                  /*!< Timer1 Base Address                              */
#define TIMER2_BASE         (APB2_BASE      + 0x10000)                  /*!< Timer2 Base Address                              */
#define TIMER3_BASE         (APB2_BASE      + 0x10020)                  /*!< Timer3 Base Address                              */

#define WDT_BASE            (APB1_BASE      + 0x4000)                   /*!< Watch Dog Timer Base Address                     */

#define WWDT_BASE           (APB1_BASE      + 0x4100)                   /*!< Window Watch Dog Timer Base Address              */

#define SPI0_BASE           (APB1_BASE      + 0x30000)                  /*!< SPI0 Base Address                                */
#define SPI1_BASE           (APB1_BASE      + 0x34000)                  /*!< SPI1 Base Address                                */
#define SPI2_BASE           (APB2_BASE      + 0x30000)                  /*!< SPI2 Base Address                                */

#define I2C0_BASE           (APB1_BASE      + 0x20000)                  /*!< I2C0 Base Address                                */
#define I2C1_BASE           (APB2_BASE      + 0x20000)                  /*!< I2C1 Base Address                                */
#define I2C2_BASE           (APB1_BASE      + 0x24000)                  /*!< I2C2 Base Address                                */

#define ADC_BASE            (APB1_BASE      + 0xE0000)                  /*!< ADC Base Address                                 */

#define CLK_BASE            (AHB_BASE       + 0x00200)                  /*!< System Clock Controller Base Address             */

#define SYS_BASE            (AHB_BASE       + 0x00000)                  /*!< System Global Controller Base Address            */

#define INT_BASE            (AHB_BASE       + 0x00300)                  /*!< Interrupt Source Controller Base Address         */

#define FMC_BASE            (AHB_BASE       + 0x0C000)                  /*!< Flash Memory Controller Base Address             */

#define BPWM0_BASE          (APB1_BASE      + 0x40000)                  /*!< BPWM0 Base Address                               */
#define BPWM1_BASE          (APB2_BASE      + 0x40000)                  /*!< BPWM1 Base Address                               */
#define BPWM2_BASE          (APB1_BASE      + 0x44000)                  /*!< BPWM2 Base Address                               */
#define BPWM3_BASE          (APB2_BASE      + 0x44000)                  /*!< BPWM3 Base Address                               */

#define CRC_BASE            (AHB_BASE       + 0x18000)                  /*!< CRC Base Address                                 */

#define USBD_BASE           (APB1_BASE      + 0x60000)                  /*!< USB Device Base Address                          */

#define PDMA_BASE           (AHB_BASE       + 0x08000)                  /*!< PDMA Base Address                                */

#define LLSI0_BASE          (APB1_BASE      + 0x54000)                  /*!< LLSI0 Base Address                               */
#define LLSI1_BASE          (APB2_BASE      + 0x54000)                  /*!< LLSI1 Base Address                               */
#define LLSI2_BASE          (APB1_BASE      + 0x54200)                  /*!< LLSI2 Base Address                               */
#define LLSI3_BASE          (APB2_BASE      + 0x54200)                  /*!< LLSI3 Base Address                               */
#define LLSI4_BASE          (APB1_BASE      + 0x54400)                  /*!< LLSI4 Base Address                               */
#define LLSI5_BASE          (APB2_BASE      + 0x54400)                  /*!< LLSI5 Base Address                               */

#define ACMP01_BASE         (APB1_BASE      + 0xD0000)                  /*!< ACMP01 Base Address                              */
#define ACMP23_BASE         (APB2_BASE      + 0xD0000)                  /*!< ACMP23 Base Address                              */


#define DAC0_BASE            (APB1_BASE     + 0xF0000UL)                 /*!< DAC0 Base Address                                */
#define DAC1_BASE            (APB1_BASE     + 0xF0040UL)                 /*!< DAC1 Base Address                                */
#define DAC2_BASE            (APB1_BASE     + 0xF0080UL)                 /*!< DAC2 Base Address                                */
#define DAC3_BASE            (APB1_BASE     + 0xF00C0UL)                 /*!< DAC3 Base Address                                */

/**@}*/ /* PERIPHERAL_BASE */

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/** @addtogroup PMODULE Peripheral Pointer
  The Declaration of Peripheral Pointer
  @{
 */
#define PA                  ((GPIO_T *) PA_BASE)                        /*!< GPIO PORTA Configuration Struct                        */
#define PB                  ((GPIO_T *) PB_BASE)                        /*!< GPIO PORTB Configuration Struct                        */
#define PC                  ((GPIO_T *) PC_BASE)                        /*!< GPIO PORTC Configuration Struct                        */
#define PD                  ((GPIO_T *) PD_BASE)                        /*!< GPIO PORTD Configuration Struct                        */
#define PF                  ((GPIO_T *) PF_BASE)                        /*!< GPIO PORTF Configuration Struct                        */
#define GPIO                ((GPIO_DBCTL_T *) GPIO_DBCTL_BASE)          /*!< Interrupt De-bounce Cycle Control Configuration Struct */

#define UART0               ((UART_T *) UART0_BASE)                     /*!< UART0 Configuration Struct                       */
#define UART1               ((UART_T *) UART1_BASE)                     /*!< UART1 Configuration Struct                       */
#define UART2               ((UART_T *) UART2_BASE)                     /*!< UART2 Configuration Struct                       */

#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< TIMER0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< TIMER1 Configuration Struct                      */
#define TIMER2              ((TIMER_T *) TIMER2_BASE)                   /*!< TIMER2 Configuration Struct                      */
#define TIMER3              ((TIMER_T *) TIMER3_BASE)                   /*!< TIMER3 Configuration Struct                      */

#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watch Dog Timer Configuration Struct             */

#define WWDT                ((WWDT_T *) WWDT_BASE)                      /*!< Window Watch Dog Timer Configuration Struct      */

#define SPI0                ((SPI_T *) SPI0_BASE)                       /*!< SPI0 Configuration Struct                        */
#define SPI1                ((SPI_T *) SPI1_BASE)                       /*!< SPI1 Configuration Struct                        */
#define SPI2                ((SPI_T *) SPI2_BASE)                       /*!< SPI2 Configuration Struct                        */

#define I2C0                ((I2C_T *) I2C0_BASE)                       /*!< I2C0 Configuration Struct                        */
#define I2C1                ((I2C_T *) I2C1_BASE)                       /*!< I2C1 Configuration Struct                        */
#define I2C2                ((I2C_T *) I2C2_BASE)                       /*!< I2C2 Configuration Struct                        */

#define ADC                 ((ADC_T *) ADC_BASE)                        /*!< ADC Configuration Struct                         */

#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */

#define SYS                 ((SYS_T *) SYS_BASE)                        /*!< System Global Controller Configuration Struct    */

#define SYSINT              ((SYS_INT_T *) INT_BASE)                    /*!< Interrupt Source Controller Configuration Struct */

#define FMC                 ((FMC_T *) FMC_BASE)                        /*!< Flash Memory Controller */

#define BPWM0               ((BPWM_T *) BPWM0_BASE)                     /*!< BPWM0 Configuration Struct                        */
#define BPWM1               ((BPWM_T *) BPWM1_BASE)                     /*!< BPWM1 Configuration Struct                        */
#define BPWM2               ((BPWM_T *) BPWM2_BASE)                     /*!< BPWM2 Configuration Struct                        */
#define BPWM3               ((BPWM_T *) BPWM3_BASE)                     /*!< BPWM3 Configuration Struct                        */
                                                                     
#define CRC                 ((CRC_T *) CRC_BASE)                        /*!< CRC Configuration Struct                          */
                                                                                                                        
#define USBD                ((USBD_T *) USBD_BASE)                      /*!< USB Device Configuration Struct                   */
                                                                                                                        
#define PDMA                ((PDMA_T *) PDMA_BASE)                      /*!< PDMA Configuration Struct                         */
                                                                                                                        
#define LLSI0               ((LLSI_T *) LLSI0_BASE)                     /*!< LLSI0 Configuration Struct                        */
#define LLSI1               ((LLSI_T *) LLSI1_BASE)                     /*!< LLSI1 Configuration Struct                        */
#define LLSI2               ((LLSI_T *) LLSI2_BASE)                     /*!< LLSI2 Configuration Struct                        */
#define LLSI3               ((LLSI_T *) LLSI3_BASE)                     /*!< LLSI3 Configuration Struct                        */
#define LLSI4               ((LLSI_T *) LLSI4_BASE)                     /*!< LLSI4 Configuration Struct                        */
#define LLSI5               ((LLSI_T *) LLSI5_BASE)                     /*!< LLSI5 Configuration Struct                        */
                                                                                                                        
#define ACMP01              ((ACMP_T *) ACMP01_BASE)                    /*!< ACMP01 Configuration Struct                       */
#define ACMP23              ((ACMP_T *) ACMP23_BASE)                    /*!< ACMP23 Configuration Struct                       */
                                                                     
#define DAC0                ((DAC_T *) DAC0_BASE)                       /*!< DAC0 Configuration Struct                         */
#define DAC1                ((DAC_T *) DAC1_BASE)                       /*!< DAC1 Configuration Struct                         */
#define DAC2                ((DAC_T *) DAC2_BASE)                       /*!< DAC2 Configuration Struct                         */
#define DAC3                ((DAC_T *) DAC3_BASE)                       /*!< DAC3 Configuration Struct                         */
/**@}*/ /* end of group PMODULE */


//=============================================================================
typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

#define outpw(port,value)   (*((volatile unsigned int *)(port))=(value))
#define inpw(port)          ((*((volatile unsigned int *)(port))))
#define outpb(port,value)   (*((volatile unsigned char *)(port))=(value))
#define inpb(port)          ((*((volatile unsigned char *)(port))))
#define outps(port,value)   (*((volatile unsigned short *)(port))=(value))
#define inps(port)          ((*((volatile unsigned short *)(port))))

#define outp32(port,value)  (*((volatile unsigned int *)(port))=(value))
#define inp32(port)         ((*((volatile unsigned int *)(port))))
#define outp8(port,value)   (*((volatile unsigned char *)(port))=(value))
#define inp8(port)          ((*((volatile unsigned char *)(port))))
#define outp16(port,value)  (*((volatile unsigned short *)(port))=(value))
#define inp16(port)         ((*((volatile unsigned short *)(port))))


#define E_SUCCESS   0
#ifndef NULL
#define NULL        0
#endif

#define TRUE        1
#define FALSE       0

#define ENABLE      1
#define DISABLE     0

/* Bit Mask Definitions */
#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000


/* Byte Mask Definitions */
#define BYTE0_Msk               (0x000000FF)
#define BYTE1_Msk               (0x0000FF00)
#define BYTE2_Msk               (0x00FF0000)
#define BYTE3_Msk               (0xFF000000)

#define _GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "adc.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "bpwm.h"
#include "spi.h"
#include "timer.h"
#include "wdt.h"
#include "wwdt.h"
#include "uart.h"
#include "crc.h"
#include "usbd.h"
#include "pdma.h"
#include "llsi.h"
#include "dac.h"
#include "acmp.h"
#endif

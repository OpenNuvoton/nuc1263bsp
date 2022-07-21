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


/*---------------------- Analog to Digital Converter -------------------------*/
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
    @{ 
*/

typedef struct
{


    /**
     * @var ADC_T::ADDR
     * Offset: 0x00~0x3C, 0x74~0x78  ADC Data Register 0~15, 29~30
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1.
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwrote.
     * |        |          |1 = Data in RSLT bits is overwrote..
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed.
     * |        |          |This bit will be cleared to 0 by hardware after ADC_ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var ADC_T::ADCR
     * Offset: 0x80  ADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADEN      |A/D Converter Enable
     * |        |          |0 = A/D converter Disabled.
     * |        |          |1 = A/D converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADIE      |A/D Interrupt Enable Control
     * |        |          |A/D conversion end interrupt request is generated if ADIE bit is set to 1.
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |[3:2]   |ADMD      |A/D Converter Operation Mode Control
     * |        |          |00 = Single conversion.
     * |        |          |01 = Burst conversion.
     * |        |          |10 = Single-cycle Scan.
     * |        |          |11 = Continuous Scan.
     * |        |          |Note1: When changing the operation mode, software should clear ADST bit first.
     * |        |          |Note2: In Burst mode, the A/D result data is always at ADC Data Register 0.
     * |[5:4]   |TRGS      |Hardware Trigger Source
     * |        |          |00 = A/D conversion is started by external STADC pin.
     * |        |          |01 = Timer0 ~ Timer3 overflow pulse trigger.
     * |        |          |10 = Reserved.
     * |        |          |11 = A/D conversion is started by BPWM trigger.
     * |        |          |Note: Software should clear TRGEN bit and ADST bit to 0 before changing TRGS bits.
     * |[7:6]   |TRGCOND   |External Trigger Condition
     * |        |          |These two bits decide external pin STADC trigger event is level or edge.
     * |        |          |The signal must be kept at stable state at least 8 PCLKs for level trigger and at least 4 PCLKs for edge trigger.
     * |        |          |00 = Low level.
     * |        |          |01 = High level.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Rising edge.
     * |[8]     |TRGEN     |External Trigger Enable Control
     * |        |          |Enable or disable triggering of A/D conversion by external STADC pin, PWM trigger and Timer trigger.
     * |        |          |If external trigger is enabled, the ADST bit can be set to 1 by the selected hardware trigger source.
     * |        |          |0 = External trigger Disabled.
     * |        |          |1 = External trigger Enabled.
     * |        |          |Note: The ADC external trigger function is only supported in Single-cycle Scan mode.
     * |[9]     |PTEN      |PDMA Transfer Enable Bit
     * |        |          |When A/D conversion is completed, the converted data is loaded into ADDR0~15, ADDR29~ADDR30.
     * |        |          |Software can enable this bit to generate a PDMA data transfer request.
     * |        |          |0 = PDMA data transfer Disabled.
     * |        |          |1 = PDMA data transfer in ADDR0~15, ADDR29~ADDR30 Enabled.
     * |        |          |Note: When PTEN=1, software must set ADIE=0 to disable interrupt.
     * |[10]    |DIFFEN    |Differential Input Mode Control
     * |        |          |0 = Single-end analog input mode.
     * |        |          |1 = Differential analog input mode.
     * |        |          |Differential input voltage (Vdiff) = Vplus - Vminus,
     * |        |          |where Vplus is the analog input; Vminus is the inverted analog input.
     * |        |          |The Vplus of differential input paired channel x is from ADC0_CHy pin; Vminus is from ADC0_CHz pin, x=0,1..3, y=2*x, z=y+1.
     * |        |          |0 = Single-end analog input mode.
     * |        |          |1 = Differential analog input mode.
     * |        |          |Note: In Differential Input mode, only the even number of the two corresponding channels needs to be enabled in ADCHER register
     * |        |          |The conversion result will be placed to the corresponding data register of the enabled channel.
     * |[11]    |ADST      |A/D Conversion Start
     * |        |          |ADST bit can be set to 1 from four sources: software, external pin STADC, PWM trigger and Timer trigger.
     * |        |          |ADST bit will be cleared to 0 by hardware automatically at the ends of Single mode and Single-cycle Scan mode.
     * |        |          |In Continuous Scan mode and Burst mode, A/D conversion is continuously performed until software writes 0 to this bit or chip is reset.
     * |        |          |0 = Conversion stops and A/D converter enters idle state.
     * |        |          |1 = Conversion starts.
     * |[18:16] |SMPTSEL   |ADC Internal Sampling Time Selection
     * |        |          |Total ADC conversion cycle = sampling cycle + 12
     * |        |          |000 = 4 ADC clock for sampling; 16 ADC clock for complete conversion.
     * |        |          |001 = 5 ADC clock for sampling; 17 ADC clock for complete conversion.
     * |        |          |010 = 6 ADC clock for sampling; 18 ADC clock for complete conversion.
     * |        |          |011 = 7 ADC clock for sampling; 19 ADC clock for complete conversion.
     * |        |          |100 = 8 ADC clock for sampling; 20 ADC clock for complete conversion.
     * |        |          |101 = 9 ADC clock for sampling; 21 ADC clock for complete conversion.
     * |        |          |110 = 10 ADC clock for sampling; 22 ADC clock for complete conversion.
     * |        |          |111 = 11 ADC clock for sampling; 23 ADC clock for complete conversion.
     * |[31]    |DMOF      |Differential Input Mode Output Format
     * |        |          |If user enables differential input mode, the conversion result can be expressed with binary straight format (unsigned format) or 2's complement format (signed format).
     * |        |          |0 = A/D Conversion result will be filled in RSLT at ADC_ADDRx registers with unsigned format (straight binary format).
     * |        |          |1 = A/D Conversion result will be filled in RSLT at ADC_ADDRx registers with 2's complement format.
     * @var ADC_T::ADCHER
     * Offset: 0x84  ADC Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CHEN      |Analog Input Channel Enable Control
     * |        |          |Set ADCHENR[15:0] bits to enable the corresponding analog input channel 15 ~ 0.
     * |        |          |If DIFFEN(ADC_ADCR[10]) bit is set to 1, only the even number channel needs to be enabled.
     * |        |          |Besides, set ADCHENR[29] to ADCHENR[30] bits will enable internal channel for band-gap voltage and SPDHUB HSA input respectively.
     * |        |          |Other bits are reserved.
     * |        |          |0 = Channel Disabled.
     * |        |          |1 = Channel Enabled.
     * |        |          |Note : If the internal channel for band-gap voltage (CHEN[29]) is active, the maximum sampling rate will be 300k SPS.
     * @var ADC_T::ADCMPR
     * Offset: 0x88/0x8C  ADC Compare Register 0/1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMPEN     |Compare Enable Control
     * |        |          |Set this bit to 1 to enable ADC controller to compare CMPD with specified channel conversion result when converted data is loaded into ADC_ADDRx register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |CMPIE     |Compare Interrupt Enable Control
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPFx(ADC_ADSR0[x+1], x = 0,1)  bit will be asserted, in the meanwhile, if CMPIE bit is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD bits, the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater than or equal to the 12-bit CMPD bits, the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches to (CMPMATCNT +1), the CMPFx(ADC_ADSR0[x+1], x = 0,1) bit will be set.
     * |[7:3]   |CMPCH     |Compare Channel Selection
     * |        |          |00000 = Channel 0 conversion result is selected to be compared.
     * |        |          |00001 = Channel 1 conversion result is selected to be compared.
     * |        |          |00010 = Channel 2 conversion result is selected to be compared.
     * |        |          |00011 = Channel 3 conversion result is selected to be compared.
     * |        |          |00100 = Channel 4 conversion result is selected to be compared.
     * |        |          |00101 = Channel 5 conversion result is selected to be compared.
     * |        |          |00110 = Channel 6 conversion result is selected to be compared.
     * |        |          |00111 = Channel 7 conversion result is selected to be compared.
     * |        |          |11101 = Band-gap voltage conversion result is selected to be compared.
     * |        |          |11110 = Temperature sensor conversion result is selected to be compared.
     * |        |          |Others = Reserved.
     * |[11:8]  |CMPMATCNT |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND bit, the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMATCNT +1), the CMPFx(ADC_ADSR0[x+1], x = 0,1) bit will be set.
     * |[15]    |CMPWEN    |Compare Window Mode Enable Bit
     * |        |          |0 = Compare Window Mode Disabled.
     * |        |          |1 = Compare Window Mode Enabled.
     * |        |          |Note: This bit is only presented in ADC_ADCMPR0 register.
     * |[27:16] |CMPD      |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
     * |        |          |Note: CMPD bits should be filled in unsigned format (straight binary format).
     * @var ADC_T::ADSR0
     * Offset: 0x90  ADC Status Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADF       |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion. Software can write 1 to clear this bit.
     * |        |          |ADF bit is set to 1 at the following three conditions:
     * |        |          |1. When A/D conversion ends in Single mode.
     * |        |          |2. When A/D conversion ends on all specified channels in Single-cycle Scan mode and Continuous Scan mode.
     * |        |          |3. When more than or equal to 8 samples in FIFO in Burst mode.
     * |[1]     |CMPF0     |Compare Flag 0
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADC_ADCMPR0 register then this bit is set to 1.
     * |        |          |This bit is cleared by writing 1 to it.
     * |        |          |0 = Conversion result in ADDR does not meet ADC_ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets ADC_ADCMPR0 setting.
     * |[2]     |CMPF1     |Compare Flag 1
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADC_ADCMPR1 register then this bit is set to 1; it is cleared by writing 1 to it.
     * |        |          |0 = Conversion result in ADDR does not meet ADC_ADCMPR1 setting.
     * |        |          |1 = Conversion result in ADDR meets ADC_ADCMPR1 setting.
     * |[7]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is a mirror of ADST bit in ADC_ADCR register.
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[8]     |VALIDF    |Data Valid Flag (Read Only)
     * |        |          |If any one of VALID (ADC_ADDRx[17]) is set, this flag will be set to 1.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, this flag will be set to 1.
     * |[16]    |OVERRUNF  |Overrun Flag (Read Only)
     * |        |          |If any one of OVERRUN (ADC_ADDRx[16]) is set, this flag will be set to 1.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, this flag will be set to 1.
     * |[31:27] |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |When BUSY=1, this filed reflects current conversion channel.
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * @var ADC_T::ADSR1
     * Offset: 0x94  ADC Status Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VALID     |Data Valid Flag (Read Only)
     * |        |          |VALID[30:29], VALID[7:0] are the mirror of the VALID bits in ADC_ADDR30[17] ~ ADC_ADDR29[17], ADC_ADDR7[17]~ ADC_ADDR0[17].
     * |        |          |The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, VALID[30:29], VALID[7:0] will be set to 1.
     * @var ADC_T::ADSR2
     * Offset: 0x98  ADC Status Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |OVERRUN[30:29], OVERRUN[7:0] are the mirror of the OVERRUN bit in ADDR30[16] ~ ADDR29[16], ADDR7[16] ~ ADDR0[16].
     * |        |          |The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, OVERRUN[30:29], OVERRUN[7:0] will be set to 1.
     * @var ADC_T::ADTDCR
     * Offset: 0x9C  ADC Trigger Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PTDT      |BPWM Trigger Delay Time
     * |        |          |Set this field will delay ADC start conversion time after BPWM trigger.
     * |        |          |BPWM trigger delay time is (4 * PTDT) * system clock
     * @var ADC_T::ADPDMA
     * Offset: 0x100  ADC PDMA Current Transfer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[17:0]  |CURDAT    |ADC PDMA Current Transfer Data Register (Read Only)
     * |        |          |When PDMA transferring, read this register can monitor current PDMA transfer data.
     * |        |          |Current PDMA transfer data could be the content of ADC_ADDR0 ~ ADC_ADDR7 and ADC_ADDR29 ~ ADC_ADDR30 registers.
     */

    __I  uint32_t ADDR[31];              /*!< [0x0000 ~ 0x0078] ADC Data Register 30                                    */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t ADCR;                  /*!< [0x0080] ADC Control Register                                             */
    __IO uint32_t ADCHER;                /*!< [0x0084] ADC Channel Enable Register                                      */
    __IO uint32_t ADCMPR[2];             /*!< [0x0088 ~ 0x008C] ADC Compare Register 0 & 1                              */
    __IO uint32_t ADSR0;                 /*!< [0x0090] ADC Status Register0                                             */
    __I  uint32_t ADSR1;                 /*!< [0x0094] ADC Status Register1                                             */
    __I  uint32_t ADSR2;                 /*!< [0x0098] ADC Status Register2                                             */
    __IO uint32_t ADTDCR;                /*!< [0x009c] ADC Trigger Delay Control Register                               */
    __I  uint32_t RESERVE1[24];
    __I  uint32_t ADPDMA;                /*!< [0x0100] ADC PDMA Current Transfer Data Register                          */

} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
    @{ 
*/

#define ADC_ADDR_RSLT_Pos                (0)                                               /*!< ADC_T::ADDR: RSLT Position             */
#define ADC_ADDR_RSLT_Msk                (0xfffful << ADC_ADDR_RSLT_Pos)                   /*!< ADC_T::ADDR: RSLT Mask                 */

#define ADC_ADDR_OVERRUN_Pos             (16)                                              /*!< ADC_T::ADDR: OVERRUN Position          */
#define ADC_ADDR_OVERRUN_Msk             (0x1ul << ADC_ADDR_OVERRUN_Pos)                   /*!< ADC_T::ADDR: OVERRUN Mask              */

#define ADC_ADDR_VALID_Pos               (17)                                              /*!< ADC_T::ADDR: VALID Position            */
#define ADC_ADDR_VALID_Msk               (0x1ul << ADC_ADDR_VALID_Pos)                     /*!< ADC_T::ADDR: VALID Mask                */

#define ADC_ADCR_ADEN_Pos                (0)                                               /*!< ADC_T::ADCR: ADEN Position             */
#define ADC_ADCR_ADEN_Msk                (0x1ul << ADC_ADCR_ADEN_Pos)                      /*!< ADC_T::ADCR: ADEN Mask                 */

#define ADC_ADCR_ADIE_Pos                (1)                                               /*!< ADC_T::ADCR: ADIE Position             */
#define ADC_ADCR_ADIE_Msk                (0x1ul << ADC_ADCR_ADIE_Pos)                      /*!< ADC_T::ADCR: ADIE Mask                 */

#define ADC_ADCR_ADMD_Pos                (2)                                               /*!< ADC_T::ADCR: ADMD Position             */
#define ADC_ADCR_ADMD_Msk                (0x3ul << ADC_ADCR_ADMD_Pos)                      /*!< ADC_T::ADCR: ADMD Mask                 */

#define ADC_ADCR_TRGS_Pos                (4)                                               /*!< ADC_T::ADCR: TRGS Position             */
#define ADC_ADCR_TRGS_Msk                (0x3ul << ADC_ADCR_TRGS_Pos)                      /*!< ADC_T::ADCR: TRGS Mask                 */

#define ADC_ADCR_TRGCOND_Pos             (6)                                               /*!< ADC_T::ADCR: TRGCOND Position          */
#define ADC_ADCR_TRGCOND_Msk             (0x3ul << ADC_ADCR_TRGCOND_Pos)                   /*!< ADC_T::ADCR: TRGCOND Mask              */

#define ADC_ADCR_TRGEN_Pos               (8)                                               /*!< ADC_T::ADCR: TRGEN Position            */
#define ADC_ADCR_TRGEN_Msk               (0x1ul << ADC_ADCR_TRGEN_Pos)                     /*!< ADC_T::ADCR: TRGEN Mask                */

#define ADC_ADCR_PTEN_Pos                (9)                                               /*!< ADC_T::ADCR: PTEN Position             */
#define ADC_ADCR_PTEN_Msk                (0x1ul << ADC_ADCR_PTEN_Pos)                      /*!< ADC_T::ADCR: PTEN Mask                 */

#define ADC_ADCR_DIFFEN_Pos              (10)                                              /*!< ADC_T::ADCR: DIFFEN Position           */
#define ADC_ADCR_DIFFEN_Msk              (0x1ul << ADC_ADCR_DIFFEN_Pos)                    /*!< ADC_T::ADCR: DIFFEN Mask               */

#define ADC_ADCR_ADST_Pos                (11)                                              /*!< ADC_T::ADCR: ADST Position             */
#define ADC_ADCR_ADST_Msk                (0x1ul << ADC_ADCR_ADST_Pos)                      /*!< ADC_T::ADCR: ADST Mask                 */

#define ADC_ADCR_SMPTSEL_Pos             (16)                                              /*!< ADC_T::ADCR: SMPTSEL Position          */
#define ADC_ADCR_SMPTSEL_Msk             (0x7ul << ADC_ADCR_SMPTSEL_Pos)                   /*!< ADC_T::ADCR: SMPTSEL Mask              */

#define ADC_ADCR_DMOF_Pos                (31)                                              /*!< ADC_T::ADCR: DMOF Position             */
#define ADC_ADCR_DMOF_Msk                (0x1ul << ADC_ADCR_DMOF_Pos)                      /*!< ADC_T::ADCR: DMOF Mask                 */

#define ADC_ADCHER_CHEN_Pos              (0)                                               /*!< ADC_T::ADCHER: CHEN Position           */
#define ADC_ADCHER_CHEN_Msk              (0xfffffffful << ADC_ADCHER_CHEN_Pos)             /*!< ADC_T::ADCHER: CHEN Mask               */

#define ADC_ADCMPR_CMPEN_Pos             (0)                                               /*!< ADC_T::ADCMPR: CMPEN Position          */
#define ADC_ADCMPR_CMPEN_Msk             (0x1ul << ADC_ADCMPR_CMPEN_Pos)                   /*!< ADC_T::ADCMPR: CMPEN Mask              */

#define ADC_ADCMPR_CMPIE_Pos             (1)                                               /*!< ADC_T::ADCMPR: CMPIE Position          */
#define ADC_ADCMPR_CMPIE_Msk             (0x1ul << ADC_ADCMPR_CMPIE_Pos)                   /*!< ADC_T::ADCMPR: CMPIE Mask              */

#define ADC_ADCMPR_CMPCOND_Pos           (2)                                               /*!< ADC_T::ADCMPR: CMPCOND Position        */
#define ADC_ADCMPR_CMPCOND_Msk           (0x1ul << ADC_ADCMPR_CMPCOND_Pos)                 /*!< ADC_T::ADCMPR: CMPCOND Mask            */

#define ADC_ADCMPR_CMPCH_Pos             (3)                                               /*!< ADC_T::ADCMPR: CMPCH Position          */
#define ADC_ADCMPR_CMPCH_Msk             (0x1ful << ADC_ADCMPR_CMPCH_Pos)                  /*!< ADC_T::ADCMPR: CMPCH Mask              */

#define ADC_ADCMPR_CMPMATCNT_Pos         (8)                                               /*!< ADC_T::ADCMPR: CMPMATCNT Position      */
#define ADC_ADCMPR_CMPMATCNT_Msk         (0xful << ADC_ADCMPR_CMPMATCNT_Pos)               /*!< ADC_T::ADCMPR: CMPMATCNT Mask          */

#define ADC_ADCMPR_CMPWEN_Pos            (15)                                              /*!< ADC_T::ADCMPR: CMPWEN Position         */
#define ADC_ADCMPR_CMPWEN_Msk            (0x1ul << ADC_ADCMPR_CMPWEN_Pos)                  /*!< ADC_T::ADCMPR: CMPWEN Mask             */

#define ADC_ADCMPR_CMPD_Pos              (16)                                              /*!< ADC_T::ADCMPR: CMPD Position           */
#define ADC_ADCMPR_CMPD_Msk              (0xffful << ADC_ADCMPR_CMPD_Pos)                  /*!< ADC_T::ADCMPR: CMPD Mask               */

#define ADC_ADSR0_ADF_Pos                (0)                                               /*!< ADC_T::ADSR0: ADF Position             */
#define ADC_ADSR0_ADF_Msk                (0x1ul << ADC_ADSR0_ADF_Pos)                      /*!< ADC_T::ADSR0: ADF Mask                 */

#define ADC_ADSR0_CMPF0_Pos              (1)                                               /*!< ADC_T::ADSR0: CMPF0 Position           */
#define ADC_ADSR0_CMPF0_Msk              (0x1ul << ADC_ADSR0_CMPF0_Pos)                    /*!< ADC_T::ADSR0: CMPF0 Mask               */

#define ADC_ADSR0_CMPF1_Pos              (2)                                               /*!< ADC_T::ADSR0: CMPF1 Position           */
#define ADC_ADSR0_CMPF1_Msk              (0x1ul << ADC_ADSR0_CMPF1_Pos)                    /*!< ADC_T::ADSR0: CMPF1 Mask               */

#define ADC_ADSR0_BUSY_Pos               (7)                                               /*!< ADC_T::ADSR0: BUSY Position            */
#define ADC_ADSR0_BUSY_Msk               (0x1ul << ADC_ADSR0_BUSY_Pos)                     /*!< ADC_T::ADSR0: BUSY Mask                */

#define ADC_ADSR0_VALIDF_Pos             (8)                                               /*!< ADC_T::ADSR0: VALIDF Position          */
#define ADC_ADSR0_VALIDF_Msk             (0x1ul << ADC_ADSR0_VALIDF_Pos)                   /*!< ADC_T::ADSR0: VALIDF Mask              */

#define ADC_ADSR0_OVERRUNF_Pos           (16)                                              /*!< ADC_T::ADSR0: OVERRUNF Position        */
#define ADC_ADSR0_OVERRUNF_Msk           (0x1ul << ADC_ADSR0_OVERRUNF_Pos)                 /*!< ADC_T::ADSR0: OVERRUNF Mask            */

#define ADC_ADSR0_CHANNEL_Pos            (27)                                              /*!< ADC_T::ADSR0: CHANNEL Position         */
#define ADC_ADSR0_CHANNEL_Msk            (0x1ful << ADC_ADSR0_CHANNEL_Pos)                 /*!< ADC_T::ADSR0: CHANNEL Mask             */

#define ADC_ADSR1_VALID_Pos              (0)                                               /*!< ADC_T::ADSR1: VALID Position           */
#define ADC_ADSR1_VALID_Msk              (0xfffffffful << ADC_ADSR1_VALID_Pos)             /*!< ADC_T::ADSR1: VALID Mask               */

#define ADC_ADSR2_OVERRUN_Pos            (0)                                               /*!< ADC_T::ADSR2: OVERRUN Position         */
#define ADC_ADSR2_OVERRUN_Msk            (0xfffffffful << ADC_ADSR2_OVERRUN_Pos)           /*!< ADC_T::ADSR2: OVERRUN Mask             */

#define ADC_ADTDCR_PTDT_Pos              (0)                                               /*!< ADC_T::ADTDCR: PTDT Position           */
#define ADC_ADTDCR_PTDT_Msk              (0xfful << ADC_ADTDCR_PTDT_Pos)                   /*!< ADC_T::ADTDCR: PTDT Mask               */

#define ADC_ADPDMA_CURDAT_Pos            (0)                                               /*!< ADC_T::ADPDMA: CURDAT Position         */
#define ADC_ADPDMA_CURDAT_Msk            (0x3fffful << ADC_ADPDMA_CURDAT_Pos)              /*!< ADC_T::ADPDMA: CURDAT Mask             */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */

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


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller(PDMA)
    Memory Mapped Structure for PDMA Controller
    @{ 
*/



typedef struct
{
    /**
     * @var DSCT_T::CTL
     * Offset: 0x00/0x10/0x20/0x30/0x40/0x50/0x60/0x70/0x80/0x90  Descriptor Table Control Register of PDMA Channel 0~9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |OPMODE    |PDMA Operation Mode Selection
     * |        |          |00 = Idle state: Channel is stopped or this table is complete, when PDMA finish channel table task, OPMODE will be cleared to idle state automatically.
     * |        |          |01 = Basic mode: The descriptor table only has one task
     * |        |          |When this task is finished, the TDIF(PDMA_INTSTS[1]) will be asserted.
     * |        |          |10 = Scatter-Gather mode: When operating in this mode, user must give the first descriptor table address in PDMA_DSCT_FIRST register; PDMA controller will ignore this task, then load the next task to execute.
     * |        |          |11 = Reserved.
     * |        |          |Note: Before filling transfer task in the Descriptor Table, user must check if the descriptor table is complete.
     * |[2]     |TXTYPE    |Transfer Type
     * |        |          |0 = Burst transfer type.
     * |        |          |1 = Single transfer type.
     * |[6:4]   |BURSIZE   |Burst Size
     * |        |          |This field is used for peripheral to determine the burst size or used for determine the re-arbitration size.
     * |        |          |000 = 128 Transfers.
     * |        |          |001 = 64 Transfers.
     * |        |          |010 = 32 Transfers.
     * |        |          |011 = 16 Transfers.
     * |        |          |100 = 8 Transfers.
     * |        |          |101 = 4 Transfers.
     * |        |          |110 = 2 Transfers.
     * |        |          |111 = 1 Transfers.
     * |        |          |Note: This field is only useful in burst transfer type.
     * |[7]     |TBINTDIS  |Table Interrupt Disable Bit
     * |        |          |This field can be used to decide whether to enable table interrupt or not
     * |        |          |If the TBINTDIS bit is enabled when PDMA controller finishes transfer task, it will not generates transfer done interrupt.
     * |        |          |0 = Table interrupt Enabled.
     * |        |          |1 = Table interrupt Disabled.
     * |        |          |Note: If this bit set to 1, the TEMPTYF will not be set.
     * |[9:8]   |SAINC     |Source Address Increment
     * |        |          |This Field Is Used To Set The Source Address Increment Size.
     * |        |          |11 = No Increment (Fixed Address).
     * |        |          |Others = Increment And Size Is Depended On TXWIDTH Selection.
     * |[11:10] |DAINC     |Destination Address Increment
     * |        |          |This field is used to set the destination address increment size.
     * |        |          |11 = No increment (fixed address).
     * |        |          |Others = Increment and size is depended on TXWIDTH selection.
     * |[13:12] |TXWIDTH   |Transfer Width Selection
     * |        |          |This field is used for transfer width.
     * |        |          |00 = One byte (8 bit) is transferred for every operation.
     * |        |          |01 = One half-word (16 bit) is transferred for every operation.
     * |        |          |10 = One word (32-bit) is transferred for every operation.
     * |        |          |11 = Reserved.
     * |        |          |Note: The PDMA transfer source address (PDMA_DSCT_SA) and PDMA transfer destination address (PDMA_DSCT_DA) should be alignment under the TXWIDTH selection
     * |        |          |For example, if source address is 0x2000_0202, but TXWIDTH is word transfer, the source address is not word alignment
     * |        |          |The source address is aligned when TXWIDTH is byte or half-word transfer.
     * |[29:16] |TXCNT     |Transfer Count
     * |        |          |The TXCNT represents the required number of PDMA transfer, the real transfer count is (TXCNT + 1); The maximum transfer count is 16384 , every transfer may be byte, half-word or word that is dependent on TXWIDTH field.
     * |        |          |Note: When PDMA finish each transfer data, this field will be decrease immediately.
     * @var DSCT_T::SA
     * Offset: 0x04/0x14/0x24/0x34/0x44/0x54/0x64/0x74/0x84/0x94  Source Address Register of PDMA Channel 0~9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |SA        |PDMA Transfer Source Address Register
     * |        |          |This field indicates a 32-bit source address of PDMA controller.
     * |        |          |Note: The PDMA transfer source address should be aligned with the TXWIDTH(PDMA_DSCTn_CTL[13:12], n=0,1..9) selection.
     * @var DSCT_T::DA
     * Offset: 0x08/0x18/0x28/0x38/0x48/0x58/0x68/0x78/0x88/0x98  Destination Address Register of PDMA Channel 0~9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DA        |PDMA Transfer Destination Address Register
     * |        |          |This field indicates a 32-bit destination address of PDMA controller.
     * |        |          |Note: The PDMA transfer destination address should be aligned with the TXWIDTH(PDMA_DSCTn_CTL[13:12], n=0,1..9) selection.
     * @var DSCT_T::FIRST
     * Offset: 0x0C/0x1C/0x2C/0x3C/0x4C/0x5C/0x6C/0x7C/0x8C/0x9C  First Scatter-Gather Descriptor Table Offset of PDMA Channel 0~9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FIRST     |PDMA First Descriptor Table Offset
     * |        |          |This field indicates the offset of the first descriptor table address in system memory.
     * |        |          |Write Operation:
     * |        |          |If the system memory based address is 0x2000_0000 (PDMA_SCATBA), and the first descriptor table is start from 0x2000_0100, then this field must fill in 0x0100.
     * |        |          |Read Operation:
     * |        |          |When operating in scatter-gather mode, the last two bits FIRST[1:0] will become reserved.
     * |        |          |Note1: The first descriptor table address must be word boundary.
     * |        |          |Note2: Before filled transfer task in the descriptor table, user must check if the descriptor table is complete.
     * |[31:16] |NEXT      |PDMA Next Descriptor Table Offset
     * |        |          |This field indicates the offset of next descriptor table address in system memory.
     * |        |          |Note: write operation is useless in this field.
     */

    __IO uint32_t CTL;             /*!< [0x00/0x10/0x20/0x30/0x40/0x50/0x60/0x70/0x80/0x90] Descriptor Table Control Register of PDMA Channel 0~9              */
    __IO uint32_t SA;              /*!< [0x04/0x14/0x24/0x34/0x44/0x54/0x64/0x74/0x84/0x94] Source Address Register of PDMA Channel 0~9                        */
    __IO uint32_t DA;              /*!< [0x08/0x18/0x28/0x38/0x48/0x58/0x68/0x78/0x88/0x98] Destination Address Register of PDMA Channel 0~9                   */
    union
    {
        __IO uint32_t FIRST;       /*!< [0x0C/0x1C/0x2C/0x3C/0x4C/0x5C/0x6C/0x7C/0x8C/0x9C] First Scatter-Gather Descriptor Table Offset of PDMA Channel 0~9   */
        __IO uint32_t NEXT;        /*!< Next Scatter-Gather Descriptor Table Offset                                                   */
    };

} DSCT_T;

typedef struct
{


    /**
     * @var PDMA_T::CURSCAT
     * Offset: 0xA0/0xA4/0xA8/0xAC/0xB0/0xB4/0xB8/0xBC/0xC0/0xC4  Current Scatter-Gather Descriptor Table Address of PDMA Channel 0~9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CURADDR   |PDMA Current Description Address Register (Read Only)
     * |        |          |This field indicates a 32-bit current external description address of PDMA controller.
     * |        |          |Note: This field is read only and only used for Scatter-Gather mode to indicate the current external description address.
     * @var PDMA_T::CHCTL
     * Offset: 0x400  PDMA Channel Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN0     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[1]     |CHEN1     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[2]     |CHEN2     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[3]     |CHEN3     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[4]     |CHEN4     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[5]     |CHEN5     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[6]     |CHEN6     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[7]     |CHEN7     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[8]     |CHEN8     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * |[9]     |CHEN9     |PDMA Channel N Enable Bit
     * |        |          |Set this bit to 1 to enable PDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = PDMA channel [n] Disabled.
     * |        |          |1 = PDMA channel [n] Enabled.
     * |        |          |Note: Set PDMA_PAUSE or PDMA_RESET register will also clear this bit.
     * @var PDMA_T::PAUSE
     * Offset: 0x404  PDMA Transfer Pause Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PAUSE0    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[1]     |PAUSE1    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[2]     |PAUSE2    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[3]     |PAUSE3    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[4]     |PAUSE4    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[5]     |PAUSE5    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[6]     |PAUSE6    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[7]     |PAUSE7    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[8]     |PAUSE8    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * |[9]     |PAUSE9    |PDMA Channel N Transfer Pause Control Register (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the PDMA transfer
     * |        |          |When user sets PAUSEn bit, the PDMA controller will pause the on-going transfer, then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1..9) and clear request active flag
     * |        |          |If re-enable the paused channel agian, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause PDMA channel n transfer.
     * @var PDMA_T::SWREQ
     * Offset: 0x408  PDMA Software Request Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SWREQ0    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[1]     |SWREQ1    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[2]     |SWREQ2    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[3]     |SWREQ3    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[4]     |SWREQ4    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[5]     |SWREQ5    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[6]     |SWREQ6    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[7]     |SWREQ7    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[8]     |SWREQ8    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * |[9]     |SWREQ9    |PDMA Channel N Software Request Register (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to PDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note1: User can read PDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note2: If user does not enable corresponding PDMA channel, the software request will be ignored.
     * @var PDMA_T::TRGSTS
     * Offset: 0x40C  PDMA Channel Request Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REQSTS0   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[1]     |REQSTS1   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[2]     |REQSTS2   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[3]     |REQSTS3   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[4]     |REQSTS4   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[5]     |REQSTS5   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[6]     |REQSTS6   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[7]     |REQSTS7   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[8]     |REQSTS8   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * |[9]     |REQSTS9   |PDMA Channel N Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When PDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = PDMA Channel n has no request.
     * |        |          |1 = PDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each PDMA transfer by setting PDMA_PAUSE or PDMA_RESET register respectively, this bit will be cleared automatically after finishing current transfer.
     * @var PDMA_T::PRISET
     * Offset: 0x410  PDMA Fixed Priority Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FPRISET0  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[1]     |FPRISET1  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[2]     |FPRISET2  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[3]     |FPRISET3  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[4]     |FPRISET4  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[5]     |FPRISET5  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[6]     |FPRISET6  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[7]     |FPRISET7  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[8]     |FPRISET8  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * |[9]     |FPRISET9  |PDMA Channel N Fixed Priority Setting Register
     * |        |          |Set this bit to 1 to enable fixed priority level
     * |        |          |The fixed priority channel has higher priority than round-robin priority channel
     * |        |          |If multiple channels are set as the same priority, the higher number of channels have higher priority.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set PDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding PDMA channel is round-robin priority.
     * |        |          |1 = Corresponding PDMA channel is fixed priority.
     * |        |          |Note: This field only set to fixed priority, clear fixed priority use PDMA_PRICLR register.
     * @var PDMA_T::PRICLR
     * Offset: 0x414  PDMA Fixed Priority Clear Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FPRICLR0  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[1]     |FPRICLR1  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[2]     |FPRICLR2  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[3]     |FPRICLR3  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[4]     |FPRICLR4  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[5]     |FPRICLR5  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[6]     |FPRICLR6  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[7]     |FPRICLR7  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[8]     |FPRICLR8  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * |[9]     |FPRICLR9  |PDMA Channel N Fixed Priority Clear Register (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear PDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read PDMA_PRISET register to know the channel priority.
     * @var PDMA_T::INTEN
     * Offset: 0x418  PDMA Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTEN0    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[1]     |INTEN1    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[2]     |INTEN2    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[3]     |INTEN3    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[4]     |INTEN4    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[5]     |INTEN5    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[6]     |INTEN6    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[7]     |INTEN7    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[8]     |INTEN8    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * |[9]     |INTEN9    |PDMA Channel N Interrupt Enable Register
     * |        |          |This field is used for enabling PDMA channel[n] interrupt.
     * |        |          |0 = PDMA channel n interrupt Disabled.
     * |        |          |1 = PDMA channel n interrupt Enabled.
     * @var PDMA_T::INTSTS
     * Offset: 0x41C  PDMA Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ABTIF     |PDMA Read/Write Target Abort Interrupt Flag (Read Only)
     * |        |          |This bit indicates that PDMA has target abort error; Software can read PDMA_ABTSTS register to find which channel has target abort error.
     * |        |          |0 = No AHB bus ERROR response received.
     * |        |          |1 = AHB bus ERROR response received.
     * |[1]     |TDIF      |Transfer Done Interrupt Flag (Read Only)
     * |        |          |This bit indicates that PDMA controller has finished transmission; User can read PDMA_TDSTS register to indicate which channel finished transfer.
     * |        |          |0 = Not finished yet.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[2]     |TEIF      |Table Empty Interrupt Flag (Read Only)
     * |        |          |This bit indicates PDMA channel scatter-gather table is empty
     * |        |          |User can read PDMA_SCATSTS register to indicate which channel scatter-gather table is empty.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty.
     * |[8]     |REQTOF0   |PDMA Channel N Request Time-out Flag for Each Channel [N]
     * |        |          |This flag indicates that PDMA controller has waited peripheral request for a period defined by PDMA_TOC0, user can write 1 to clear these bits.
     * |        |          |0 = No request time-out.
     * |        |          |1 = Peripheral request time-out.
     * |[9]     |REQTOF1   |PDMA Channel N Request Time-out Flag for Each Channel [N]
     * |        |          |This flag indicates that PDMA controller has waited peripheral request for a period defined by PDMA_TOC10, user can write 1 to clear these bits.
     * |        |          |0 = No request time-out.
     * |        |          |1 = Peripheral request time-out.
     * @var PDMA_T::ABTSTS
     * Offset: 0x420  PDMA Channel Read/Write Target Abort Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ABTIF0    |PDMA Channel 0 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[1]     |ABTIF1    |PDMA Channel 1 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[2]     |ABTIF2    |PDMA Channel 2 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[3]     |ABTIF3    |PDMA Channel 3 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[4]     |ABTIF4    |PDMA Channel 4 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[5]     |ABTIF5    |PDMA Channel 5 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[6]     |ABTIF6    |PDMA Channel 6 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[7]     |ABTIF7    |PDMA Channel 7 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[8]     |ABTIF8    |PDMA Channel 8 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |[9]     |ABTIF9    |PDMA Channel 9 Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which PDMA controller has target abort error; User can write 1 to clear these bits.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * @var PDMA_T::TDSTS
     * Offset: 0x424  PDMA Channel Transfer Done Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TDIF0     |PDMA Channel 0 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[1]     |TDIF1     |PDMA Channel 1 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[2]     |TDIF2     |PDMA Channel 2 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[3]     |TDIF3     |PDMA Channel 3 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[4]     |TDIF4     |PDMA Channel 4 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[5]     |TDIF5     |PDMA Channel 5 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[6]     |TDIF6     |PDMA Channel 6 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[7]     |TDIF7     |PDMA Channel 7 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[8]     |TDIF8     |PDMA Channel 8 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * |[9]     |TDIF9     |PDMA Channel 9 Transfer Done Flag Register
     * |        |          |This bit indicates whether PDMA controller channel transfer has been finished or not, user can write 1 to clear these bits.
     * |        |          |0 = PDMA channel transfer has not finished.
     * |        |          |1 = PDMA channel has finished transmission.
     * @var PDMA_T::SCATSTS
     * Offset: 0x428  PDMA Scatter-Gather Table Empty Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TEMPTYF0  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[1]     |TEMPTYF1  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[2]     |TEMPTYF2  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[3]     |TEMPTYF3  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[4]     |TEMPTYF4  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[5]     |TEMPTYF5  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[6]     |TEMPTYF6  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[7]     |TEMPTYF7  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[8]     |TEMPTYF8  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * |[9]     |TEMPTYF9  |Table Empty Flag Register
     * |        |          |This bit indicates which PDMA channel table is empty when channel have a request , no matter request from software or peripheral, but operation mode of channel descriptor table is idle state, or channel has finished current transfer and next table operation mode is idle state for PDMA Scatter-Gather mode
     * |        |          |User can write 1 to clear these bits.
     * |        |          |0 = PDMA channel scatter-gather table is not empty.
     * |        |          |1 = PDMA channel scatter-gather table is empty and PDMA SWREQ has be set.
     * @var PDMA_T::TACTSTS
     * Offset: 0x42C  PDMA Transfer Active Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXACTF0   |PDMA Channel 0 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[1]     |TXACTF1   |PDMA Channel 1 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[2]     |TXACTF2   |PDMA Channel 2 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[3]     |TXACTF3   |PDMA Channel 3 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[4]     |TXACTF4   |PDMA Channel 4 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[5]     |TXACTF5   |PDMA Channel 5 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[6]     |TXACTF6   |PDMA Channel 6 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[7]     |TXACTF7   |PDMA Channel 7 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[8]     |TXACTF8   |PDMA Channel 8 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * |[9]     |TXACTF9   |PDMA Channel 9 Transfer on Active Flag Register (Read Only)
     * |        |          |This bit indicates which PDMA channel is in active.
     * |        |          |0 = PDMA channel is not finished.
     * |        |          |1 = PDMA channel is active.
     * @var PDMA_T::TOUTPSC
     * Offset: 0x430  PDMA Time-out Prescaler Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TOUTPSC0  |PDMA Channel 0 Time-out Clock Source Prescaler Bits
     * |        |          |000 = PDMA channel 0 time-out clock source is HCLK/2^8.
     * |        |          |001 = PDMA channel 0 time-out clock source is HCLK/2^9.
     * |        |          |010 = PDMA channel 0 time-out clock source is HCLK/2^10.
     * |        |          |011 = PDMA channel 0 time-out clock source is HCLK/2^11.
     * |        |          |100 = PDMA channel 0 time-out clock source is HCLK/2^12.
     * |        |          |101 = PDMA channel 0 time-out clock source is HCLK/2^13.
     * |        |          |110 = PDMA channel 0 time-out clock source is HCLK/2^14.
     * |        |          |111 = PDMA channel 0 time-out clock source is HCLK/2^15.
     * |[6:4]   |TOUTPSC1  |PDMA Channel 1 Time-out Clock Source Prescaler Bits
     * |        |          |000 = PDMA channel 1 time-out clock source is HCLK/2^8.
     * |        |          |001 = PDMA channel 1 time-out clock source is HCLK/2^9.
     * |        |          |010 = PDMA channel 1 time-out clock source is HCLK/2^10.
     * |        |          |011 = PDMA channel 1 time-out clock source is HCLK/2^11.
     * |        |          |100 = PDMA channel 1 time-out clock source is HCLK/2^12.
     * |        |          |101 = PDMA channel 1 time-out clock source is HCLK/2^13.
     * |        |          |110 = PDMA channel 1 time-out clock source is HCLK/2^14.
     * |        |          |111 = PDMA channel 1 time-out clock source is HCLK/2^15.
     * @var PDMA_T::TOUTEN
     * Offset: 0x434  PDMA Time-out Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TOUTEN0   |PDMA Channel 0 Time-out Enable Bit
     * |        |          |0 = PDMA Channel 0 time-out function Disable.
     * |        |          |1 = PDMA Channel 0 time-out function Enable.
     * |[1]     |TOUTEN1   |PDMA Channel 1 Time-out Enable Bit
     * |        |          |0 = PDMA Channel 1 time-out function Disable.
     * |        |          |1 = PDMA Channel 1 time-out function Enable.
     * @var PDMA_T::TOUTIEN
     * Offset: 0x438  PDMA Time-out Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TOUTIEN0  |PDMA Channel 0 Time-out Interrupt Enable Bit
     * |        |          |0 = PDMA Channel 0 time-out interrupt Disable.
     * |        |          |1 = PDMA Channel 0 time-out interrupt Enable.
     * |[1]     |TOUTIEN1  |PDMA Channel 1 Time-out Interrupt Enable Bit
     * |        |          |0 = PDMA Channel 1 time-out interrupt Disable.
     * |        |          |1 = PDMA Channel 1 time-out interrupt Enable.
     * @var PDMA_T::SCATBA
     * Offset: 0x43C  PDMA Scatter-Gather Descriptor Table Base Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:16] |SCATBA    |PDMA Scatter-gather Descriptor Table Address Register
     * |        |          |In Scatter-Gather mode, this is the base address for calculating the next link - list address
     * |        |          |The next link address equation is
     * |        |          |Next Link Address = PDMA_SCATBA + PDMA_DSCT_FIRST.
     * |        |          |Note: Only useful in Scatter-Gather mode.
     * @var PDMA_T::TOC0_1
     * Offset: 0x440  PDMA Channel 0 and Channel 1 Time-out Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |TOC0      |Time-out Counter for Channel 0
     * |        |          |This controls the period of time-out function for channel 0
     * |        |          |The calculation unit is based on TOUTPSC0 (PDMA_TOUTPSC[2:0]) clock.
     * |        |          |Time-out period = (Period of time-out clock) * (16-bit TOCn),n = 0,1.
     * |[31:16] |TOC1      |Time-out Counter for Channel 1
     * |        |          |This controls the period of time-out function for channel 1
     * |        |          |The calculation unit is based on TOUTPSC1 (PDMA_TOUTPSC[5:3]) clock
     * |        |          |The example of time-out period can refer TOC0 bit description.
     * @var PDMA_T::RESET
     * Offset: 0x460  PDMA Channel Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RESET0    |PDMA Channel 0 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 0.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[1]     |RESET1    |PDMA Channel 1 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 1.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[2]     |RESET2    |PDMA Channel 2 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 2.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[3]     |RESET3    |PDMA Channel 3 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 3.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[4]     |RESET4    |PDMA Channel 4 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 4.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[5]     |RESET5    |PDMA Channel 5 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 5.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[6]     |RESET6    |PDMA Channel 6 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 6.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[7]     |RESET7    |PDMA Channel 7 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 7.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[8]     |RESET8    |PDMA Channel 8 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 8.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * |[9]     |RESET9    |PDMA Channel 9 Reset Control Register
     * |        |          |User can set this bit field to reset the PDMA channel
     * |        |          |When user sets RESETn bit, the PDMA controller will finish the on-going transfer then clear the channel enable bit CHEN(PDMA_CHCTL [n], n=0,1...9)) and clear request active flag
     * |        |          |If re-enable channel after channel reset, PDMA will re-load the channel description table to execute PDMA task.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset PDMA channel 9.
     * |        |          |Note: This bit will be cleared automatically after finishing reset process.
     * @var PDMA_T::REQSEL0_3
     * Offset: 0x480  PDMA Channel 0 to Channel 3 Request Source Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |REQSRC0   |Channel 0 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 0
     * |        |          |User can configure the peripheral by setting REQSRC0.
     * |        |          |0 = Disable PDMA.
     * |        |          |1 = Reserved.
     * |        |          |4 = Channel connects to UART0_TX.
     * |        |          |5 = Channel connects to UART0_RX.
     * |        |          |6 = Channel connects to UART1_TX.
     * |        |          |7 = Channel connects to UART1_RX.
     * |        |          |16 = Channel connects to SPI0_TX.
     * |        |          |17 = Channel connects to SPI0_RX.
     * |        |          |18 = Channel connects to SPI1_TX.
     * |        |          |19 = Channel connects to SPI1_RX.
     * |        |          |20 = Channel connects to ADC_RX.
     * |        |          |28 = Channel connects to I2C0_TX.
     * |        |          |29 = Channel connects to I2C0_RX.
     * |        |          |30 = Channel connects to I2C1_TX.
     * |        |          |31 = Channel connects to I2C1_RX.
     * |        |          |32 = Channel connects to TMR0.
     * |        |          |33 = Channel connects to TMR1.
     * |        |          |34 = Channel connects to TMR2.
     * |        |          |35 = Channel connects to TMR3.
     * |        |          |36 = Channel connects to LLSI0.
     * |        |          |37 = Channel connects to LLSI1.
     * |        |          |38 = Channel connects to LLSI2.
     * |        |          |39 = Channel connects to LLSI3.
     * |        |          |40 = Channel connects to LLSI4.
     * |        |          |41 = Channel connects to LLSI5.
     * |        |          |42 = Channel connects to LLSI6.
     * |        |          |43 = Channel connects to LLSI7.
     * |        |          |44 = Channel connects to LLSI8.
     * |        |          |45 = Channel connects to LLSI9.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: A request source cannot assign to two channels at the same time.
     * |        |          |Note 2: This field is useless when transfer between memory and memory.
     * |[13:8]  |REQSRC1   |Channel 1 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 1
     * |        |          |User can configure the peripheral setting by REQSRC1.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[21:16] |REQSRC2   |Channel 2 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 2
     * |        |          |User can configure the peripheral setting by REQSRC2.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[29:24] |REQSRC3   |Channel 3 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 3
     * |        |          |User can configure the peripheral setting by REQSRC3.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * @var PDMA_T::REQSEL4_7
     * Offset: 0x484  PDMA Channel 4 to Channel 7 Request Source Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |REQSRC4   |Channel 4 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 4
     * |        |          |User can configure the peripheral setting by REQSRC4.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[13:8]  |REQSRC5   |Channel 5 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 5
     * |        |          |User can configure the peripheral setting by REQSRC5.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[21:16] |REQSRC6   |Channel 6 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 6
     * |        |          |User can configure the peripheral setting by REQSRC6.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[29:24] |REQSRC7   |Channel 7 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 7
     * |        |          |User can configure the peripheral setting by REQSRC5.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * @var PDMA_T::REQSEL8_9
     * Offset: 0x488  PDMA Channel 8 to Channel 9 Request Source Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |REQSRC8   |Channel 8 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 8
     * |        |          |User can configure the peripheral setting by REQSRC8.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[13:8]  |REQSRC9   |Channel 9 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to PDMA channel 9
     * |        |          |User can configure the peripheral setting by REQSRC9.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     */

    DSCT_T        DSCT[10];               /*!< [0x0000 ~ 0x009C] DMA Embedded Description Table 0~9                      */
    __I  uint32_t CURSCAT[10];            /*!< [0x00A0~0x00C0] Current Scatter-Gather Descriptor Table Address of PDMA Channel 0~9 */
    __I  uint32_t RESERVE0[206];
    __IO uint32_t CHCTL;                 /*!< [0x0400] PDMA Channel Control Register                                    */
    __O  uint32_t PAUSE;                 /*!< [0x0404] PDMA Transfer Pause Control Register                             */
    __O  uint32_t SWREQ;                 /*!< [0x0408] PDMA Software Request Register                                   */
    __I  uint32_t TRGSTS;                /*!< [0x040c] PDMA Channel Request Status Register                             */
    __IO uint32_t PRISET;                /*!< [0x0410] PDMA Fixed Priority Setting Register                             */
    __O  uint32_t PRICLR;                /*!< [0x0414] PDMA Fixed Priority Clear Register                               */
    __IO uint32_t INTEN;                 /*!< [0x0418] PDMA Interrupt Enable Register                                   */
    __IO uint32_t INTSTS;                /*!< [0x041c] PDMA Interrupt Status Register                                   */
    __IO uint32_t ABTSTS;                /*!< [0x0420] PDMA Channel Read/Write Target Abort Flag Register               */
    __IO uint32_t TDSTS;                 /*!< [0x0424] PDMA Channel Transfer Done Flag Register                         */
    __IO uint32_t SCATSTS;               /*!< [0x0428] PDMA Scatter-Gather Table Empty Status Register                  */
    __I  uint32_t TACTSTS;               /*!< [0x042c] PDMA Transfer Active Flag Register                               */
    __IO uint32_t TOUTPSC;               /*!< [0x0430] PDMA Time-out Prescaler Register                                 */
    __IO uint32_t TOUTEN;                /*!< [0x0434] PDMA Time-out Enable Register                                    */
    __IO uint32_t TOUTIEN;               /*!< [0x0438] PDMA Time-out Interrupt Enable Register                          */
    __IO uint32_t SCATBA;                /*!< [0x043c] PDMA Scatter-Gather Descriptor Table Base Address Register       */
    __IO uint32_t TOC0_1;                /*!< [0x0440] PDMA Channel 0 and Channel 1 Time-out Counter Register           */
    __I  uint32_t RESERVE1[7];
    __IO uint32_t RESET;                 /*!< [0x0460] PDMA Channel Reset Control Register                              */
    __I  uint32_t RESERVE2[7];
    __IO uint32_t REQSEL0_3;             /*!< [0x0480] PDMA Channel 0 to Channel 3 Request Source Select Register       */
    __IO uint32_t REQSEL4_7;             /*!< [0x0484] PDMA Channel 4 to Channel 7 Request Source Select Register       */
    __IO uint32_t REQSEL8_9;             /*!< [0x0488] PDMA Channel 8 to Channel 9 Request Source Select Register       */

} PDMA_T;




/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for PDMA Controller
    @{ 
*/

#define PDMA_DSCT_CTL_OPMODE_Pos         (0)                                               /*!< PDMA_T::DSCT_CTL: OPMODE Position     */
#define PDMA_DSCT_CTL_OPMODE_Msk         (0x3ul << PDMA_DSCT_CTL_OPMODE_Pos)               /*!< PDMA_T::DSCT_CTL: OPMODE Mask         */

#define PDMA_DSCT_CTL_TXTYPE_Pos         (2)                                               /*!< PDMA_T::DSCT_CTL: TXTYPE Position     */
#define PDMA_DSCT_CTL_TXTYPE_Msk         (0x1ul << PDMA_DSCT_CTL_TXTYPE_Pos)               /*!< PDMA_T::DSCT_CTL: TXTYPE Mask         */

#define PDMA_DSCT_CTL_BURSIZE_Pos        (4)                                               /*!< PDMA_T::DSCT_CTL: BURSIZE Position    */
#define PDMA_DSCT_CTL_BURSIZE_Msk        (0x7ul << PDMA_DSCT_CTL_BURSIZE_Pos)              /*!< PDMA_T::DSCT_CTL: BURSIZE Mask        */

#define PDMA_DSCT_CTL_TBINTDIS_Pos       (7)                                               /*!< PDMA_T::DSCT_CTL: TBINTDIS Position   */
#define PDMA_DSCT_CTL_TBINTDIS_Msk       (0x1ul << PDMA_DSCT_CTL_TBINTDIS_Pos)             /*!< PDMA_T::DSCT_CTL: TBINTDIS Mask       */

#define PDMA_DSCT_CTL_SAINC_Pos          (8)                                               /*!< PDMA_T::DSCT_CTL: SAINC Position      */
#define PDMA_DSCT_CTL_SAINC_Msk          (0x3ul << PDMA_DSCT_CTL_SAINC_Pos)                /*!< PDMA_T::DSCT_CTL: SAINC Mask          */

#define PDMA_DSCT_CTL_DAINC_Pos          (10)                                              /*!< PDMA_T::DSCT_CTL: DAINC Position      */
#define PDMA_DSCT_CTL_DAINC_Msk          (0x3ul << PDMA_DSCT_CTL_DAINC_Pos)                /*!< PDMA_T::DSCT_CTL: DAINC Mask          */

#define PDMA_DSCT_CTL_TXWIDTH_Pos        (12)                                              /*!< PDMA_T::DSCT_CTL: TXWIDTH Position    */
#define PDMA_DSCT_CTL_TXWIDTH_Msk        (0x3ul << PDMA_DSCT_CTL_TXWIDTH_Pos)              /*!< PDMA_T::DSCT_CTL: TXWIDTH Mask        */

#define PDMA_DSCT_CTL_TXCNT_Pos          (16)                                              /*!< PDMA_T::DSCT_CTL: TXCNT Position      */
#define PDMA_DSCT_CTL_TXCNT_Msk          (0x3ffful << PDMA_DSCT_CTL_TXCNT_Pos)             /*!< PDMA_T::DSCT_CTL: TXCNT Mask          */

#define PDMA_DSCT_SA_SA_Pos              (0)                                               /*!< PDMA_T::DSCT_SA: SA Position          */
#define PDMA_DSCT_SA_SA_Msk              (0xfffffffful << PDMA_DSCT_SA_SA_Pos)             /*!< PDMA_T::DSCT_SA: SA Mask              */

#define PDMA_DSCT_DA_DA_Pos              (0)                                               /*!< PDMA_T::DSCT_DA: DA Position          */
#define PDMA_DSCT_DA_DA_Msk              (0xfffffffful << PDMA_DSCT_DA_DA_Pos)             /*!< PDMA_T::DSCT_DA: DA Mask              */

#define PDMA_DSCT_FIRST_FIRST_Pos        (0)                                               /*!< PDMA_T::DSCT_FIRST: FIRST Position    */
#define PDMA_DSCT_FIRST_FIRST_Msk        (0xfffful << PDMA_DSCT_FIRST_FIRST_Pos)           /*!< PDMA_T::DSCT_FIRST: FIRST Mask        */

#define PDMA_DSCT_FIRST_NEXT_Pos         (16)                                              /*!< PDMA_T::DSCT_FIRST: NEXT Position     */
#define PDMA_DSCT_FIRST_NEXT_Msk         (0xfffful << PDMA_DSCT_FIRST_NEXT_Pos)            /*!< PDMA_T::DSCT_FIRST: NEXT Mask         */

#define PDMA_CURSCAT_CURADDR_Pos         (0)                                               /*!< PDMA_T::CURSCAT: CURADDR Position     */
#define PDMA_CURSCAT_CURADDR_Msk         (0xfffffffful << PDMA_CURSCAT_CURADDR_Pos)        /*!< PDMA_T::CURSCAT: CURADDR Mask         */

#define PDMA_CHCTL_CHEN0_Pos             (0)                                               /*!< PDMA_T::CHCTL: CHEN0 Position          */
#define PDMA_CHCTL_CHEN0_Msk             (0x1ul << PDMA_CHCTL_CHEN0_Pos)                   /*!< PDMA_T::CHCTL: CHEN0 Mask              */

#define PDMA_CHCTL_CHEN1_Pos             (1)                                               /*!< PDMA_T::CHCTL: CHEN1 Position          */
#define PDMA_CHCTL_CHEN1_Msk             (0x1ul << PDMA_CHCTL_CHEN1_Pos)                   /*!< PDMA_T::CHCTL: CHEN1 Mask              */

#define PDMA_CHCTL_CHEN2_Pos             (2)                                               /*!< PDMA_T::CHCTL: CHEN2 Position          */
#define PDMA_CHCTL_CHEN2_Msk             (0x1ul << PDMA_CHCTL_CHEN2_Pos)                   /*!< PDMA_T::CHCTL: CHEN2 Mask              */

#define PDMA_CHCTL_CHEN3_Pos             (3)                                               /*!< PDMA_T::CHCTL: CHEN3 Position          */
#define PDMA_CHCTL_CHEN3_Msk             (0x1ul << PDMA_CHCTL_CHEN3_Pos)                   /*!< PDMA_T::CHCTL: CHEN3 Mask              */

#define PDMA_CHCTL_CHEN4_Pos             (4)                                               /*!< PDMA_T::CHCTL: CHEN4 Position          */
#define PDMA_CHCTL_CHEN4_Msk             (0x1ul << PDMA_CHCTL_CHEN4_Pos)                   /*!< PDMA_T::CHCTL: CHEN4 Mask              */

#define PDMA_CHCTL_CHEN5_Pos             (5)                                               /*!< PDMA_T::CHCTL: CHEN5 Position          */
#define PDMA_CHCTL_CHEN5_Msk             (0x1ul << PDMA_CHCTL_CHEN5_Pos)                   /*!< PDMA_T::CHCTL: CHEN5 Mask              */

#define PDMA_CHCTL_CHEN6_Pos             (6)                                               /*!< PDMA_T::CHCTL: CHEN6 Position          */
#define PDMA_CHCTL_CHEN6_Msk             (0x1ul << PDMA_CHCTL_CHEN6_Pos)                   /*!< PDMA_T::CHCTL: CHEN6 Mask              */

#define PDMA_CHCTL_CHEN7_Pos             (7)                                               /*!< PDMA_T::CHCTL: CHEN7 Position          */
#define PDMA_CHCTL_CHEN7_Msk             (0x1ul << PDMA_CHCTL_CHEN7_Pos)                   /*!< PDMA_T::CHCTL: CHEN7 Mask              */

#define PDMA_CHCTL_CHEN8_Pos             (8)                                               /*!< PDMA_T::CHCTL: CHEN8 Position          */
#define PDMA_CHCTL_CHEN8_Msk             (0x1ul << PDMA_CHCTL_CHEN8_Pos)                   /*!< PDMA_T::CHCTL: CHEN8 Mask              */

#define PDMA_CHCTL_CHEN9_Pos             (9)                                               /*!< PDMA_T::CHCTL: CHEN9 Position          */
#define PDMA_CHCTL_CHEN9_Msk             (0x1ul << PDMA_CHCTL_CHEN9_Pos)                   /*!< PDMA_T::CHCTL: CHEN9 Mask              */

#define PDMA_PAUSE_PAUSE0_Pos            (0)                                               /*!< PDMA_T::PAUSE: PAUSE0 Position         */
#define PDMA_PAUSE_PAUSE0_Msk            (0x1ul << PDMA_PAUSE_PAUSE0_Pos)                  /*!< PDMA_T::PAUSE: PAUSE0 Mask             */

#define PDMA_PAUSE_PAUSE1_Pos            (1)                                               /*!< PDMA_T::PAUSE: PAUSE1 Position         */
#define PDMA_PAUSE_PAUSE1_Msk            (0x1ul << PDMA_PAUSE_PAUSE1_Pos)                  /*!< PDMA_T::PAUSE: PAUSE1 Mask             */

#define PDMA_PAUSE_PAUSE2_Pos            (2)                                               /*!< PDMA_T::PAUSE: PAUSE2 Position         */
#define PDMA_PAUSE_PAUSE2_Msk            (0x1ul << PDMA_PAUSE_PAUSE2_Pos)                  /*!< PDMA_T::PAUSE: PAUSE2 Mask             */

#define PDMA_PAUSE_PAUSE3_Pos            (3)                                               /*!< PDMA_T::PAUSE: PAUSE3 Position         */
#define PDMA_PAUSE_PAUSE3_Msk            (0x1ul << PDMA_PAUSE_PAUSE3_Pos)                  /*!< PDMA_T::PAUSE: PAUSE3 Mask             */

#define PDMA_PAUSE_PAUSE4_Pos            (4)                                               /*!< PDMA_T::PAUSE: PAUSE4 Position         */
#define PDMA_PAUSE_PAUSE4_Msk            (0x1ul << PDMA_PAUSE_PAUSE4_Pos)                  /*!< PDMA_T::PAUSE: PAUSE4 Mask             */

#define PDMA_PAUSE_PAUSE5_Pos            (5)                                               /*!< PDMA_T::PAUSE: PAUSE5 Position         */
#define PDMA_PAUSE_PAUSE5_Msk            (0x1ul << PDMA_PAUSE_PAUSE5_Pos)                  /*!< PDMA_T::PAUSE: PAUSE5 Mask             */

#define PDMA_PAUSE_PAUSE6_Pos            (6)                                               /*!< PDMA_T::PAUSE: PAUSE6 Position         */
#define PDMA_PAUSE_PAUSE6_Msk            (0x1ul << PDMA_PAUSE_PAUSE6_Pos)                  /*!< PDMA_T::PAUSE: PAUSE6 Mask             */

#define PDMA_PAUSE_PAUSE7_Pos            (7)                                               /*!< PDMA_T::PAUSE: PAUSE7 Position         */
#define PDMA_PAUSE_PAUSE7_Msk            (0x1ul << PDMA_PAUSE_PAUSE7_Pos)                  /*!< PDMA_T::PAUSE: PAUSE7 Mask             */

#define PDMA_PAUSE_PAUSE8_Pos            (8)                                               /*!< PDMA_T::PAUSE: PAUSE8 Position         */
#define PDMA_PAUSE_PAUSE8_Msk            (0x1ul << PDMA_PAUSE_PAUSE8_Pos)                  /*!< PDMA_T::PAUSE: PAUSE8 Mask             */

#define PDMA_PAUSE_PAUSE9_Pos            (9)                                               /*!< PDMA_T::PAUSE: PAUSE9 Position         */
#define PDMA_PAUSE_PAUSE9_Msk            (0x1ul << PDMA_PAUSE_PAUSE9_Pos)                  /*!< PDMA_T::PAUSE: PAUSE9 Mask             */

#define PDMA_SWREQ_SWREQ0_Pos            (0)                                               /*!< PDMA_T::SWREQ: SWREQ0 Position         */
#define PDMA_SWREQ_SWREQ0_Msk            (0x1ul << PDMA_SWREQ_SWREQ0_Pos)                  /*!< PDMA_T::SWREQ: SWREQ0 Mask             */

#define PDMA_SWREQ_SWREQ1_Pos            (1)                                               /*!< PDMA_T::SWREQ: SWREQ1 Position         */
#define PDMA_SWREQ_SWREQ1_Msk            (0x1ul << PDMA_SWREQ_SWREQ1_Pos)                  /*!< PDMA_T::SWREQ: SWREQ1 Mask             */

#define PDMA_SWREQ_SWREQ2_Pos            (2)                                               /*!< PDMA_T::SWREQ: SWREQ2 Position         */
#define PDMA_SWREQ_SWREQ2_Msk            (0x1ul << PDMA_SWREQ_SWREQ2_Pos)                  /*!< PDMA_T::SWREQ: SWREQ2 Mask             */

#define PDMA_SWREQ_SWREQ3_Pos            (3)                                               /*!< PDMA_T::SWREQ: SWREQ3 Position         */
#define PDMA_SWREQ_SWREQ3_Msk            (0x1ul << PDMA_SWREQ_SWREQ3_Pos)                  /*!< PDMA_T::SWREQ: SWREQ3 Mask             */

#define PDMA_SWREQ_SWREQ4_Pos            (4)                                               /*!< PDMA_T::SWREQ: SWREQ4 Position         */
#define PDMA_SWREQ_SWREQ4_Msk            (0x1ul << PDMA_SWREQ_SWREQ4_Pos)                  /*!< PDMA_T::SWREQ: SWREQ4 Mask             */

#define PDMA_SWREQ_SWREQ5_Pos            (5)                                               /*!< PDMA_T::SWREQ: SWREQ5 Position         */
#define PDMA_SWREQ_SWREQ5_Msk            (0x1ul << PDMA_SWREQ_SWREQ5_Pos)                  /*!< PDMA_T::SWREQ: SWREQ5 Mask             */

#define PDMA_SWREQ_SWREQ6_Pos            (6)                                               /*!< PDMA_T::SWREQ: SWREQ6 Position         */
#define PDMA_SWREQ_SWREQ6_Msk            (0x1ul << PDMA_SWREQ_SWREQ6_Pos)                  /*!< PDMA_T::SWREQ: SWREQ6 Mask             */

#define PDMA_SWREQ_SWREQ7_Pos            (7)                                               /*!< PDMA_T::SWREQ: SWREQ7 Position         */
#define PDMA_SWREQ_SWREQ7_Msk            (0x1ul << PDMA_SWREQ_SWREQ7_Pos)                  /*!< PDMA_T::SWREQ: SWREQ7 Mask             */

#define PDMA_SWREQ_SWREQ8_Pos            (8)                                               /*!< PDMA_T::SWREQ: SWREQ8 Position         */
#define PDMA_SWREQ_SWREQ8_Msk            (0x1ul << PDMA_SWREQ_SWREQ8_Pos)                  /*!< PDMA_T::SWREQ: SWREQ8 Mask             */

#define PDMA_SWREQ_SWREQ9_Pos            (9)                                               /*!< PDMA_T::SWREQ: SWREQ9 Position         */
#define PDMA_SWREQ_SWREQ9_Msk            (0x1ul << PDMA_SWREQ_SWREQ9_Pos)                  /*!< PDMA_T::SWREQ: SWREQ9 Mask             */

#define PDMA_TRGSTS_REQSTS0_Pos          (0)                                               /*!< PDMA_T::TRGSTS: REQSTS0 Position       */
#define PDMA_TRGSTS_REQSTS0_Msk          (0x1ul << PDMA_TRGSTS_REQSTS0_Pos)                /*!< PDMA_T::TRGSTS: REQSTS0 Mask           */

#define PDMA_TRGSTS_REQSTS1_Pos          (1)                                               /*!< PDMA_T::TRGSTS: REQSTS1 Position       */
#define PDMA_TRGSTS_REQSTS1_Msk          (0x1ul << PDMA_TRGSTS_REQSTS1_Pos)                /*!< PDMA_T::TRGSTS: REQSTS1 Mask           */

#define PDMA_TRGSTS_REQSTS2_Pos          (2)                                               /*!< PDMA_T::TRGSTS: REQSTS2 Position       */
#define PDMA_TRGSTS_REQSTS2_Msk          (0x1ul << PDMA_TRGSTS_REQSTS2_Pos)                /*!< PDMA_T::TRGSTS: REQSTS2 Mask           */

#define PDMA_TRGSTS_REQSTS3_Pos          (3)                                               /*!< PDMA_T::TRGSTS: REQSTS3 Position       */
#define PDMA_TRGSTS_REQSTS3_Msk          (0x1ul << PDMA_TRGSTS_REQSTS3_Pos)                /*!< PDMA_T::TRGSTS: REQSTS3 Mask           */

#define PDMA_TRGSTS_REQSTS4_Pos          (4)                                               /*!< PDMA_T::TRGSTS: REQSTS4 Position       */
#define PDMA_TRGSTS_REQSTS4_Msk          (0x1ul << PDMA_TRGSTS_REQSTS4_Pos)                /*!< PDMA_T::TRGSTS: REQSTS4 Mask           */

#define PDMA_TRGSTS_REQSTS5_Pos          (5)                                               /*!< PDMA_T::TRGSTS: REQSTS5 Position       */
#define PDMA_TRGSTS_REQSTS5_Msk          (0x1ul << PDMA_TRGSTS_REQSTS5_Pos)                /*!< PDMA_T::TRGSTS: REQSTS5 Mask           */

#define PDMA_TRGSTS_REQSTS6_Pos          (6)                                               /*!< PDMA_T::TRGSTS: REQSTS6 Position       */
#define PDMA_TRGSTS_REQSTS6_Msk          (0x1ul << PDMA_TRGSTS_REQSTS6_Pos)                /*!< PDMA_T::TRGSTS: REQSTS6 Mask           */

#define PDMA_TRGSTS_REQSTS7_Pos          (7)                                               /*!< PDMA_T::TRGSTS: REQSTS7 Position       */
#define PDMA_TRGSTS_REQSTS7_Msk          (0x1ul << PDMA_TRGSTS_REQSTS7_Pos)                /*!< PDMA_T::TRGSTS: REQSTS7 Mask           */

#define PDMA_TRGSTS_REQSTS8_Pos          (8)                                               /*!< PDMA_T::TRGSTS: REQSTS8 Position       */
#define PDMA_TRGSTS_REQSTS8_Msk          (0x1ul << PDMA_TRGSTS_REQSTS8_Pos)                /*!< PDMA_T::TRGSTS: REQSTS8 Mask           */

#define PDMA_TRGSTS_REQSTS9_Pos          (9)                                               /*!< PDMA_T::TRGSTS: REQSTS9 Position       */
#define PDMA_TRGSTS_REQSTS9_Msk          (0x1ul << PDMA_TRGSTS_REQSTS9_Pos)                /*!< PDMA_T::TRGSTS: REQSTS9 Mask           */

#define PDMA_PRISET_FPRISET0_Pos         (0)                                               /*!< PDMA_T::PRISET: FPRISET0 Position      */
#define PDMA_PRISET_FPRISET0_Msk         (0x1ul << PDMA_PRISET_FPRISET0_Pos)               /*!< PDMA_T::PRISET: FPRISET0 Mask          */

#define PDMA_PRISET_FPRISET1_Pos         (1)                                               /*!< PDMA_T::PRISET: FPRISET1 Position      */
#define PDMA_PRISET_FPRISET1_Msk         (0x1ul << PDMA_PRISET_FPRISET1_Pos)               /*!< PDMA_T::PRISET: FPRISET1 Mask          */

#define PDMA_PRISET_FPRISET2_Pos         (2)                                               /*!< PDMA_T::PRISET: FPRISET2 Position      */
#define PDMA_PRISET_FPRISET2_Msk         (0x1ul << PDMA_PRISET_FPRISET2_Pos)               /*!< PDMA_T::PRISET: FPRISET2 Mask          */

#define PDMA_PRISET_FPRISET3_Pos         (3)                                               /*!< PDMA_T::PRISET: FPRISET3 Position      */
#define PDMA_PRISET_FPRISET3_Msk         (0x1ul << PDMA_PRISET_FPRISET3_Pos)               /*!< PDMA_T::PRISET: FPRISET3 Mask          */

#define PDMA_PRISET_FPRISET4_Pos         (4)                                               /*!< PDMA_T::PRISET: FPRISET4 Position      */
#define PDMA_PRISET_FPRISET4_Msk         (0x1ul << PDMA_PRISET_FPRISET4_Pos)               /*!< PDMA_T::PRISET: FPRISET4 Mask          */

#define PDMA_PRISET_FPRISET5_Pos         (5)                                               /*!< PDMA_T::PRISET: FPRISET5 Position      */
#define PDMA_PRISET_FPRISET5_Msk         (0x1ul << PDMA_PRISET_FPRISET5_Pos)               /*!< PDMA_T::PRISET: FPRISET5 Mask          */

#define PDMA_PRISET_FPRISET6_Pos         (6)                                               /*!< PDMA_T::PRISET: FPRISET6 Position      */
#define PDMA_PRISET_FPRISET6_Msk         (0x1ul << PDMA_PRISET_FPRISET6_Pos)               /*!< PDMA_T::PRISET: FPRISET6 Mask          */

#define PDMA_PRISET_FPRISET7_Pos         (7)                                               /*!< PDMA_T::PRISET: FPRISET7 Position      */
#define PDMA_PRISET_FPRISET7_Msk         (0x1ul << PDMA_PRISET_FPRISET7_Pos)               /*!< PDMA_T::PRISET: FPRISET7 Mask          */

#define PDMA_PRISET_FPRISET8_Pos         (8)                                               /*!< PDMA_T::PRISET: FPRISET8 Position      */
#define PDMA_PRISET_FPRISET8_Msk         (0x1ul << PDMA_PRISET_FPRISET8_Pos)               /*!< PDMA_T::PRISET: FPRISET8 Mask          */

#define PDMA_PRISET_FPRISET9_Pos         (9)                                               /*!< PDMA_T::PRISET: FPRISET9 Position      */
#define PDMA_PRISET_FPRISET9_Msk         (0x1ul << PDMA_PRISET_FPRISET9_Pos)               /*!< PDMA_T::PRISET: FPRISET9 Mask          */

#define PDMA_PRICLR_FPRICLR0_Pos         (0)                                               /*!< PDMA_T::PRICLR: FPRICLR0 Position      */
#define PDMA_PRICLR_FPRICLR0_Msk         (0x1ul << PDMA_PRICLR_FPRICLR0_Pos)               /*!< PDMA_T::PRICLR: FPRICLR0 Mask          */

#define PDMA_PRICLR_FPRICLR1_Pos         (1)                                               /*!< PDMA_T::PRICLR: FPRICLR1 Position      */
#define PDMA_PRICLR_FPRICLR1_Msk         (0x1ul << PDMA_PRICLR_FPRICLR1_Pos)               /*!< PDMA_T::PRICLR: FPRICLR1 Mask          */

#define PDMA_PRICLR_FPRICLR2_Pos         (2)                                               /*!< PDMA_T::PRICLR: FPRICLR2 Position      */
#define PDMA_PRICLR_FPRICLR2_Msk         (0x1ul << PDMA_PRICLR_FPRICLR2_Pos)               /*!< PDMA_T::PRICLR: FPRICLR2 Mask          */

#define PDMA_PRICLR_FPRICLR3_Pos         (3)                                               /*!< PDMA_T::PRICLR: FPRICLR3 Position      */
#define PDMA_PRICLR_FPRICLR3_Msk         (0x1ul << PDMA_PRICLR_FPRICLR3_Pos)               /*!< PDMA_T::PRICLR: FPRICLR3 Mask          */

#define PDMA_PRICLR_FPRICLR4_Pos         (4)                                               /*!< PDMA_T::PRICLR: FPRICLR4 Position      */
#define PDMA_PRICLR_FPRICLR4_Msk         (0x1ul << PDMA_PRICLR_FPRICLR4_Pos)               /*!< PDMA_T::PRICLR: FPRICLR4 Mask          */

#define PDMA_PRICLR_FPRICLR5_Pos         (5)                                               /*!< PDMA_T::PRICLR: FPRICLR5 Position      */
#define PDMA_PRICLR_FPRICLR5_Msk         (0x1ul << PDMA_PRICLR_FPRICLR5_Pos)               /*!< PDMA_T::PRICLR: FPRICLR5 Mask          */

#define PDMA_PRICLR_FPRICLR6_Pos         (6)                                               /*!< PDMA_T::PRICLR: FPRICLR6 Position      */
#define PDMA_PRICLR_FPRICLR6_Msk         (0x1ul << PDMA_PRICLR_FPRICLR6_Pos)               /*!< PDMA_T::PRICLR: FPRICLR6 Mask          */

#define PDMA_PRICLR_FPRICLR7_Pos         (7)                                               /*!< PDMA_T::PRICLR: FPRICLR7 Position      */
#define PDMA_PRICLR_FPRICLR7_Msk         (0x1ul << PDMA_PRICLR_FPRICLR7_Pos)               /*!< PDMA_T::PRICLR: FPRICLR7 Mask          */

#define PDMA_PRICLR_FPRICLR8_Pos         (8)                                               /*!< PDMA_T::PRICLR: FPRICLR8 Position      */
#define PDMA_PRICLR_FPRICLR8_Msk         (0x1ul << PDMA_PRICLR_FPRICLR8_Pos)               /*!< PDMA_T::PRICLR: FPRICLR8 Mask          */

#define PDMA_PRICLR_FPRICLR9_Pos         (9)                                               /*!< PDMA_T::PRICLR: FPRICLR9 Position      */
#define PDMA_PRICLR_FPRICLR9_Msk         (0x1ul << PDMA_PRICLR_FPRICLR9_Pos)               /*!< PDMA_T::PRICLR: FPRICLR9 Mask          */

#define PDMA_INTEN_INTEN0_Pos            (0)                                               /*!< PDMA_T::INTEN: INTEN0 Position         */
#define PDMA_INTEN_INTEN0_Msk            (0x1ul << PDMA_INTEN_INTEN0_Pos)                  /*!< PDMA_T::INTEN: INTEN0 Mask             */

#define PDMA_INTEN_INTEN1_Pos            (1)                                               /*!< PDMA_T::INTEN: INTEN1 Position         */
#define PDMA_INTEN_INTEN1_Msk            (0x1ul << PDMA_INTEN_INTEN1_Pos)                  /*!< PDMA_T::INTEN: INTEN1 Mask             */

#define PDMA_INTEN_INTEN2_Pos            (2)                                               /*!< PDMA_T::INTEN: INTEN2 Position         */
#define PDMA_INTEN_INTEN2_Msk            (0x1ul << PDMA_INTEN_INTEN2_Pos)                  /*!< PDMA_T::INTEN: INTEN2 Mask             */

#define PDMA_INTEN_INTEN3_Pos            (3)                                               /*!< PDMA_T::INTEN: INTEN3 Position         */
#define PDMA_INTEN_INTEN3_Msk            (0x1ul << PDMA_INTEN_INTEN3_Pos)                  /*!< PDMA_T::INTEN: INTEN3 Mask             */

#define PDMA_INTEN_INTEN4_Pos            (4)                                               /*!< PDMA_T::INTEN: INTEN4 Position         */
#define PDMA_INTEN_INTEN4_Msk            (0x1ul << PDMA_INTEN_INTEN4_Pos)                  /*!< PDMA_T::INTEN: INTEN4 Mask             */

#define PDMA_INTEN_INTEN5_Pos            (5)                                               /*!< PDMA_T::INTEN: INTEN5 Position         */
#define PDMA_INTEN_INTEN5_Msk            (0x1ul << PDMA_INTEN_INTEN5_Pos)                  /*!< PDMA_T::INTEN: INTEN5 Mask             */

#define PDMA_INTEN_INTEN6_Pos            (6)                                               /*!< PDMA_T::INTEN: INTEN6 Position         */
#define PDMA_INTEN_INTEN6_Msk            (0x1ul << PDMA_INTEN_INTEN6_Pos)                  /*!< PDMA_T::INTEN: INTEN6 Mask             */

#define PDMA_INTEN_INTEN7_Pos            (7)                                               /*!< PDMA_T::INTEN: INTEN7 Position         */
#define PDMA_INTEN_INTEN7_Msk            (0x1ul << PDMA_INTEN_INTEN7_Pos)                  /*!< PDMA_T::INTEN: INTEN7 Mask             */

#define PDMA_INTEN_INTEN8_Pos            (8)                                               /*!< PDMA_T::INTEN: INTEN8 Position         */
#define PDMA_INTEN_INTEN8_Msk            (0x1ul << PDMA_INTEN_INTEN8_Pos)                  /*!< PDMA_T::INTEN: INTEN8 Mask             */

#define PDMA_INTEN_INTEN9_Pos            (9)                                               /*!< PDMA_T::INTEN: INTEN9 Position         */
#define PDMA_INTEN_INTEN9_Msk            (0x1ul << PDMA_INTEN_INTEN9_Pos)                  /*!< PDMA_T::INTEN: INTEN9 Mask             */

#define PDMA_INTSTS_ABTIF_Pos            (0)                                               /*!< PDMA_T::INTSTS: ABTIF Position         */
#define PDMA_INTSTS_ABTIF_Msk            (0x1ul << PDMA_INTSTS_ABTIF_Pos)                  /*!< PDMA_T::INTSTS: ABTIF Mask             */

#define PDMA_INTSTS_TDIF_Pos             (1)                                               /*!< PDMA_T::INTSTS: TDIF Position          */
#define PDMA_INTSTS_TDIF_Msk             (0x1ul << PDMA_INTSTS_TDIF_Pos)                   /*!< PDMA_T::INTSTS: TDIF Mask              */

#define PDMA_INTSTS_TEIF_Pos             (2)                                               /*!< PDMA_T::INTSTS: TEIF Position          */
#define PDMA_INTSTS_TEIF_Msk             (0x1ul << PDMA_INTSTS_TEIF_Pos)                   /*!< PDMA_T::INTSTS: TEIF Mask              */

#define PDMA_INTSTS_REQTOF0_Pos          (8)                                               /*!< PDMA_T::INTSTS: REQTOF0 Position       */
#define PDMA_INTSTS_REQTOF0_Msk          (0x1ul << PDMA_INTSTS_REQTOF0_Pos)                /*!< PDMA_T::INTSTS: REQTOF0 Mask           */

#define PDMA_INTSTS_REQTOF1_Pos          (9)                                               /*!< PDMA_T::INTSTS: REQTOF1 Position       */
#define PDMA_INTSTS_REQTOF1_Msk          (0x1ul << PDMA_INTSTS_REQTOF1_Pos)                /*!< PDMA_T::INTSTS: REQTOF1 Mask           */

#define PDMA_ABTSTS_ABTIF0_Pos           (0)                                               /*!< PDMA_T::ABTSTS: ABTIF0 Position        */
#define PDMA_ABTSTS_ABTIF0_Msk           (0x1ul << PDMA_ABTSTS_ABTIF0_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF0 Mask            */

#define PDMA_ABTSTS_ABTIF1_Pos           (1)                                               /*!< PDMA_T::ABTSTS: ABTIF1 Position        */
#define PDMA_ABTSTS_ABTIF1_Msk           (0x1ul << PDMA_ABTSTS_ABTIF1_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF1 Mask            */

#define PDMA_ABTSTS_ABTIF2_Pos           (2)                                               /*!< PDMA_T::ABTSTS: ABTIF2 Position        */
#define PDMA_ABTSTS_ABTIF2_Msk           (0x1ul << PDMA_ABTSTS_ABTIF2_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF2 Mask            */

#define PDMA_ABTSTS_ABTIF3_Pos           (3)                                               /*!< PDMA_T::ABTSTS: ABTIF3 Position        */
#define PDMA_ABTSTS_ABTIF3_Msk           (0x1ul << PDMA_ABTSTS_ABTIF3_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF3 Mask            */

#define PDMA_ABTSTS_ABTIF4_Pos           (4)                                               /*!< PDMA_T::ABTSTS: ABTIF4 Position        */
#define PDMA_ABTSTS_ABTIF4_Msk           (0x1ul << PDMA_ABTSTS_ABTIF4_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF4 Mask            */

#define PDMA_ABTSTS_ABTIF5_Pos           (5)                                               /*!< PDMA_T::ABTSTS: ABTIF5 Position        */
#define PDMA_ABTSTS_ABTIF5_Msk           (0x1ul << PDMA_ABTSTS_ABTIF5_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF5 Mask            */

#define PDMA_ABTSTS_ABTIF6_Pos           (6)                                               /*!< PDMA_T::ABTSTS: ABTIF6 Position        */
#define PDMA_ABTSTS_ABTIF6_Msk           (0x1ul << PDMA_ABTSTS_ABTIF6_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF6 Mask            */

#define PDMA_ABTSTS_ABTIF7_Pos           (7)                                               /*!< PDMA_T::ABTSTS: ABTIF7 Position        */
#define PDMA_ABTSTS_ABTIF7_Msk           (0x1ul << PDMA_ABTSTS_ABTIF7_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF7 Mask            */

#define PDMA_ABTSTS_ABTIF8_Pos           (8)                                               /*!< PDMA_T::ABTSTS: ABTIF8 Position        */
#define PDMA_ABTSTS_ABTIF8_Msk           (0x1ul << PDMA_ABTSTS_ABTIF8_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF8 Mask            */

#define PDMA_ABTSTS_ABTIF9_Pos           (9)                                               /*!< PDMA_T::ABTSTS: ABTIF9 Position        */
#define PDMA_ABTSTS_ABTIF9_Msk           (0x1ul << PDMA_ABTSTS_ABTIF9_Pos)                 /*!< PDMA_T::ABTSTS: ABTIF9 Mask            */

#define PDMA_TDSTS_TDIF0_Pos             (0)                                               /*!< PDMA_T::TDSTS: TDIF0 Position          */
#define PDMA_TDSTS_TDIF0_Msk             (0x1ul << PDMA_TDSTS_TDIF0_Pos)                   /*!< PDMA_T::TDSTS: TDIF0 Mask              */

#define PDMA_TDSTS_TDIF1_Pos             (1)                                               /*!< PDMA_T::TDSTS: TDIF1 Position          */
#define PDMA_TDSTS_TDIF1_Msk             (0x1ul << PDMA_TDSTS_TDIF1_Pos)                   /*!< PDMA_T::TDSTS: TDIF1 Mask              */

#define PDMA_TDSTS_TDIF2_Pos             (2)                                               /*!< PDMA_T::TDSTS: TDIF2 Position          */
#define PDMA_TDSTS_TDIF2_Msk             (0x1ul << PDMA_TDSTS_TDIF2_Pos)                   /*!< PDMA_T::TDSTS: TDIF2 Mask              */

#define PDMA_TDSTS_TDIF3_Pos             (3)                                               /*!< PDMA_T::TDSTS: TDIF3 Position          */
#define PDMA_TDSTS_TDIF3_Msk             (0x1ul << PDMA_TDSTS_TDIF3_Pos)                   /*!< PDMA_T::TDSTS: TDIF3 Mask              */

#define PDMA_TDSTS_TDIF4_Pos             (4)                                               /*!< PDMA_T::TDSTS: TDIF4 Position          */
#define PDMA_TDSTS_TDIF4_Msk             (0x1ul << PDMA_TDSTS_TDIF4_Pos)                   /*!< PDMA_T::TDSTS: TDIF4 Mask              */

#define PDMA_TDSTS_TDIF5_Pos             (5)                                               /*!< PDMA_T::TDSTS: TDIF5 Position          */
#define PDMA_TDSTS_TDIF5_Msk             (0x1ul << PDMA_TDSTS_TDIF5_Pos)                   /*!< PDMA_T::TDSTS: TDIF5 Mask              */

#define PDMA_TDSTS_TDIF6_Pos             (6)                                               /*!< PDMA_T::TDSTS: TDIF6 Position          */
#define PDMA_TDSTS_TDIF6_Msk             (0x1ul << PDMA_TDSTS_TDIF6_Pos)                   /*!< PDMA_T::TDSTS: TDIF6 Mask              */

#define PDMA_TDSTS_TDIF7_Pos             (7)                                               /*!< PDMA_T::TDSTS: TDIF7 Position          */
#define PDMA_TDSTS_TDIF7_Msk             (0x1ul << PDMA_TDSTS_TDIF7_Pos)                   /*!< PDMA_T::TDSTS: TDIF7 Mask              */

#define PDMA_TDSTS_TDIF8_Pos             (8)                                               /*!< PDMA_T::TDSTS: TDIF8 Position          */
#define PDMA_TDSTS_TDIF8_Msk             (0x1ul << PDMA_TDSTS_TDIF8_Pos)                   /*!< PDMA_T::TDSTS: TDIF8 Mask              */

#define PDMA_TDSTS_TDIF9_Pos             (9)                                               /*!< PDMA_T::TDSTS: TDIF9 Position          */
#define PDMA_TDSTS_TDIF9_Msk             (0x1ul << PDMA_TDSTS_TDIF9_Pos)                   /*!< PDMA_T::TDSTS: TDIF9 Mask              */

#define PDMA_SCATSTS_TEMPTYF0_Pos        (0)                                               /*!< PDMA_T::SCATSTS: TEMPTYF0 Position     */
#define PDMA_SCATSTS_TEMPTYF0_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF0_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF0 Mask         */

#define PDMA_SCATSTS_TEMPTYF1_Pos        (1)                                               /*!< PDMA_T::SCATSTS: TEMPTYF1 Position     */
#define PDMA_SCATSTS_TEMPTYF1_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF1_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF1 Mask         */

#define PDMA_SCATSTS_TEMPTYF2_Pos        (2)                                               /*!< PDMA_T::SCATSTS: TEMPTYF2 Position     */
#define PDMA_SCATSTS_TEMPTYF2_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF2_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF2 Mask         */

#define PDMA_SCATSTS_TEMPTYF3_Pos        (3)                                               /*!< PDMA_T::SCATSTS: TEMPTYF3 Position     */
#define PDMA_SCATSTS_TEMPTYF3_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF3_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF3 Mask         */

#define PDMA_SCATSTS_TEMPTYF4_Pos        (4)                                               /*!< PDMA_T::SCATSTS: TEMPTYF4 Position     */
#define PDMA_SCATSTS_TEMPTYF4_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF4_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF4 Mask         */

#define PDMA_SCATSTS_TEMPTYF5_Pos        (5)                                               /*!< PDMA_T::SCATSTS: TEMPTYF5 Position     */
#define PDMA_SCATSTS_TEMPTYF5_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF5_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF5 Mask         */

#define PDMA_SCATSTS_TEMPTYF6_Pos        (6)                                               /*!< PDMA_T::SCATSTS: TEMPTYF6 Position     */
#define PDMA_SCATSTS_TEMPTYF6_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF6_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF6 Mask         */

#define PDMA_SCATSTS_TEMPTYF7_Pos        (7)                                               /*!< PDMA_T::SCATSTS: TEMPTYF7 Position     */
#define PDMA_SCATSTS_TEMPTYF7_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF7_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF7 Mask         */

#define PDMA_SCATSTS_TEMPTYF8_Pos        (8)                                               /*!< PDMA_T::SCATSTS: TEMPTYF8 Position     */
#define PDMA_SCATSTS_TEMPTYF8_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF8_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF8 Mask         */

#define PDMA_SCATSTS_TEMPTYF9_Pos        (9)                                               /*!< PDMA_T::SCATSTS: TEMPTYF9 Position     */
#define PDMA_SCATSTS_TEMPTYF9_Msk        (0x1ul << PDMA_SCATSTS_TEMPTYF9_Pos)              /*!< PDMA_T::SCATSTS: TEMPTYF9 Mask         */

#define PDMA_TACTSTS_TXACTF0_Pos         (0)                                               /*!< PDMA_T::TACTSTS: TXACTF0 Position      */
#define PDMA_TACTSTS_TXACTF0_Msk         (0x1ul << PDMA_TACTSTS_TXACTF0_Pos)               /*!< PDMA_T::TACTSTS: TXACTF0 Mask          */

#define PDMA_TACTSTS_TXACTF1_Pos         (1)                                               /*!< PDMA_T::TACTSTS: TXACTF1 Position      */
#define PDMA_TACTSTS_TXACTF1_Msk         (0x1ul << PDMA_TACTSTS_TXACTF1_Pos)               /*!< PDMA_T::TACTSTS: TXACTF1 Mask          */

#define PDMA_TACTSTS_TXACTF2_Pos         (2)                                               /*!< PDMA_T::TACTSTS: TXACTF2 Position      */
#define PDMA_TACTSTS_TXACTF2_Msk         (0x1ul << PDMA_TACTSTS_TXACTF2_Pos)               /*!< PDMA_T::TACTSTS: TXACTF2 Mask          */

#define PDMA_TACTSTS_TXACTF3_Pos         (3)                                               /*!< PDMA_T::TACTSTS: TXACTF3 Position      */
#define PDMA_TACTSTS_TXACTF3_Msk         (0x1ul << PDMA_TACTSTS_TXACTF3_Pos)               /*!< PDMA_T::TACTSTS: TXACTF3 Mask          */

#define PDMA_TACTSTS_TXACTF4_Pos         (4)                                               /*!< PDMA_T::TACTSTS: TXACTF4 Position      */
#define PDMA_TACTSTS_TXACTF4_Msk         (0x1ul << PDMA_TACTSTS_TXACTF4_Pos)               /*!< PDMA_T::TACTSTS: TXACTF4 Mask          */

#define PDMA_TACTSTS_TXACTF5_Pos         (5)                                               /*!< PDMA_T::TACTSTS: TXACTF5 Position      */
#define PDMA_TACTSTS_TXACTF5_Msk         (0x1ul << PDMA_TACTSTS_TXACTF5_Pos)               /*!< PDMA_T::TACTSTS: TXACTF5 Mask          */

#define PDMA_TACTSTS_TXACTF6_Pos         (6)                                               /*!< PDMA_T::TACTSTS: TXACTF6 Position      */
#define PDMA_TACTSTS_TXACTF6_Msk         (0x1ul << PDMA_TACTSTS_TXACTF6_Pos)               /*!< PDMA_T::TACTSTS: TXACTF6 Mask          */

#define PDMA_TACTSTS_TXACTF7_Pos         (7)                                               /*!< PDMA_T::TACTSTS: TXACTF7 Position      */
#define PDMA_TACTSTS_TXACTF7_Msk         (0x1ul << PDMA_TACTSTS_TXACTF7_Pos)               /*!< PDMA_T::TACTSTS: TXACTF7 Mask          */

#define PDMA_TACTSTS_TXACTF8_Pos         (8)                                               /*!< PDMA_T::TACTSTS: TXACTF8 Position      */
#define PDMA_TACTSTS_TXACTF8_Msk         (0x1ul << PDMA_TACTSTS_TXACTF8_Pos)               /*!< PDMA_T::TACTSTS: TXACTF8 Mask          */

#define PDMA_TACTSTS_TXACTF9_Pos         (9)                                               /*!< PDMA_T::TACTSTS: TXACTF9 Position      */
#define PDMA_TACTSTS_TXACTF9_Msk         (0x1ul << PDMA_TACTSTS_TXACTF9_Pos)               /*!< PDMA_T::TACTSTS: TXACTF9 Mask          */

#define PDMA_TOUTPSC_TOUTPSC0_Pos        (0)                                               /*!< PDMA_T::TOUTPSC: TOUTPSC0 Position     */
#define PDMA_TOUTPSC_TOUTPSC0_Msk        (0x7ul << PDMA_TOUTPSC_TOUTPSC0_Pos)              /*!< PDMA_T::TOUTPSC: TOUTPSC0 Mask         */

#define PDMA_TOUTPSC_TOUTPSC1_Pos        (4)                                               /*!< PDMA_T::TOUTPSC: TOUTPSC1 Position     */
#define PDMA_TOUTPSC_TOUTPSC1_Msk        (0x7ul << PDMA_TOUTPSC_TOUTPSC1_Pos)              /*!< PDMA_T::TOUTPSC: TOUTPSC1 Mask         */

#define PDMA_TOUTEN_TOUTEN0_Pos          (0)                                               /*!< PDMA_T::TOUTEN: TOUTEN0 Position       */
#define PDMA_TOUTEN_TOUTEN0_Msk          (0x1ul << PDMA_TOUTEN_TOUTEN0_Pos)                /*!< PDMA_T::TOUTEN: TOUTEN0 Mask           */

#define PDMA_TOUTEN_TOUTEN1_Pos          (1)                                               /*!< PDMA_T::TOUTEN: TOUTEN1 Position       */
#define PDMA_TOUTEN_TOUTEN1_Msk          (0x1ul << PDMA_TOUTEN_TOUTEN1_Pos)                /*!< PDMA_T::TOUTEN: TOUTEN1 Mask           */

#define PDMA_TOUTIEN_TOUTIEN0_Pos        (0)                                               /*!< PDMA_T::TOUTIEN: TOUTIEN0 Position     */
#define PDMA_TOUTIEN_TOUTIEN0_Msk        (0x1ul << PDMA_TOUTIEN_TOUTIEN0_Pos)              /*!< PDMA_T::TOUTIEN: TOUTIEN0 Mask         */

#define PDMA_TOUTIEN_TOUTIEN1_Pos        (1)                                               /*!< PDMA_T::TOUTIEN: TOUTIEN1 Position     */
#define PDMA_TOUTIEN_TOUTIEN1_Msk        (0x1ul << PDMA_TOUTIEN_TOUTIEN1_Pos)              /*!< PDMA_T::TOUTIEN: TOUTIEN1 Mask         */

#define PDMA_SCATBA_SCATBA_Pos           (16)                                              /*!< PDMA_T::SCATBA: SCATBA Position        */
#define PDMA_SCATBA_SCATBA_Msk           (0xfffful << PDMA_SCATBA_SCATBA_Pos)              /*!< PDMA_T::SCATBA: SCATBA Mask            */

#define PDMA_TOC0_1_TOC0_Pos             (0)                                               /*!< PDMA_T::TOC0_1: TOC0 Position          */
#define PDMA_TOC0_1_TOC0_Msk             (0xfffful << PDMA_TOC0_1_TOC0_Pos)                /*!< PDMA_T::TOC0_1: TOC0 Mask              */

#define PDMA_TOC0_1_TOC1_Pos             (16)                                              /*!< PDMA_T::TOC0_1: TOC1 Position          */
#define PDMA_TOC0_1_TOC1_Msk             (0xfffful << PDMA_TOC0_1_TOC1_Pos)                /*!< PDMA_T::TOC0_1: TOC1 Mask              */

#define PDMA_RESET_RESET0_Pos            (0)                                               /*!< PDMA_T::RESET: RESET0 Position         */
#define PDMA_RESET_RESET0_Msk            (0x1ul << PDMA_RESET_RESET0_Pos)                  /*!< PDMA_T::RESET: RESET0 Mask             */

#define PDMA_RESET_RESET1_Pos            (1)                                               /*!< PDMA_T::RESET: RESET1 Position         */
#define PDMA_RESET_RESET1_Msk            (0x1ul << PDMA_RESET_RESET1_Pos)                  /*!< PDMA_T::RESET: RESET1 Mask             */

#define PDMA_RESET_RESET2_Pos            (2)                                               /*!< PDMA_T::RESET: RESET2 Position         */
#define PDMA_RESET_RESET2_Msk            (0x1ul << PDMA_RESET_RESET2_Pos)                  /*!< PDMA_T::RESET: RESET2 Mask             */

#define PDMA_RESET_RESET3_Pos            (3)                                               /*!< PDMA_T::RESET: RESET3 Position         */
#define PDMA_RESET_RESET3_Msk            (0x1ul << PDMA_RESET_RESET3_Pos)                  /*!< PDMA_T::RESET: RESET3 Mask             */

#define PDMA_RESET_RESET4_Pos            (4)                                               /*!< PDMA_T::RESET: RESET4 Position         */
#define PDMA_RESET_RESET4_Msk            (0x1ul << PDMA_RESET_RESET4_Pos)                  /*!< PDMA_T::RESET: RESET4 Mask             */

#define PDMA_RESET_RESET5_Pos            (5)                                               /*!< PDMA_T::RESET: RESET5 Position         */
#define PDMA_RESET_RESET5_Msk            (0x1ul << PDMA_RESET_RESET5_Pos)                  /*!< PDMA_T::RESET: RESET5 Mask             */

#define PDMA_RESET_RESET6_Pos            (6)                                               /*!< PDMA_T::RESET: RESET6 Position         */
#define PDMA_RESET_RESET6_Msk            (0x1ul << PDMA_RESET_RESET6_Pos)                  /*!< PDMA_T::RESET: RESET6 Mask             */

#define PDMA_RESET_RESET7_Pos            (7)                                               /*!< PDMA_T::RESET: RESET7 Position         */
#define PDMA_RESET_RESET7_Msk            (0x1ul << PDMA_RESET_RESET7_Pos)                  /*!< PDMA_T::RESET: RESET7 Mask             */

#define PDMA_RESET_RESET8_Pos            (8)                                               /*!< PDMA_T::RESET: RESET8 Position         */
#define PDMA_RESET_RESET8_Msk            (0x1ul << PDMA_RESET_RESET8_Pos)                  /*!< PDMA_T::RESET: RESET8 Mask             */

#define PDMA_RESET_RESET9_Pos            (9)                                               /*!< PDMA_T::RESET: RESET9 Position         */
#define PDMA_RESET_RESET9_Msk            (0x1ul << PDMA_RESET_RESET9_Pos)                  /*!< PDMA_T::RESET: RESET9 Mask             */

#define PDMA_REQSEL0_3_REQSRC0_Pos       (0)                                               /*!< PDMA_T::REQSEL0_3: REQSRC0 Position    */
#define PDMA_REQSEL0_3_REQSRC0_Msk       (0x3ful << PDMA_REQSEL0_3_REQSRC0_Pos)            /*!< PDMA_T::REQSEL0_3: REQSRC0 Mask        */

#define PDMA_REQSEL0_3_REQSRC1_Pos       (8)                                               /*!< PDMA_T::REQSEL0_3: REQSRC1 Position    */
#define PDMA_REQSEL0_3_REQSRC1_Msk       (0x3ful << PDMA_REQSEL0_3_REQSRC1_Pos)            /*!< PDMA_T::REQSEL0_3: REQSRC1 Mask        */

#define PDMA_REQSEL0_3_REQSRC2_Pos       (16)                                              /*!< PDMA_T::REQSEL0_3: REQSRC2 Position    */
#define PDMA_REQSEL0_3_REQSRC2_Msk       (0x3ful << PDMA_REQSEL0_3_REQSRC2_Pos)            /*!< PDMA_T::REQSEL0_3: REQSRC2 Mask        */

#define PDMA_REQSEL0_3_REQSRC3_Pos       (24)                                              /*!< PDMA_T::REQSEL0_3: REQSRC3 Position    */
#define PDMA_REQSEL0_3_REQSRC3_Msk       (0x3ful << PDMA_REQSEL0_3_REQSRC3_Pos)            /*!< PDMA_T::REQSEL0_3: REQSRC3 Mask        */

#define PDMA_REQSEL4_7_REQSRC4_Pos       (0)                                               /*!< PDMA_T::REQSEL4_7: REQSRC4 Position    */
#define PDMA_REQSEL4_7_REQSRC4_Msk       (0x3ful << PDMA_REQSEL4_7_REQSRC4_Pos)            /*!< PDMA_T::REQSEL4_7: REQSRC4 Mask        */

#define PDMA_REQSEL4_7_REQSRC5_Pos       (8)                                               /*!< PDMA_T::REQSEL4_7: REQSRC5 Position    */
#define PDMA_REQSEL4_7_REQSRC5_Msk       (0x3ful << PDMA_REQSEL4_7_REQSRC5_Pos)            /*!< PDMA_T::REQSEL4_7: REQSRC5 Mask        */

#define PDMA_REQSEL4_7_REQSRC6_Pos       (16)                                              /*!< PDMA_T::REQSEL4_7: REQSRC6 Position    */
#define PDMA_REQSEL4_7_REQSRC6_Msk       (0x3ful << PDMA_REQSEL4_7_REQSRC6_Pos)            /*!< PDMA_T::REQSEL4_7: REQSRC6 Mask        */

#define PDMA_REQSEL4_7_REQSRC7_Pos       (24)                                              /*!< PDMA_T::REQSEL4_7: REQSRC7 Position    */
#define PDMA_REQSEL4_7_REQSRC7_Msk       (0x3ful << PDMA_REQSEL4_7_REQSRC7_Pos)            /*!< PDMA_T::REQSEL4_7: REQSRC7 Mask        */

#define PDMA_REQSEL8_9_REQSRC8_Pos       (0)                                               /*!< PDMA_T::REQSEL8_9: REQSRC8 Position    */
#define PDMA_REQSEL8_9_REQSRC8_Msk       (0x3ful << PDMA_REQSEL8_9_REQSRC8_Pos)            /*!< PDMA_T::REQSEL8_9: REQSRC8 Mask        */

#define PDMA_REQSEL8_9_REQSRC9_Pos       (8)                                               /*!< PDMA_T::REQSEL8_9: REQSRC9 Position    */
#define PDMA_REQSEL8_9_REQSRC9_Msk       (0x3ful << PDMA_REQSEL8_9_REQSRC9_Pos)            /*!< PDMA_T::REQSEL8_9: REQSRC9 Mask        */

/**@}*/ /* PDMA_CONST */
/**@}*/ /* end of PDMA register group */


/*---------------------- Basic Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup BPWM Basic Pulse Width Modulation Controller(BPWM)
    Memory Mapped Structure for BPWM Controller
    @{ 
*/

typedef struct
{
    /**
     * @var BCAPDAT_T::RCAPDAT
     * Offset: 0x20C  BPWM Rising Capture Data Register 0~5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RCAPDAT   |BPWM Rising Capture Data (Read Only)
     * |        |          |When rising capture condition happened, the BPWM counter value will be saved in this register.
     * @var BCAPDAT_T::FCAPDAT
     * Offset: 0x210  BPWM Falling Capture Data Register 0~5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FCAPDAT   |BPWM Falling Capture Data (Read Only)
     * |        |          |When falling capture condition happened, the BPWM counter value will be saved in this register.
     */
    __IO uint32_t RCAPDAT; /*!< [0x20C/0x214/0x21C/0x224/0x22C/0x234] BPWM Rising Capture Data Register 0~5 */
    __IO uint32_t FCAPDAT; /*!< [0x210/0x218/0x220/0x228/0x230/0x238] BPWM Falling Capture Data Register 0~5 */
} BCAPDAT_T;


typedef struct
{
    /**
     * @var BPWM_T::CTL0
     * Offset: 0x00  BPWM Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CTRLD0    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[1]     |CTRLD1    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[2]     |CTRLD2    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[3]     |CTRLD3    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[4]     |CTRLD4    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[5]     |CTRLD5    |Center Re-load
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |In up-down counter type, PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the center point of a period
     * |[16]    |IMMLDEN0  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is Enabled, WINLDENn and CTRLDn will be invalid.
     * |[17]    |IMMLDEN1  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is Enabled, WINLDENn and CTRLDn will be invalid.
     * |[18]    |IMMLDEN2  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is Enabled, WINLDENn and CTRLDn will be invalid.
     * |[19]    |IMMLDEN3  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is Enabled, WINLDENn and CTRLDn will be invalid.
     * |[20]    |IMMLDEN4  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is Enabled, WINLDENn and CTRLDn will be invalid.
     * |[21]    |IMMLDEN5  |Immediately Load Enable Bit(S)
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = PERIOD will load to PBUF at the end point of each period
     * |        |          |CMPDAT will load to CMPBUF at the end point or center point of each period by setting CTRLD bit.
     * |        |          |1 = PERIOD/CMPDAT will load to PBUF and CMPBUF immediately when software update PERIOD/CMPDAT.
     * |        |          |Note: If IMMLDENn is Enabled, WINLDENn and CTRLDn will be invalid.
     * |[30]    |DBGHALT   |ICE Debug Mode Counter Halt (Write Protect)
     * |        |          |If counter halt is enabled, BPWM all counters will keep current value until exit ICE debug mode.
     * |        |          |0 = ICE debug mode counter halt Disable.
     * |        |          |1 = ICE debug mode counter halt Enable.
     * |        |          |Note: This register is write protected. Refer to SYS_REGLCTL register.
     * |[31]    |DBGTRIOFF |ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects BPWM output.
     * |        |          |BPWM pin will be forced as tri-state while ICE debug mode acknowledged.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |BPWM pin will keep output no matter ICE debug mode acknowledged or not.
     * |        |          |Note: This register is write protected. Refer to SYS_REGLCTL register.
     * @var BPWM_T::CTL1
     * Offset: 0x04  BPWM Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CNTTYPE0  |BPWM Counter Behavior Type 0
     * |        |          |Each bit n controls corresponding BPWM channel n.
     * |        |          |00 = Up counter type (supports in capture mode).
     * |        |          |01 = Down count type (supports in capture mode).
     * |        |          |10 = Up-down counter type.
     * |        |          |11 = Reserved.
     * @var BPWM_T::CLKSRC
     * Offset: 0x10  BPWM Clock Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |ECLKSRC0  |BPWM_CH01 External Clock Source Select
     * |        |          |000 = BPWMx_CLK, x denotes 0 or 1.
     * |        |          |001 = TIMER0 overflow.
     * |        |          |010 = TIMER1 overflow.
     * |        |          |011 = TIMER2 overflow.
     * |        |          |100 = TIMER3 overflow.
     * |        |          |Others = Reserved.
     * @var BPWM_T::CLKPSC
     * Offset: 0x14  BPWM Clock Prescale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |CLKPSC    |BPWM Counter Clock Prescale
     * |        |          |The clock of BPWM counter is decided by clock prescaler
     * |        |          |Each BPWM pair share one BPWM counter clock prescaler
     * |        |          |The clock of BPWM counter is divided by (CLKPSC+ 1)
     * @var BPWM_T::CNTEN
     * Offset: 0x20  BPWM Counter Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |BPWM Counter 0 Enable Bit
     * |        |          |0 = BPWM Counter and clock prescaler stop running.
     * |        |          |1 = BPWM Counter and clock prescaler start running.
     * @var BPWM_T::CNTCLR
     * Offset: 0x24  BPWM Clear Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTCLR0   |Clear BPWM Counter Control Bit 0
     * |        |          |It is automatically cleared by hardware.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear 16-bit BPWM counter to 0000H.
     * @var BPWM_T::PERIOD
     * Offset: 0x30  BPWM Period Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |BPWM Period Register
     * |        |          |Up-Count mode: In this mode, BPWM counter counts from 0 to PERIOD, and restarts from 0.
     * |        |          |Down-Count mode: In this mode, BPWM counter counts from PERIOD to 0, and restarts from PERIOD.
     * |        |          |BPWM period time = (PERIOD+1) * BPWM_CLK period.
     * |        |          |Up-Down-Count mode: In this mode, BPWM counter counts from 0 to PERIOD, then decrements to 0 and repeats again.
     * |        |          |BPWM period time = 2 * PERIOD * BPWM_CLK period.
     * @var BPWM_T::CMPDAT[6]
     * Offset: 0x50  BPWM Comparator Register 0~5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMPDAT    |BPWM Comparator Register
     * |        |          |CMPDAT use to compare with CNT to generate BPWM waveform, interrupt and trigger ADC.
     * |        |          |In independent mode, BPWM_CMPDATn, n=0,1..5 denote as 6 independent BPWM_CH0~5 compared point.
     * @var BPWM_T::CNT
     * Offset: 0x90  BPWM Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNT       |BPWM Data Register (Read Only)
     * |        |          |Monitor CNT to know the current value in 16-bit period counter.
     * |[16]    |DIRF      |BPWM Direction Indicator Flag (Read Only)
     * |        |          |0 = Counter is Down count.
     * |        |          |1 = Counter is UP count.
     * @var BPWM_T::WGCTL0
     * Offset: 0xB0  BPWM Generation Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |ZPCTL0    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[3:2]   |ZPCTL1    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[5:4]   |ZPCTL2    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[7:6]   |ZPCTL3    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[9:8]   |ZPCTL4    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[11:10] |ZPCTL5    |BPWM Zero Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM zero point output Low.
     * |        |          |10 = BPWM zero point output High.
     * |        |          |11 = BPWM zero point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to zero.
     * |[17:16] |PRDPCTL0  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[19:18] |PRDPCTL1  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[21:20] |PRDPCTL2  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[23:22] |PRDPCTL3  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[25:24] |PRDPCTL4  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * |[27:26] |PRDPCTL5  |BPWM Period (Center) Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM period (center) point output Low.
     * |        |          |10 = BPWM period (center) point output High.
     * |        |          |11 = BPWM period (center) point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter count to (PERIOD+1).
     * |        |          |Note: This bit is center point control when BPWM counter operating in up-down counter type.
     * @var BPWM_T::WGCTL1
     * Offset: 0xB4  BPWM Generation Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CMPUCTL0  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[3:2]   |CMPUCTL1  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[5:4]   |CMPUCTL2  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[7:6]   |CMPUCTL3  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[9:8]   |CMPUCTL4  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[11:10] |CMPUCTL5  |BPWM Compare Up Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare up point output Low.
     * |        |          |10 = BPWM compare up point output High.
     * |        |          |11 = BPWM compare up point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter up count to CMPDAT.
     * |[17:16] |CMPDCTL0  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[19:18] |CMPDCTL1  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[21:20] |CMPDCTL2  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[23:22] |CMPDCTL3  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[25:24] |CMPDCTL4  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * |[27:26] |CMPDCTL5  |BPWM Compare Down Point Control
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |00 = Do nothing.
     * |        |          |01 = BPWM compare down point output Low.
     * |        |          |10 = BPWM compare down point output High.
     * |        |          |11 = BPWM compare down point output Toggle.
     * |        |          |BPWM can control output level when BPWM counter down count to CMPDAT.
     * @var BPWM_T::MSKEN
     * Offset: 0xB8  BPWM Mask Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKEN0    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[1]     |MSKEN1    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[2]     |MSKEN2    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[3]     |MSKEN3    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[4]     |MSKEN4    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * |[5]     |MSKEN5    |BPWM Mask Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |The BPWM output signal will be masked when this bit is enabled
     * |        |          |The corresponding BPWM channel n will output MSKDATn (BPWM_MSK[5:0]) data.
     * |        |          |0 = BPWM output signal is non-masked.
     * |        |          |1 = BPWM output signal is masked and output MSKDATn data.
     * @var BPWM_T::MSK
     * Offset: 0xBC  BPWM Mask Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKDAT0   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[1]     |MSKDAT1   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[2]     |MSKDAT2   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[3]     |MSKDAT3   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[4]     |MSKDAT4   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * |[5]     |MSKDAT5   |BPWM Mask Data Bit
     * |        |          |This data bit control the state of BPWMn output pin, if corresponding mask function is enabled
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Output logic low to BPWMn.
     * |        |          |1 = Output logic high to BPWMn.
     * @var BPWM_T::POLCTL
     * Offset: 0xD4  BPWM Pin Polar Inverse Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PINV0     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWMx_CHn output.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn output polar inverse Disabled.
     * |        |          |1 = BPWMx_CHn output polar inverse Enabled.
     * |[1]     |PINV1     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWMx_CHn output.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn output polar inverse Disabled.
     * |        |          |1 = BPWMx_CHn output polar inverse Enabled.
     * |[2]     |PINV2     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWMx_CHn output.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn output polar inverse Disabled.
     * |        |          |1 = BPWMx_CHn output polar inverse Enabled.
     * |[3]     |PINV3     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWMx_CHn output.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn output polar inverse Disabled.
     * |        |          |1 = BPWMx_CHn output polar inverse Enabled.
     * |[4]     |PINV4     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWMx_CHn output.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn output polar inverse Disabled.
     * |        |          |1 = BPWMx_CHn output polar inverse Enabled.
     * |[5]     |PINV5     |BPWM PIN Polar Inverse Control
     * |        |          |The register controls polarity state of BPWMx_CHn output.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn output polar inverse Disabled.
     * |        |          |1 = BPWMx_CHn output polar inverse Enabled.
     * @var BPWM_T::POEN
     * Offset: 0xD8  BPWM Output Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN0     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn pin at tri-state.
     * |        |          |1 = BPWMx_CHn pin in output mode.
     * |[1]     |POEN1     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn pin at tri-state.
     * |        |          |1 = BPWMx_CHn pin in output mode.
     * |[2]     |POEN2     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn pin at tri-state.
     * |        |          |1 = BPWMx_CHn pin in output mode.
     * |[3]     |POEN3     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn pin at tri-state.
     * |        |          |1 = BPWMx_CHn pin in output mode.
     * |[4]     |POEN4     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn pin at tri-state.
     * |        |          |1 = BPWMx_CHn pin in output mode.
     * |[5]     |POEN5     |BPWM Pin Output Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWMx_CHn pin at tri-state.
     * |        |          |1 = BPWMx_CHn pin in output mode.
     * @var BPWM_T::INTEN
     * Offset: 0xE0  BPWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZIEN0     |BPWM Zero Point Interrupt 0 Enable Bit
     * |        |          |0 = Zero point interrupt Disabled.
     * |        |          |1 = Zero point interrupt Enabled.
     * |[8]     |PIEN0     |BPWM Period Point Interrupt 0 Enable Bit
     * |        |          |0 = Period point interrupt Disabled.
     * |        |          |1 = Period point interrupt Enabled.
     * |        |          |Note: When up-down counter type period point means center point.
     * |[16]    |CMPUIEN0  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * |[17]    |CMPUIEN1  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * |[18]    |CMPUIEN2  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * |[19]    |CMPUIEN3  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * |[20]    |CMPUIEN4  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * |[21]    |CMPUIEN5  |BPWM Compare Up Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * |[24]    |CMPDIEN0  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt Disabled.
     * |        |          |1 = Compare down count interrupt Enabled.
     * |[25]    |CMPDIEN1  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt Disabled.
     * |        |          |1 = Compare down count interrupt Enabled.
     * |[26]    |CMPDIEN2  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt Disabled.
     * |        |          |1 = Compare down count interrupt Enabled.
     * |[27]    |CMPDIEN3  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt Disabled.
     * |        |          |1 = Compare down count interrupt Enabled.
     * |[28]    |CMPDIEN4  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt Disabled.
     * |        |          |1 = Compare down count interrupt Enabled.
     * |[29]    |CMPDIEN5  |BPWM Compare Down Count Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Compare down count interrupt Disabled.
     * |        |          |1 = Compare down count interrupt Enabled.
     * @var BPWM_T::INTSTS
     * Offset: 0xE8  BPWM Interrupt Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZIF0      |BPWM Zero Point Interrupt Flag 0
     * |        |          |This bit is set by hardware when BPWM_CH0 counter reaches zero, software can write 1 to clear this bit to zero.
     * |[8]     |PIF0      |BPWM Period Point Interrupt Flag 0
     * |        |          |This bit is set by hardware when BPWM_CH0 counter reaches BPWM_PERIOD0, software can write 1 to clear this bit to zero.
     * |[16]    |CMPUIF0   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[17]    |CMPUIF1   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[18]    |CMPUIF2   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[19]    |CMPUIF3   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[20]    |CMPUIF4   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[21]    |CMPUIF5   |BPWM Compare Up Count Interrupt Flag
     * |        |          |Flag is set by hardware when BPWM counter up count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in up counter type selection.
     * |[24]    |CMPDIF0   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[25]    |CMPDIF1   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[26]    |CMPDIF2   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[27]    |CMPDIF3   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[28]    |CMPDIF4   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * |[29]    |CMPDIF5   |BPWM Compare Down Count Interrupt Flag
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Flag is set by hardware when BPWM counter down count and reaches BPWM_CMPDATn, software can clear this bit by writing 1 to it.
     * |        |          |Note: If CMPDAT equal to PERIOD, this flag is not working in down counter type selection.
     * @var BPWM_T::ADCTS0
     * Offset: 0xF8  BPWM Trigger ADC Source Select Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRGSEL0   |BPWM_CH0 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH0 zero point.
     * |        |          |0001 = BPWM_CH0 period point.
     * |        |          |0010 = BPWM_CH0 zero or period point.
     * |        |          |0011 = BPWM_CH0 up-count compared point.
     * |        |          |0100 = BPWM_CH0 down-count compared point.
     * |        |          |1000 = BPWM_CH1 up-count compared point.
     * |        |          |1001 = BPWM_CH1 down-count compared point.
     * |        |          |Others reserved
     * |[7]     |TRGEN0    |BPWM_CH0 Trigger ADC Enable Bit
     * |[11:8]  |TRGSEL1   |BPWM_CH1 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH0 zero point.
     * |        |          |0001 = BPWM_CH0 period point.
     * |        |          |0010 = BPWM_CH0 zero or period point.
     * |        |          |0011 = BPWM_CH0 up-count compared point.
     * |        |          |0100 = BPWM_CH0 down-count compared point.
     * |        |          |1000 = BPWM_CH1 up-count compared point.
     * |        |          |1001 = BPWM_CH1 down-count compared point.
     * |        |          |Others reserved
     * |[15]    |TRGEN1    |BPWM_CH1 Trigger ADC Enable Bit
     * |[19:16] |TRGSEL2   |BPWM_CH2 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH2 zero point.
     * |        |          |0001 = BPWM_CH2 period point.
     * |        |          |0010 = BPWM_CH2 zero or period point.
     * |        |          |0011 = BPWM_CH2 up-count compared point.
     * |        |          |0100 = BPWM_CH2 down-count compared point.
     * |        |          |1000 = BPWM_CH3 up-count compared point.
     * |        |          |1001 = BPWM_CH3 down-count compared point.
     * |        |          |Others reserved
     * |[23]    |TRGEN2    |BPWM_CH2 Trigger ADC Enable Bit
     * |[27:24] |TRGSEL3   |BPWM_CH3 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH2 zero point.
     * |        |          |0001 = BPWM_CH2 period point.
     * |        |          |0010 = BPWM_CH2 zero or period point.
     * |        |          |0011 = BPWM_CH2 up-count compared point.
     * |        |          |0100 = BPWM_CH2 down-count compared point.
     * |        |          |1000 = BPWM_CH3 up-count compared point.
     * |        |          |1001 = BPWM_CH3 down-count compared point.
     * |        |          |Others reserved.
     * |[31]    |TRGEN3    |BPWM_CH3 Trigger ADC Enable Bit
     * @var BPWM_T::ADCTS1
     * Offset: 0xFC  BPWM Trigger ADC Source Select Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRGSEL4   |BPWM_CH4 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH4 zero point.
     * |        |          |0001 = BPWM_CH4 period point.
     * |        |          |0010 = BPWM_CH4 zero or period point.
     * |        |          |0011 = BPWM_CH4 up-count compared point.
     * |        |          |0100 = BPWM_CH4 down-count compared point.
     * |        |          |1000 = BPWM_CH5 up-count compared point.
     * |        |          |1001 = BPWM_CH5 down-count compared point.
     * |        |          |Others reserved
     * |[7]     |TRGEN4    |BPWM_CH4 Trigger ADC Enable Bit
     * |[11:8]  |TRGSEL5   |BPWM_CH5 Trigger ADC Source Select
     * |        |          |0000 = BPWM_CH4 zero point.
     * |        |          |0001 = BPWM_CH4 period point.
     * |        |          |0010 = BPWM_CH4 zero or period point.
     * |        |          |0011 = BPWM_CH4 up-count compared point.
     * |        |          |0100 = BPWM_CH4 down-count compared point.
     * |        |          |1000 = BPWM_CH5 up-count compared point.
     * |        |          |1001 = BPWM_CH5 down-count compared point.
     * |        |          |Others reserved
     * |[15]    |TRGEN5    |BPWM_CH5 Trigger ADC Enable Bit
     * @var BPWM_T::SSCTL
     * Offset: 0x110  BPWM Synchronous Start Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SSEN0     |BPWM Synchronous Start Function 0 Enable Bit
     * |        |          |When synchronous start function is enabled, the BPWM_CH0 counter enable bit (CNTEN0) can be enabled by writing BPWM synchronous start trigger bit (CNTSEN).
     * |        |          |0 = BPWM synchronous start function Disabled.
     * |        |          |1 = BPWM synchronous start function Enabled.
     * |[9:8]   |SSRC      |BPWM Synchronous Start Source Select
     * |        |          |00 = Synchronous start source come from BPWM0.
     * |        |          |01 = Synchronous start source come from BPWM1.
     * |        |          |10 = Synchronous start source come from BPWM2.
     * |        |          |11 = Synchronous start source come from BPWM3.
     * @var BPWM_T::SSTRG
     * Offset: 0x114  BPWM Synchronous Start Trigger Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTSEN    |BPWM Counter Synchronous Start Enable Bit(Write Only)
     * |        |          |BPMW counter synchronous enable function is used to make PWM or BPWM channels start counting at the same time.
     * |        |          |Writing this bit to 1 will also set the counter enable bit if correlated BPWM channel counter synchronous start function is enabled.
     * @var BPWM_T::STATUS
     * Offset: 0x120  BPWM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTMAX0   |Time-base Counter 0 Equal to 0xFFFF Latched Status
     * |        |          |0 = indicates the time-base counter never reached its maximum value 0xFFFF.
     * |        |          |1 = indicates the time-base counter reached its maximum value, software can write 1 to clear this bit.
     * |[16]    |ADCTRG0   |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No ADC start of conversion trigger event has occurred.
     * |        |          |1 = An ADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[17]    |ADCTRG1   |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No EADC start of conversion trigger event has occurred.
     * |        |          |1 = An EADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[18]    |ADCTRG2   |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No EADC start of conversion trigger event has occurred.
     * |        |          |1 = An EADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[19]    |ADCTRG3   |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No EADC start of conversion trigger event has occurred.
     * |        |          |1 = An EADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[20]    |ADCTRG4   |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No EADC start of conversion trigger event has occurred.
     * |        |          |1 = An EADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * |[21]    |ADCTRG5   |ADC Start of Conversion Status
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No EADC start of conversion trigger event has occurred.
     * |        |          |1 = An EADC start of conversion trigger event has occurred, software can write 1 to clear this bit.
     * @var BPWM_T::CAPINEN
     * Offset: 0x200  BPWM Capture Input Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPINEN0  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path Disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path Enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[1]     |CAPINEN1  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path Disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path Enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[2]     |CAPINEN2  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path Disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path Enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[3]     |CAPINEN3  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path Disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path Enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[4]     |CAPINEN4  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path Disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path Enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * |[5]     |CAPINEN5  |Capture Input Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = BPWM Channel capture input path Disabled
     * |        |          |The input of BPWM channel capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel capture input path Enabled
     * |        |          |The input of BPWM channel capture function comes from correlative multifunction pin.
     * @var BPWM_T::CAPCTL
     * Offset: 0x204  BPWM Capture Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPEN0    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function Disabled. BPWM_RCAPDAT/BPWM_FCAPDAT register will not be updated.
     * |        |          |1 = Capture function Enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[1]     |CAPEN1    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function Disabled. BPWM_RCAPDAT/BPWM_FCAPDAT register will not be updated.
     * |        |          |1 = Capture function Enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[2]     |CAPEN2    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function Disabled. BPWM_RCAPDAT/BPWM_FCAPDAT register will not be updated.
     * |        |          |1 = Capture function Enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[3]     |CAPEN3    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function Disabled. BPWM_RCAPDAT/BPWM_FCAPDAT register will not be updated.
     * |        |          |1 = Capture function Enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[4]     |CAPEN4    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function Disabled. BPWM_RCAPDAT/BPWM_FCAPDAT register will not be updated.
     * |        |          |1 = Capture function Enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[5]     |CAPEN5    |Capture Function Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture function Disabled. BPWM_RCAPDAT/BPWM_FCAPDAT register will not be updated.
     * |        |          |1 = Capture function Enabled
     * |        |          |Capture latched the BPWM counter value when detected rising or falling edge of input signal and saved to RCAPDAT (Rising latch) and FCAPDAT (Falling latch).
     * |[8]     |CAPINV0   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter Disabled.
     * |        |          |1 = Capture source inverter Enabled. Reverse the input signal from GPIO.
     * |[9]     |CAPINV1   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter Disabled.
     * |        |          |1 = Capture source inverter Enabled. Reverse the input signal from GPIO.
     * |[10]    |CAPINV2   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter Disabled.
     * |        |          |1 = Capture source inverter Enabled. Reverse the input signal from GPIO.
     * |[11]    |CAPINV3   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter Disabled.
     * |        |          |1 = Capture source inverter Enabled. Reverse the input signal from GPIO.
     * |[12]    |CAPINV4   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter Disabled.
     * |        |          |1 = Capture source inverter Enabled. Reverse the input signal from GPIO.
     * |[13]    |CAPINV5   |Capture Inverter Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture source inverter Disabled.
     * |        |          |1 = Capture source inverter Enabled. Reverse the input signal from GPIO.
     * |[16]    |RCRLDEN0  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter Disabled.
     * |        |          |1 = Rising capture reload counter Enabled.
     * |[17]    |RCRLDEN1  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter Disabled.
     * |        |          |1 = Rising capture reload counter Enabled.
     * |[18]    |RCRLDEN2  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter Disabled.
     * |        |          |1 = Rising capture reload counter Enabled.
     * |[19]    |RCRLDEN3  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter Disabled.
     * |        |          |1 = Rising capture reload counter Enabled.
     * |[20]    |RCRLDEN4  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter Disabled.
     * |        |          |1 = Rising capture reload counter Enabled.
     * |[21]    |RCRLDEN5  |Rising Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Rising capture reload counter Disabled.
     * |        |          |1 = Rising capture reload counter Enabled.
     * |[24]    |FCRLDEN0  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter Disabled.
     * |        |          |1 = Falling capture reload counter Enabled.
     * |[25]    |FCRLDEN1  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter Disabled.
     * |        |          |1 = Falling capture reload counter Enabled.
     * |[26]    |FCRLDEN2  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter Disabled.
     * |        |          |1 = Falling capture reload counter Enabled.
     * |[27]    |FCRLDEN3  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter Disabled.
     * |        |          |1 = Falling capture reload counter Enabled.
     * |[28]    |FCRLDEN4  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter Disabled.
     * |        |          |1 = Falling capture reload counter Enabled.
     * |[29]    |FCRLDEN5  |Falling Capture Reload Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Falling capture reload counter Disabled.
     * |        |          |1 = Falling capture reload counter Enabled.
     * @var BPWM_T::CAPSTS
     * Offset: 0x208  BPWM Capture Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRIFOV0   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[1]     |CRIFOV1   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[2]     |CRIFOV2   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[3]     |CRIFOV3   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[4]     |CRIFOV4   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[5]     |CRIFOV5   |Capture Rising Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if rising latch happened when the corresponding CAPRIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPRIF.
     * |[8]     |CFIFOV0   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[9]     |CFIFOV1   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[10]    |CFIFOV2   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[11]    |CFIFOV3   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[12]    |CFIFOV4   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * |[13]    |CFIFOV5   |Capture Falling Interrupt Flag Overrun Status (Read Only)
     * |        |          |This flag indicates if falling latch happened when the corresponding CAPFIF is 1.
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |Note: This bit will be cleared automatically when user clear corresponding CAPFIF.
     * @var BPWM_T::CAPIEN
     * Offset: 0x250  BPWM Capture Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CAPRIENn  |BPWM Capture Rising Latch Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture rising edge latch interrupt Disabled.
     * |        |          |1 = Capture rising edge latch interrupt Enabled.
     * |[13:8]  |CAPFIENn  |BPWM Capture Falling Latch Interrupt Enable Bits
     * |        |          |Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = Capture falling edge latch interrupt Disabled.
     * |        |          |1 = Capture falling edge latch interrupt Enabled.
     * @var BPWM_T::CAPIF
     * Offset: 0x254  BPWM Capture Interrupt Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPRIF0   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[1]     |CAPRIF1   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[2]     |CAPRIF2   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[3]     |CAPRIF3   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[4]     |CAPRIF4   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[5]     |CAPRIF5   |BPWM Capture Rising Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture rising latch condition happened.
     * |        |          |1 = Capture rising latch condition happened, this flag will be set to high.
     * |[8]     |CAPFIF0   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[9]     |CAPFIF1   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[10]    |CAPFIF2   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[11]    |CAPFIF3   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[12]    |CAPFIF4   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * |[13]    |CAPFIF5   |BPWM Capture Falling Latch Interrupt Flag
     * |        |          |This bit is writing 1 to clear. Each bit n controls the corresponding BPWM channel n.
     * |        |          |0 = No capture falling latch condition happened.
     * |        |          |1 = Capture falling latch condition happened, this flag will be set to high.
     * @var BPWM_T::PBUF
     * Offset: 0x304  BPWM PERIOD Buffer
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PBUF      |BPWM Period Buffer (Read Only)
     * |        |          |Used as PERIOD active register.
     * @var BPWM_T::CMPBUF[6]
     * Offset: 0x31C  BPWM CMPDAT 0~5 Buffer
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMPBUF    |BPWM Comparator Buffer (Read Only)
     * |        |          |Used as CMPDAT active register.
     */
    __IO uint32_t CTL0;                  /*!< [0x0000] BPWM Control Register 0                                          */
    __IO uint32_t CTL1;                  /*!< [0x0004] BPWM Control Register 1                                          */
    __I  uint32_t RESERVED0[2];
    __IO uint32_t CLKSRC;                /*!< [0x0010] BPWM Clock Source Register                                       */
    __IO uint32_t CLKPSC;                /*!< [0x0014] BPWM Clock Prescale Register                                     */
    __I  uint32_t RESERVED1[2];
    __IO uint32_t CNTEN;                 /*!< [0x0020] BPWM Counter Enable Register                                     */
    __IO uint32_t CNTCLR;                /*!< [0x0024] BPWM Clear Counter Register                                      */
    __I  uint32_t RESERVED2[2];
    __IO uint32_t PERIOD;                /*!< [0x0030] BPWM Period Register                                             */
    __I  uint32_t RESERVED3[7];
    __IO uint32_t CMPDAT[6];             /*!< [0x0050~0x0064] BPWM Comparator Register 0~5                              */
    __I  uint32_t RESERVED4[10];
    __I  uint32_t CNT;                   /*!< [0x0090] BPWM Counter Register                                            */
    __I  uint32_t RESERVED5[7];
    __IO uint32_t WGCTL0;                /*!< [0x00b0] BPWM Generation Register 0                                       */
    __IO uint32_t WGCTL1;                /*!< [0x00b4] BPWM Generation Register 1                                       */
    __IO uint32_t MSKEN;                 /*!< [0x00b8] BPWM Mask Enable Register                                        */
    __IO uint32_t MSK;                   /*!< [0x00bc] BPWM Mask Data Register                                          */
    __I  uint32_t RESERVED6[5];
    __IO uint32_t POLCTL;                /*!< [0x00d4] BPWM Pin Polar Inverse Register                                  */
    __IO uint32_t POEN;                  /*!< [0x00d8] BPWM Output Enable Register                                      */
    __I  uint32_t RESERVED7[1];
    __IO uint32_t INTEN;                 /*!< [0x00e0] BPWM Interrupt Enable Register                                   */
    __I  uint32_t RESERVED8[1];
    __IO uint32_t INTSTS;                /*!< [0x00e8] BPWM Interrupt Flag Register                                     */
    __I  uint32_t RESERVED9[3];
    __IO uint32_t ADCTS0;                /*!< [0x00f8] BPWM Trigger ADC Source Select Register 0                       */
    __IO uint32_t ADCTS1;                /*!< [0x00fc] BPWM Trigger ADC Source Select Register 1                       */
    __I  uint32_t RESERVED10[4];
    __IO uint32_t SSCTL;                 /*!< [0x0110] BPWM Synchronous Start Control Register                          */
    __O  uint32_t SSTRG;                 /*!< [0x0114] BPWM Synchronous Start Trigger Register                          */
    __I  uint32_t RESERVED11[2];
    __IO uint32_t STATUS;                /*!< [0x0120] BPWM Status Register                                             */
    __I  uint32_t RESERVED12[55];
    __IO uint32_t CAPINEN;               /*!< [0x0200] BPWM Capture Input Enable Register                               */
    __IO uint32_t CAPCTL;                /*!< [0x0204] BPWM Capture Control Register                                    */
    __I  uint32_t CAPSTS;                /*!< [0x0208] BPWM Capture Status Register                                     */
    BCAPDAT_T CAPDAT[6];                  /*!< [0x020c~0x0238] BPWM Rising and Falling Capture Data Register 0~5         */
    __I  uint32_t RESERVED13[5];
    __IO uint32_t CAPIEN;                /*!< [0x0250] BPWM Capture Interrupt Enable Register                           */
    __IO uint32_t CAPIF;                 /*!< [0x0254] BPWM Capture Interrupt Flag Register                             */
    __I  uint32_t RESERVED14[43];
    __I  uint32_t PBUF;                  /*!< [0x0304] BPWM PERIOD Buffer                                               */
    __I  uint32_t RESERVED15[5];
    __I  uint32_t CMPBUF[6];             /*!< [0x031c~0x0330] BPWM CMPDAT 0~5 Buffer                                  */

} BPWM_T;

/**
    @addtogroup BPWM_CONST BPWM Bit Field Definition
    Constant Definitions for BPWM Controller
    @{ 
*/

#define BPWM_CTL0_CTRLD0_Pos             (0)                                               /*!< BPWM_T::CTL0: CTRLD0 Position          */
#define BPWM_CTL0_CTRLD0_Msk             (0x1ul << BPWM_CTL0_CTRLD0_Pos)                   /*!< BPWM_T::CTL0: CTRLD0 Mask              */

#define BPWM_CTL0_CTRLD1_Pos             (1)                                               /*!< BPWM_T::CTL0: CTRLD1 Position          */
#define BPWM_CTL0_CTRLD1_Msk             (0x1ul << BPWM_CTL0_CTRLD1_Pos)                   /*!< BPWM_T::CTL0: CTRLD1 Mask              */

#define BPWM_CTL0_CTRLD2_Pos             (2)                                               /*!< BPWM_T::CTL0: CTRLD2 Position          */
#define BPWM_CTL0_CTRLD2_Msk             (0x1ul << BPWM_CTL0_CTRLD2_Pos)                   /*!< BPWM_T::CTL0: CTRLD2 Mask              */

#define BPWM_CTL0_CTRLD3_Pos             (3)                                               /*!< BPWM_T::CTL0: CTRLD3 Position          */
#define BPWM_CTL0_CTRLD3_Msk             (0x1ul << BPWM_CTL0_CTRLD3_Pos)                   /*!< BPWM_T::CTL0: CTRLD3 Mask              */

#define BPWM_CTL0_CTRLD4_Pos             (4)                                               /*!< BPWM_T::CTL0: CTRLD4 Position          */
#define BPWM_CTL0_CTRLD4_Msk             (0x1ul << BPWM_CTL0_CTRLD4_Pos)                   /*!< BPWM_T::CTL0: CTRLD4 Mask              */

#define BPWM_CTL0_CTRLD5_Pos             (5)                                               /*!< BPWM_T::CTL0: CTRLD5 Position          */
#define BPWM_CTL0_CTRLD5_Msk             (0x1ul << BPWM_CTL0_CTRLD5_Pos)                   /*!< BPWM_T::CTL0: CTRLD5 Mask              */

#define BPWM_CTL0_IMMLDEN0_Pos           (16)                                              /*!< BPWM_T::CTL0: IMMLDEN0 Position        */
#define BPWM_CTL0_IMMLDEN0_Msk           (0x1ul << BPWM_CTL0_IMMLDEN0_Pos)                 /*!< BPWM_T::CTL0: IMMLDEN0 Mask            */

#define BPWM_CTL0_IMMLDEN1_Pos           (17)                                              /*!< BPWM_T::CTL0: IMMLDEN1 Position        */
#define BPWM_CTL0_IMMLDEN1_Msk           (0x1ul << BPWM_CTL0_IMMLDEN1_Pos)                 /*!< BPWM_T::CTL0: IMMLDEN1 Mask            */

#define BPWM_CTL0_IMMLDEN2_Pos           (18)                                              /*!< BPWM_T::CTL0: IMMLDEN2 Position        */
#define BPWM_CTL0_IMMLDEN2_Msk           (0x1ul << BPWM_CTL0_IMMLDEN2_Pos)                 /*!< BPWM_T::CTL0: IMMLDEN2 Mask            */

#define BPWM_CTL0_IMMLDEN3_Pos           (19)                                              /*!< BPWM_T::CTL0: IMMLDEN3 Position        */
#define BPWM_CTL0_IMMLDEN3_Msk           (0x1ul << BPWM_CTL0_IMMLDEN3_Pos)                 /*!< BPWM_T::CTL0: IMMLDEN3 Mask            */

#define BPWM_CTL0_IMMLDEN4_Pos           (20)                                              /*!< BPWM_T::CTL0: IMMLDEN4 Position        */
#define BPWM_CTL0_IMMLDEN4_Msk           (0x1ul << BPWM_CTL0_IMMLDEN4_Pos)                 /*!< BPWM_T::CTL0: IMMLDEN4 Mask            */

#define BPWM_CTL0_IMMLDEN5_Pos           (21)                                              /*!< BPWM_T::CTL0: IMMLDEN5 Position        */
#define BPWM_CTL0_IMMLDEN5_Msk           (0x1ul << BPWM_CTL0_IMMLDEN5_Pos)                 /*!< BPWM_T::CTL0: IMMLDEN5 Mask            */

#define BPWM_CTL0_DBGHALT_Pos            (30)                                              /*!< BPWM_T::CTL0: DBGHALT Position         */
#define BPWM_CTL0_DBGHALT_Msk            (0x1ul << BPWM_CTL0_DBGHALT_Pos)                  /*!< BPWM_T::CTL0: DBGHALT Mask             */

#define BPWM_CTL0_DBGTRIOFF_Pos          (31)                                              /*!< BPWM_T::CTL0: DBGTRIOFF Position       */
#define BPWM_CTL0_DBGTRIOFF_Msk          (0x1ul << BPWM_CTL0_DBGTRIOFF_Pos)                /*!< BPWM_T::CTL0: DBGTRIOFF Mask           */

#define BPWM_CTL1_CNTTYPE0_Pos           (0)                                               /*!< BPWM_T::CTL1: CNTTYPE0 Position        */
#define BPWM_CTL1_CNTTYPE0_Msk           (0x3ul << BPWM_CTL1_CNTTYPE0_Pos)                 /*!< BPWM_T::CTL1: CNTTYPE0 Mask            */

#define BPWM_CLKSRC_ECLKSRC0_Pos         (0)                                               /*!< BPWM_T::CLKSRC: ECLKSRC0 Position      */
#define BPWM_CLKSRC_ECLKSRC0_Msk         (0x7ul << BPWM_CLKSRC_ECLKSRC0_Pos)               /*!< BPWM_T::CLKSRC: ECLKSRC0 Mask          */

#define BPWM_CLKPSC_CLKPSC_Pos           (0)                                               /*!< BPWM_T::CLKPSC: CLKPSC Position        */
#define BPWM_CLKPSC_CLKPSC_Msk           (0xffful << BPWM_CLKPSC_CLKPSC_Pos)               /*!< BPWM_T::CLKPSC: CLKPSC Mask            */

#define BPWM_CNTEN_CNTEN0_Pos            (0)                                               /*!< BPWM_T::CNTEN: CNTEN0 Position         */
#define BPWM_CNTEN_CNTEN0_Msk            (0x1ul << BPWM_CNTEN_CNTEN0_Pos)                  /*!< BPWM_T::CNTEN: CNTEN0 Mask             */

#define BPWM_CNTCLR_CNTCLR0_Pos          (0)                                               /*!< BPWM_T::CNTCLR: CNTCLR0 Position       */
#define BPWM_CNTCLR_CNTCLR0_Msk          (0x1ul << BPWM_CNTCLR_CNTCLR0_Pos)                /*!< BPWM_T::CNTCLR: CNTCLR0 Mask           */

#define BPWM_PERIOD_PERIOD_Pos           (0)                                               /*!< BPWM_T::PERIOD: PERIOD Position        */
#define BPWM_PERIOD_PERIOD_Msk           (0xfffful << BPWM_PERIOD_PERIOD_Pos)              /*!< BPWM_T::PERIOD: PERIOD Mask            */

#define BPWM_CMPDAT0_CMPDAT_Pos          (0)                                               /*!< BPWM_T::CMPDAT0: CMPDAT Position       */
#define BPWM_CMPDAT0_CMPDAT_Msk          (0xfffful << BPWM_CMPDAT0_CMPDAT_Pos)             /*!< BPWM_T::CMPDAT0: CMPDAT Mask           */

#define BPWM_CMPDAT1_CMPDAT_Pos          (0)                                               /*!< BPWM_T::CMPDAT1: CMPDAT Position       */
#define BPWM_CMPDAT1_CMPDAT_Msk          (0xfffful << BPWM_CMPDAT1_CMPDAT_Pos)             /*!< BPWM_T::CMPDAT1: CMPDAT Mask           */

#define BPWM_CMPDAT2_CMPDAT_Pos          (0)                                               /*!< BPWM_T::CMPDAT2: CMPDAT Position       */
#define BPWM_CMPDAT2_CMPDAT_Msk          (0xfffful << BPWM_CMPDAT2_CMPDAT_Pos)             /*!< BPWM_T::CMPDAT2: CMPDAT Mask           */

#define BPWM_CMPDAT3_CMPDAT_Pos          (0)                                               /*!< BPWM_T::CMPDAT3: CMPDAT Position       */
#define BPWM_CMPDAT3_CMPDAT_Msk          (0xfffful << BPWM_CMPDAT3_CMPDAT_Pos)             /*!< BPWM_T::CMPDAT3: CMPDAT Mask           */

#define BPWM_CMPDAT4_CMPDAT_Pos          (0)                                               /*!< BPWM_T::CMPDAT4: CMPDAT Position       */
#define BPWM_CMPDAT4_CMPDAT_Msk          (0xfffful << BPWM_CMPDAT4_CMPDAT_Pos)             /*!< BPWM_T::CMPDAT4: CMPDAT Mask           */

#define BPWM_CMPDAT5_CMPDAT_Pos          (0)                                               /*!< BPWM_T::CMPDAT5: CMPDAT Position       */
#define BPWM_CMPDAT5_CMPDAT_Msk          (0xfffful << BPWM_CMPDAT5_CMPDAT_Pos)             /*!< BPWM_T::CMPDAT5: CMPDAT Mask           */

#define BPWM_CNT_CNT_Pos                 (0)                                               /*!< BPWM_T::CNT: CNT Position              */
#define BPWM_CNT_CNT_Msk                 (0xfffful << BPWM_CNT_CNT_Pos)                    /*!< BPWM_T::CNT: CNT Mask                  */

#define BPWM_CNT_DIRF_Pos                (16)                                              /*!< BPWM_T::CNT: DIRF Position             */
#define BPWM_CNT_DIRF_Msk                (0x1ul << BPWM_CNT_DIRF_Pos)                      /*!< BPWM_T::CNT: DIRF Mask                 */

#define BPWM_WGCTL0_ZPCTL0_Pos           (0)                                               /*!< BPWM_T::WGCTL0: ZPCTL0 Position        */
#define BPWM_WGCTL0_ZPCTL0_Msk           (0x3ul << BPWM_WGCTL0_ZPCTL0_Pos)                 /*!< BPWM_T::WGCTL0: ZPCTL0 Mask            */

#define BPWM_WGCTL0_ZPCTL1_Pos           (2)                                               /*!< BPWM_T::WGCTL0: ZPCTL1 Position        */
#define BPWM_WGCTL0_ZPCTL1_Msk           (0x3ul << BPWM_WGCTL0_ZPCTL1_Pos)                 /*!< BPWM_T::WGCTL0: ZPCTL1 Mask            */

#define BPWM_WGCTL0_ZPCTL2_Pos           (4)                                               /*!< BPWM_T::WGCTL0: ZPCTL2 Position        */
#define BPWM_WGCTL0_ZPCTL2_Msk           (0x3ul << BPWM_WGCTL0_ZPCTL2_Pos)                 /*!< BPWM_T::WGCTL0: ZPCTL2 Mask            */

#define BPWM_WGCTL0_ZPCTL3_Pos           (6)                                               /*!< BPWM_T::WGCTL0: ZPCTL3 Position        */
#define BPWM_WGCTL0_ZPCTL3_Msk           (0x3ul << BPWM_WGCTL0_ZPCTL3_Pos)                 /*!< BPWM_T::WGCTL0: ZPCTL3 Mask            */

#define BPWM_WGCTL0_ZPCTL4_Pos           (8)                                               /*!< BPWM_T::WGCTL0: ZPCTL4 Position        */
#define BPWM_WGCTL0_ZPCTL4_Msk           (0x3ul << BPWM_WGCTL0_ZPCTL4_Pos)                 /*!< BPWM_T::WGCTL0: ZPCTL4 Mask            */

#define BPWM_WGCTL0_ZPCTL5_Pos           (10)                                              /*!< BPWM_T::WGCTL0: ZPCTL5 Position        */
#define BPWM_WGCTL0_ZPCTL5_Msk           (0x3ul << BPWM_WGCTL0_ZPCTL5_Pos)                 /*!< BPWM_T::WGCTL0: ZPCTL5 Mask            */

#define BPWM_WGCTL0_ZPCTLn_Pos           (0)                                               /*!< BPWM_T::WGCTL0: ZPCTLn Position        */
#define BPWM_WGCTL0_ZPCTLn_Msk           (0xffful << BPWM_WGCTL0_ZPCTLn_Pos)               /*!< BPWM_T::WGCTL0: ZPCTLn Mask            */

#define BPWM_WGCTL0_PRDPCTL0_Pos         (16)                                              /*!< BPWM_T::WGCTL0: PRDPCTL0 Position      */
#define BPWM_WGCTL0_PRDPCTL0_Msk         (0x3ul << BPWM_WGCTL0_PRDPCTL0_Pos)               /*!< BPWM_T::WGCTL0: PRDPCTL0 Mask          */

#define BPWM_WGCTL0_PRDPCTL1_Pos         (18)                                              /*!< BPWM_T::WGCTL0: PRDPCTL1 Position      */
#define BPWM_WGCTL0_PRDPCTL1_Msk         (0x3ul << BPWM_WGCTL0_PRDPCTL1_Pos)               /*!< BPWM_T::WGCTL0: PRDPCTL1 Mask          */

#define BPWM_WGCTL0_PRDPCTL2_Pos         (20)                                              /*!< BPWM_T::WGCTL0: PRDPCTL2 Position      */
#define BPWM_WGCTL0_PRDPCTL2_Msk         (0x3ul << BPWM_WGCTL0_PRDPCTL2_Pos)               /*!< BPWM_T::WGCTL0: PRDPCTL2 Mask          */

#define BPWM_WGCTL0_PRDPCTL3_Pos         (22)                                              /*!< BPWM_T::WGCTL0: PRDPCTL3 Position      */
#define BPWM_WGCTL0_PRDPCTL3_Msk         (0x3ul << BPWM_WGCTL0_PRDPCTL3_Pos)               /*!< BPWM_T::WGCTL0: PRDPCTL3 Mask          */

#define BPWM_WGCTL0_PRDPCTL4_Pos         (24)                                              /*!< BPWM_T::WGCTL0: PRDPCTL4 Position      */
#define BPWM_WGCTL0_PRDPCTL4_Msk         (0x3ul << BPWM_WGCTL0_PRDPCTL4_Pos)               /*!< BPWM_T::WGCTL0: PRDPCTL4 Mask          */

#define BPWM_WGCTL0_PRDPCTL5_Pos         (26)                                              /*!< BPWM_T::WGCTL0: PRDPCTL5 Position      */
#define BPWM_WGCTL0_PRDPCTL5_Msk         (0x3ul << BPWM_WGCTL0_PRDPCTL5_Pos)               /*!< BPWM_T::WGCTL0: PRDPCTL5 Mask          */

#define BPWM_WGCTL0_PRDPCTLn_Pos         (16)                                              /*!< BPWM_T::WGCTL0: PRDPCTLn Position      */
#define BPWM_WGCTL0_PRDPCTLn_Msk         (0xffful << BPWM_WGCTL0_PRDPCTLn_Pos)             /*!< BPWM_T::WGCTL0: PRDPCTLn Mask          */

#define BPWM_WGCTL1_CMPUCTL0_Pos         (0)                                               /*!< BPWM_T::WGCTL1: CMPUCTL0 Position      */
#define BPWM_WGCTL1_CMPUCTL0_Msk         (0x3ul << BPWM_WGCTL1_CMPUCTL0_Pos)               /*!< BPWM_T::WGCTL1: CMPUCTL0 Mask          */

#define BPWM_WGCTL1_CMPUCTL1_Pos         (2)                                               /*!< BPWM_T::WGCTL1: CMPUCTL1 Position      */
#define BPWM_WGCTL1_CMPUCTL1_Msk         (0x3ul << BPWM_WGCTL1_CMPUCTL1_Pos)               /*!< BPWM_T::WGCTL1: CMPUCTL1 Mask          */

#define BPWM_WGCTL1_CMPUCTL2_Pos         (4)                                               /*!< BPWM_T::WGCTL1: CMPUCTL2 Position      */
#define BPWM_WGCTL1_CMPUCTL2_Msk         (0x3ul << BPWM_WGCTL1_CMPUCTL2_Pos)               /*!< BPWM_T::WGCTL1: CMPUCTL2 Mask          */

#define BPWM_WGCTL1_CMPUCTL3_Pos         (6)                                               /*!< BPWM_T::WGCTL1: CMPUCTL3 Position      */
#define BPWM_WGCTL1_CMPUCTL3_Msk         (0x3ul << BPWM_WGCTL1_CMPUCTL3_Pos)               /*!< BPWM_T::WGCTL1: CMPUCTL3 Mask          */

#define BPWM_WGCTL1_CMPUCTL4_Pos         (8)                                               /*!< BPWM_T::WGCTL1: CMPUCTL4 Position      */
#define BPWM_WGCTL1_CMPUCTL4_Msk         (0x3ul << BPWM_WGCTL1_CMPUCTL4_Pos)               /*!< BPWM_T::WGCTL1: CMPUCTL4 Mask          */

#define BPWM_WGCTL1_CMPUCTL5_Pos         (10)                                              /*!< BPWM_T::WGCTL1: CMPUCTL5 Position      */
#define BPWM_WGCTL1_CMPUCTL5_Msk         (0x3ul << BPWM_WGCTL1_CMPUCTL5_Pos)               /*!< BPWM_T::WGCTL1: CMPUCTL5 Mask          */

#define BPWM_WGCTL1_CMPUCTLn_Pos         (0)                                               /*!< BPWM_T::WGCTL1: CMPUCTLn Position      */
#define BPWM_WGCTL1_CMPUCTLn_Msk         (0xffful << BPWM_WGCTL1_CMPUCTLn_Pos)             /*!< BPWM_T::WGCTL1: CMPUCTLn Mask          */

#define BPWM_WGCTL1_CMPDCTL0_Pos         (16)                                              /*!< BPWM_T::WGCTL1: CMPDCTL0 Position      */
#define BPWM_WGCTL1_CMPDCTL0_Msk         (0x3ul << BPWM_WGCTL1_CMPDCTL0_Pos)               /*!< BPWM_T::WGCTL1: CMPDCTL0 Mask          */

#define BPWM_WGCTL1_CMPDCTL1_Pos         (18)                                              /*!< BPWM_T::WGCTL1: CMPDCTL1 Position      */
#define BPWM_WGCTL1_CMPDCTL1_Msk         (0x3ul << BPWM_WGCTL1_CMPDCTL1_Pos)               /*!< BPWM_T::WGCTL1: CMPDCTL1 Mask          */

#define BPWM_WGCTL1_CMPDCTL2_Pos         (20)                                              /*!< BPWM_T::WGCTL1: CMPDCTL2 Position      */
#define BPWM_WGCTL1_CMPDCTL2_Msk         (0x3ul << BPWM_WGCTL1_CMPDCTL2_Pos)               /*!< BPWM_T::WGCTL1: CMPDCTL2 Mask          */

#define BPWM_WGCTL1_CMPDCTL3_Pos         (22)                                              /*!< BPWM_T::WGCTL1: CMPDCTL3 Position      */
#define BPWM_WGCTL1_CMPDCTL3_Msk         (0x3ul << BPWM_WGCTL1_CMPDCTL3_Pos)               /*!< BPWM_T::WGCTL1: CMPDCTL3 Mask          */

#define BPWM_WGCTL1_CMPDCTL4_Pos         (24)                                              /*!< BPWM_T::WGCTL1: CMPDCTL4 Position      */
#define BPWM_WGCTL1_CMPDCTL4_Msk         (0x3ul << BPWM_WGCTL1_CMPDCTL4_Pos)               /*!< BPWM_T::WGCTL1: CMPDCTL4 Mask          */

#define BPWM_WGCTL1_CMPDCTL5_Pos         (26)                                              /*!< BPWM_T::WGCTL1: CMPDCTL5 Position      */
#define BPWM_WGCTL1_CMPDCTL5_Msk         (0x3ul << BPWM_WGCTL1_CMPDCTL5_Pos)               /*!< BPWM_T::WGCTL1: CMPDCTL5 Mask          */

#define BPWM_WGCTL1_CMPDCTLn_Pos         (16)                                              /*!< BPWM_T::WGCTL1: CMPDCTLn Position      */
#define BPWM_WGCTL1_CMPDCTLn_Msk         (0xffful << BPWM_WGCTL1_CMPDCTLn_Pos)             /*!< BPWM_T::WGCTL1: CMPDCTLn Mask          */

#define BPWM_MSKEN_MSKEN0_Pos            (0)                                               /*!< BPWM_T::MSKEN: MSKEN0 Position         */
#define BPWM_MSKEN_MSKEN0_Msk            (0x1ul << BPWM_MSKEN_MSKEN0_Pos)                  /*!< BPWM_T::MSKEN: MSKEN0 Mask             */

#define BPWM_MSKEN_MSKEN1_Pos            (1)                                               /*!< BPWM_T::MSKEN: MSKEN1 Position         */
#define BPWM_MSKEN_MSKEN1_Msk            (0x1ul << BPWM_MSKEN_MSKEN1_Pos)                  /*!< BPWM_T::MSKEN: MSKEN1 Mask             */

#define BPWM_MSKEN_MSKEN2_Pos            (2)                                               /*!< BPWM_T::MSKEN: MSKEN2 Position         */
#define BPWM_MSKEN_MSKEN2_Msk            (0x1ul << BPWM_MSKEN_MSKEN2_Pos)                  /*!< BPWM_T::MSKEN: MSKEN2 Mask             */

#define BPWM_MSKEN_MSKEN3_Pos            (3)                                               /*!< BPWM_T::MSKEN: MSKEN3 Position         */
#define BPWM_MSKEN_MSKEN3_Msk            (0x1ul << BPWM_MSKEN_MSKEN3_Pos)                  /*!< BPWM_T::MSKEN: MSKEN3 Mask             */

#define BPWM_MSKEN_MSKEN4_Pos            (4)                                               /*!< BPWM_T::MSKEN: MSKEN4 Position         */
#define BPWM_MSKEN_MSKEN4_Msk            (0x1ul << BPWM_MSKEN_MSKEN4_Pos)                  /*!< BPWM_T::MSKEN: MSKEN4 Mask             */

#define BPWM_MSKEN_MSKEN5_Pos            (5)                                               /*!< BPWM_T::MSKEN: MSKEN5 Position         */
#define BPWM_MSKEN_MSKEN5_Msk            (0x1ul << BPWM_MSKEN_MSKEN5_Pos)                  /*!< BPWM_T::MSKEN: MSKEN5 Mask             */

#define BPWM_MSKEN_MSKENn_Pos            (0)                                               /*!< BPWM_T::MSKEN: MSKENn Position         */
#define BPWM_MSKEN_MSKENn_Msk            (0x3ful << BPWM_MSKEN_MSKENn_Pos)                 /*!< BPWM_T::MSKEN: MSKENn Mask             */

#define BPWM_MSK_MSKDAT0_Pos             (0)                                               /*!< BPWM_T::MSK: MSKDAT0 Position          */
#define BPWM_MSK_MSKDAT0_Msk             (0x1ul << BPWM_MSK_MSKDAT0_Pos)                   /*!< BPWM_T::MSK: MSKDAT0 Mask              */

#define BPWM_MSK_MSKDAT1_Pos             (1)                                               /*!< BPWM_T::MSK: MSKDAT1 Position          */
#define BPWM_MSK_MSKDAT1_Msk             (0x1ul << BPWM_MSK_MSKDAT1_Pos)                   /*!< BPWM_T::MSK: MSKDAT1 Mask              */

#define BPWM_MSK_MSKDAT2_Pos             (2)                                               /*!< BPWM_T::MSK: MSKDAT2 Position          */
#define BPWM_MSK_MSKDAT2_Msk             (0x1ul << BPWM_MSK_MSKDAT2_Pos)                   /*!< BPWM_T::MSK: MSKDAT2 Mask              */

#define BPWM_MSK_MSKDAT3_Pos             (3)                                               /*!< BPWM_T::MSK: MSKDAT3 Position          */
#define BPWM_MSK_MSKDAT3_Msk             (0x1ul << BPWM_MSK_MSKDAT3_Pos)                   /*!< BPWM_T::MSK: MSKDAT3 Mask              */

#define BPWM_MSK_MSKDAT4_Pos             (4)                                               /*!< BPWM_T::MSK: MSKDAT4 Position          */
#define BPWM_MSK_MSKDAT4_Msk             (0x1ul << BPWM_MSK_MSKDAT4_Pos)                   /*!< BPWM_T::MSK: MSKDAT4 Mask              */

#define BPWM_MSK_MSKDAT5_Pos             (5)                                               /*!< BPWM_T::MSK: MSKDAT5 Position          */
#define BPWM_MSK_MSKDAT5_Msk             (0x1ul << BPWM_MSK_MSKDAT5_Pos)                   /*!< BPWM_T::MSK: MSKDAT5 Mask              */

#define BPWM_MSK_MSKDATn_Pos             (0)                                               /*!< BPWM_T::MSK: MSKDATn Position          */
#define BPWM_MSK_MSKDATn_Msk             (0x3ful << BPWM_MSK_MSKDATn_Pos)                  /*!< BPWM_T::MSK: MSKDATn Mask              */

#define BPWM_POLCTL_PINV0_Pos            (0)                                               /*!< BPWM_T::POLCTL: PINV0 Position         */
#define BPWM_POLCTL_PINV0_Msk            (0x1ul << BPWM_POLCTL_PINV0_Pos)                  /*!< BPWM_T::POLCTL: PINV0 Mask             */

#define BPWM_POLCTL_PINV1_Pos            (1)                                               /*!< BPWM_T::POLCTL: PINV1 Position         */
#define BPWM_POLCTL_PINV1_Msk            (0x1ul << BPWM_POLCTL_PINV1_Pos)                  /*!< BPWM_T::POLCTL: PINV1 Mask             */

#define BPWM_POLCTL_PINV2_Pos            (2)                                               /*!< BPWM_T::POLCTL: PINV2 Position         */
#define BPWM_POLCTL_PINV2_Msk            (0x1ul << BPWM_POLCTL_PINV2_Pos)                  /*!< BPWM_T::POLCTL: PINV2 Mask             */

#define BPWM_POLCTL_PINV3_Pos            (3)                                               /*!< BPWM_T::POLCTL: PINV3 Position         */
#define BPWM_POLCTL_PINV3_Msk            (0x1ul << BPWM_POLCTL_PINV3_Pos)                  /*!< BPWM_T::POLCTL: PINV3 Mask             */

#define BPWM_POLCTL_PINV4_Pos            (4)                                               /*!< BPWM_T::POLCTL: PINV4 Position         */
#define BPWM_POLCTL_PINV4_Msk            (0x1ul << BPWM_POLCTL_PINV4_Pos)                  /*!< BPWM_T::POLCTL: PINV4 Mask             */

#define BPWM_POLCTL_PINV5_Pos            (5)                                               /*!< BPWM_T::POLCTL: PINV5 Position         */
#define BPWM_POLCTL_PINV5_Msk            (0x1ul << BPWM_POLCTL_PINV5_Pos)                  /*!< BPWM_T::POLCTL: PINV5 Mask             */

#define BPWM_POLCTL_PINVn_Pos            (0)                                               /*!< BPWM_T::POLCTL: PINVn Position         */
#define BPWM_POLCTL_PINVn_Msk            (0x3ful << BPWM_POLCTL_PINVn_Pos)                 /*!< BPWM_T::POLCTL: PINVn Mask             */

#define BPWM_POEN_POEN0_Pos              (0)                                               /*!< BPWM_T::POEN: POEN0 Position           */
#define BPWM_POEN_POEN0_Msk              (0x1ul << BPWM_POEN_POEN0_Pos)                    /*!< BPWM_T::POEN: POEN0 Mask               */

#define BPWM_POEN_POEN1_Pos              (1)                                               /*!< BPWM_T::POEN: POEN1 Position           */
#define BPWM_POEN_POEN1_Msk              (0x1ul << BPWM_POEN_POEN1_Pos)                    /*!< BPWM_T::POEN: POEN1 Mask               */

#define BPWM_POEN_POEN2_Pos              (2)                                               /*!< BPWM_T::POEN: POEN2 Position           */
#define BPWM_POEN_POEN2_Msk              (0x1ul << BPWM_POEN_POEN2_Pos)                    /*!< BPWM_T::POEN: POEN2 Mask               */

#define BPWM_POEN_POEN3_Pos              (3)                                               /*!< BPWM_T::POEN: POEN3 Position           */
#define BPWM_POEN_POEN3_Msk              (0x1ul << BPWM_POEN_POEN3_Pos)                    /*!< BPWM_T::POEN: POEN3 Mask               */

#define BPWM_POEN_POEN4_Pos              (4)                                               /*!< BPWM_T::POEN: POEN4 Position           */
#define BPWM_POEN_POEN4_Msk              (0x1ul << BPWM_POEN_POEN4_Pos)                    /*!< BPWM_T::POEN: POEN4 Mask               */

#define BPWM_POEN_POEN5_Pos              (5)                                               /*!< BPWM_T::POEN: POEN5 Position           */
#define BPWM_POEN_POEN5_Msk              (0x1ul << BPWM_POEN_POEN5_Pos)                    /*!< BPWM_T::POEN: POEN5 Mask               */

#define BPWM_POEN_POENn_Pos              (0)                                               /*!< BPWM_T::POEN: POENn Position           */
#define BPWM_POEN_POENn_Msk              (0x3ful << BPWM_POEN_POENn_Pos)                   /*!< BPWM_T::POEN: POENn Mask               */

#define BPWM_INTEN_ZIEN0_Pos             (0)                                               /*!< BPWM_T::INTEN: ZIEN0 Position          */
#define BPWM_INTEN_ZIEN0_Msk             (0x1ul << BPWM_INTEN_ZIEN0_Pos)                   /*!< BPWM_T::INTEN: ZIEN0 Mask              */

#define BPWM_INTEN_PIEN0_Pos             (8)                                               /*!< BPWM_T::INTEN: PIEN0 Position          */
#define BPWM_INTEN_PIEN0_Msk             (0x1ul << BPWM_INTEN_PIEN0_Pos)                   /*!< BPWM_T::INTEN: PIEN0 Mask              */

#define BPWM_INTEN_CMPUIEN0_Pos          (16)                                              /*!< BPWM_T::INTEN: CMPUIEN0 Position       */
#define BPWM_INTEN_CMPUIEN0_Msk          (0x1ul << BPWM_INTEN_CMPUIEN0_Pos)                /*!< BPWM_T::INTEN: CMPUIEN0 Mask           */

#define BPWM_INTEN_CMPUIEN1_Pos          (17)                                              /*!< BPWM_T::INTEN: CMPUIEN1 Position       */
#define BPWM_INTEN_CMPUIEN1_Msk          (0x1ul << BPWM_INTEN_CMPUIEN1_Pos)                /*!< BPWM_T::INTEN: CMPUIEN1 Mask           */

#define BPWM_INTEN_CMPUIEN2_Pos          (18)                                              /*!< BPWM_T::INTEN: CMPUIEN2 Position       */
#define BPWM_INTEN_CMPUIEN2_Msk          (0x1ul << BPWM_INTEN_CMPUIEN2_Pos)                /*!< BPWM_T::INTEN: CMPUIEN2 Mask           */

#define BPWM_INTEN_CMPUIEN3_Pos          (19)                                              /*!< BPWM_T::INTEN: CMPUIEN3 Position       */
#define BPWM_INTEN_CMPUIEN3_Msk          (0x1ul << BPWM_INTEN_CMPUIEN3_Pos)                /*!< BPWM_T::INTEN: CMPUIEN3 Mask           */

#define BPWM_INTEN_CMPUIEN4_Pos          (20)                                              /*!< BPWM_T::INTEN: CMPUIEN4 Position       */
#define BPWM_INTEN_CMPUIEN4_Msk          (0x1ul << BPWM_INTEN_CMPUIEN4_Pos)                /*!< BPWM_T::INTEN: CMPUIEN4 Mask           */

#define BPWM_INTEN_CMPUIEN5_Pos          (21)                                              /*!< BPWM_T::INTEN: CMPUIEN5 Position       */
#define BPWM_INTEN_CMPUIEN5_Msk          (0x1ul << BPWM_INTEN_CMPUIEN5_Pos)                /*!< BPWM_T::INTEN: CMPUIEN5 Mask           */

#define BPWM_INTEN_CMPUIENn_Pos          (16)                                              /*!< BPWM_T::INTEN: CMPUIENn Position       */
#define BPWM_INTEN_CMPUIENn_Msk          (0x3ful << BPWM_INTEN_CMPUIENn_Pos)               /*!< BPWM_T::INTEN: CMPUIENn Mask           */

#define BPWM_INTEN_CMPDIEN0_Pos          (24)                                              /*!< BPWM_T::INTEN: CMPDIEN0 Position       */
#define BPWM_INTEN_CMPDIEN0_Msk          (0x1ul << BPWM_INTEN_CMPDIEN0_Pos)                /*!< BPWM_T::INTEN: CMPDIEN0 Mask           */

#define BPWM_INTEN_CMPDIEN1_Pos          (25)                                              /*!< BPWM_T::INTEN: CMPDIEN1 Position       */
#define BPWM_INTEN_CMPDIEN1_Msk          (0x1ul << BPWM_INTEN_CMPDIEN1_Pos)                /*!< BPWM_T::INTEN: CMPDIEN1 Mask           */

#define BPWM_INTEN_CMPDIEN2_Pos          (26)                                              /*!< BPWM_T::INTEN: CMPDIEN2 Position       */
#define BPWM_INTEN_CMPDIEN2_Msk          (0x1ul << BPWM_INTEN_CMPDIEN2_Pos)                /*!< BPWM_T::INTEN: CMPDIEN2 Mask           */

#define BPWM_INTEN_CMPDIEN3_Pos          (27)                                              /*!< BPWM_T::INTEN: CMPDIEN3 Position       */
#define BPWM_INTEN_CMPDIEN3_Msk          (0x1ul << BPWM_INTEN_CMPDIEN3_Pos)                /*!< BPWM_T::INTEN: CMPDIEN3 Mask           */

#define BPWM_INTEN_CMPDIEN4_Pos          (28)                                              /*!< BPWM_T::INTEN: CMPDIEN4 Position       */
#define BPWM_INTEN_CMPDIEN4_Msk          (0x1ul << BPWM_INTEN_CMPDIEN4_Pos)                /*!< BPWM_T::INTEN: CMPDIEN4 Mask           */

#define BPWM_INTEN_CMPDIEN5_Pos          (29)                                              /*!< BPWM_T::INTEN: CMPDIEN5 Position       */
#define BPWM_INTEN_CMPDIEN5_Msk          (0x1ul << BPWM_INTEN_CMPDIEN5_Pos)                /*!< BPWM_T::INTEN: CMPDIEN5 Mask           */

#define BPWM_INTEN_CMPDIENn_Pos          (24)                                              /*!< BPWM_T::INTEN: CMPDIENn Position       */
#define BPWM_INTEN_CMPDIENn_Msk          (0x3ful << BPWM_INTEN_CMPDIENn_Pos)               /*!< BPWM_T::INTEN: CMPDIENn Mask           */

#define BPWM_INTSTS_ZIF0_Pos             (0)                                               /*!< BPWM_T::INTSTS: ZIF0 Position          */
#define BPWM_INTSTS_ZIF0_Msk             (0x1ul << BPWM_INTSTS_ZIF0_Pos)                   /*!< BPWM_T::INTSTS: ZIF0 Mask              */

#define BPWM_INTSTS_PIF0_Pos             (8)                                               /*!< BPWM_T::INTSTS: PIF0 Position          */
#define BPWM_INTSTS_PIF0_Msk             (0x1ul << BPWM_INTSTS_PIF0_Pos)                   /*!< BPWM_T::INTSTS: PIF0 Mask              */

#define BPWM_INTSTS_CMPUIF0_Pos          (16)                                              /*!< BPWM_T::INTSTS: CMPUIF0 Position       */
#define BPWM_INTSTS_CMPUIF0_Msk          (0x1ul << BPWM_INTSTS_CMPUIF0_Pos)                /*!< BPWM_T::INTSTS: CMPUIF0 Mask           */

#define BPWM_INTSTS_CMPUIF1_Pos          (17)                                              /*!< BPWM_T::INTSTS: CMPUIF1 Position       */
#define BPWM_INTSTS_CMPUIF1_Msk          (0x1ul << BPWM_INTSTS_CMPUIF1_Pos)                /*!< BPWM_T::INTSTS: CMPUIF1 Mask           */

#define BPWM_INTSTS_CMPUIF2_Pos          (18)                                              /*!< BPWM_T::INTSTS: CMPUIF2 Position       */
#define BPWM_INTSTS_CMPUIF2_Msk          (0x1ul << BPWM_INTSTS_CMPUIF2_Pos)                /*!< BPWM_T::INTSTS: CMPUIF2 Mask           */

#define BPWM_INTSTS_CMPUIF3_Pos          (19)                                              /*!< BPWM_T::INTSTS: CMPUIF3 Position       */
#define BPWM_INTSTS_CMPUIF3_Msk          (0x1ul << BPWM_INTSTS_CMPUIF3_Pos)                /*!< BPWM_T::INTSTS: CMPUIF3 Mask           */

#define BPWM_INTSTS_CMPUIF4_Pos          (20)                                              /*!< BPWM_T::INTSTS: CMPUIF4 Position       */
#define BPWM_INTSTS_CMPUIF4_Msk          (0x1ul << BPWM_INTSTS_CMPUIF4_Pos)                /*!< BPWM_T::INTSTS: CMPUIF4 Mask           */

#define BPWM_INTSTS_CMPUIF5_Pos          (21)                                              /*!< BPWM_T::INTSTS: CMPUIF5 Position       */
#define BPWM_INTSTS_CMPUIF5_Msk          (0x1ul << BPWM_INTSTS_CMPUIF5_Pos)                /*!< BPWM_T::INTSTS: CMPUIF5 Mask           */

#define BPWM_INTSTS_CMPUIFn_Pos          (16)                                              /*!< BPWM_T::INTSTS: CMPUIFn Position       */
#define BPWM_INTSTS_CMPUIFn_Msk          (0x3ful << BPWM_INTSTS_CMPUIFn_Pos)               /*!< BPWM_T::INTSTS: CMPUIFn Mask           */

#define BPWM_INTSTS_CMPDIF0_Pos          (24)                                              /*!< BPWM_T::INTSTS: CMPDIF0 Position       */
#define BPWM_INTSTS_CMPDIF0_Msk          (0x1ul << BPWM_INTSTS_CMPDIF0_Pos)                /*!< BPWM_T::INTSTS: CMPDIF0 Mask           */

#define BPWM_INTSTS_CMPDIF1_Pos          (25)                                              /*!< BPWM_T::INTSTS: CMPDIF1 Position       */
#define BPWM_INTSTS_CMPDIF1_Msk          (0x1ul << BPWM_INTSTS_CMPDIF1_Pos)                /*!< BPWM_T::INTSTS: CMPDIF1 Mask           */

#define BPWM_INTSTS_CMPDIF2_Pos          (26)                                              /*!< BPWM_T::INTSTS: CMPDIF2 Position       */
#define BPWM_INTSTS_CMPDIF2_Msk          (0x1ul << BPWM_INTSTS_CMPDIF2_Pos)                /*!< BPWM_T::INTSTS: CMPDIF2 Mask           */

#define BPWM_INTSTS_CMPDIF3_Pos          (27)                                              /*!< BPWM_T::INTSTS: CMPDIF3 Position       */
#define BPWM_INTSTS_CMPDIF3_Msk          (0x1ul << BPWM_INTSTS_CMPDIF3_Pos)                /*!< BPWM_T::INTSTS: CMPDIF3 Mask           */

#define BPWM_INTSTS_CMPDIF4_Pos          (28)                                              /*!< BPWM_T::INTSTS: CMPDIF4 Position       */
#define BPWM_INTSTS_CMPDIF4_Msk          (0x1ul << BPWM_INTSTS_CMPDIF4_Pos)                /*!< BPWM_T::INTSTS: CMPDIF4 Mask           */

#define BPWM_INTSTS_CMPDIF5_Pos          (29)                                              /*!< BPWM_T::INTSTS: CMPDIF5 Position       */
#define BPWM_INTSTS_CMPDIF5_Msk          (0x1ul << BPWM_INTSTS_CMPDIF5_Pos)                /*!< BPWM_T::INTSTS: CMPDIF5 Mask           */

#define BPWM_INTSTS_CMPDIFn_Pos          (24)                                              /*!< BPWM_T::INTSTS: CMPDIFn Position       */
#define BPWM_INTSTS_CMPDIFn_Msk          (0x3ful << BPWM_INTSTS_CMPDIFn_Pos)               /*!< BPWM_T::INTSTS: CMPDIFn Mask           */

#define BPWM_ADCTS0_TRGSEL0_Pos          (0)                                               /*!< BPWM_T::ADCTS0: TRGSEL0 Position       */
#define BPWM_ADCTS0_TRGSEL0_Msk          (0xful << BPWM_ADCTS0_TRGSEL0_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL0 Mask           */

#define BPWM_ADCTS0_TRGEN0_Pos           (7)                                               /*!< BPWM_T::ADCTS0: TRGEN0 Position        */
#define BPWM_ADCTS0_TRGEN0_Msk           (0x1ul << BPWM_ADCTS0_TRGEN0_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN0 Mask            */

#define BPWM_ADCTS0_TRGSEL1_Pos          (8)                                               /*!< BPWM_T::ADCTS0: TRGSEL1 Position       */
#define BPWM_ADCTS0_TRGSEL1_Msk          (0xful << BPWM_ADCTS0_TRGSEL1_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL1 Mask           */

#define BPWM_ADCTS0_TRGEN1_Pos           (15)                                              /*!< BPWM_T::ADCTS0: TRGEN1 Position        */
#define BPWM_ADCTS0_TRGEN1_Msk           (0x1ul << BPWM_ADCTS0_TRGEN1_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN1 Mask            */

#define BPWM_ADCTS0_TRGSEL2_Pos          (16)                                              /*!< BPWM_T::ADCTS0: TRGSEL2 Position       */
#define BPWM_ADCTS0_TRGSEL2_Msk          (0xful << BPWM_ADCTS0_TRGSEL2_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL2 Mask           */

#define BPWM_ADCTS0_TRGEN2_Pos           (23)                                              /*!< BPWM_T::ADCTS0: TRGEN2 Position        */
#define BPWM_ADCTS0_TRGEN2_Msk           (0x1ul << BPWM_ADCTS0_TRGEN2_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN2 Mask            */

#define BPWM_ADCTS0_TRGSEL3_Pos          (24)                                              /*!< BPWM_T::ADCTS0: TRGSEL3 Position       */
#define BPWM_ADCTS0_TRGSEL3_Msk          (0xful << BPWM_ADCTS0_TRGSEL3_Pos)                /*!< BPWM_T::ADCTS0: TRGSEL3 Mask           */

#define BPWM_ADCTS0_TRGEN3_Pos           (31)                                              /*!< BPWM_T::ADCTS0: TRGEN3 Position        */
#define BPWM_ADCTS0_TRGEN3_Msk           (0x1ul << BPWM_ADCTS0_TRGEN3_Pos)                 /*!< BPWM_T::ADCTS0: TRGEN3 Mask            */

#define BPWM_ADCTS1_TRGSEL4_Pos          (0)                                               /*!< BPWM_T::ADCTS1: TRGSEL4 Position       */
#define BPWM_ADCTS1_TRGSEL4_Msk          (0xful << BPWM_ADCTS1_TRGSEL4_Pos)                /*!< BPWM_T::ADCTS1: TRGSEL4 Mask           */

#define BPWM_ADCTS1_TRGEN4_Pos           (7)                                               /*!< BPWM_T::ADCTS1: TRGEN4 Position        */
#define BPWM_ADCTS1_TRGEN4_Msk           (0x1ul << BPWM_ADCTS1_TRGEN4_Pos)                 /*!< BPWM_T::ADCTS1: TRGEN4 Mask            */

#define BPWM_ADCTS1_TRGSEL5_Pos          (8)                                               /*!< BPWM_T::ADCTS1: TRGSEL5 Position       */
#define BPWM_ADCTS1_TRGSEL5_Msk          (0xful << BPWM_ADCTS1_TRGSEL5_Pos)                /*!< BPWM_T::ADCTS1: TRGSEL5 Mask           */

#define BPWM_ADCTS1_TRGEN5_Pos           (15)                                              /*!< BPWM_T::ADCTS1: TRGEN5 Position        */
#define BPWM_ADCTS1_TRGEN5_Msk           (0x1ul << BPWM_ADCTS1_TRGEN5_Pos)                 /*!< BPWM_T::ADCTS1: TRGEN5 Mask            */

#define BPWM_SSCTL_SSEN0_Pos             (0)                                               /*!< BPWM_T::SSCTL: SSEN0 Position          */
#define BPWM_SSCTL_SSEN0_Msk             (0x1ul << BPWM_SSCTL_SSEN0_Pos)                   /*!< BPWM_T::SSCTL: SSEN0 Mask              */

#define BPWM_SSCTL_SSRC_Pos              (8)                                               /*!< BPWM_T::SSCTL: SSRC Position           */
#define BPWM_SSCTL_SSRC_Msk              (0x3ul << BPWM_SSCTL_SSRC_Pos)                    /*!< BPWM_T::SSCTL: SSRC Mask               */

#define BPWM_SSTRG_CNTSEN_Pos            (0)                                               /*!< BPWM_T::SSTRG: CNTSEN Position         */
#define BPWM_SSTRG_CNTSEN_Msk            (0x1ul << BPWM_SSTRG_CNTSEN_Pos)                  /*!< BPWM_T::SSTRG: CNTSEN Mask             */

#define BPWM_STATUS_CNTMAX0_Pos          (0)                                               /*!< BPWM_T::STATUS: CNTMAX0 Position       */
#define BPWM_STATUS_CNTMAX0_Msk          (0x1ul << BPWM_STATUS_CNTMAX0_Pos)                /*!< BPWM_T::STATUS: CNTMAX0 Mask           */

#define BPWM_STATUS_ADCTRG0_Pos          (16)                                              /*!< BPWM_T::STATUS: ADCTRG0 Position       */
#define BPWM_STATUS_ADCTRG0_Msk          (0x1ul << BPWM_STATUS_ADCTRG0_Pos)                /*!< BPWM_T::STATUS: ADCTRG0 Mask           */

#define BPWM_STATUS_ADCTRG1_Pos          (17)                                              /*!< BPWM_T::STATUS: ADCTRG1 Position       */
#define BPWM_STATUS_ADCTRG1_Msk          (0x1ul << BPWM_STATUS_ADCTRG1_Pos)                /*!< BPWM_T::STATUS: ADCTRG1 Mask           */

#define BPWM_STATUS_ADCTRG2_Pos          (18)                                              /*!< BPWM_T::STATUS: ADCTRG2 Position       */
#define BPWM_STATUS_ADCTRG2_Msk          (0x1ul << BPWM_STATUS_ADCTRG2_Pos)                /*!< BPWM_T::STATUS: ADCTRG2 Mask           */

#define BPWM_STATUS_ADCTRG3_Pos          (19)                                              /*!< BPWM_T::STATUS: ADCTRG3 Position       */
#define BPWM_STATUS_ADCTRG3_Msk          (0x1ul << BPWM_STATUS_ADCTRG3_Pos)                /*!< BPWM_T::STATUS: ADCTRG3 Mask           */

#define BPWM_STATUS_ADCTRG4_Pos          (20)                                              /*!< BPWM_T::STATUS: ADCTRG4 Position       */
#define BPWM_STATUS_ADCTRG4_Msk          (0x1ul << BPWM_STATUS_ADCTRG4_Pos)                /*!< BPWM_T::STATUS: ADCTRG4 Mask           */

#define BPWM_STATUS_ADCTRG5_Pos          (21)                                              /*!< BPWM_T::STATUS: ADCTRG5 Position       */
#define BPWM_STATUS_ADCTRG5_Msk          (0x1ul << BPWM_STATUS_ADCTRG5_Pos)                /*!< BPWM_T::STATUS: ADCTRG5 Mask           */

#define BPWM_STATUS_ADCTRGn_Pos          (16)                                              /*!< BPWM_T::STATUS: ADCTRGn Position       */
#define BPWM_STATUS_ADCTRGn_Msk          (0x3ful << BPWM_STATUS_ADCTRGn_Pos)               /*!< BPWM_T::STATUS: ADCTRGn Mask           */

#define BPWM_CAPINEN_CAPINEN0_Pos        (0)                                               /*!< BPWM_T::CAPINEN: CAPINEN0 Position     */
#define BPWM_CAPINEN_CAPINEN0_Msk        (0x1ul << BPWM_CAPINEN_CAPINEN0_Pos)              /*!< BPWM_T::CAPINEN: CAPINEN0 Mask         */

#define BPWM_CAPINEN_CAPINEN1_Pos        (1)                                               /*!< BPWM_T::CAPINEN: CAPINEN1 Position     */
#define BPWM_CAPINEN_CAPINEN1_Msk        (0x1ul << BPWM_CAPINEN_CAPINEN1_Pos)              /*!< BPWM_T::CAPINEN: CAPINEN1 Mask         */

#define BPWM_CAPINEN_CAPINEN2_Pos        (2)                                               /*!< BPWM_T::CAPINEN: CAPINEN2 Position     */
#define BPWM_CAPINEN_CAPINEN2_Msk        (0x1ul << BPWM_CAPINEN_CAPINEN2_Pos)              /*!< BPWM_T::CAPINEN: CAPINEN2 Mask         */

#define BPWM_CAPINEN_CAPINEN3_Pos        (3)                                               /*!< BPWM_T::CAPINEN: CAPINEN3 Position     */
#define BPWM_CAPINEN_CAPINEN3_Msk        (0x1ul << BPWM_CAPINEN_CAPINEN3_Pos)              /*!< BPWM_T::CAPINEN: CAPINEN3 Mask         */

#define BPWM_CAPINEN_CAPINEN4_Pos        (4)                                               /*!< BPWM_T::CAPINEN: CAPINEN4 Position     */
#define BPWM_CAPINEN_CAPINEN4_Msk        (0x1ul << BPWM_CAPINEN_CAPINEN4_Pos)              /*!< BPWM_T::CAPINEN: CAPINEN4 Mask         */

#define BPWM_CAPINEN_CAPINEN5_Pos        (5)                                               /*!< BPWM_T::CAPINEN: CAPINEN5 Position     */
#define BPWM_CAPINEN_CAPINEN5_Msk        (0x1ul << BPWM_CAPINEN_CAPINEN5_Pos)              /*!< BPWM_T::CAPINEN: CAPINEN5 Mask         */

#define BPWM_CAPINEN_CAPINENn_Pos        (0)                                               /*!< BPWM_T::CAPINEN: CAPINENn Position     */
#define BPWM_CAPINEN_CAPINENn_Msk        (0x3ful << BPWM_CAPINEN_CAPINENn_Pos)             /*!< BPWM_T::CAPINEN: CAPINENn Mask         */

#define BPWM_CAPCTL_CAPEN0_Pos           (0)                                               /*!< BPWM_T::CAPCTL: CAPEN0 Position        */
#define BPWM_CAPCTL_CAPEN0_Msk           (0x1ul << BPWM_CAPCTL_CAPEN0_Pos)                 /*!< BPWM_T::CAPCTL: CAPEN0 Mask            */

#define BPWM_CAPCTL_CAPEN1_Pos           (1)                                               /*!< BPWM_T::CAPCTL: CAPEN1 Position        */
#define BPWM_CAPCTL_CAPEN1_Msk           (0x1ul << BPWM_CAPCTL_CAPEN1_Pos)                 /*!< BPWM_T::CAPCTL: CAPEN1 Mask            */

#define BPWM_CAPCTL_CAPEN2_Pos           (2)                                               /*!< BPWM_T::CAPCTL: CAPEN2 Position        */
#define BPWM_CAPCTL_CAPEN2_Msk           (0x1ul << BPWM_CAPCTL_CAPEN2_Pos)                 /*!< BPWM_T::CAPCTL: CAPEN2 Mask            */

#define BPWM_CAPCTL_CAPEN3_Pos           (3)                                               /*!< BPWM_T::CAPCTL: CAPEN3 Position        */
#define BPWM_CAPCTL_CAPEN3_Msk           (0x1ul << BPWM_CAPCTL_CAPEN3_Pos)                 /*!< BPWM_T::CAPCTL: CAPEN3 Mask            */

#define BPWM_CAPCTL_CAPEN4_Pos           (4)                                               /*!< BPWM_T::CAPCTL: CAPEN4 Position        */
#define BPWM_CAPCTL_CAPEN4_Msk           (0x1ul << BPWM_CAPCTL_CAPEN4_Pos)                 /*!< BPWM_T::CAPCTL: CAPEN4 Mask            */

#define BPWM_CAPCTL_CAPEN5_Pos           (5)                                               /*!< BPWM_T::CAPCTL: CAPEN5 Position        */
#define BPWM_CAPCTL_CAPEN5_Msk           (0x1ul << BPWM_CAPCTL_CAPEN5_Pos)                 /*!< BPWM_T::CAPCTL: CAPEN5 Mask            */

#define BPWM_CAPCTL_CAPENn_Pos           (0)                                               /*!< BPWM_T::CAPCTL: CAPENn Position        */
#define BPWM_CAPCTL_CAPENn_Msk           (0x3ful << BPWM_CAPCTL_CAPENn_Pos)                /*!< BPWM_T::CAPCTL: CAPENn Mask            */

#define BPWM_CAPCTL_CAPINV0_Pos          (8)                                               /*!< BPWM_T::CAPCTL: CAPINV0 Position       */
#define BPWM_CAPCTL_CAPINV0_Msk          (0x1ul << BPWM_CAPCTL_CAPINV0_Pos)                /*!< BPWM_T::CAPCTL: CAPINV0 Mask           */

#define BPWM_CAPCTL_CAPINV1_Pos          (9)                                               /*!< BPWM_T::CAPCTL: CAPINV1 Position       */
#define BPWM_CAPCTL_CAPINV1_Msk          (0x1ul << BPWM_CAPCTL_CAPINV1_Pos)                /*!< BPWM_T::CAPCTL: CAPINV1 Mask           */

#define BPWM_CAPCTL_CAPINV2_Pos          (10)                                              /*!< BPWM_T::CAPCTL: CAPINV2 Position       */
#define BPWM_CAPCTL_CAPINV2_Msk          (0x1ul << BPWM_CAPCTL_CAPINV2_Pos)                /*!< BPWM_T::CAPCTL: CAPINV2 Mask           */

#define BPWM_CAPCTL_CAPINV3_Pos          (11)                                              /*!< BPWM_T::CAPCTL: CAPINV3 Position       */
#define BPWM_CAPCTL_CAPINV3_Msk          (0x1ul << BPWM_CAPCTL_CAPINV3_Pos)                /*!< BPWM_T::CAPCTL: CAPINV3 Mask           */

#define BPWM_CAPCTL_CAPINV4_Pos          (12)                                              /*!< BPWM_T::CAPCTL: CAPINV4 Position       */
#define BPWM_CAPCTL_CAPINV4_Msk          (0x1ul << BPWM_CAPCTL_CAPINV4_Pos)                /*!< BPWM_T::CAPCTL: CAPINV4 Mask           */

#define BPWM_CAPCTL_CAPINV5_Pos          (13)                                              /*!< BPWM_T::CAPCTL: CAPINV5 Position       */
#define BPWM_CAPCTL_CAPINV5_Msk          (0x1ul << BPWM_CAPCTL_CAPINV5_Pos)                /*!< BPWM_T::CAPCTL: CAPINV5 Mask           */

#define BPWM_CAPCTL_CAPINVn_Pos          (8)                                               /*!< BPWM_T::CAPCTL: CAPINVn Position       */
#define BPWM_CAPCTL_CAPINVn_Msk          (0x3ful << BPWM_CAPCTL_CAPINVn_Pos)               /*!< BPWM_T::CAPCTL: CAPINVn Mask           */

#define BPWM_CAPCTL_RCRLDEN0_Pos         (16)                                              /*!< BPWM_T::CAPCTL: RCRLDEN0 Position      */
#define BPWM_CAPCTL_RCRLDEN0_Msk         (0x1ul << BPWM_CAPCTL_RCRLDEN0_Pos)               /*!< BPWM_T::CAPCTL: RCRLDEN0 Mask          */

#define BPWM_CAPCTL_RCRLDEN1_Pos         (17)                                              /*!< BPWM_T::CAPCTL: RCRLDEN1 Position      */
#define BPWM_CAPCTL_RCRLDEN1_Msk         (0x1ul << BPWM_CAPCTL_RCRLDEN1_Pos)               /*!< BPWM_T::CAPCTL: RCRLDEN1 Mask          */

#define BPWM_CAPCTL_RCRLDEN2_Pos         (18)                                              /*!< BPWM_T::CAPCTL: RCRLDEN2 Position      */
#define BPWM_CAPCTL_RCRLDEN2_Msk         (0x1ul << BPWM_CAPCTL_RCRLDEN2_Pos)               /*!< BPWM_T::CAPCTL: RCRLDEN2 Mask          */

#define BPWM_CAPCTL_RCRLDEN3_Pos         (19)                                              /*!< BPWM_T::CAPCTL: RCRLDEN3 Position      */
#define BPWM_CAPCTL_RCRLDEN3_Msk         (0x1ul << BPWM_CAPCTL_RCRLDEN3_Pos)               /*!< BPWM_T::CAPCTL: RCRLDEN3 Mask          */

#define BPWM_CAPCTL_RCRLDEN4_Pos         (20)                                              /*!< BPWM_T::CAPCTL: RCRLDEN4 Position      */
#define BPWM_CAPCTL_RCRLDEN4_Msk         (0x1ul << BPWM_CAPCTL_RCRLDEN4_Pos)               /*!< BPWM_T::CAPCTL: RCRLDEN4 Mask          */

#define BPWM_CAPCTL_RCRLDEN5_Pos         (21)                                              /*!< BPWM_T::CAPCTL: RCRLDEN5 Position      */
#define BPWM_CAPCTL_RCRLDEN5_Msk         (0x1ul << BPWM_CAPCTL_RCRLDEN5_Pos)               /*!< BPWM_T::CAPCTL: RCRLDEN5 Mask          */

#define BPWM_CAPCTL_RCRLDENn_Pos         (16)                                              /*!< BPWM_T::CAPCTL: RCRLDENn Position      */
#define BPWM_CAPCTL_RCRLDENn_Msk         (0x3ful << BPWM_CAPCTL_RCRLDENn_Pos)              /*!< BPWM_T::CAPCTL: RCRLDENn Mask          */

#define BPWM_CAPCTL_FCRLDEN0_Pos         (24)                                              /*!< BPWM_T::CAPCTL: FCRLDEN0 Position      */
#define BPWM_CAPCTL_FCRLDEN0_Msk         (0x1ul << BPWM_CAPCTL_FCRLDEN0_Pos)               /*!< BPWM_T::CAPCTL: FCRLDEN0 Mask          */

#define BPWM_CAPCTL_FCRLDEN1_Pos         (25)                                              /*!< BPWM_T::CAPCTL: FCRLDEN1 Position      */
#define BPWM_CAPCTL_FCRLDEN1_Msk         (0x1ul << BPWM_CAPCTL_FCRLDEN1_Pos)               /*!< BPWM_T::CAPCTL: FCRLDEN1 Mask          */

#define BPWM_CAPCTL_FCRLDEN2_Pos         (26)                                              /*!< BPWM_T::CAPCTL: FCRLDEN2 Position      */
#define BPWM_CAPCTL_FCRLDEN2_Msk         (0x1ul << BPWM_CAPCTL_FCRLDEN2_Pos)               /*!< BPWM_T::CAPCTL: FCRLDEN2 Mask          */

#define BPWM_CAPCTL_FCRLDEN3_Pos         (27)                                              /*!< BPWM_T::CAPCTL: FCRLDEN3 Position      */
#define BPWM_CAPCTL_FCRLDEN3_Msk         (0x1ul << BPWM_CAPCTL_FCRLDEN3_Pos)               /*!< BPWM_T::CAPCTL: FCRLDEN3 Mask          */

#define BPWM_CAPCTL_FCRLDEN4_Pos         (28)                                              /*!< BPWM_T::CAPCTL: FCRLDEN4 Position      */
#define BPWM_CAPCTL_FCRLDEN4_Msk         (0x1ul << BPWM_CAPCTL_FCRLDEN4_Pos)               /*!< BPWM_T::CAPCTL: FCRLDEN4 Mask          */

#define BPWM_CAPCTL_FCRLDEN5_Pos         (29)                                              /*!< BPWM_T::CAPCTL: FCRLDEN5 Position      */
#define BPWM_CAPCTL_FCRLDEN5_Msk         (0x1ul << BPWM_CAPCTL_FCRLDEN5_Pos)               /*!< BPWM_T::CAPCTL: FCRLDEN5 Mask          */

#define BPWM_CAPCTL_FCRLDENn_Pos         (24)                                              /*!< BPWM_T::CAPCTL: FCRLDENn Position      */
#define BPWM_CAPCTL_FCRLDENn_Msk         (0x3ful << BPWM_CAPCTL_FCRLDENn_Pos)              /*!< BPWM_T::CAPCTL: FCRLDENn Mask          */

#define BPWM_CAPSTS_CRIFOV0_Pos          (0)                                               /*!< BPWM_T::CAPSTS: CRIFOV0 Position       */
#define BPWM_CAPSTS_CRIFOV0_Msk          (0x1ul << BPWM_CAPSTS_CRIFOV0_Pos)                /*!< BPWM_T::CAPSTS: CRIFOV0 Mask           */

#define BPWM_CAPSTS_CRIFOV1_Pos          (1)                                               /*!< BPWM_T::CAPSTS: CRIFOV1 Position       */
#define BPWM_CAPSTS_CRIFOV1_Msk          (0x1ul << BPWM_CAPSTS_CRIFOV1_Pos)                /*!< BPWM_T::CAPSTS: CRIFOV1 Mask           */

#define BPWM_CAPSTS_CRIFOV2_Pos          (2)                                               /*!< BPWM_T::CAPSTS: CRIFOV2 Position       */
#define BPWM_CAPSTS_CRIFOV2_Msk          (0x1ul << BPWM_CAPSTS_CRIFOV2_Pos)                /*!< BPWM_T::CAPSTS: CRIFOV2 Mask           */

#define BPWM_CAPSTS_CRIFOV3_Pos          (3)                                               /*!< BPWM_T::CAPSTS: CRIFOV3 Position       */
#define BPWM_CAPSTS_CRIFOV3_Msk          (0x1ul << BPWM_CAPSTS_CRIFOV3_Pos)                /*!< BPWM_T::CAPSTS: CRIFOV3 Mask           */

#define BPWM_CAPSTS_CRIFOV4_Pos          (4)                                               /*!< BPWM_T::CAPSTS: CRIFOV4 Position       */
#define BPWM_CAPSTS_CRIFOV4_Msk          (0x1ul << BPWM_CAPSTS_CRIFOV4_Pos)                /*!< BPWM_T::CAPSTS: CRIFOV4 Mask           */

#define BPWM_CAPSTS_CRIFOV5_Pos          (5)                                               /*!< BPWM_T::CAPSTS: CRIFOV5 Position       */
#define BPWM_CAPSTS_CRIFOV5_Msk          (0x1ul << BPWM_CAPSTS_CRIFOV5_Pos)                /*!< BPWM_T::CAPSTS: CRIFOV5 Mask           */

#define BPWM_CAPSTS_CRIFOVn_Pos          (0)                                               /*!< BPWM_T::CAPSTS: CRIFOVn Position       */
#define BPWM_CAPSTS_CRIFOVn_Msk          (0x3ful << BPWM_CAPSTS_CRIFOVn_Pos)               /*!< BPWM_T::CAPSTS: CRIFOVn Mask           */

#define BPWM_CAPSTS_CFIFOV0_Pos          (8)                                               /*!< BPWM_T::CAPSTS: CFIFOV0 Position       */
#define BPWM_CAPSTS_CFIFOV0_Msk          (0x1ul << BPWM_CAPSTS_CFIFOV0_Pos)                /*!< BPWM_T::CAPSTS: CFIFOV0 Mask           */

#define BPWM_CAPSTS_CFIFOV1_Pos          (9)                                               /*!< BPWM_T::CAPSTS: CFIFOV1 Position       */
#define BPWM_CAPSTS_CFIFOV1_Msk          (0x1ul << BPWM_CAPSTS_CFIFOV1_Pos)                /*!< BPWM_T::CAPSTS: CFIFOV1 Mask           */

#define BPWM_CAPSTS_CFIFOV2_Pos          (10)                                              /*!< BPWM_T::CAPSTS: CFIFOV2 Position       */
#define BPWM_CAPSTS_CFIFOV2_Msk          (0x1ul << BPWM_CAPSTS_CFIFOV2_Pos)                /*!< BPWM_T::CAPSTS: CFIFOV2 Mask           */

#define BPWM_CAPSTS_CFIFOV3_Pos          (11)                                              /*!< BPWM_T::CAPSTS: CFIFOV3 Position       */
#define BPWM_CAPSTS_CFIFOV3_Msk          (0x1ul << BPWM_CAPSTS_CFIFOV3_Pos)                /*!< BPWM_T::CAPSTS: CFIFOV3 Mask           */

#define BPWM_CAPSTS_CFIFOV4_Pos          (12)                                              /*!< BPWM_T::CAPSTS: CFIFOV4 Position       */
#define BPWM_CAPSTS_CFIFOV4_Msk          (0x1ul << BPWM_CAPSTS_CFIFOV4_Pos)                /*!< BPWM_T::CAPSTS: CFIFOV4 Mask           */

#define BPWM_CAPSTS_CFIFOV5_Pos          (13)                                              /*!< BPWM_T::CAPSTS: CFIFOV5 Position       */
#define BPWM_CAPSTS_CFIFOV5_Msk          (0x1ul << BPWM_CAPSTS_CFIFOV5_Pos)                /*!< BPWM_T::CAPSTS: CFIFOV5 Mask           */

#define BPWM_CAPSTS_CFIFOVn_Pos          (8)                                               /*!< BPWM_T::CAPSTS: CFIFOVn Position       */
#define BPWM_CAPSTS_CFIFOVn_Msk          (0x3ful << BPWM_CAPSTS_CFIFOVn_Pos)               /*!< BPWM_T::CAPSTS: CFIFOVn Mask           */

#define BPWM_RCAPDAT0_RCAPDAT_Pos        (0)                                               /*!< BPWM_T::RCAPDAT0: RCAPDAT Position     */
#define BPWM_RCAPDAT0_RCAPDAT_Msk        (0xfffful << BPWM_RCAPDAT0_RCAPDAT_Pos)           /*!< BPWM_T::RCAPDAT0: RCAPDAT Mask         */

#define BPWM_FCAPDAT0_FCAPDAT_Pos        (0)                                               /*!< BPWM_T::FCAPDAT0: FCAPDAT Position     */
#define BPWM_FCAPDAT0_FCAPDAT_Msk        (0xfffful << BPWM_FCAPDAT0_FCAPDAT_Pos)           /*!< BPWM_T::FCAPDAT0: FCAPDAT Mask         */

#define BPWM_RCAPDAT1_RCAPDAT_Pos        (0)                                               /*!< BPWM_T::RCAPDAT1: RCAPDAT Position     */
#define BPWM_RCAPDAT1_RCAPDAT_Msk        (0xfffful << BPWM_RCAPDAT1_RCAPDAT_Pos)           /*!< BPWM_T::RCAPDAT1: RCAPDAT Mask         */

#define BPWM_FCAPDAT1_FCAPDAT_Pos        (0)                                               /*!< BPWM_T::FCAPDAT1: FCAPDAT Position     */
#define BPWM_FCAPDAT1_FCAPDAT_Msk        (0xfffful << BPWM_FCAPDAT1_FCAPDAT_Pos)           /*!< BPWM_T::FCAPDAT1: FCAPDAT Mask         */

#define BPWM_RCAPDAT2_RCAPDAT_Pos        (0)                                               /*!< BPWM_T::RCAPDAT2: RCAPDAT Position     */
#define BPWM_RCAPDAT2_RCAPDAT_Msk        (0xfffful << BPWM_RCAPDAT2_RCAPDAT_Pos)           /*!< BPWM_T::RCAPDAT2: RCAPDAT Mask         */

#define BPWM_FCAPDAT2_FCAPDAT_Pos        (0)                                               /*!< BPWM_T::FCAPDAT2: FCAPDAT Position     */
#define BPWM_FCAPDAT2_FCAPDAT_Msk        (0xfffful << BPWM_FCAPDAT2_FCAPDAT_Pos)           /*!< BPWM_T::FCAPDAT2: FCAPDAT Mask         */

#define BPWM_RCAPDAT3_RCAPDAT_Pos        (0)                                               /*!< BPWM_T::RCAPDAT3: RCAPDAT Position     */
#define BPWM_RCAPDAT3_RCAPDAT_Msk        (0xfffful << BPWM_RCAPDAT3_RCAPDAT_Pos)           /*!< BPWM_T::RCAPDAT3: RCAPDAT Mask         */

#define BPWM_FCAPDAT3_FCAPDAT_Pos        (0)                                               /*!< BPWM_T::FCAPDAT3: FCAPDAT Position     */
#define BPWM_FCAPDAT3_FCAPDAT_Msk        (0xfffful << BPWM_FCAPDAT3_FCAPDAT_Pos)           /*!< BPWM_T::FCAPDAT3: FCAPDAT Mask         */

#define BPWM_RCAPDAT4_RCAPDAT_Pos        (0)                                               /*!< BPWM_T::RCAPDAT4: RCAPDAT Position     */
#define BPWM_RCAPDAT4_RCAPDAT_Msk        (0xfffful << BPWM_RCAPDAT4_RCAPDAT_Pos)           /*!< BPWM_T::RCAPDAT4: RCAPDAT Mask         */

#define BPWM_FCAPDAT4_FCAPDAT_Pos        (0)                                               /*!< BPWM_T::FCAPDAT4: FCAPDAT Position     */
#define BPWM_FCAPDAT4_FCAPDAT_Msk        (0xfffful << BPWM_FCAPDAT4_FCAPDAT_Pos)           /*!< BPWM_T::FCAPDAT4: FCAPDAT Mask         */

#define BPWM_RCAPDAT5_RCAPDAT_Pos        (0)                                               /*!< BPWM_T::RCAPDAT5: RCAPDAT Position     */
#define BPWM_RCAPDAT5_RCAPDAT_Msk        (0xfffful << BPWM_RCAPDAT5_RCAPDAT_Pos)           /*!< BPWM_T::RCAPDAT5: RCAPDAT Mask         */

#define BPWM_FCAPDAT5_FCAPDAT_Pos        (0)                                               /*!< BPWM_T::FCAPDAT5: FCAPDAT Position     */
#define BPWM_FCAPDAT5_FCAPDAT_Msk        (0xfffful << BPWM_FCAPDAT5_FCAPDAT_Pos)           /*!< BPWM_T::FCAPDAT5: FCAPDAT Mask         */

#define BPWM_CAPIEN_CAPRIENn_Pos         (0)                                               /*!< BPWM_T::CAPIEN: CAPRIENn Position      */
#define BPWM_CAPIEN_CAPRIENn_Msk         (0x3ful << BPWM_CAPIEN_CAPRIENn_Pos)              /*!< BPWM_T::CAPIEN: CAPRIENn Mask          */

#define BPWM_CAPIEN_CAPFIENn_Pos         (8)                                               /*!< BPWM_T::CAPIEN: CAPFIENn Position      */
#define BPWM_CAPIEN_CAPFIENn_Msk         (0x3ful << BPWM_CAPIEN_CAPFIENn_Pos)              /*!< BPWM_T::CAPIEN: CAPFIENn Mask          */

#define BPWM_CAPIF_CAPRIF0_Pos           (0)                                               /*!< BPWM_T::CAPIF: CAPRIF0 Position        */
#define BPWM_CAPIF_CAPRIF0_Msk           (0x1ul << BPWM_CAPIF_CAPRIF0_Pos)                 /*!< BPWM_T::CAPIF: CAPRIF0 Mask            */

#define BPWM_CAPIF_CAPRIF1_Pos           (1)                                               /*!< BPWM_T::CAPIF: CAPRIF1 Position        */
#define BPWM_CAPIF_CAPRIF1_Msk           (0x1ul << BPWM_CAPIF_CAPRIF1_Pos)                 /*!< BPWM_T::CAPIF: CAPRIF1 Mask            */

#define BPWM_CAPIF_CAPRIF2_Pos           (2)                                               /*!< BPWM_T::CAPIF: CAPRIF2 Position        */
#define BPWM_CAPIF_CAPRIF2_Msk           (0x1ul << BPWM_CAPIF_CAPRIF2_Pos)                 /*!< BPWM_T::CAPIF: CAPRIF2 Mask            */

#define BPWM_CAPIF_CAPRIF3_Pos           (3)                                               /*!< BPWM_T::CAPIF: CAPRIF3 Position        */
#define BPWM_CAPIF_CAPRIF3_Msk           (0x1ul << BPWM_CAPIF_CAPRIF3_Pos)                 /*!< BPWM_T::CAPIF: CAPRIF3 Mask            */

#define BPWM_CAPIF_CAPRIF4_Pos           (4)                                               /*!< BPWM_T::CAPIF: CAPRIF4 Position        */
#define BPWM_CAPIF_CAPRIF4_Msk           (0x1ul << BPWM_CAPIF_CAPRIF4_Pos)                 /*!< BPWM_T::CAPIF: CAPRIF4 Mask            */

#define BPWM_CAPIF_CAPRIF5_Pos           (5)                                               /*!< BPWM_T::CAPIF: CAPRIF5 Position        */
#define BPWM_CAPIF_CAPRIF5_Msk           (0x1ul << BPWM_CAPIF_CAPRIF5_Pos)                 /*!< BPWM_T::CAPIF: CAPRIF5 Mask            */

#define BPWM_CAPIF_CAPRIFn_Pos           (0)                                               /*!< BPWM_T::CAPIF: CAPRIFn Position        */
#define BPWM_CAPIF_CAPRIFn_Msk           (0x3ful << BPWM_CAPIF_CAPRIFn_Pos)                /*!< BPWM_T::CAPIF: CAPRIFn Mask            */

#define BPWM_CAPIF_CAPFIF0_Pos           (8)                                               /*!< BPWM_T::CAPIF: CAPFIF0 Position        */
#define BPWM_CAPIF_CAPFIF0_Msk           (0x1ul << BPWM_CAPIF_CAPFIF0_Pos)                 /*!< BPWM_T::CAPIF: CAPFIF0 Mask            */

#define BPWM_CAPIF_CAPFIF1_Pos           (9)                                               /*!< BPWM_T::CAPIF: CAPFIF1 Position        */
#define BPWM_CAPIF_CAPFIF1_Msk           (0x1ul << BPWM_CAPIF_CAPFIF1_Pos)                 /*!< BPWM_T::CAPIF: CAPFIF1 Mask            */

#define BPWM_CAPIF_CAPFIF2_Pos           (10)                                              /*!< BPWM_T::CAPIF: CAPFIF2 Position        */
#define BPWM_CAPIF_CAPFIF2_Msk           (0x1ul << BPWM_CAPIF_CAPFIF2_Pos)                 /*!< BPWM_T::CAPIF: CAPFIF2 Mask            */

#define BPWM_CAPIF_CAPFIF3_Pos           (11)                                              /*!< BPWM_T::CAPIF: CAPFIF3 Position        */
#define BPWM_CAPIF_CAPFIF3_Msk           (0x1ul << BPWM_CAPIF_CAPFIF3_Pos)                 /*!< BPWM_T::CAPIF: CAPFIF3 Mask            */

#define BPWM_CAPIF_CAPFIF4_Pos           (12)                                              /*!< BPWM_T::CAPIF: CAPFIF4 Position        */
#define BPWM_CAPIF_CAPFIF4_Msk           (0x1ul << BPWM_CAPIF_CAPFIF4_Pos)                 /*!< BPWM_T::CAPIF: CAPFIF4 Mask            */

#define BPWM_CAPIF_CAPFIF5_Pos           (13)                                              /*!< BPWM_T::CAPIF: CAPFIF5 Position        */
#define BPWM_CAPIF_CAPFIF5_Msk           (0x1ul << BPWM_CAPIF_CAPFIF5_Pos)                 /*!< BPWM_T::CAPIF: CAPFIF5 Mask            */

#define BPWM_CAPIF_CAPFIFn_Pos           (8)                                               /*!< BPWM_T::CAPIF: CAPFIFn Position        */
#define BPWM_CAPIF_CAPFIFn_Msk           (0x3ful << BPWM_CAPIF_CAPFIFn_Pos)                /*!< BPWM_T::CAPIF: CAPFIFn Mask            */

#define BPWM_PBUF_PBUF_Pos               (0)                                               /*!< BPWM_T::PBUF: PBUF Position            */
#define BPWM_PBUF_PBUF_Msk               (0xfffful << BPWM_PBUF_PBUF_Pos)                  /*!< BPWM_T::PBUF: PBUF Mask                */

#define BPWM_CMPBUF0_CMPBUF_Pos          (0)                                               /*!< BPWM_T::CMPBUF0: CMPBUF Position       */
#define BPWM_CMPBUF0_CMPBUF_Msk          (0xfffful << BPWM_CMPBUF0_CMPBUF_Pos)             /*!< BPWM_T::CMPBUF0: CMPBUF Mask           */

#define BPWM_CMPBUF1_CMPBUF_Pos          (0)                                               /*!< BPWM_T::CMPBUF1: CMPBUF Position       */
#define BPWM_CMPBUF1_CMPBUF_Msk          (0xfffful << BPWM_CMPBUF1_CMPBUF_Pos)             /*!< BPWM_T::CMPBUF1: CMPBUF Mask           */

#define BPWM_CMPBUF2_CMPBUF_Pos          (0)                                               /*!< BPWM_T::CMPBUF2: CMPBUF Position       */
#define BPWM_CMPBUF2_CMPBUF_Msk          (0xfffful << BPWM_CMPBUF2_CMPBUF_Pos)             /*!< BPWM_T::CMPBUF2: CMPBUF Mask           */

#define BPWM_CMPBUF3_CMPBUF_Pos          (0)                                               /*!< BPWM_T::CMPBUF3: CMPBUF Position       */
#define BPWM_CMPBUF3_CMPBUF_Msk          (0xfffful << BPWM_CMPBUF3_CMPBUF_Pos)             /*!< BPWM_T::CMPBUF3: CMPBUF Mask           */

#define BPWM_CMPBUF4_CMPBUF_Pos          (0)                                               /*!< BPWM_T::CMPBUF4: CMPBUF Position       */
#define BPWM_CMPBUF4_CMPBUF_Msk          (0xfffful << BPWM_CMPBUF4_CMPBUF_Pos)             /*!< BPWM_T::CMPBUF4: CMPBUF Mask           */

#define BPWM_CMPBUF5_CMPBUF_Pos          (0)                                               /*!< BPWM_T::CMPBUF5: CMPBUF Position       */
#define BPWM_CMPBUF5_CMPBUF_Msk          (0xfffful << BPWM_CMPBUF5_CMPBUF_Pos)             /*!< BPWM_T::CMPBUF5: CMPBUF Mask           */

/**@}*/ /* BPWM_CONST */
/**@}*/ /* end of BPWM register group */

/**
    @addtogroup DAC Digital to Analog Converter(DAC)
    Memory Mapped Structure for DAC Controller
@{ */

typedef struct
{


    /**
     * @var DAC_T::CTL
     * Offset: 0x00  DAC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DACEN     |DAC Enable Bit
     * |        |          |0 = DAC is Disabled.
     * |        |          |1 = DAC is Enabled.
     * |[1]     |DACIEN    |DAC Interrupt Enable Bit
     * |        |          |0 = Interrupt is Disabled.
     * |        |          |1 = Interrupt is Enabled.
     * |[2]     |DMAEN     |DMA Mode Enable Bit
     * |        |          |0 = DMA mode Disabled.
     * |        |          |1 = DMA mode Enabled.
     * |[3]     |DMAURIEN  |DMA Under-run Interrupt Enable Bit
     * |        |          |0 = DMA under-run interrupt Disabled.
     * |        |          |1 = DMA under-run interrupt Enabled.
     * |[4]     |TRGEN     |Trigger Mode Enable Bit
     * |        |          |0 = DAC event trigger mode Disabled.
     * |        |          |1 = DAC event trigger mode Enabled.
     * |[7:5]   |TRGSEL    |Trigger Source Selection
     * |        |          |000 = Software trigger.
     * |        |          |001 = External pin DAC0_ST trigger.
     * |        |          |010 = Timer 0 trigger.
     * |        |          |011 = Timer 1 trigger.
     * |        |          |100 = Timer 2 trigger.
     * |        |          |101 = Timer 3 trigger.
     * |[13:12] |ETRGSEL   |External Pin Trigger Selection
     * |        |          |00 = Low level trigger.
     * |        |          |01 = High level trigger.
     * |        |          |10 = Falling edge trigger.
     * |        |          |11 = Rising edge trigger.
     * @var DAC_T::SWTRG
     * Offset: 0x04  DAC Software Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SWTRG     |Software Trigger
     * |        |          |0 = Software trigger Disabled.
     * |        |          |1 = Software trigger Enabled.
     * |        |          |User writes this bit to generate one shot pulse and it is cleared to 0 by hardware automatically; Reading this bit will always get 0.
     * @var DAC_T::DAT
     * Offset: 0x08  DAC Data Holding Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DACDAT    |DAC 12-bit Holding Data
     * |        |          |These bits are written by user software which specifies 12-bit conversion data for DAC output
     * |        |          |The unused bits (DAC_DAT[3:0] in left-alignment mode and DAC_DAT[15:12] in right alignment mode) are ignored by DAC controller hardware.
     * |        |          |12 bit left alignment: user has to load data into DAC_DAT[15:4] bits.
     * |        |          |12 bit right alignment: user has to load data into DAC_DAT[11:0] bits.
     * @var DAC_T::DATOUT
     * Offset: 0x0C  DAC Data Output Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |DATOUT    |DAC 12-bit Output Data
     * |        |          |These bits are current digital data for DAC output conversion.
     * |        |          |It is loaded from DAC_DAT register and user cannot write it directly.
     * @var DAC_T::STATUS
     * Offset: 0x10  DAC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FINISH    |DAC Conversion Complete Finish Flag
     * |        |          |0 = DAC is in conversion state.
     * |        |          |1 = DAC conversion finish.
     * |        |          |This bit set to 1 when conversion time counter counts to SETTLET
     * |        |          |It is cleared to 0 when DAC starts a new conversion
     * |        |          |User writes 1 to clear this bit to 0.
     * |[1]     |DMAUDR    |DMA Under-run Interrupt Flag
     * |        |          |0 = No DMA under-run error condition occurred.
     * |        |          |1 = DMA under-run error condition occurred.
     * |        |          |User writes 1 to clear this bit.
     * |[8]     |BUSY      |DAC Busy Flag (Read Only)
     * |        |          |0 = DAC is ready for next conversion.
     * |        |          |1 = DAC is busy in conversion.
     * |        |          |This is read only bit.
     * @var DAC_T::TCTL
     * Offset: 0x14  DAC Timing Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |SETTLET   |DAC Output Settling Time
     * |        |          |User software needs to write appropriate value to these bits to meet DAC conversion settling time base on PCLK (APB clock) speed.
     * |        |          |For example, DAC controller clock speed is 80MHz and DAC conversion settling time is 1 us, SETTLETvalue must be greater than 0x50.
     * |        |          |SELTTLET = DAC controller clock speed x settling time.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] DAC Control Register                                             */
    __IO uint32_t SWTRG;                 /*!< [0x0004] DAC Software Trigger Control Register                            */
    __IO uint32_t DAT;                   /*!< [0x0008] DAC Data Holding Register                                        */
    __I  uint32_t DATOUT;                /*!< [0x000c] DAC Data Output Register                                         */
    __IO uint32_t STATUS;                /*!< [0x0010] DAC Status Register                                              */
    __IO uint32_t TCTL;                  /*!< [0x0014] DAC Timing Control Register                                      */
} DAC_T;

/**
    @addtogroup DAC_CONST DAC Bit Field Definition
    Constant Definitions for DAC Controller
@{ */

#define DAC_CTL_DACEN_Pos                (0)                                               /*!< DAC_T::CTL: DACEN Position             */
#define DAC_CTL_DACEN_Msk                (0x1ul << DAC_CTL_DACEN_Pos)                      /*!< DAC_T::CTL: DACEN Mask                 */

#define DAC_CTL_DACIEN_Pos               (1)                                               /*!< DAC_T::CTL: DACIEN Position            */
#define DAC_CTL_DACIEN_Msk               (0x1ul << DAC_CTL_DACIEN_Pos)                     /*!< DAC_T::CTL: DACIEN Mask                */

#define DAC_CTL_DMAEN_Pos                (2)                                               /*!< DAC_T::CTL: DMAEN Position             */
#define DAC_CTL_DMAEN_Msk                (0x1ul << DAC_CTL_DMAEN_Pos)                      /*!< DAC_T::CTL: DMAEN Mask                 */

#define DAC_CTL_DMAURIEN_Pos             (3)                                               /*!< DAC_T::CTL: DMAURIEN Position          */
#define DAC_CTL_DMAURIEN_Msk             (0x1ul << DAC_CTL_DMAURIEN_Pos)                   /*!< DAC_T::CTL: DMAURIEN Mask              */

#define DAC_CTL_TRGEN_Pos                (4)                                               /*!< DAC_T::CTL: TRGEN Position             */
#define DAC_CTL_TRGEN_Msk                (0x1ul << DAC_CTL_TRGEN_Pos)                      /*!< DAC_T::CTL: TRGEN Mask                 */

#define DAC_CTL_TRGSEL_Pos               (5)                                               /*!< DAC_T::CTL: TRGSEL Position            */
#define DAC_CTL_TRGSEL_Msk               (0x7ul << DAC_CTL_TRGSEL_Pos)                     /*!< DAC_T::CTL: TRGSEL Mask                */

#define DAC_CTL_ETRGSEL_Pos              (12)                                              /*!< DAC_T::CTL: ETRGSEL Position           */
#define DAC_CTL_ETRGSEL_Msk              (0x3ul << DAC_CTL_ETRGSEL_Pos)                    /*!< DAC_T::CTL: ETRGSEL Mask               */

#define DAC_SWTRG_SWTRG_Pos              (0)                                               /*!< DAC_T::SWTRG: SWTRG Position           */
#define DAC_SWTRG_SWTRG_Msk              (0x1ul << DAC_SWTRG_SWTRG_Pos)                    /*!< DAC_T::SWTRG: SWTRG Mask               */

#define DAC_DAT_DACDAT_Pos               (0)                                               /*!< DAC_T::DAT: DACDAT Position            */
#define DAC_DAT_DACDAT_Msk               (0xfffful << DAC_DAT_DACDAT_Pos)                  /*!< DAC_T::DAT: DACDAT Mask                */

#define DAC_DATOUT_DATOUT_Pos            (0)                                               /*!< DAC_T::DATOUT: DATOUT Position         */
#define DAC_DATOUT_DATOUT_Msk            (0xffful << DAC_DATOUT_DATOUT_Pos)                /*!< DAC_T::DATOUT: DATOUT Mask             */

#define DAC_STATUS_FINISH_Pos            (0)                                               /*!< DAC_T::STATUS: FINISH Position         */
#define DAC_STATUS_FINISH_Msk            (0x1ul << DAC_STATUS_FINISH_Pos)                  /*!< DAC_T::STATUS: FINISH Mask             */

#define DAC_STATUS_DMAUDR_Pos            (1)                                               /*!< DAC_T::STATUS: DMAUDR Position         */
#define DAC_STATUS_DMAUDR_Msk            (0x1ul << DAC_STATUS_DMAUDR_Pos)                  /*!< DAC_T::STATUS: DMAUDR Mask             */

#define DAC_STATUS_BUSY_Pos              (8)                                               /*!< DAC_T::STATUS: BUSY Position           */
#define DAC_STATUS_BUSY_Msk              (0x1ul << DAC_STATUS_BUSY_Pos)                    /*!< DAC_T::STATUS: BUSY Mask               */

#define DAC_TCTL_SETTLET_Pos             (0)                                               /*!< DAC_T::TCTL: SETTLET Position          */
#define DAC_TCTL_SETTLET_Msk             (0x3fful << DAC_TCTL_SETTLET_Pos)                 /*!< DAC_T::TCTL: SETTLET Mask              */

/**@}*/ /* DAC_CONST */
/**@}*/ /* end of DAC register group */



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

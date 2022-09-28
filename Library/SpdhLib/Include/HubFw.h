/**************************************************************************//**
 * @file     HubFw.h
 * @version  V3.00
 * @brief    Hub control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HUBFW_H__
#define __HUBFW_H__
#include "stdio.h"
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

//#define DBGLOG printf
#define DBGLOG(...)

#define WRNLOG printf
//#define WRNLOG(...)

#define ERRLOG printf
//#define ERRLOG(...)

#define ADC_HSA_VALUE_HID0 620
#define ADC_HSA_VALUE_HID1 880
#define ADC_HSA_VALUE_HID2 1210
#define ADC_HSA_VALUE_HID3 1600
#define ADC_HSA_VALUE_HID4 2030
#define ADC_HSA_VALUE_HID5 2470
#define ADC_HSA_VALUE_HID6 2840
#define ADC_HSA_VALUE_HID7 3170
#define ADC_HSA_VALUE_OFFLINE 0
#define ADC_HSA_VALUE_DEVIATION 100

/**
 * @brief Initial the SPD5 Hub.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To initail the SPD5 Hub function.
 *          
 */
int8_t Hub_Init(void);

/**
 * @brief Re-Initial the SPD5 Hub after bus reset.
 *
 * @param[in] u32I3cAddr The I3C slave address.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To re-Initial the SPD5 Hub function after bus reset.
 *          
 */
int8_t Hub_ResetInit(uint32_t u32I3cAddr);

/**
 * @brief Update temperature sensor value.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To update temperature sensor value to MR registers.
 *          
 */
int8_t Hub_UpdateTempReg(void);

/**
 * @brief Initial temperature sensor.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To initial temperature sensor function.
 *          
 */
int8_t Temp_Init(void);

/**
 * @brief Check if Hub device has changed from I3C mode to I2C mode.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To check if Hub device has changed from I3C mode to I2C mode.
 *          
 */
int8_t Hub_CheckInterfaceSel(void);

/**
 * @brief Update Hub MR register for interface has changed.
 *
 * @param[in] u8InfSel The transfer interface. 0 measn I2C mode and 1 meas I3C mode.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To Update Hub MR register for changed to I3C mode or I2C mode.
 *          
 */
int8_t Hub_UpdateInterfaceSel(uint8_t u8InfSel);

/**
 * @brief Set the pull-up resistor for local bus of SPD5 Hub.
 *
 * @param[in] u32LsdaSel LSDA pull-up resistor.
 * @param[in] u32LsclSel LSCL pull-up resistor.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To set the pull-up resistor for local bus of SPD5 Hub.
 *          
 */
int8_t Hub_SetLocalBusPullUp(uint32_t u32LsdaSel, uint32_t u32LsclSel);

/**
 * @brief Check if need to send IBI request.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To check if need to send IBI request.
 *          
 */
int8_t Hub_CheckIBIReg(void);

/**
 * @brief Get the flag of the bus reset event.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To get if a bus reset event has occurred.
 *          
 */
int8_t Hub_GetBusRstEvt(void);

/**
 * @brief Clear the flag of the bus reset event.
 *
 * @param[in] None.
 *
 * @retval   0 Successful
 * @retval  -1 Unsuccessful
 *
 * @details To clear the flag of the bus reset event.
 *          
 */
int8_t Hub_ClrBusRstEvt(void);

/**
 * @brief The interrupt handler for SPD5 Hub.
 *
 * @param[in] None.
 *
 * @return      None
 *
 * @details To handle interrupt event for SPD5 Hub.
 *          
 */
void Hub_SPDHIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* HUBFW_H */
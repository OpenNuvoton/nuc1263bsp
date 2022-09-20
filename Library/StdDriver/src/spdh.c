/**************************************************************************//**
 * @file     spdh.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 16/10/25 4:25p $
 * @brief    SPDH driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPDH_Driver SPDH Driver
  @{
*/


/** @addtogroup SPDH_EXPORTED_FUNCTIONS SPDH Exported Functions
  @{
*/

/**
 * @brief       Set bus reset timeout counter
 *
 * @param[in]   u32TimeOutCnt    timeout counter. Valid value range is from 0 to 255.
 *                               tTIMEOUT = (u32TimeOutCnt+1) x 16384 (14-bit) x TPCLK
 *
 * @param[in]   u32OnOff         Enable/disable timeout function
 *
 * @return      None
 *
 * @details     This function sets bus reset timeout counter.
 */
void SPDH_SetBusResetTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt)
{
    SPDH->BUSRST = u32TimeOutCnt;

    if(u32OnOff)
        SPDH->CTL |= SPDH_CTL_BUSRSTEN_Msk;
    else
        SPDH->CTL &= ~SPDH_CTL_BUSRSTEN_Msk;
}


/**
 * @brief       Set HSDA switch timeout counter
 *
 * @param[in]   u32TimeOutCnt     timeout counter. Valid value range is from 0 to 63.
 *                                tTIMEOUT = ((u32TimeOutCnt+1) x 16384 (14-bit) x TPCLK
 *
 * @param[in]   u32OnOff          Enable/disable timeout function
 *
 * @return      None
 *
 * @details     This function sets HSDA switch timeout counter.
 */
void SPDH_SetHSDASwitchTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt)
{
    SPDH->HSDASW = u32TimeOutCnt;

    if(u32OnOff)
        SPDH->CTL |= SPDH_CTL_HSDATOEN_Msk;
    else
        SPDH->CTL &= ~SPDH_CTL_HSDATOEN_Msk;
}


/**
 * @brief       Set power down timeout counter
 *
 * @param[in]   u32TimeOutCnt     timeout counter. Valid value range is from 0 to 255.
 *                                tTIMEOUT = (u32TimeOutCnt+1) x 65536 (16-bit) x TPCLK
 *
 * @param[in]   u32OnOff          Enable/disable timeout function
 *
 * @return      None
 *
 * @details     This function sets power down detect timeout counter.
 */
void SPDH_SetPowerDownTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt)
{
    SPDH->PWRD = u32TimeOutCnt;

    if(u32OnOff)
        SPDH->CTL |= SPDH_CTL_PWRWUEN_Msk;
    else
        SPDH->CTL &= ~SPDH_CTL_PWRWUEN_Msk;
}


/*@}*/ /* end of group SPDH_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SPDH_Driver */

/*@}*/ /* end of group Standard_Driver */


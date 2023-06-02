/**************************************************************************//**
 * @file     spdh_device.h
 * @version  V3.00
 * @brief    SPDH driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SPDH_DEVICE_H__
#define __SPDH_DEVICE_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPDH_Driver SPDH Driver
  @{
*/

/** @addtogroup SPDH_EXPORTED_CONSTANTS SPDH Exported Constants
  @{
*/

 /*---------------------------------------------------------------------------------------------------------*/
 /*  pull-up resistor select Definitions                                                                    */
 /*---------------------------------------------------------------------------------------------------------*/
 #define LSCL_EXT_LSDA_EXT_PULLUP    0UL       /*!< LSCL External and LSDA External Pull-Up resistor  \hideinitializer */
 #define LSCL_EXT_LSDA_INT_PULLUP    1UL       /*!< LSCL External and LSDA Internal Pull-Up resistor  \hideinitializer */
 #define LSCL_INT_LSDA_EXT_PULLUP    2UL       /*!< LSCL Internal and LSDA External Pull-Up resistor  \hideinitializer */
 #define LSCL_INT_LSDA_INT_PULLUP    3UL       /*!< LSCL Internal and LSDA Internal Pull-Up resistor  \hideinitializer */

 #define PULLUP_500    0UL            /*!< Pull-up resistor 0.5K(500) ohm  \hideinitializer */
 #define PULLUP_1K     1UL            /*!< Pull-up resistor 1K ohm  \hideinitializer */
 #define PULLUP_2K     2UL            /*!< Pull-up resistor 2K ohm  \hideinitializer */
 #define PULLUP_4K     3UL            /*!< Pull-up resistor 4K ohm  \hideinitializer */


/*@}*/ /* end of group SPDH_EXPORTED_CONSTANTS */

/** @addtogroup SPDH_EXPORTED_FUNCTIONS SPDH Exported Functions
  @{
*/

/**
 * @brief       Set the pull-up configuration for local bus of SPD5 Hub
 *
 * @param[in]   u32PullUpSel. Valid values are:
 *                            - \ref LSCL_EXT_LSDA_EXT_PULLUP    : LSCL External and LSDA External Pull-Up resistor.
 *                            - \ref LSCL_EXT_LSDA_INT_PULLUP    : LSCL External and LSDA Internal Pull-Up resistor.
 *                            - \ref LSCL_INT_LSDA_EXT_PULLUP    : LSCL Internal and LSDA External Pull-Up resistor.
 *                            - \ref LSCL_INT_LSDA_INT_PULLUP    : LSCL Internal and LSDA Internal Pull-Up resistor.
 *
 * @return      None
 *
 * @details     This function sets the pull-up configuration for local bus(LSCL and LSDA) of SPD5 Hub.
 */
void SPDH_SetLocalBusCfg(uint32_t u32PullUpSel);


/**
 * @brief       Set the pull-up resistor for local bus of SPD5 Hub
 *
 * @param[in]   u32SdaPullUp. Valid values are:
 *                            - \ref PULLUP_500    : LSDA pull-up resistor 0.5K(500) ohm.
 *                            - \ref PULLUP_1K     : LSDA pull-up resistor 1K ohm.
 *                            - \ref PULLUP_2K     : LSDA pull-up resistor 2K ohm.
 *                            - \ref PULLUP_4K     : LSDA pull-up resistor 4K ohm.
 *
 * @param[in]   u32SclPullUp. Valid values are:
 *                            - \ref PULLUP_500    : LSCL pull-up resistor 0.5K(500) ohm.
 *                            - \ref PULLUP_1K     : LSCL pull-up resistor 1K ohm.
 *                            - \ref PULLUP_2K     : LSCL pull-up resistor 2K ohm.
 *                            - \ref PULLUP_4K     : LSCL pull-up resistor 4K ohm.
 *
 * @return      None
 *
 * @details     This function sets the pull-up resistor for local bus(LSCL and LSDA) of SPD5 Hub.
 */
void SPDH_SetLocalBusPullUp(uint32_t u32SdaPullUp, uint32_t u32SclPullUp);



/**
 * @brief       Enable specified SPD5 Hub interrupt
 *
 * @param[in]   u32IntSel     The interrupt type select, should be
 *                            - \ref SPDH_INTEN_BUSRTOEN_Msk    : Bus reset time-out interrupt
 *                            - \ref SPDH_INTEN_HDEVCTLEN_Msk   : SPD5 Hub receive DEVCTRL CCC interrupt
 *                            - \ref SPDH_INTEN_DDEVCTLEN_Msk   : SPD5 Hub local device receive DEVCTRL CCC interrupt
 *                            - \ref SPDH_INTEN_HDEVCAPEN_Msk   : SPD5 Hub receive DEVCAP CCC interrupt
 *                            - \ref SPDH_INTEN_DDEVCAPEN_Msk   : SPD5 Hub local device receive DEVCAP CCC interrupt
 *                            - \ref SPDH_INTEN_DSETHIDEN_Msk   : SPD5 Hub local device receive SEDHID CCC interrupt
 *                            - \ref SPDH_INTEN_HUBPCEN_Msk     : SPD5 Hub PEC error check interrupt
 *                            - \ref SPDH_INTEN_DEVPCEN_Msk     : SPD5 Hub local device PEC error check interrupt
 *                            - \ref SPDH_INTEN_HSDATOEN_Msk    : HSDA switch time-out interrupt
 *                            - \ref SPDH_INTEN_HUBIHDEN_Msk    : SPD5 Hub IBI header detect interrupt
 *                            - \ref SPDH_INTEN_DEVIHDEN_Msk    : SPD5 Hub local device IBI header detect interrupt
 *                            - \ref SPDH_INTEN_PWRDTOEN_Msk    : Power down detect time-out interrupt
 *                            - \ref SPDH_INTEN_WKUPEN_Msk      : Wake up interrupt
 *
 * @retval      None
 *
 * @details     This function enables specified SPD5 Hub interrupt.
 */
void SPDH_EnableINT(uint32_t u32IntSel);


/**
 * @brief       Disable specified SPD5 Hub interrupt
 *
 * @param[in]   u32IntSel     The interrupt type select, should be
 *                            - \ref SPDH_INTEN_BUSRTOEN_Msk    : Bus reset time-out interrupt
 *                            - \ref SPDH_INTEN_HDEVCTLEN_Msk   : SPD5 Hub receive DEVCTRL CCC interrupt
 *                            - \ref SPDH_INTEN_DDEVCTLEN_Msk   : SPD5 Hub local device receive DEVCTRL CCC interrupt
 *                            - \ref SPDH_INTEN_HDEVCAPEN_Msk   : SPD5 Hub receive DEVCAP CCC interrupt
 *                            - \ref SPDH_INTEN_DDEVCAPEN_Msk   : SPD5 Hub local device receive DEVCAP CCC interrupt
 *                            - \ref SPDH_INTEN_DSETHIDEN_Msk   : SPD5 Hub local device receive SEDHID CCC interrupt
 *                            - \ref SPDH_INTEN_HUBPCEN_Msk     : SPD5 Hub PEC error check interrupt
 *                            - \ref SPDH_INTEN_DEVPCEN_Msk     : SPD5 Hub local device PEC error check interrupt
 *                            - \ref SPDH_INTEN_HSDATOEN_Msk    : HSDA switch time-out interrupt
 *                            - \ref SPDH_INTEN_HUBIHDEN_Msk    : SPD5 Hub IBI header detect interrupt
 *                            - \ref SPDH_INTEN_DEVIHDEN_Msk    : SPD5 Hub local device IBI header detect interrupt
 *                            - \ref SPDH_INTEN_PWRDTOEN_Msk    : Power down detect time-out interrupt
 *                            - \ref SPDH_INTEN_WKUPEN_Msk      : Wake up interrupt
 *
 * @retval      None
 *
 * @details     This function disables specified SPD5 Hub interrupt.
 */
void SPDH_DisableINT(uint32_t u32IntSel);


/**
 * @brief       Get specified SPD5 Hub interrupt status
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function get specified interrupt status.
 */
uint32_t SPDH_GetINTStatus(void);


/**
 * @brief       Get specified SPD5 Hub interrupt flag
 *
 * @param[in]   u32IntSel     The interrupt type select, should be
 *                            - \ref SPDH_INTSTS_BUSRTOIF_Msk    : Bus reset time-out interrupt flag
 *                            - \ref SPDH_INTSTS_HDEVCTLIF_Msk   : SPD5 Hub receive DEVCTRL CCC interrupt flag
 *                            - \ref SPDH_INTSTS_DDEVCTLIF_Msk   : SPD5 Hub local device receive DEVCTRL CCC interrupt flag
 *                            - \ref SPDH_INTSTS_HDEVCAPIF_Msk   : SPD5 Hub receive DEVCAP CCC interrupt flag
 *                            - \ref SPDH_INTSTS_DDEVCAPIF_Msk   : SPD5 Hub local device receive DEVCAP CCC interrupt flag
 *                            - \ref SPDH_INTSTS_DSETHIDIF_Msk   : SPD5 Hub local device receive SEDHID CCC interrupt flag
 *                            - \ref SPDH_INTSTS_HUBPCIF_Msk     : SPD5 Hub PEC error check interrupt flag
 *                            - \ref SPDH_INTSTS_DEVPCIF_Msk     : SPD5 Hub local device PEC error check interrupt flag
 *                            - \ref SPDH_INTSTS_HSDATOIF_Msk    : HSDA switch time-out interrupt flag
 *                            - \ref SPDH_INTSTS_HUBIHDIF_Msk    : SPD5 Hub IBI header detect interrupt flag
 *                            - \ref SPDH_INTSTS_DEVIHDIF_Msk    : SPD5 Hub local device IBI header detect interrupt flag
 *                            - \ref SPDH_INTSTS_PWRDTOIF_Msk    : Power down detect time-out interrupt flag
 *                            - \ref SPDH_INTSTS_WKUPIF_Msk      : Wake-up interrupt flag
 *
 * @retval      0 The specified interrupt is not happened.
 *              1 The specified interrupt is happened.
 *
 * @details     This function get specified interrupt flag.
 */
uint32_t SPDH_GetINTFlag(uint32_t u32IntFlag);


/**
 * @brief       Clear specified SPD5 Hub interrupt flag
 *
 * @param[in]   u32IntSel     The interrupt type select, should be
 *                            - \ref SPDH_INTSTS_BUSRTOIF_Msk    : Bus reset time-out interrupt flag
 *                            - \ref SPDH_INTSTS_HDEVCTLIF_Msk   : SPD5 Hub receive DEVCTRL CCC interrupt flag
 *                            - \ref SPDH_INTSTS_DDEVCTLIF_Msk   : SPD5 Hub local device receive DEVCTRL CCC interrupt flag
 *                            - \ref SPDH_INTSTS_HDEVCAPIF_Msk   : SPD5 Hub receive DEVCAP CCC interrupt flag
 *                            - \ref SPDH_INTSTS_DDEVCAPIF_Msk   : SPD5 Hub local device receive DEVCAP CCC interrupt flag
 *                            - \ref SPDH_INTSTS_DSETHIDIF_Msk   : SPD5 Hub local device receive SEDHID CCC interrupt flag
 *                            - \ref SPDH_INTSTS_HUBPCIF_Msk     : SPD5 Hub PEC error check interrupt flag
 *                            - \ref SPDH_INTSTS_DEVPCIF_Msk     : SPD5 Hub local device PEC error check interrupt flag
 *                            - \ref SPDH_INTSTS_HSDATOIF_Msk    : HSDA switch time-out interrupt flag
 *                            - \ref SPDH_INTSTS_HUBIHDIF_Msk    : SPD5 Hub IBI header detect interrupt flag
 *                            - \ref SPDH_INTSTS_DEVIHDIF_Msk    : SPD5 Hub local device IBI header detect interrupt flag
 *                            - \ref SPDH_INTSTS_PWRDTOIF_Msk    : Power down detect time-out interrupt flag
 *                            - \ref SPDH_INTSTS_WKUPIF_Msk      : Wake-up interrupt flag
 *
 * @retval      0 The specified interrupt is not happened.
 *              1 The specified interrupt is happened.
 *
 * @details     This function clear specified interrupt flag.
 */
void SPDH_ClearINTFlag(uint32_t u32IntFlag);


/**
 * @brief       Enable slave interrupt request of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function enables slave interrupt request of SPD5 Hub local device.
 * @note        This function needs to be called before setting I3C1 enable bit.
 */
void SPDH_EnableDEVSIR(void);


/**
 * @brief       Disable slave interrupt request of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function disables slave interrupt request of SPD5 Hub local device.
 * @note        This function needs to be called before setting I3C1 enable bit.
 */
void SPDH_DisableDEVSIR(void);


/**
 * @brief       Enable CRC function of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function enables CRC function of SPD5 Hub local device.
 */
void SPDH_EnableDEVCRC(void);


/**
 * @brief       Disable CRC function of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function disables CRC function of SPD5 Hub local device.
 */
void SPDH_DisableDEVCRC(void);


/**
 * @brief       Get status of SPD5 Hub local device
 *
 * @param[in]   None 
 *
 * @return      Status of SPD5 Hub local device
 *
 * @details     This function gets the status of device behind SPD5 Hub.
 */
uint32_t SPDH_GetDEVStatus(void);


/**
 * @brief       Check The Specified status of SPD5 Hub local device
 *
 * @param[in]   u32StsMsk     The status mask, should be
 *                            - \ref SPDH_DSTS_PECSTS_Msk   : PEC enable status
 *                            - \ref SPDH_DSTS_PARDIS_Msk   : Parity disable bit
 *                            - \ref SPDH_DSTS_IBICLR_Msk   : IBI clear bit
 *                            - \ref SPDH_DSTS_PECCHK_Msk   : PEC check result
 *                            - \ref SPDH_DSTS_MODE_Msk     : I3C/I2C MODE status
 *                            - \ref SPDH_DSTS_IBIHEAD_Msk  : IBI Header status
 *                            - \ref SPDH_DSTS_PENDIBI_Msk  : Pending in band interrupt status
 *                            - \ref SPDH_DSTS_PROERR_Msk   : Protocol error
 *                            - \ref SPDH_DSTS_PECERR_Msk   : PEC error 
 *
 * @retval      0 The specified status is not happened.
 *              1 The specified status is happened.
 *
 * @details     This function checks the status of device behind SPD5 Hub.
 */
uint32_t SPDH_IsDEVINTStatus(uint32_t u32StsMsk);


/**
 * @brief       Get the Protocol Mode of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      0    I2C mode.
 * @retval      1    I3C mode.
 *
 * @details     This function gets the protocol mode of device behind SPD5 Hub.
 */
uint32_t SPDH_GetDEVMode(void);


/**
 *    @brief        Check the PEC mode of SPD5 Hub local device is enabled.
 *
 *    @param[in]    None
 *
 *    @retval       0 PEC is not enabled
 *    @retval       1 PEC is enabled
 *
 *    @details      This function returns PEC enable status register bit value.
 *                  It indicates if PEC of device behind SPD5 Hub is enabled nor not.
 */
uint32_t SPDH_IsDEVPECEnable(void); 


/**
 * @brief       Get HID of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function gets the HID value of device behind SPD5 Hub.
 */
uint32_t SPDH_GetDEVHID(void);


/**
 * @brief       Set LID of SPD5 Hub local device
 *
 * @param[in]   4-bits LID
 *
 * @retval      None
 *
 * @details     This function sets the LID value of device behind SPD5 Hub.
 */
void SPDH_SetDEVLID(uint8_t u8LID);


/**
 * @brief       Set DEVCAP value of SPD5 Hub local device
 *
 * @param[in]   DEVCAP value of HUB's device
 *
 * @retval      None
 *
 * @details     This function sets the DEVCAP value of device behind SPD5 Hub.
 */
void SPDH_SetDEVDEVCAP(uint32_t u32Value);


/**
 * @brief       Get DEVCTRL0 value
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function gets the DEVCTRL0 value.
 */
uint32_t SPDH_GetDEVCTRL0(void);


/**
 * @brief       Get DEVCTRL1 value
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This function gets the DEVCTRL1 value.
 */
uint32_t SPDH_GetDEVCTRL1(void);



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
void SPDH_SetBusResetTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt);


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
void SPDH_SetHSDASwitchTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt);


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
void SPDH_SetPowerDownTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt);

/*@}*/ /* end of group SPDH_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SPDH_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPDH_DEVICE_H__


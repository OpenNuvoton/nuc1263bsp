/**************************************************************************//**
 * @file     spdh.h
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 16/10/25 4:25p $
 * @brief    SPDH driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SPDH_H__
#define __SPDH_H__

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
 * @details     This macro sets the pull-up configuration for local bus(LSCL and LSDA) of SPD5 Hub.
 */
#define SPDH_SET_LBUS(u32PullUpSel) ((uint32_t)(SPDH->CTL = (SPDH->CTL &~(SPDH_CTL_LPUSEL_Msk)) | ((u32PullUpSel) << SPDH_CTL_LPUSEL_Pos)))


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
 * @details     This macro sets the pull-up resistor for local bus(LSCL and LSDA) of SPD5 Hub.
 */
#define SPDH_SET_LBUS_PULLUP(u32SdaPullUp, u32SclPullUp) ((uint32_t)(SPDH->CTL = (SPDH->CTL &~(SPDH_CTL_LSDAPUS_Msk|SPDH_CTL_LSCLPUS_Msk)) | \
                                                         ((u32SdaPullUp) << SPDH_CTL_LSDAPUS_Pos)|((u32SclPullUp) << SPDH_CTL_LSCLPUS_Pos)))


/**
 * @brief       Enable SPD5 Hub
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro enables SPD5 Hub.
 * @note        This macro needs to be called before setting I3CS0 enable bit.
 */
#define SPDH_ENABLE_HUB() (SPDH->CTL |= (SPDH_CTL_HUBEN_Msk|SPDH_CTL_SIRENVAL_Msk))


/**
 * @brief       Disable SPD5 Hub
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro disables SPD5 Hub.
 */
#define SPDH_DISABLE_HUB() (SPDH->CTL &= ~SPDH_CTL_HUBEN_Msk)


/**
 * @brief       Enable slave interrupt request of SPD5 Hub
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro enables slave interrupt request of SPD5 Hub.
 * @note        This macro needs to be called before setting I3CS0 enable bit.
 */
#define SPDH_ENABLE_HUB_SIR() (SPDH->CTL &= ~SPDH_CTL_SIRENVAL_Msk)


/**
 * @brief       Disable slave interrupt request of SPD5 Hub
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro disables slave interrupt request of SPD5 Hub.
 * @note        This macro needs to be called before setting I3CS0 enable bit.
 */
#define SPDH_DISABLE_HUB_SIR() (SPDH->CTL |= SPDH_CTL_SIRENVAL_Msk)


/**
 * @brief       Enable SPD5 Hub CRC function
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro enables SPD5 Hub CRC function.
 */
#define SPDH_ENABLE_HUB_CRC() (SPDH->CTL |= SPDH_CTL_CRCEN_Msk)


/**
 * @brief       Disable SPD5 Hub CRC function
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro disables SPD5 Hub CRC function.
 */
#define SPDH_DISABLE_HUB_CRC() (SPDH->CTL &= ~SPDH_CTL_CRCEN_Msk)


/**
 * @brief       Set SPD5 Hub HID
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro sets the HID value of SPD5 Hub.
 */
#define SPDH_SET_HUB_HID(u32Hid) (SPDH->HID = (u32Hid))


/**
 * @brief       Set DEVCAP value of SPD5 Hub
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro sets the DEVCAP value of SPD5 Hub.
 */
#define SPDH_SET_HUB_DEVCAP(u32Value) (SPDH->HCAP = (u32Value))


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
 * @details     This macro enables specified SPD5 Hub interrupt.
 */
#define SPDH_ENABLE_INT(u32IntSel) (SPDH->INTEN |= (u32IntSel))


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
 * @details     This macro disables specified SPD5 Hub interrupt.
 */
#define SPDH_DISABLE_INT(u32IntSel) (SPDH->INTEN &= (~(u32IntSel)))


/**
 * @brief       Get specified SPD5 Hub interrupt status
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro get specified interrupt status.
 */
#define SPDH_GET_INT_STATUS() ((uint32_t)(SPDH->INTSTS))


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
 * @details     This macro get specified interrupt flag.
 */
#define SPDH_GET_INT_FLAG(u32IntFlag) ((SPDH->INTSTS & (u32IntFlag))?1:0)


/**
 * @brief       Get SPD5 Hub status
 *
 * @param[in]   None
 *
 * @return      The SPD5 Hub status
 *
 * @details     This macro gets the SPD5 Hub status.
 */
#define SPDH_GET_HUB_STATUS() (SPDH->STS)


/**
 * @brief       Check The Specified SPD5 Hub status
 *
 * @param[in]   u32StsMsk     The status mask, should be
 *                            - \ref SPDH_STS_PECSTS_Msk   : PEC enable status
 *                            - \ref SPDH_STS_PARDIS_Msk   : Parity disable bit
 *                            - \ref SPDH_STS_IBICLR_Msk   : IBI clear bit
 *                            - \ref SPDH_STS_PECCHK_Msk   : PEC check result
 *                            - \ref SPDH_STS_MODE_Msk     : I3C/I2C MODE status
 *                            - \ref SPDH_STS_IBIHEAD_Msk  : IBI Header status
 *                            - \ref SPDH_STS_PENDIBI_Msk  : Pending in band interrupt status
 *                            - \ref SPDH_STS_PROERR_Msk   : Protocol error
 *                            - \ref SPDH_STS_PECERR_Msk   : PEC error 
 *
 * @retval      0 The specified status is not happened.
 *              1 The specified status is happened.
 *
 * @details     This macro checks the SPD5 Hub status.
 */
#define SPDH_IS_HUB_INT_STATUS(u32StsMsk) ((SPDH->STS & (u32StsMsk))?1:0)

/**
 * @brief       Get SPD5 Hub protocol mode
 *
 * @param[in]   None
 *
 * @retval      0    I2C mode.
 * @retval      1    I3C mode.
 *
 * @details     This macro gets the prototcol mode of SPD5 Hub.
 */
#define SPDH_GET_HUB_MODE() ((uint32_t)((SPDH->STS & SPDH_STS_MODE_Msk) >> SPDH_STS_MODE_Pos))


/**
 *    @brief        Check the PEC mode of SPD5 Hub is enabled.
 *
 *    @param[in]    None
 *
 *    @retval       0 PEC is not enabled
 *    @retval       1 PEC is enabled
 *
 *    @details      This macro returns PEC enable status register bit value.
 *                  It indicates if PEC of SPD5 Hub is enabled nor not.
 */
#define SPDH_IS_HUB_PEC_ENABLE() ((uint32_t)(SPDH->STS & SPDH_STS_PECSTS_Msk) >> SPDH_STS_PECSTS_Pos)


/**
 * @brief       Clear the PEC error status of SPD5 Hub
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This macro clears the PEC error status of SPD5 Hub.
 */
#define SPDH_CLEAR_HUB_PEC_ERR() ((uint32_t)(SPDH->CTL = SPDH_CTL_PECCLR_Msk)


/**
 * @brief       Clear the global event status of SPD5 Hub
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This macro clears the global event status of SPD5 Hub.
 */
#define SPDH_CLEAR_HUB_EVENT() ((uint32_t)(SPDH->CTL = SPDH_CTL_EVENTCLR_Msk)


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
 * @details     This macro clear specified interrupt flag.
 */
#define SPDH_CLEAR_INT_FLAG(u32IntFlag) (SPDH->INTSTS = (u32IntFlag))


/**
 * @brief       Enable slave interrupt request of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro enables slave interrupt request of SPD5 Hub local device.
 * @note        This macro needs to be called before setting I3C1 enable bit.
 */
#define SPDH_ENABLE_DEV_SIR() (SPDH->DCTL &= ~SPDH_DCTL_SIRENVAL_Msk)


/**
 * @brief       Disable slave interrupt request of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro disables slave interrupt request of SPD5 Hub local device.
 * @note        This macro needs to be called before setting I3C1 enable bit.
 */
#define SPDH_DISABLE_DEV_SIR() (SPDH->DCTL |= SPDH_DCTL_SIRENVAL_Msk)


/**
 * @brief       Enable CRC function of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro enables CRC function of SPD5 Hub local device.
 */
#define SPDH_ENABLE_DEV_CRC() (SPDH->DCTL |= SPDH_DCTL_CRCEN_Msk)


/**
 * @brief       Disable CRC function of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro disables CRC function of SPD5 Hub local device.
 */
#define SPDH_DISABLE_DEV_CRC() (SPDH->DCTL &= ~SPDH_DCTL_CRCEN_Msk)


/**
 * @brief       Get status of SPD5 Hub local device
 *
 * @param[in]   None 
 *
 * @return      Status of SPD5 Hub local device
 *
 * @details     This macro gets the status of device behind SPD5 Hub.
 */
#define SPDH_GET_DEV_STATUS() (SPDH->DSTS)


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
 * @details     This macro checks the status of device behind SPD5 Hub.
 */
#define SPDH_IS_DEV_INT_STATUS(u32StsMsk) ((SPDH->DSTS & (u32StsMsk))?1:0)

/**
 * @brief       Get the Protocol Mode of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      0    I2C mode.
 * @retval      1    I3C mode.
 *
 * @details     This macro gets the protocol mode of device behind SPD5 Hub.
 */
#define SPDH_GET_DEV_MODE() ((uint32_t)((SPDH->DSTS & SPDH_DSTS_MODE_Msk) >> SPDH_DSTS_MODE_Pos))


/**
 *    @brief        Check the PEC mode of SPD5 Hub local device is enabled.
 *
 *    @param[in]    None
 *
 *    @retval       0 PEC is not enabled
 *    @retval       1 PEC is enabled
 *
 *    @details      This macro returns PEC enable status register bit value.
 *                  It indicates if PEC of device behind SPD5 Hub is enabled nor not.
 */
#define SPDH_IS_DEV_PEC_ENABLE() ((uint32_t)(SPDH->DSTS & SPDH_DSTS_PECSTS_Msk) >> SPDH_DSTS_PECSTS_Pos)


/**
 * @brief       Clear the PEC error status of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This macro clears the PEC error status of device behind SPD5 Hub.
 */
#define SPDH_CLEAR_DEV_PEC_ERR() ((uint32_t)(SPDH->DCTL = SPDH_DCTL_PECCLR_Msk)


/**
 * @brief       Clear the global event status of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This macro clears the global event status of device behind SPD5 Hub.
 */
#define SPDH_CLEAR_DEV_EVENT() ((uint32_t)(SPDH->DCTL = SPDH_DCTL_EVENTCLR_Msk)


/**
 * @brief       Get HID of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro gets the HID value of device behind SPD5 Hub.
 */
#define SPDH_GET_DEV_HID() (SPDH->DCTL & SPDH_DCTL_HID_Msk)


/**
 * @brief       Set LID of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro sets the LID value of device behind SPD5 Hub.
 */
#define SPDH_SET_DEV_LID(u32Lid) (SPDH->DCTL = ((SPDH->DCTL & ~SPDH_DCTL_LID_Msk) | ((u32Lid) << SPDH_DCTL_LID_Pos)))


/**
 * @brief       Set DEVCAP value of SPD5 Hub local device
 *
 * @param[in]   None
 *
 * @retval      None
 *
 * @details     This macro sets the DEVCAP value of device behind SPD5 Hub.
 */
#define SPDH_SET_DEV_DEVCAP(u32Value) (SPDH->DCAP = (u32Value))



/*---------------------------------------------------------------------------------------------------------*/
/* Define SPDH functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void SPDH_SetBusResetTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void SPDH_SetHSDASwitchTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void SPDH_SetPowerDownTimeout(uint32_t u32OnOff, uint32_t u32TimeOutCnt);

/*@}*/ /* end of group SPDH_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SPDH_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPDH_H__


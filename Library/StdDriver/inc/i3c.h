/**************************************************************************//**
 * @file     i3c.h
 * @version  V3.00
 * @brief    NUC1263 series I3C Controller (I3C) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __I3C_H__
#define __I3C_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup I3C_Driver I3C Driver
  @{
*/


/** @addtogroup I3C_EXPORTED_CONSTANTS I3C Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Maximum RX/TX FIFO and Response/Command Queue Constant Definitions                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_DEVICE_RX_BUF_CNT               (16UL)  /*!< Maximum RX FIFO count of I3C */
#define I3C_DEVICE_TX_BUF_CNT               (16UL)  /*!< Maximum TX FIFO count of I3C */
#define I3C_DEVICE_RESP_QUEUE_CNT           (2UL)   /*!< Maximum Response Queue count of I3C */
#define I3C_DEVICE_CMD_QUEUE_CNT            (4UL)   /*!< Maximum Command Queue count of I3C */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Support ENTDAA CCC or Hot-Join Generation Constant Definitions                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_SUPPORT_ENTDAA                  (0UL)   /*!< Support to receive ENTDAA CCC */
#define I3C_SUPPORT_ADAPTIVE_HJ             (1UL)   /*!< Support to initiate Hot-Join after receiving broadcast header 7'h7E */
#define I3C_SUPPORT_IMMEDIATE_HJ            (2UL)   /*!< Support to initiate Hot-Join immediately after I3C controller enabled */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Error Status in Response Queue Constant Definitions                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_RESP_NO_ERR                     (0UL << I3C_RESPQUE_ERRSTS_Pos)     /*!< No error in response queue */
#define I3C_RESP_CRC_ERR                    (1UL << I3C_RESPQUE_ERRSTS_Pos)     /*!< CRC error in response queue */
#define I3C_RESP_PARITY_ERR                 (2UL << I3C_RESPQUE_ERRSTS_Pos)     /*!< Parity error in response queue */
#define I3C_RESP_FRAME_ERRR                 (3UL << I3C_RESPQUE_ERRSTS_Pos)     /*!< Frame error in response queue */
#define I3C_RESP_FLOW_ERR                   (6UL << I3C_RESPQUE_ERRSTS_Pos)     /*!< Underflow/Overflow in response queue */
#define I3C_RESP_MASTER_TERMINATE_ERR       (10UL << I3C_RESPQUE_ERRSTS_Pos)    /*!< Master early termination error in response queue */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Reset Operation Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_RESET_CMD_QUEUE                 (I3C_RSTCTL_CMDRST_Msk)     /*!< Command Queue software reset */
#define I3C_RESET_RESP_QUEUE                (I3C_RSTCTL_RESPRST_Msk)    /*!< Response Queue software reset */
#define I3C_RESET_TX_BUF                    (I3C_RSTCTL_TXRST_Msk)      /*!< TX FIFO buffer software reset */
#define I3C_RESET_RX_BUF                    (I3C_RSTCTL_RXRST_Msk)      /*!< RX FIFO buffer software reset */
#define I3C_RESET_ALL_QUEUE_AND_BUF         (0x1EUL)                    /*!< Command, Response Queue and TX, RX FIFO buffer software reset */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Interrupt Status Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_INTSTS_TX_EMPTY_THLD            (I3C_INTSTS_TXTH_Msk)       /*!< TX FIFO empty count threshold status */
#define I3C_INTSTS_RX_THLD                  (I3C_INTSTS_RXTH_Msk)       /*!< RX FIFO received count threshold status */
#define I3C_INTSTS_CMDQ_EMPTY_THLD          (I3C_INTSTS_CMDRDY_Msk)     /*!< Command Queue empty count threshold status */
#define I3C_INTSTS_RESPQ_READY              (I3C_INTSTS_RESPRDY_Msk)    /*!< Response Queue received count threshold status */
#define I3C_INTSTS_CCC_UPDATED              (I3C_INTSTS_CCCUPD_Msk)     /*!< CCC register value updated status */
#define I3C_INTSTS_DA_ASSIGNED              (I3C_INTSTS_DAA_Msk)        /*!< Dynamic Address assigned status */
#define I3C_INTSTS_TRANSFER_ERR             (I3C_INTSTS_TFRERR_Msk)     /*!< Transfer error status */
#define I3C_INTSTS_READ_REQUEST             (I3C_INTSTS_READREQ_Msk)    /*!< Read request received status */
#define I3C_INTSTS_IBI_UPDATED              (I3C_INTSTS_IBIUPD_Msk)     /*!< IBI request status */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Interrupt Enable Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_INTEN_TX_EMPTY_THLD             (I3C_INTEN_TXTH_Msk)        /*!< Enable TX FIFO empty count threshold interrupt */
#define I3C_INTEN_RX_THLD                   (I3C_INTEN_RXTH_Msk)        /*!< Enable RX FIFO received count threshold interrupt */
#define I3C_INTEN_CMDQ_EMPTY_THLD           (I3C_INTEN_CMDRDY_Msk)      /*!< Enable Command Queue empty count threshold interrupt */
#define I3C_INTEN_RESPQ_READY               (I3C_INTEN_RESPRDY_Msk)     /*!< Enable Response Queue received count threshold interrupt */
#define I3C_INTEN_CCC_UPDATED               (I3C_INTEN_CCCUPD_Msk)      /*!< Enable CCC register value updated interrupt */
#define I3C_INTEN_DA_ASSIGNED               (I3C_INTEN_DAA_Msk)         /*!< Enable Dynamic Address Assigned interrupt */
#define I3C_INTEN_TRANSFER_ERR              (I3C_INTEN_TFRERR_Msk)      /*!< Enable transfer error interrupt */
#define I3C_INTEN_READ_REQUEST              (I3C_INTEN_READREQ_Msk)     /*!< Enable read request received interrupt */
#define I3C_INTEN_IBI_UPDATED               (I3C_INTEN_IBIUPD_Msk)      /*!< Enable IBI request interrupt */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C IBI Response Status Constant Definitions                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_IBI_ACCEPTED                    (1UL << I3C_SIRRESP_IBISTS_Pos) /*!< IBI request accepted by the Master */
#define I3C_IBI_MASTER_TERMINATE            (2UL << I3C_SIRRESP_IBISTS_Pos) /*!< IBI request early terminate by the Master */
#define I3C_IBI_NOT_ATTEMPTED               (3UL << I3C_SIRRESP_IBISTS_Pos) /*!< IBI request not attempted */

/*---------------------------------------------------------------------------------------------------------*/
/*  I3C Operation Return Status Constant Definitions                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C_STS_NO_ERR                      (0L)        /*!< Return no error */
#define I3C_STS_SUCCESS                     (0L)        /*!< Return success */
#define I3C_STS_TX_FULL                     (-1L)       /*!< Return TX FIFO full */
#define I3C_STS_CMDQ_FULL                   (-3L)       /*!< Return Command Queue full */
#define I3C_STS_RESPQ_EMPTY                 (-4L)       /*!< Return Response Queue empty */  
#define I3C_STS_DMA_ERR                     (-5L)       /*!< Return DMA error */ 
#define I3C_STS_INVALID_INPUT               (-1000L)    /*!< Return invalid input parameter */ 
#define I3C_STS_INVALID_STATE               (-2000L)    /*!< Return invalid/unexpected state */ 

#define I3C_TIMEOUT_ERR                     (-1L)       /*!< I3C operation abort due to timeout error */

/*@}*/ /* end of group I3C_EXPORTED_CONSTANTS */


/** @addtogroup I3C_EXPORTED_FUNCTIONS I3C Exported Functions
  @{
*/

/**
  * @brief      Check DMA is enabled
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       DMA function is disabled
  * @retval     1       DMA function is enabled
  *
  * @details    This macro checks if the DMA function is enabled.
  */
#define I3C_IS_ENABLE_DMA(i3c)              ((((i3c)->DEVCTL&I3C_DEVCTL_DMAEN_Msk)==I3C_DEVCTL_DMAEN_Msk)? 1:0)

/**
  * @brief      Get I2C Static Address
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The I2C Static Address value
  *
  * @details    This macro gets the I2C Static Address.
  */
#define I3C_GET_I2C_SA(i3c)                 (((i3c)->DEVADDR&I3C_DEVADDR_SA_Msk) >> I3C_DEVADDR_SA_Pos)

/**
  * @brief      Check I2C Sataic Address is valid
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       I2C Static Address is invalid
  * @retval     1       I2C Static Address is valid
  *
  * @details    This macro checks if the I2C Sataic Address is valid.
  */
#define I3C_IS_SA_VALID(i3c)                ((((i3c)->DEVADDR&I3C_DEVADDR_SAVALID_Msk)==I3C_DEVADDR_SAVALID_Msk)? 1:0)

        /**
  * @brief      Get I3C Dynamic Address
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The I3C Dynamic Address value
  *
  * @details    This macro gets the I3C Dynamic Address.
  */
#define I3C_GET_I3C_DA(i3c)                 (((i3c)->DEVADDR&I3C_DEVADDR_DA_Msk) >> I3C_DEVADDR_DA_Pos)

/**
  * @brief      Check I3C Dymanic Address is valid
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       I3C Dymanic Address is invalid
  * @retval     1       I3C Dymanic Address is valid
  *
  * @details    This macro checks if the I3C Dymanic Address is valid.
  */
#define I3C_IS_DA_VALID(i3c)            ((((i3c)->DEVADDR&I3C_DEVADDR_DAVALID_Msk)==I3C_DEVADDR_DAVALID_Msk)? 1:0)

/**
  * @brief      Get Response Queue Threshold
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The Response Queue threshold value
  *
  * @details    This macro gets the Response Queue threshold value.
  *             It affects the I3C_INTSTS[4] RESPRDY (Response Queue Ready Status).
  *             A value of N sets the threshold for N+1 entries to generate a RESPRDY status when N+1 queues received.
  */
#define I3C_GET_RESPQ_THLD(i3c)             (((i3c)->QUETHCTL&I3C_QUETHCTL_RESPTH_Msk) >> I3C_QUETHCTL_RESPTH_Pos)

/**
  * @brief      Get Command Queue Empty Threshold
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The Command Queue empty threshold value
  *
  * @details    This macro gets the Command Queue empty threshold value.
  *             It affects the I3C_INTSTS[3] CMDRDY (Command Queue Ready Status).  
  *             A value of 0 sets the threshold to generate a CMDRDY status when the queue is completely empty.
  *             A value of N sets the threshold to generate a CMDRDY status when at least N empty queues.
  */
#define I3C_GET_CMDQ_EMPTY_THLD(i3c)        (((i3c)->QUETHCTL&I3C_QUETHCTL_CMDETH_Msk) >> I3C_QUETHCTL_CMDETH_Pos)

/**
  * @brief      Check Response Queue is Empty
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Response Queue is not empty
  * @retval     1       Response Queue is empty
  *
  * @details    This macro checks if the Response Queue is empty.
  */
#define I3C_IS_RESPQ_EMPTY(i3c)             ((((i3c)->QUESTSLV&I3C_QUESTSLV_RESPLV_Msk)==0)? 1:0)

/**
  * @brief      Get Response Queue Count
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The number of Response Queue received
  *
  * @details    This macro gets the number of Response Queue received.
  */
#define I3C_GET_RESPQ_CNT(i3c)              (((i3c)->QUESTSLV&I3C_QUESTSLV_RESPLV_Msk) >> I3C_QUESTSLV_RESPLV_Pos)

/**
  * @brief      Check Command Queue is Full
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Command Queue is not full
  * @retval     1       Command Queue is full
  * 
  * @details    This macro checks if the Command Queue is full determined by I3C_QUESTSLV[7:0] CMDELOC (Command Queue Empty Locations).
  */
#define I3C_IS_CMDQ_FULL(i3c)               ((((i3c)->QUESTSLV&I3C_QUESTSLV_CMDELOC_Msk)==0)? 1:0)

/**
  * @brief      Check TX FIFO is Full
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       TX FIFO is not full
  * @retval     1       TX FIFO is full
  * 
  * @details    This macro checks if the TX FIFO is full determined by I3C_DBSTSLV[7:0] TXELV (Transmit Buffer Empty Level Value).
  */
#define I3C_IS_TX_FULL(i3c)                 ((((i3c)->DBSTSLV&I3C_DBSTSLV_TXELV_Msk)==0)? 1:0)

/**
  * @brief      Enable Specified I3C Interrupt
  *
  * @param[in]  i3c         The pointer of the specified I3C module
  * @param[in]  u32IntSel   Interrupt type select
  *                             - \ref I3C_INTEN_TX_EMPTY_THLD  : TX FIFO empty count threshold interrupt
  *                             - \ref I3C_INTEN_RX_THLD        : RX FIFO received count threshold interrupt
  *                             - \ref I3C_INTEN_CMDQ_EMPTY_THLD: Command Queue empty count threshold interrupt
  *                             - \ref I3C_INTEN_RESPQ_READY    : Response Queue received count threshold interrupt
  *                             - \ref I3C_INTEN_CCC_UPDATED    : CCC register value updated interrupt
  *                             - \ref I3C_INTEN_DA_ASSIGNED    : Dynamic Address Assigned interrupt
  *                             - \ref I3C_INTEN_TRANSFER_ERR   : Transfer error interrupt
  *                             - \ref I3C_INTEN_READ_REQUEST   : Read request received interrupt
  *                             - \ref I3C_INTEN_IBI_UPDATED    : IBI request interrupt
  *
  * @return       None
  *
  * @details      This macro enables specified I3C interrupts.
  */
#define I3C_ENABLE_INT(i3c, u32IntSel)      ((i3c)->INTEN |= (u32IntSel))

/**
  * @brief      Disable Specified I3C Interrupt
  *
  * @param[in]  i3c         The pointer of the specified I3C module
  * @param[in]  u32IntSel   Interrupt type select
  *                             - \ref I3C_INTEN_TX_EMPTY_THLD  : TX FIFO empty count threshold interrupt
  *                             - \ref I3C_INTEN_RX_THLD        : RX FIFO received count threshold interrupt
  *                             - \ref I3C_INTEN_CMDQ_EMPTY_THLD: Command Queue empty count threshold interrupt
  *                             - \ref I3C_INTEN_RESPQ_READY    : Response Queue received count threshold interrupt
  *                             - \ref I3C_INTEN_CCC_UPDATED    : CCC register value updated interrupt
  *                             - \ref I3C_INTEN_DA_ASSIGNED    : Dynamic Address Assigned interrupt
  *                             - \ref I3C_INTEN_TRANSFER_ERR   : Transfer error interrupt
  *                             - \ref I3C_INTEN_READ_REQUEST   : Read request received interrupt
  *                             - \ref I3C_INTEN_IBI_UPDATED    : IBI request interrupt
  *
  * @return       None
  *
  * @details      This macro disables specified I3C interrupts.
  */
#define I3C_DISABLE_INT(i3c, u32IntSel)     ((i3c)->INTEN &= ~(u32IntSel))

/**
  * @brief      Check Specified Interrupt Status Occurred
  *
  * @param[in]  i3c         The pointer of the specified I3C module
  * @param[in]  u32IntSel   Interrupt type select
  *                             - \ref I3C_INTSTS_TX_EMPTY_THLD  : TX FIFO empty count threshold status
  *                             - \ref I3C_INTSTS_RX_THLD        : RX FIFO received count threshold status
  *                             - \ref I3C_INTSTS_CMDQ_EMPTY_THLD: Command Queue empty count threshold status
  *                             - \ref I3C_INTSTS_RESPQ_READY    : Response Queue received count threshold status
  *                             - \ref I3C_INTSTS_CCC_UPDATED    : CCC register value updated status
  *                             - \ref I3C_INTSTS_DA_ASSIGNED    : Dynamic Address Assigned status
  *                             - \ref I3C_INTSTS_TRANSFER_ERR   : Transfer error status
  *                             - \ref I3C_INTSTS_READ_REQUEST   : Read request received status
  *                             - \ref I3C_INTSTS_IBI_UPDATED    : IBI request status
  *
  * @retval     0       Specified interrupts did not occur
  * @retval     1       Specified interrupts occurred
  *
  * @details    This macro checks if the specific interrupts occurred.
  */
#define I3C_IS_INT_STATUS(i3c, u32IntSel)   ((((i3c)->INTSTS&(u32IntSel))==(u32IntSel))? 1:0)

/**
  * @brief      Clear CCC UPDATED Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  * 
  * @details    This macro clears CCC UPDATED status.
  */
#define I3C_CLEAR_CCC_UPDATED_STATUS(i3c)   ((i3c)->INTSTS = I3C_INTSTS_CCCUPD_Msk)

/**
  * @brief      Clear Dynamic Address Assigned Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  * 
  * @details    This macro clears Dynamic Address Assigned status.
  */
#define I3C_CLEAR_DA_ASSIGNED_STATUS(i3c)   ((i3c)->INTSTS = I3C_INTSTS_DAA_Msk)

/**
  * @brief      Clear Transfer Error Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  * 
  * @details    This macro clears transfer error status.
  */
#define I3C_CLEAR_TRANSFER_ERR_STATUS(i3c)  ((i3c)->INTSTS = I3C_INTSTS_TFRERR_Msk)

/**
  * @brief      Clear Read Request Received Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  * 
  * @details    This macro clears read request received status.
  */
#define I3C_CLEAR_READ_REQUEST_STATUS(i3c)  ((i3c)->INTSTS = I3C_INTSTS_READREQ_Msk)

/**
  * @brief      Clear IBI Request Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  * 
  * @details    This macro clears IBI request status.
  */
#define I3C_CLEAR_IBI_UPDATED_STATUS(i3c)   ((i3c)->INTSTS = I3C_INTSTS_IBIUPD_Msk)

/**
  * @brief      Get Response Data
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The data in the Response Queue
  *
  * @details    This macro gets the data from the Response Queue.
  */
#define I3C_GET_RESP_DATA(i3c)              ((i3c)->RESPQUE)

/**
  * @brief      Check Response Status is No Error
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Response has error
  * @retval     1       Response has no error
  * 
  * @details    This macro checks if response status is error free.
  */
#define I3C_IS_RESP_NO_ERR(resp)            ((((resp)&I3C_RESPQUE_ERRSTS_Msk)==0)? 1:0)

/**
  * @brief      Check Response Status is Receive or Transmit
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Transmit response status
  * @retval     1       Receive response status
  * 
  * @details    This macro checks if receive response status.
  */
#define I3C_IS_RESP_RX(resp)                ((((resp)&I3C_RESPQUE_RXRESP_Msk)==0)? 0:1)

/**
  * @brief      Get Transmit Transaction ID
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The Transmit Transaction ID
  *
  * @details    This macro gets the Transmit Transaction ID from the response data for the transmit command.
  */
#define I3C_GET_RESP_TID(resp)              (((resp)&I3C_RESPQUE_TID_Msk) >> I3C_RESPQUE_TID_Pos)

/**
  * @brief      Get HDR Command Code
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The HDR Command Code
  *
  * @details    This macro gets the HDR Command Code from the response data.
  */
#define I3C_GET_RESP_HDR_CODE(resp)         (((resp)&I3C_RESPQUE_HDRCODE_Msk) >> I3C_RESPQUE_HDRCODE_Pos)

/**
  * @brief      Get Data Length
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The data length in bytes
  *
  * @details    This macro gets the data length in bytes of Master write transfer or the remaining length of Master read Transfer.
  */
#define I3C_GET_RESP_DATA_LEN(resp)         (((resp)&I3C_RESPQUE_DATLEN_Msk) >> I3C_RESPQUE_DATLEN_Pos)

/**
  * @brief      Get Present Transmit Transaction ID
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The Transmit Transaction ID in the Present State register
  *
  * @details    This macro gets the Transmit Transaction ID of the current executing command in the Present State register.
  */
#define I3C_GET_PRESENT_TID(i3c)            (((i3c)->PRESENTS&I3C_PRESENTS_TID_Msk) >> I3C_PRESENTS_TID_Pos)

/**
  * @brief      Get Present Transfer Type Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The transfer type status in the Present State register
  *
  * @details    This macro gets the transfer type status in the Present State register.
  */
#define I3C_GET_PRESENT_STATUS(i3c)         (((i3c)->PRESENTS&I3C_PRESENTS_TFRTYPE_Msk) >> I3C_PRESENTS_TFRTYPE_Pos)

/**
  * @brief      Get Device Operation Status
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The device operation status
  *
  * @details    This macro gets the device operation status, 
  *             this data is also used for the I3C Slave controller sends in response to GETSTATUS CCC by the Master.
  */
#define I3C_GET_DEVICE_STATUS(i3c)          ((i3c)->CCCDEVS)

/**
  * @brief      Get Pending Interrupt
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     The pending interrupt number
  *
  * @details    This macro gets the pending interrupt number. 
  */
#define I3C_GET_PENDING_INT(i3c)            (((i3c)->CCCDEVS&I3C_CCCDEVS_PENDINT_Msk) >> I3C_CCCDEVS_PENDINT_Pos)

/**
  * @brief      Check Protocol Error Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Protocol error did not occur
  * @retval     1       Protocol error occurred
  * 
  * @details    This macro checks if protocol error occurred. This status can be cleared after receiving GETSTATUS CCC.
  */
#define I3C_IS_PROTOCOL_ERR(i3c)            ((((i3c)->CCCDEVS&I3C_CCCDEVS_PROTERR_Msk)==0)? 0:1)
  
/**
  * @brief      Check Underflow Error Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Underflow error did not occur
  * @retval     1       Underflow error occurred
  * 
  * @details    This macro checks if underflow error occurred. This status can be cleared after receiving GETSTATUS CCC.
  */
#define I3C_IS_UNDERFLOW_ERR(i3c)           ((((i3c)->CCCDEVS&I3C_CCCDEVS_UDFERR_Msk)==0)? 0:1)

/**
  * @brief      Check Slave Busy Status Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Slave busy status error did not occur
  * @retval     1       Slave busy status occurred
  * 
  * @details    This macro checks if Slave busy status occurred. This status can be cleared after the RESUME operation is complete.
  */
#define I3C_IS_SLAVE_BUSY(i3c)              ((((i3c)->CCCDEVS&I3C_CCCDEVS_SLVBUSY_Msk)==0)? 0:1)

/**
  * @brief      Check Overflow Error Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Overflow error did not occur
  * @retval     1       Overflow error occurred
  * 
  * @details    This macro checks if overflow error occurred. This status can be cleared after receiving GETSTATUS CCC.
  */
#define I3C_IS_OVERFLOW_ERR(i3c)            ((((i3c)->CCCDEVS&I3C_CCCDEVS_OVFERR_Msk)==0)? 0:1)

/**
  * @brief      Check Data Not Ready Status Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Data not ready status did not occur
  * @retval     1       Data not ready status occurred
  * 
  * @details    This macro checks if data not ready status occurred. 
  *             This status can be cleared after receiving GETSTATUS CCC or successful the next Slave write transfer.
  */
#define I3C_IS_DATA_NOT_READY(i3c)          ((((i3c)->CCCDEVS&I3C_CCCDEVS_DATNRDY_Msk)==0)? 0:1)

/**
  * @brief      Check Buffer Not Available Status Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Buffer not available status did not occur
  * @retval     1       Buffer not available status occurred
  * 
  * @details    This macro checks if buffer not available status occurred. 
  *             This status can be cleared after receiving GETSTATUS CCC or successful the next Slave read transfer.
  */
#define I3C_IS_BUFFER_NOT_AVAIL(i3c)        ((((i3c)->CCCDEVS&I3C_CCCDEVS_BFNAVAIL_Msk)==0)? 0:1)

/**
  * @brief      Check Frame Error Occurred
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     0       Frame error did not occur
  * @retval     1       Frame error occurred
  * 
  * @details    This macro checks if frame error occurred. This status can be cleared after receiving GETSTATUS CCC.
  */
#define I3C_IS_FRAME_ERR(i3c)               ((((i3c)->CCCDEVS&I3C_CCCDEVS_FRAMEERR_Msk)==0)? 0:1)


/*---------------------------------------------------------------------------------------------------------*/
/*  Static Inline Functions                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE int32_t I3C_Enable(I3C_T *i3c);
__STATIC_INLINE int32_t I3C_Disable(I3C_T *i3c);
__STATIC_INLINE void    I3C_EnableDMA(I3C_T *i3c);
__STATIC_INLINE void    I3C_DisableDMA(I3C_T *i3c);

/**
  * @brief      Enable I3C Controller
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Enable I3C time-out
  *
  * @details    This function is used to enable I3C controller.
  */
__STATIC_INLINE int32_t I3C_Enable(I3C_T *i3c)
{
    volatile uint32_t u32Timeout;
    
    i3c->DEVCTL |= I3C_DEVCTL_ENABLE_Msk;
    u32Timeout = (SystemCoreClock / 1000);
    while(((i3c->DEVCTL&I3C_DEVCTL_SYNC_Msk) == I3C_DEVCTL_SYNC_Msk) && (--u32Timeout)) {}
    if(u32Timeout == 0)
        return I3C_TIMEOUT_ERR;
    
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Disable I3C Controller
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Disable I3C time-out
  *
  * @details    This function is used to disable I3C controller.
  */
__STATIC_INLINE int32_t I3C_Disable(I3C_T *i3c)
{
    volatile uint32_t u32Timeout;
    
    i3c->DEVCTL &= ~I3C_DEVCTL_ENABLE_Msk;
    u32Timeout = (SystemCoreClock / 1000);
    while(((i3c->DEVCTL&I3C_DEVCTL_SYNC_Msk) == I3C_DEVCTL_SYNC_Msk) && (--u32Timeout)) {}
    if(u32Timeout == 0)
        return I3C_TIMEOUT_ERR;
        
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Enable I3C DMA Function
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  *
  * @details    This function is used to enable I3C DMA function.
  */
__STATIC_INLINE void I3C_EnableDMA(I3C_T *i3c)
{
    i3c->DEVCTL &= ~I3C_DEVCTL_DMAEN_Msk;
    i3c->DEVCTL |= I3C_DEVCTL_DMAEN_Msk;
}

/**
  * @brief      Disable I3C DMA Function
  *
  * @param[in]  i3c     The pointer of the specified I3C module
  *
  * @return     None
  *
  * @details    This function is used to disable I3C DMA function.
  */
__STATIC_INLINE void I3C_DisableDMA(I3C_T *i3c)
{
    i3c->DEVCTL &= ~I3C_DEVCTL_DMAEN_Msk;
}

//typedef int32_t (*I3C_FUNC_PDMA)(I3C_T *i3c, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts);  /*!< Functional pointer type definition for perform I3C DMA operation */

void    I3C_Open(I3C_T *i3c, uint8_t u8StaticAddr, uint32_t u32ModeSel);
int32_t I3C_ResetAndResume(I3C_T *i3c, uint32_t u32ResetMask, uint32_t u32EnableResume);
int32_t I3C_ParseRespQueue(I3C_T *i3c, uint32_t *pu32RespQ);
int32_t I3C_SetCmdQueueAndData(I3C_T *i3c, uint8_t u8TID, uint32_t *pu32TxBuf, uint16_t u16WriteBytes);
int32_t I3C_SendIBIRequest(I3C_T *i3c, uint8_t u8MandatoryData, uint32_t u32PayloadData, uint8_t u8PayloadLen);
int32_t I3C_EnableHJRequest(I3C_T *i3c, uint32_t u32ModeSel); 
int32_t I3C_DisableHJRequest(I3C_T *i3c); 
int32_t I3C_RespErrorRecovery(I3C_T *i3c, uint32_t u32RespStatus);

/*@}*/ /* end of group I3C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I3C_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__I3C_H__

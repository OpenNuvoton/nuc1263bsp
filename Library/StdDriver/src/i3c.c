/**************************************************************************//**
 * @file     i3c.c
 * @version  V3.00
 * @brief    NUC1263 series I3C Controller (I3C) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup I3C_Driver I3C Driver
  @{
*/

/** @addtogroup I3C_EXPORTED_FUNCTIONS I3C Exported Functions
  @{
*/

/**
  * @brief      Open I3C with Static Address and Initial Mode
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u8StaticAddr    7-bit slave address for I2C operation.
  * @param[in]  u32ModeSel      Initial mode selection to support ENTDAA CCC or Hot-Join generation. Valid values are:
  *                                 - \ref I3C_SUPPORT_ENTDAA
  *                                 - \ref I3C_SUPPORT_HJ_ADAPTIVE
  *                                 - \ref I3C_SUPPORT_HJ_IMMEDIATE
  *
  * @return     None
  *
  * @details    This API is used to configure I3C controller can receive ENTDAA CCC or generate a Hot-Join request.
  *             The 7-bit slave address is used for related I2C operations until I3C controller has a valid Dynamic Address.
  * @note       This API also enables all interrupt status and configures TX, RX FIFO and Response, Command Queue threshold to 0.
  */
void I3C_Open(I3C_T *i3c, uint8_t u8StaticAddr, uint32_t u32ModeSel)
{        
    if(u8StaticAddr == 0)
    {
        i3c->DEVADDR &= ~I3C_DEVADDR_SAVALID_Msk;
    }
    else
    {
        i3c->DEVADDR &= ~I3C_DEVADDR_SA_Msk;
        i3c->DEVADDR |= (I3C_DEVADDR_SAVALID_Msk | u8StaticAddr);
    }     
    
    /* Enable all interrupt status */
    i3c->INTSTSEN = 0xFFFFFFFF;
    
    /* Set Tx/Rx/CmdQ/ReapQ threshold to 0 */
    i3c->DBTHCTL = 0x0;
    i3c->QUETHCTL = 0x0;
        
    /* Select the slave ability for enter I3C mode */
    if(u32ModeSel == I3C_SUPPORT_ENTDAA)
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    }
    else if(u32ModeSel == I3C_SUPPORT_HJ_ADAPTIVE)
    {
        /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3C header 7'h7E on the bus */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else if(u32ModeSel == I3C_SUPPORT_HJ_IMMEDIATE)
    {
        /* Only HJEN enabled: Slave generates a Hot-Join request immediately */
        i3c->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    }
}

/**
  * @brief      Reset and Resume I3C Controller
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32ResetMask    Software reset operation of I3C module. Valid values are:
  *                                 - \ref I3C_RESET_CMD_QUEUE
  *                                 - \ref I3C_RESET_RESP_QUEUE
  *                                 - \ref I3C_RESET_TX_BUF
  *                                 - \ref I3C_RESET_RX_BUF
  *                                 - \ref I3C_RESET_ALL_QUEUE_AND_BUF
  * @param[in]  u32EnableResume Enable resume I3C controller function. Valid values are TRUE and FALSE.
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Function time-out
  *
  * @details    This API is used to reset specified FIFO and Queue or resume the I3C controller from the halt state.
  */
int32_t I3C_ResetAndResume(I3C_T *i3c, uint32_t u32ResetMask, uint32_t u32EnableResume)
{
    volatile uint32_t u32Timeout;
    
    if(u32ResetMask)
    {
        /* Disable I3C controller for reset buffer and queue */
        if(I3C_Disable(i3c) != I3C_STS_NO_ERR)
            return I3C_TIMEOUT_ERR;

        /* Reset specify source */
        i3c->RSTCTL = u32ResetMask;
        u32Timeout = SystemCoreClock;
        while((i3c->RSTCTL != 0) && (--u32Timeout)) {}
        if(u32Timeout == 0)
            return I3C_TIMEOUT_ERR;
                    
        /* Enable I3C controller again */
        if(I3C_Enable(i3c) != I3C_STS_NO_ERR)
            return I3C_TIMEOUT_ERR;
    }
    
    if(u32EnableResume)
    {
        /* The application has to take necessary action to handle the error condition and 
            then set RESUME bit to resume the controller. */
        /* Slave will receive GETSTATUS CCC to clear specify status in I3C_CCCDEVS register. */
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        while((i3c->DEVCTL&I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}
            
        /* RESUME bit is auto-cleared once the controller is ready to accept new transfers. */
    }
    
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Get Resopnse Status and Receive Data
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  pRespQ          The response data structure to load current response data.
  * @param[in]  pu32RxBuf       The buffer to receive the data of receive FIFO.
  * @param[in]  pfnI3CPDMARx    RX PDMA request callback function.
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_RESPQ_EMPTY     No Response Queue data
  * @retval     I3C_STS_DMA_ERR         Perform DMA fail
  * @return     Response error status
  *
  * @details    This API is used to get the response data and the received data on Master write operation.
  */
int32_t I3C_ParseRespQueue(I3C_T *i3c, S_I3C_RESP_QUEUE_T *pRespQ, uint32_t *pu32RxBuf, I3C_FUNC_PDMA pfnI3CPDMARx)
{
    uint32_t i;
    
    /* Check if RespQ buffer is empty */
    if(pRespQ == NULL)
        return I3C_STS_INVALID_INPUT;
    
    /* Check if RespQ is empty */
    if(I3C_IS_RESPQ_EMPTY(i3c))
        return I3C_STS_RESPQ_EMPTY;
    
    pRespQ->w = I3C_GET_RESP_DATA(i3c);
    
    if(I3C_IS_RESP_NO_ERR(pRespQ->w))
    {
        if(I3C_IS_RESP_RX(pRespQ->w))
        {
            /* Check if pu32RxBuf buffer is empty */
            if(pu32RxBuf == NULL)
                return I3C_STS_INVALID_INPUT;
            
            if(I3C_IS_ENABLE_DMA(i3c))
            {
                if(pfnI3CPDMARx == NULL)
                    return I3C_STS_INVALID_INPUT;
                
                if(!pfnI3CPDMARx(i3c, (uint32_t)&(i3c->TXRXDAT), (uint32_t)pu32RxBuf, I3C_GET_RESP_DATA_LEN(pRespQ->w)))
                    return I3C_STS_DMA_ERR;
            }
            else
            {
                for(i=0; i<((I3C_GET_RESP_DATA_LEN(pRespQ->w)+3)/4); i++)
                {
                    pu32RxBuf[i] = i3c->TXRXDAT;
                }
            }
        }
        
        return I3C_STS_NO_ERR;
    }
    
    return pRespQ->b.ERRSTS;
}

/**
  * @brief      Prepare Transmit Data for Master Read
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u8TID           Specified Transmit Transaction ID in Command Queue.
  * @param[in]  pu32TxBuf       The buffer to send the data to transmit FIFO.
  * @param[in]  u16WriteBytes   The byte number of TX data.
  * @param[in]  pfnI3CPDMATx    TX PDMA request callback function.
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3C_STS_TX_FULL         TX FIFO is full
  * @retval     I3C_STS_DMA_ERR         Perform DMA fail
  *
  * @details    This API is used to prepare TX response data for Master read operation.
  */
int32_t I3C_PrepareTXData(I3C_T *i3c, uint8_t u8TID, uint32_t *pu32TxBuf, uint16_t u16WriteBytes, I3C_FUNC_PDMA pfnI3CPDMATx)
{
    uint32_t i;
    
    /* Check if input data buffer is empty */
    if(pu32TxBuf == NULL)
        return I3C_STS_INVALID_INPUT;
    
    /* Check if write bytes is exceeded */
    if(u16WriteBytes > (I3C_DEVICE_TX_BUF_CNT *4 ))
        return I3C_STS_INVALID_INPUT;

    /* Check if CmdQ is full */
    if(I3C_IS_CMDQ_FULL(i3c))
        return I3C_STS_CMDQ_FULL;
        
    if(I3C_IS_ENABLE_DMA(i3c))
    {
        if(pfnI3CPDMATx == NULL)
            return I3C_STS_INVALID_INPUT;
        
        if(!pfnI3CPDMATx(i3c, (uint32_t)pu32TxBuf, (uint32_t)&(i3c->TXRXDAT), u16WriteBytes))
            return I3C_STS_DMA_ERR;
    }
    else
    {
        for(i=0; i<((u16WriteBytes+3)/4); i++)
        {
            /* Check if TX buffer is full */
            if(I3C_IS_TX_FULL(i3c))
                return I3C_STS_TX_FULL;
            
            i3c->TXRXDAT = pu32TxBuf[i];
        }
    }
    
    i3c->CMDQUE = ((u8TID << I3C_CMDQUE_TID_Pos) | (u16WriteBytes << I3C_CMDQUE_DATLEN_Pos));
        
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Generate IBI Request
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u8MDB           The mandatory data byte.
  * @param[in]  pu32PayloadBuf  The payload data buffer.
  * @param[in]  u8PayloadLen    The byte number of payload data. The maximum length is 4 bytes.
  *
  * @retval     I3C_STS_NO_ERR              No error, IBI request accepted by the Master
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @retval     I3C_STS_INVALID_INPUT       Invalid input parameter
  * @retval     I3C_TIMEOUT_ERR             IBI request time-out
  * @retval     I3C_IBI_MASTER_TERMINATE    Master early terminate
  * @retval     I3C_IBI_NOT_ATTEMPTED       IBI not attempted
  *
  * @details    This API is used to generate an IBI request on the bus.
  */
int32_t I3C_SendIBIRequest(I3C_T *i3c, uint8_t u8MDB, uint32_t *pu32PayloadBuf, uint8_t u8PayloadLen)
{
    volatile uint32_t u32Timeout;
    
    /* Check if SIR function enabled */
    if(!(i3c->SLVEVNTS&I3C_SLVEVNTS_SIREN_Msk))
        return I3C_STS_INVALID_STATE;
    
    /* Clear CmdQ and Tx buffer */
    if(I3C_ResetAndResume(i3c, (I3C_RESET_CMD_QUEUE | I3C_RESET_TX_BUF), 0) != 0)
        return I3C_STS_INVALID_STATE;
    
    /* Check if payload buffer is empty */
    if(pu32PayloadBuf == NULL)
        return I3C_STS_INVALID_INPUT;

    /* Check if payload length > 4-bytes */
    if(u8PayloadLen > 4)
        return I3C_STS_INVALID_INPUT;

    /* Program IBI payload data, payload length and MDB */
    i3c->SIR = (u8PayloadLen << I3C_SIR_DATLEN_Pos) | (u8MDB << I3C_SIR_MDB_Pos) | (0 << I3C_SIR_CTL_Pos);
    i3c->SIRDAT = pu32PayloadBuf[0];
        
    /* Trigger IBI */
    i3c->SIR |= I3C_SIR_EN_Msk;
    u32Timeout = SystemCoreClock;
    while(((i3c->SIR&I3C_SIR_EN_Msk) == I3C_SIR_EN_Msk) && (--u32Timeout)) {}
    if(u32Timeout == 0)
        return I3C_TIMEOUT_ERR;
    
    /* IBI master early terminate */
    if(i3c->SIRRESP&I3C_IBI_MASTER_TERMINATE)
    {
        /* Clear CmdQ, Tx buffer and resume the I3C controller */
        if(I3C_ResetAndResume(i3c, (I3C_RESET_CMD_QUEUE | I3C_RESET_TX_BUF), 1) != 0)
            return I3C_IBI_MASTER_TERMINATE;
    }
    
    /* IBI NOT attempted */
    if(i3c->SIRRESP&I3C_IBI_NOT_ATTEMPTED)
        return I3C_IBI_NOT_ATTEMPTED;
    
    /* IBI accepted by the Master or unexpected error */
    if(i3c->SIRRESP&I3C_IBI_ACCEPTED)
        return I3C_STS_NO_ERR; // IBI accepted by the Master (ACK response received)
    else
        return I3C_STS_INVALID_STATE;
}

/**
  * @brief      Enable HJ Generation
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32ModeSel      Select the Hot-Join generation method. Valid values are:
  *                                 - \ref I3C_SUPPORT_HJ_ADAPTIVE
  *                                 - \ref I3C_SUPPORT_HJ_IMMEDIATE
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_TIMEOUT_ERR         Enable/Disable I3C time-out
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  *
  * @details    This API is used to enable the specified HJ generation after enable I3C controller again.
  * @note       I3C controller does not recognize the ENTDAA CCC after enabling the HJ generation.
  */
int32_t I3C_EnableHJRequest(I3C_T *i3c, uint32_t u32ModeSel)
{
    volatile uint32_t u32Timeout;
    
    I3C_ResetAndResume(i3c, I3C_RESET_ALL_QUEUE_AND_BUF, 0);
    
    /* Disable I3C controller */
    if(I3C_Disable(i3c) != I3C_STS_NO_ERR)
        return I3C_TIMEOUT_ERR;
        
    /* Select the slave ability for enter I3C mode */
    if(u32ModeSel == I3C_SUPPORT_HJ_ADAPTIVE)
    {
        /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3C header 7'h7E on the bus */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else if(u32ModeSel == I3C_SUPPORT_HJ_IMMEDIATE)
    {
        /* Only HJEN enabled: Slave generates a Hot-Join request immediately */
        i3c->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
        
        /* Enable I3C controller */
        if(I3C_Enable(i3c) != I3C_STS_NO_ERR)
            return I3C_TIMEOUT_ERR;
        
        return I3C_STS_INVALID_INPUT;
    }
    
    /* Enable I3C controller */
    if(I3C_Enable(i3c) != I3C_STS_NO_ERR)
        return I3C_TIMEOUT_ERR;
    
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Disable HJ Generation
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Enable/Disable I3C time-out
  *
  * @details    This API is used to disable the HJ generation.
  * @note       I3C controller can recognize the ENTDAA CCC after disabling the HJ generation.
  */
int32_t I3C_DisableHJRequest(I3C_T *i3c)
{
    /* Disable I3C controller */
    if(I3C_Disable(i3c) != I3C_STS_NO_ERR)
        return I3C_TIMEOUT_ERR;
    
    /* HJEN disabled: Slave supports ENTDAA CCC */
    i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
    i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    
    /* Enable I3C controller */
    if(I3C_Enable(i3c) != I3C_STS_NO_ERR)
        return I3C_TIMEOUT_ERR;
    
    return I3C_STS_NO_ERR;
}

/**
  * @brief      I3C Controller Error Recovery
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  *
  * @retval     I3C_STS_SUCCESS         Error recovery successfully
  * @retval     I3C_STS_INVALID_STATE   Invalid state
  *
  * @details    This API is used to perform error recovery then the I3C controller can leave busy state and enter to idle state again.
  * @note       The I3C controller may wait for the RESUME bit to clear automatically until a GETSTATUS CCC is received.
  */
int32_t I3C_ErrorRecovery(I3C_T *i3c)
{
    uint32_t u32RespQCnt;
    S_I3C_RESP_QUEUE_T sRespQ;
    
    if(!I3C_IS_INT_STATUS(i3c, I3C_INTSTS_TRANSFER_ERR))
        return I3C_STS_INVALID_STATE;

    I3C_CLEAR_TRANSFER_ERR_STATUS(i3c);
    
    /* Check if RespQ interrupt status ready */
    if(I3C_IS_INT_STATUS(i3c, I3C_INTSTS_RESPQ_READY))
    {
        /* RespQ interrupt occurs, get valid RespQ counts */
        u32RespQCnt = I3C_GET_RESPQ_THLD(i3c) + 1;
        if(u32RespQCnt > I3C_DEVICE_RESP_QUEUE_CNT)
            u32RespQCnt = 2;
    }
    else
    {
        /* No RespQ interrupt, get valid RespQ counts */
        u32RespQCnt = I3C_GET_RESPQ_CNT(i3c);
        if(u32RespQCnt == 0)
            return I3C_STS_SUCCESS;
    }
    
    /* Check RespQ error status, and u32RespQCnt may be 1 or 2 */
    do {
        sRespQ = (S_I3C_RESP_QUEUE_T)I3C_GET_RESP_DATA(i3c);
        switch(sRespQ.w&I3C_RESPQUE_ERRSTS_Msk)
        {
            case I3C_RESP_NO_ERR:
                /* No error status in RespQ */
                break;
            case I3C_RESP_CRC_ERR:
            case I3C_RESP_PARITY_ERR:
            case I3C_RESP_FRAME_ERRR:
            case I3C_RESP_FLOW_ERR:
                /* Reset RX FIFO -> apply resume */
                I3C_ResetAndResume(i3c, I3C_RESET_RX_BUF, 1);
                return I3C_STS_SUCCESS;
            default:
                /* I3C_RESP_MASTER_TERMINATE_ERR or others error */
                /* Reset TX FIFO and CMDQ FIFO  -> apply resume */
                I3C_ResetAndResume(i3c, (I3C_RESET_TX_BUF | I3C_RESET_CMD_QUEUE), 1);
                return I3C_STS_SUCCESS;
        }
    }while(u32RespQCnt--);
    
    return I3C_STS_SUCCESS;
}

/*@}*/ /* end of group I3C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I3C_Driver */

/*@}*/ /* end of group Standard_Driver */
/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use I3C0 Slave to reveive and transmit the data from a Master.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


//#define DGBINT          printf
#define DGBINT(...)

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define I3CS0_SA        (0x68)
#define I3CS0_MID       (0x8123UL)
#define I3CS0_PID       (0xA13573C0UL)

volatile uint32_t   g_RespQ[I3CS_DEVICE_RESP_QUEUE_CNT];
volatile uint32_t   g_RxBuf[I3CS_DEVICE_RX_BUF_CNT], g_TxBuf[I3CS_DEVICE_RX_BUF_CNT];
volatile uint32_t   g_u32IntSelMask = 0, g_u32IntOccurredMask = 0;
volatile uint32_t   g_u32RespStatus = I3CS_STS_NO_ERR;

int32_t I3CS_ProcessNoneRespInt(I3CS_T *i3cs, uint32_t i32IntMask);
int32_t I3CS_ProcessRespError(I3CS_T *i3cs, uint32_t u32RespStatus);
void I3CS0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
  * @brief  Process interrupt events except Response Ready interrupt.
  */
int32_t I3CS_ProcessNoneRespInt(I3CS_T *i3cs, uint32_t u32IntMask)
{
    if(u32IntMask & I3CS_INTEN_DA_ASSIGNED)
        printf("[ Set I3C Dynamic Address 0x%02x ]\n\n", (uint32_t)I3CS_GET_I3CS_DA(i3cs));
    if(u32IntMask & I3CS_INTEN_TRANSFER_ERR)
        printf("[ Transfer error ]\n\n");
    if(u32IntMask & I3CS_INTEN_READ_REQUEST)
        printf("[ Master read request event ]\n\n");
    if(u32IntMask & I3CS_INTEN_CCC_UPDATED)
    {
        printf("[ CCC Updated event ]\n");
        if(i3cs->SLVEVNTS & I3CS_SLVEVNTS_MWLUPD_Msk)
        {
            i3cs->SLVEVNTS = I3CS_SLVEVNTS_MWLUPD_Msk;
            printf("\t* Updated MWL to 0x%x\n\n", (uint32_t)((i3cs->SLVMXLEN&I3CS_SLVMXLEN_MWL_Msk) >> I3CS_SLVMXLEN_MWL_Pos));
        }
        else if(i3cs->SLVEVNTS & I3CS_SLVEVNTS_MRLUPD_Msk)
        {
            i3cs->SLVEVNTS = I3CS_SLVEVNTS_MRLUPD_Msk;
            printf("\t* Updated MRL to 0x%x\n\n", (uint32_t)((i3cs->SLVMXLEN&I3CS_SLVMXLEN_MRL_Msk) >> I3CS_SLVMXLEN_MRL_Pos));
            /* Reset TX FIFO and CMDQ FIFO -> apply resume */
            I3CS_ResetAndResume(i3cs, (I3CS_RESET_TX_BUF | I3CS_RESET_CMD_QUEUE), TRUE);
        }
        else 
        {
            printf("\t* Updated - ENTAS%d\n", (uint32_t)((i3cs->SLVEVNTS&I3CS_SLVEVNTS_ACTSTATE_Msk) >> I3CS_SLVEVNTS_ACTSTATE_Pos));
            printf("\t* Updated - HJEN %d\n", (uint32_t)((i3cs->SLVEVNTS&I3CS_SLVEVNTS_HJEN_Msk) >> I3CS_SLVEVNTS_HJEN_Pos));
            printf("\t* Updated - SIREN %d\n", (uint32_t)((i3cs->SLVEVNTS&I3CS_SLVEVNTS_SIREN_Msk) >> I3CS_SLVEVNTS_SIREN_Pos));
        }
    }
    
    return 0;
}

/**
  * @brief  Process response error event.
  */
int32_t I3CS_ProcessRespError(I3CS_T *i3cs, uint32_t u32RespStatus)
{
    printf("[ Resp Error 0x%x] ", (u32RespStatus >> I3CS_RESPQUE_ERRSTS_Pos));
    if(u32RespStatus == I3CS_RESP_CRC_ERR)            
        printf("CRC error\n");
    else if(u32RespStatus == I3CS_RESP_PARITY_ERR)
        printf("Parity error\n");
    else if(u32RespStatus == I3CS_RESP_FRAME_ERRR)
        printf("Frame error\n");
    else if(u32RespStatus == I3CS_RESP_FLOW_ERR)
        printf("Underflow/Overflow error\n");
    else if(u32RespStatus == I3CS_RESP_MASTER_TERMINATE_ERR)
        printf("Master early termination error\n");
    else
        printf("Unknow error\n");

    if(I3CS_IS_PROTOCOL_ERR(i3cs))
        printf("[ Device Protocol Error ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
    if(I3CS_IS_UNDERFLOW_ERR(i3cs))
        printf("[ Device Underflow Error ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
    if(I3CS_IS_OVERFLOW_ERR(i3cs))
        printf("[ Device Overflow Error ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
    if(I3CS_IS_DATA_NOT_READY(i3cs))
        printf("[ Device Data Not Ready Status ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
    if(I3CS_IS_BUFFER_NOT_AVAIL(i3cs))
        printf("[ Device Buffer Not Available Status ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
    if(I3CS_IS_FRAME_ERR(i3cs))
        printf("[ Device Frame Error ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
    if(I3CS_IS_SLAVE_BUSY(i3cs))
    {    
        printf("[ Device Slave Busy Status ] (0x%04x)\n\n", I3CS_GET_DEVICE_STATUS(i3cs));
        printf("\tPerform FIFO/Queue reset then wait RESUME complete ... ");
        I3CS_RespErrorRecovery(I3CS0, g_u32RespStatus);
        printf("done.\n\n");
    }
    
    return 0;
}

/**
  * @brief  The I3CS0 default IRQ, declared in startup_NUC1263.s.
  */
void I3CS0_IRQHandler(void)
{    
    DGBINT("\n");
    
    if(g_u32IntSelMask & I3CS_INTEN_TX_EMPTY_THLD)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_TX_EMPTY_THLD))
        {
            DGBINT("[ INT ] TX_EMPTY_THLD\n");
            g_u32IntOccurredMask |= I3CS_INTSTS_TX_EMPTY_THLD;
        }        
    }
    
    if(g_u32IntSelMask & I3CS_INTEN_RX_THLD)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_RX_THLD))
        {
            DGBINT("[ INT ] INTSTS_RX_THLD\n");
            g_u32IntOccurredMask |= I3CS_INTSTS_RX_THLD;
        }
    }
    
    if(g_u32IntSelMask & I3CS_INTEN_CMDQ_EMPTY_THLD)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_CMDQ_EMPTY_THLD))
        {
            DGBINT("[ INT ] CMDQ_EMPTY_THLD\n");        
            g_u32IntOccurredMask |= I3CS_INTSTS_CMDQ_EMPTY_THLD;
        }
    }
        
    if(g_u32IntSelMask & I3CS_INTEN_RESPQ_READY)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_RESPQ_READY))
        {
            DGBINT("[ INT ] RESPQ_READY\n");        
            g_u32IntOccurredMask |= I3CS_INTSTS_RESPQ_READY;
        }
    }
        
    if(g_u32IntSelMask & I3CS_INTEN_CCC_UPDATED)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_CCC_UPDATED))
        {
            DGBINT("[ INT ] CCC_UPDATED\n");
            I3CS_CLEAR_CCC_UPDATED_STATUS(I3CS0);
            g_u32IntOccurredMask |= I3CS_INTSTS_CCC_UPDATED;
        }
    }
        
    if(g_u32IntSelMask & I3CS_INTEN_DA_ASSIGNED)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_DA_ASSIGNED))
        {
            DGBINT("[ INT ] DA_ASSIGNED\n");
            I3CS_CLEAR_DA_ASSIGNED_STATUS(I3CS0);
            g_u32IntOccurredMask |= I3CS_INTSTS_DA_ASSIGNED;
        }        
    }
        
    if(g_u32IntSelMask & I3CS_INTEN_TRANSFER_ERR)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_TRANSFER_ERR))
        {
            DGBINT("[ INT ] TRANSFER_ERR\n");
            I3CS_CLEAR_TRANSFER_ERR_STATUS(I3CS0);
            g_u32IntOccurredMask |= I3CS_INTSTS_TRANSFER_ERR;
        }       
    }
        
    if(g_u32IntSelMask & I3CS_INTEN_READ_REQUEST)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_READ_REQUEST))
        {
            DGBINT("[ INT ] READ_REQUEST\n");
            I3CS_CLEAR_READ_REQUEST_STATUS(I3CS0);
            g_u32IntOccurredMask |= I3CS_INTSTS_READ_REQUEST;
        }        
    }
        
    if(g_u32IntSelMask & I3CS_INTEN_IBI_UPDATED)
    {
        if(I3CS_IS_INT_STATUS(I3CS0, I3CS_INTSTS_IBI_UPDATED))
        {
            DGBINT("[ INT ] IBI_UPDATED\n");
            I3CS_CLEAR_IBI_UPDATED_STATUS(I3CS0);
            g_u32IntOccurredMask |= I3CS_INTSTS_IBI_UPDATED;
        }        
    }
    
    if(g_u32IntOccurredMask & I3CS_INTSTS_RESPQ_READY)
    {
        g_u32RespStatus = (uint32_t)I3CS_ParseRespQueue(I3CS0, (uint32_t *)(&g_RespQ[0]));
    }
        
    DGBINT("[ INT EXIT ] INTSTS: 0x%08x. Occurred: 0x%08x.\n\n", I3CS0->INTSTS, g_u32IntOccurredMask);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);
    
    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/    
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3CS0_MODULE);

    /* Set multi-function pins for I3CS0 pin */
    SET_I3CS0_SDA_PA0();
    SET_I3CS0_SCL_PA1();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}
																
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint16_t    i, u16Len;
    uint8_t     *pu8Data, u8TID;
    uint8_t     qn, u8RespQCnt;
    uint32_t    u32ActiveIntMask;
    int32_t     iErrCode = I3CS_STS_NO_ERR;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Init();
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    I3C0 Slave Read/Write Sample Code   |\n");
    printf("+----------------------------------------+\n\n");
            
    /* Reset I3CS0 module */
    SYS_ResetModule(I3CS0_RST);
    
    /* Initial I3CS0 default settings */
    I3CS0->SLVMID = I3CS0_MID;
    I3CS0->SLVPID = I3CS0_PID;
    I3CS_Open(I3CS0, I3CS0_SA, I3CS_SUPPORT_ENTDAA);
    
    /* Enable I3CS0 interrupts */
    g_u32IntSelMask = (I3CS_INTEN_RESPQ_READY | I3CS_INTEN_CCC_UPDATED | I3CS_INTEN_DA_ASSIGNED |
                        I3CS_INTEN_TRANSFER_ERR | I3CS_INTEN_READ_REQUEST);
    I3CS_ENABLE_INT(I3CS0, g_u32IntSelMask);
    NVIC_EnableIRQ(I3CS0_IRQn);

    /* Enable I3CS0 controller */
    I3CS_Enable(I3CS0);

    printf("# I3C0 Slave settings:\n");
    printf("    - SDA on PA.0\n");
    printf("    - SCL on PA.1\n");
    printf("    - I2C Static Address 0x%02x\n", I3CS0_SA);
    printf("    - RespQ interrupt threshold %d\n", (uint32_t)(I3CS_GET_RESPQ_THLD(I3CS0) + 1));
    printf("    - The first operation of the I3CS enable bit requires at least bus SCLx4 to become active\n");
    printf("# An I3C Master can write N-bytes data to Slave,\n");
    printf("  then perform a read request to receive the N-bytes data from Slave.\n");
    printf("    - The write data should be equal to the received data\n");
    printf("\n");
        
        
    while(1)
    {
        while(g_u32IntOccurredMask != 0)
        {
            /* Get active interrupt occurred events */
            u32ActiveIntMask = g_u32IntOccurredMask;
            g_u32IntOccurredMask = 0;
            
            if(u32ActiveIntMask & I3CS_INTSTS_RESPQ_READY)
            {
                /* Process Response */

                if(g_u32RespStatus == I3CS_STS_NO_ERR)
                {
                    /* Response no error */
                    
                    u8RespQCnt = I3CS_GET_RESPQ_THLD(I3CS0) + 1;
                    
                    qn = 0; // Queue number
                    do {
                        if(I3CS_IS_RESP_RX(g_RespQ[qn]))
                        {
                            /* Master write request */
                                                        
                            u16Len = I3CS_GET_RESP_DATA_LEN(g_RespQ[qn]);
                            printf("Slave receives %d-bytes:\n\thex: ", u16Len);
                            /* Read Rx data from data port */
                            for(i=0; i<((u16Len+3)/4); i++)
                                g_RxBuf[i] = I3CS0->TXRXDAT;
                            pu8Data = (uint8_t *)(&g_RxBuf[0]);
                            for(i=0; i<u16Len; i++)
                                printf("%02x ", pu8Data[i]);
                            printf("\n\n");
                            
                            /* Set CmdQ and response data for a Master read request */
                            memcpy((uint8_t *)(&g_TxBuf[0]), (uint8_t *)(&g_RxBuf[0]), u16Len);
                            u8TID = (pu8Data[0] % 8);
                            iErrCode = I3CS_SetCmdQueueAndData(I3CS0, u8TID, (uint32_t *)&g_TxBuf[0], u16Len);
                            if(iErrCode != I3CS_STS_NO_ERR)
                                printf("\tSet TX data error, %d.\n\n", iErrCode);
                            else
                                printf("[ Set TX %d-bytes and TID %d for Master read request ]\n\n", u16Len, u8TID);
                        }
                        else
                        {
                            /* Master read request -> Slave transmits data done */
                            printf("Slave transmits ID-%d done.\n\n", (uint32_t)I3CS_GET_RESP_TID(g_RespQ[qn]));                        
                        }
                        
                        qn++;                        
                        u8RespQCnt--;
                    }while(u8RespQCnt);
                }
                else
                {                    
                    /* Response has error */
                    
                    I3CS_ProcessRespError(I3CS0, g_u32RespStatus);
                }
                    
                g_u32RespStatus = I3CS_STS_NO_ERR;
            }
            else
            {                                                   
                /* Process others interrupt event */
                
                I3CS_ProcessNoneRespInt(I3CS0, u32ActiveIntMask);
            }
        }
    }
}

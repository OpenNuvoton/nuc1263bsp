/**************************************************************************//**
 * @file     LocalDevFw.c
 * @version  V3.00
 * @brief    Local device control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "spdh_device.h"
#include "LocalDevReg.h"
#include "LocalDevFw.h"

volatile RESP_QUEUE_T     g_DevRespQue[I3CS_DEVICE_RESP_QUEUE_CNT] __attribute__((aligned(4)));
static uint32_t  s_DevRxBuf[I3CS_DEVICE_RX_BUF_CNT];

uint32_t        g_u32DevRespIntCnt;
static uint8_t   s_u8DevMRn;
static volatile uint8_t   s_u8DevCMD;
static uint8_t   s_u8DevIdx;

volatile uint32_t g_u32DeviceChangedToI3CMode = 0;
uint32_t          g_u32DevPecValue;

extern volatile uint32_t g_u32FifoClr;
extern uint8_t  g_au8DevReg[64];


static int8_t _DevSendIBIReq(void);

int32_t Dev_FIFO_ResetAndResume(I3CS_T *i3cs, uint32_t u32ResetMask, uint32_t u32EnableResume)
{
    uint8_t u8InHaltState = 0;
    volatile uint32_t u32Timeout;

    if(I3CS_IS_SLAVE_BUSY(i3cs))
        u8InHaltState = 1;

    if(u32ResetMask)
    {
        if(u8InHaltState == 0)
        {
            /* Disable I3CS controller for reset buffer and queue */
            if(I3CS_Disable(i3cs) != I3CS_STS_NO_ERR)
                return I3CS_TIMEOUT_ERR;
        }

        /* Reset specify source */
        i3cs->RSTCTL = u32ResetMask;
        u32Timeout = (SystemCoreClock / 1000);

        while((i3cs->RSTCTL != 0) && (--u32Timeout)) {}
        if(u32Timeout == 0)
            return I3CS_TIMEOUT_ERR;

        if(u8InHaltState == 0)
        {
            /* Enable I3CS controller again */
            if(I3CS_Enable(i3cs) != I3CS_STS_NO_ERR)
                return I3CS_TIMEOUT_ERR;
        }
    }

    if(u32EnableResume || u8InHaltState)
    {
        /* The application has to take necessary action to handle the error condition and
            then set RESUME bit to resume the controller. */
        /* Slave will receive GETSTATUS CCC to clear specify status in I3CS_CCCDEVS register. */
        i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
        //while((i3cs->DEVCTL&I3CS_DEVCTL_RESUME_Msk) == I3CS_DEVCTL_RESUME_Msk) {}

        /* RESUME bit is auto-cleared once the controller is ready to accept new transfers. */
    }

    return I3CS_STS_NO_ERR;
}

int32_t Dev_RespErrorRecovery(I3CS_T *i3cs, uint32_t u32RespStatus)
{
    if(u32RespStatus != I3CS_STS_NO_ERR)
    {
        if(I3CS_IS_SLAVE_BUSY(i3cs))
        {
            switch(u32RespStatus)
            {
                case I3CS_RESP_CRC_ERR:
                case I3CS_RESP_PARITY_ERR:
                case I3CS_RESP_FRAME_ERRR:
                case I3CS_RESP_FLOW_ERR:
                    /* Reset RX FIFO -> apply resume */
                    Dev_FIFO_ResetAndResume(i3cs, I3CS_RESET_RX_BUF, TRUE);
                    break;

                case I3CS_RESP_MASTER_TERMINATE_ERR:
                    while((I3CS_GET_PRESENT_STATUS(i3cs) != 6)){}
                    /* Reset TX FIFO and CMDQ Queue -> apply resume */
                    Dev_FIFO_ResetAndResume(i3cs, (I3CS_RESET_TX_BUF | I3CS_RESET_CMD_QUEUE), TRUE);
                    break;

                default:
                    /* Reset all FIFO and Queue */
                    Dev_FIFO_ResetAndResume(i3cs, I3CS_RESET_ALL_QUEUE_AND_BUF, FALSE);
                    break;
            }
        }
    }

    return I3CS_STS_NO_ERR;
}

static int32_t _ReadHandler(I3CS_T * i3cs)
{
    /* 1-byte addressing mode */
    /*
        Start  |    Bit 7    |    Bit 6    |    Bit 5    |    Bit 4    |    Bit 3    |    Bit 2    |    Bit 1    |    Bit 0    | A/N |  Stop
          S    |       1     |      0      |     1       |      0      |                     HID                 |     W=0     |  A  |
               |                                           Address[7:0]                                                        |  A  |
    */
    /* Internal register */
    i3cs->CMDQUE = 0x00010028;//1 byte data, and TID is 5;
    i3cs->TXRXDAT = g_au8DevReg[s_DevRxBuf[0]&0xFF];
    //printf("%02x %d %d\n", s_DevRxBuf[0],s_DevRxBuf[0]&0xFF, g_au8DevReg[s_DevRxBuf[0]&0xFF]);

    return 0;
}

static int32_t _ReadHandler2B(I3CS_T * i3cs)
{

    if (SPDH_IsDEVPECEnable())//if PEC enalbed in I3C mode
    {
        /* 2-bytes addressing mode and PEC enabled */
        /*
            Start  |    Bit 7    |    Bit 6    |    Bit 5    |    Bit 4    |    Bit 3    |    Bit 2    |    Bit 1    |    Bit 0    | A/N/T |  Stop
              S    |       1     |      0      |     1       |      0      |                     HID                 |     W=0     |   A   |
                   |                                           Address[7:0]                                                        |   T   |
                   |                   CMD                   |     R=1     |      0      |      0      |      0      |      0      |   T   |
        */
        #if 0
        if (SPDH_GET_PEC_CHK())
        {
            printf("Read operation is ignored because PEC value checked error.\n\n");
            return (-1);
        }
        #endif
        //Internal register
        {
            /* MemReg = 0 */
            /* Check CMD value */
            /* g_au8DevReg max size is 64, so s_u8DevMRn cannot over 63.*/
            /* for volatile memory access (i.e., MemReg = 0), when last byte (i.e., MR255) is reached (extreme rare case), the slave device sends T = 0. */
            s_u8DevCMD = (s_DevRxBuf[0]&0xE000)>>13;

            /* Addr[5:0] max value is 63. */
            /* When MemReg = 0 there is no concept of "Block Address", Block Address bits are treated simply Upper Address bits. */
            /* So the max MRn is 127.(BlkAddr[0]+Addr[5:0]) */
            //s_u8DevMRn = s_DevRxBuf[0]&0x3F;/* limited MRn to 63. */
            s_u8DevMRn = s_DevRxBuf[0]&0x7F;/* limited MRn to 127. */

            //printf("CMD:%02X\n", s_u8DevCMD);//it cause NAK, because delay the write TX FIFO timing.
            if (s_u8DevCMD == 0)/* R1R */
            {
                #if 1
                if (s_u8DevMRn > MAX_DEVREG_LEN)
                {
                    /* Addr[5:0] max value is 63. */
                    /* When MemReg = 0there is no concept of "Block Address", Block Address bits are treated simply Upper Address bits. */
                    /* So the max MRn is 127.(BlkAddr[0]+Addr[5:0]) */
                    i3cs->CMDQUE = 0x00020028;//2 byte data(include 1 byte PEC value), and TID is 5;
                    i3cs->TXRXDAT = 0; //set dummy data, because no specific MRn
                    WRNLOG("R1R:MRn Overflow(L:%d)\n", __LINE__);
                    //return (-1);
                }
                #endif
                i3cs->CMDQUE = 0x00020028;//2 byte data(include 1 byte PEC value), and TID is 5;
                i3cs->TXRXDAT = g_au8DevReg[s_u8DevMRn];
            }
            else if (s_u8DevCMD == 1)/* R2R */
            {
                if (s_u8DevMRn < (MAX_DEVREG_LEN - 1))
                {
                    i3cs->CMDQUE = 0x00030028;//2 byte data + 1 byte PEC, and TID is 5;
                    i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+1]<<8)|g_au8DevReg[s_u8DevMRn];
                }
                else if (s_u8DevMRn < MAX_DEVREG_LEN)
                {
                    WRNLOG("R2R:MRn+1 Overflow\n");
                    return (-1);
                    //i3cs->CMDQUE = 0x00020028;//1 byte data only + 1 byte PEC, response T = 0 at byte 1, and TID is 5;
                    //i3cs->TXRXDAT = g_au8DevReg[s_u8DevMRn];
                }
                else
                {
                    WRNLOG("R2R:MRn Overflow(L:%d)\n", __LINE__);
                    return (-1);
                }
            }
            else if (s_u8DevCMD == 2)/* R4R */
            {
                if (s_u8DevMRn < MAX_DEVREG_LEN - 3)
                {
                    i3cs->CMDQUE = 0x00050028;//4 byte data + 1 byte PEC, and TID is 5;
                    i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+3]<<24)|(g_au8DevReg[s_u8DevMRn+2]<<16)|(g_au8DevReg[s_u8DevMRn+1]<<8)|g_au8DevReg[s_u8DevMRn];
                    i3cs->TXRXDAT = 0x55;//a dummy byte for PEC
                }
                else
                {
                    /* MRn overflow */
                    if (s_u8DevMRn < MAX_DEVREG_LEN - 2)
                    {
                        WRNLOG("R4R:MRn+3 Overflow\n");
                        i3cs->CMDQUE = 0x00040028;//3 bytes data only + 1 byte PEC, response T = 0 at byte 3, and TID is 5;
                        i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+2]<<16)|(g_au8DevReg[s_u8DevMRn+1]<<8)|g_au8DevReg[s_u8DevMRn];
                    }
                    else if (s_u8DevMRn < MAX_DEVREG_LEN - 1)
                    {
                        WRNLOG("R4R:MRn+2 Overflow\n");
                        i3cs->CMDQUE = 0x00030028;//2 bytes data only + 1 byte PEC, response T = 0 at byte 2, and TID is 5;
                        i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+1]<<8)|g_au8DevReg[s_u8DevMRn];
                    }
                    else if (s_u8DevMRn < MAX_DEVREG_LEN)
                    {
                        WRNLOG("R4R:MRn+1 Overflow\n");
                        i3cs->CMDQUE = 0x00020028;//1 bytes data only + 1 byte PEC, response T = 0 at byte 1, and TID is 5;
                        i3cs->TXRXDAT = g_au8DevReg[s_u8DevMRn];
                    }
                    else
                    {
                        WRNLOG("R4R:MRn Overflow\n");
                        /* This case is not happend. because Addr[5:0] max value is 63. */
                        WRNLOG("R4R:MRn Overflow(L:%d)\n", __LINE__);
                        return (-1);
                    }
                }
            }
            else if (s_u8DevCMD == 3)/* R16R */
            {
                if (s_u8DevMRn < MAX_DEVREG_LEN - 15)
                {
                    i3cs->CMDQUE = 0x00110028;//16 byte data + 1 byte PEC, and TID is 5;
                    i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+3]<<24)|(g_au8DevReg[s_u8DevMRn+2]<<16)|(g_au8DevReg[s_u8DevMRn+1]<<8)|g_au8DevReg[s_u8DevMRn];
                    i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+7]<<24)|(g_au8DevReg[s_u8DevMRn+6]<<16)|(g_au8DevReg[s_u8DevMRn+5]<<8)|g_au8DevReg[s_u8DevMRn+4];
                    i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+11]<<24)|(g_au8DevReg[s_u8DevMRn+10]<<16)|(g_au8DevReg[s_u8DevMRn+9]<<8)|g_au8DevReg[s_u8DevMRn+8];
                    i3cs->TXRXDAT = (g_au8DevReg[s_u8DevMRn+15]<<24)|(g_au8DevReg[s_u8DevMRn+14]<<16)|(g_au8DevReg[s_u8DevMRn+13]<<8)|g_au8DevReg[s_u8DevMRn+12];
                    i3cs->TXRXDAT = 0x55;//a dummy byte for PEC
                }
                else
                {
                    /* This case is not happend. because Addr[5:0] max value is 63. */
                    WRNLOG("MRn Overflow(L:%d)\n", __LINE__);
                    return (-1);
                }
            }
            else
            {
                WRNLOG("[WARN] CMD: %d does not support.\n\n", s_u8DevCMD);
            }
            //printf("%02x %x %x\n", s_DevRxBuf[0],s_DevRxBuf[0]&0x3F, g_au8DevReg[s_DevRxBuf[0]&0x3F]);
        }
    }
    else
    {
        /* 1-bytes addressing mode and PEC disabled */
        /*
            Start  |    Bit 7    |    Bit 6    |    Bit 5    |    Bit 4    |    Bit 3    |    Bit 2    |    Bit 1    |    Bit 0    | A/N |  Stop
              S    |       1     |      0      |     1       |      0      |                     HID                 |     W=0     |  A  |
                   |                                           Address[7:0]                                                        |  A  |
        */
        /* Internal register */
        i3cs->CMDQUE = 0x00010028;//1 byte data, and TID is 5;
        i3cs->TXRXDAT = g_au8DevReg[s_DevRxBuf[0]&0xFF];

//      printf("R %02x %d 0x%02X(L:%d)\n", s_DevRxBuf[0],s_DevRxBuf[0]&0xFF, g_au8DevReg[s_DevRxBuf[0]&0xFF], __LINE__);
    }

    return 0;
}

static int32_t _WriteHandler(I3CS_T * i3cs)
{
    /* 1-byte addressing mode */
    /*
        Start  |    Bit 7    |    Bit 6    |    Bit 5    |    Bit 4    |    Bit 3    |    Bit 2    |    Bit 1    |    Bit 0    | A/N |  Stop
          S    |       1     |      0      |     1       |      0      |                     HID                 |     W=0     |  A  |
               |                                           Address[7:0]                                                        |  A  |
               |                                                Data                                                           |  A  |   P
    */
    //Internal register
    DevReg_WriteReg(s_DevRxBuf[0]&0xFF, (s_DevRxBuf[0]&0xFF00)>>8);
    //printf("W:%08x %d %d(%08x)\n", s_DevRxBuf[0],s_DevRxBuf[0]&0xFF, (s_DevRxBuf[0]&0xFF00)>>8, s_DevRxBuf[0]&BIT7);
    return 0;
}

static int32_t _BlockWriteHandler(I3CS_T * i3cs, uint16_t uLen)
{
    uint32_t i;
    uint8_t *pu8Buf;

    /* 1-byte addressing mode */
    /* Read Rx data from data port */
    for(i=1; i<((uLen+3)/4); i++)
        s_DevRxBuf[i] = I3CS_GET_RXD(i3cs);

    /* Check if block write was enabled */
    if (g_au8DevReg[47]&BIT0)
    {
#if 1
        /* Check if write MR43 register for LED DATA */
        if ((s_DevRxBuf[0]&0xFF) == 43)
#endif
        {
            uint16_t u16PixelCnt;
            u16PixelCnt = ((g_au8DevReg[40]&0x1)<<8)|(g_au8DevReg[39]&0xFF);
            /* Check if data length match the (pixel count * 3)/2. */
            DBGLOG("len - 1:%d, %d\n", uLen - 1, (u16PixelCnt * 3)/2);
            if ((u16PixelCnt * 3)/2 == uLen - 1)
            {
                pu8Buf = (uint8_t *)(s_DevRxBuf);
                LLSI_WriteBlockData(15, pu8Buf+1);
            }
            else
            {
                /* data length does not match the pixel count div 2. */
                WRNLOG("[WARN] data length does not match the pixel count div 2.\n");
            }
        }
    }

    return 0;
}

static int32_t _WriteHandler2B(I3CS_T * i3cs)
{
    if (SPDH_IsDEVPECEnable())//if PEC enalbed in I3C mode
    {
        /* 2-bytes addressing mode and PEC enabled */
        /*
            Start  |    Bit 7    |    Bit 6    |    Bit 5    |    Bit 4    |    Bit 3    |    Bit 2    |    Bit 1    |    Bit 0    | A/N/T |  Stop
              S    |       1     |      0      |     1       |      0      |                     HID                 |     W=0     |   A   |
                   |                                           Address[7:0]                                                        |   T   |
                   |                   CMD                   |     W=0     |      0      |       0     |      0      |      0      |   T   |
        */

        /* Internal register */

        /* Check CMD value */
        s_u8DevCMD = (s_DevRxBuf[0]&0xE000)>>13;
        //s_u8DevMRn = s_DevRxBuf[0]&0x3F;/* limited MRn to 63. */
        s_u8DevMRn = s_DevRxBuf[0]&0x7F;/* limited MRn to 127. */

        if (s_u8DevCMD == 0)/* W1R */
        {
            /* ToDo: add max s_u8DevMRn protection */
            DevReg_WriteReg(s_u8DevMRn, (s_DevRxBuf[0]&0xFF0000)>>16);
        }
        else if (s_u8DevCMD == 1)/* W2R */
        {
            /* ToDo: add max s_u8DevMRn protection */
            DevReg_WriteReg(s_u8DevMRn, (s_DevRxBuf[0]&0xFF0000)>>16);
            DevReg_WriteReg(s_u8DevMRn+1, (s_DevRxBuf[0]&0xFF000000)>>24);
            //s_DevRxBuf[1] = I3CS_GET_RXD(i3c);//read PEC value
            g_u32DevPecValue = I3CS_GET_RXD(i3cs)&0xFF;
            //printf("PEC:0x%x\n", g_u32DevPecValue);
        }
        else if (s_u8DevCMD == 2)/* W4R */
        {
            /* ToDo: add max s_u8DevMRn protection */
            DevReg_WriteReg(s_u8DevMRn, (s_DevRxBuf[0]&0xFF0000)>>16);
            DevReg_WriteReg(s_u8DevMRn+1, (s_DevRxBuf[0]&0xFF000000)>>24);
            s_DevRxBuf[1] = I3CS_GET_RXD(i3cs);
            DevReg_WriteReg(s_u8DevMRn+2, (s_DevRxBuf[1]&0xFF));
            DevReg_WriteReg(s_u8DevMRn+3, (s_DevRxBuf[1]&0xFF00)>>8);
        }
        else if (s_u8DevCMD == 3)/* W16R */
        {
            if (s_u8DevMRn > (MAX_DEVREG_LEN - 16))
            {
                WRNLOG("Write MRn has overflow.(%d)\n", s_u8DevMRn);
                return (-1);
            }
            /* ToDo: add max s_u8DevMRn protection */
            DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[0]&0xFF0000)>>16);
            DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[0]&0xFF000000)>>24);
            for(s_u8DevIdx = 1; s_u8DevIdx < 4; s_u8DevIdx++)
            {
                s_DevRxBuf[s_u8DevIdx] = I3CS_GET_RXD(i3cs);
                DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[s_u8DevIdx]&0xFF));
                DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[s_u8DevIdx]&0xFF00)>>8);
                DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[s_u8DevIdx]&0xFF0000)>>16);
                DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[s_u8DevIdx]&0xFF000000)>>24);
            }
            s_DevRxBuf[s_u8DevIdx] = I3CS_GET_RXD(i3cs);
            DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[s_u8DevIdx]&0xFF));
            DevReg_WriteReg(s_u8DevMRn++, (s_DevRxBuf[s_u8DevIdx]&0xFF00)>>8);

            //printf("PEC:0x%x(s_u8DevIdx:%d)\n", g_u32DevPecValue, s_u8DevIdx);
        }
        else
        {
            WRNLOG("[WARN] CMD: %d does not support.\n\n", s_u8DevCMD);
        }
        //printf("MR%d:0x%x(0x%x)(L:%d)\n", s_u8DevMRn, g_au8DevReg[s_DevRxBuf[0]&0x7F], s_DevRxBuf[0], __LINE__);
    }
    else
    {
        /* 1-bytes addressing mode */
        /*
            Start  |    Bit 7    |    Bit 6    |    Bit 5    |    Bit 4    |    Bit 3    |    Bit 2    |    Bit 1    |    Bit 0    | A/N |  Stop
              S    |       1     |      0      |     1       |      0      |                     HID                 |     W=0     |  A  |
                   |                                           Address[7:0]                                                        |  A  |
                   |                                                Data                                                           |  A  |   P
        */
        /* Internal register */
        DevReg_WriteReg(s_DevRxBuf[0]&0xFF, (s_DevRxBuf[0]&0xFF00)>>8);
//      printf("W MR%d:%d(L:%d)\n", s_DevRxBuf[0]&0xFF, g_au8DevReg[s_DevRxBuf[0]&0xFF], __LINE__);
    }

    return 0;
}

static int32_t I3CS_ProcessRespQueue(I3CS_T * i3cs)
{
    int32_t ret = -1;
    volatile uint32_t i;
    uint32_t          u32RespErrSts;
    RESP_QUEUE_T      *pRespQ;

    pRespQ = (RESP_QUEUE_T *)&g_DevRespQue[0];
    pRespQ->w = I3CS_GET_RESP_DATA(i3cs);
    u32RespErrSts = pRespQ->b.STATUS;

    if(u32RespErrSts == 0)
    {
        if(pRespQ->b.RXRSP == 1)
        {
            g_u32FifoClr = 0;

            /*
                I2C mode: 1-byte addressing mode.
                I3C mode: 1-byte addressing mode when PEC mode was disabled,
                          2-bytes addressing mode when PEC mode was enabled and support CMD while PEC mode enabled.
            */
            if (I3CS_IS_DA_VALID(i3cs) == 0)
            {
                /* I2C mode */
                //1-byte addressing mode //only in I2C mode
                if(pRespQ->b.LENGTH < 2)//read: -O0: cannot write data to TXRXD register before send FIFO out. So response NACK.
                                        //      -O3: can write data to TXRXD register before send FIFO out. So response correct data.
                {
                    if (pRespQ->b.LENGTH != 0)
                    {
                        _ReadHandler(i3cs);
                    }
                    else
                    {
                        DBGLOG("Default read address pointer\n");
                    }
                }
                else
                {//write
                    if (pRespQ->b.LENGTH == 2)
                    {
                        _WriteHandler(i3cs);
                    }
                    else
                    {
                        _BlockWriteHandler(i3cs, pRespQ->b.LENGTH);
                    }
                }
//                printf("\tRX resp done(len:%d)\n",pRespQ->b.LENGTH);
//                printf("len:%d\n",pRespQ->b.LENGTH);
                ret = 0;
            }
            else
            {
                /* I3C mode */
                /* PEC mode was disabled using 1-bytes addressing mode */
                /* PEC mode was enabled using 2-bytes addressing mode */

                if (SPDH_IsDEVPECEnable())//if PEC enalbed in I3C mode
                {
                    if(pRespQ->b.LENGTH < 4)//read: -O0: cannot write data to TXRXD register before send FIFO out. So response NACK.
                                            //      -O3: can write data to TXRXD register before send FIFO out. So response correct data.
                    {
                        if (pRespQ->b.LENGTH != 0)
                        {
                            if (s_DevRxBuf[0]&BIT12)//check R/W bit
                            {
                                _ReadHandler2B(i3cs);
                            }
                            else
                            {
                                ERRLOG("\t[ERR]R bit and package length are not matched.(0x%08X)(L:%d)\n", s_DevRxBuf[0], __LINE__);
                                s_DevRxBuf[0] = I3CS_GET_RXD(i3cs);
                            }
                        }
                        else
                        {
                            DBGLOG("Default read address pointer\n");
                        }
                    }
                    else
                    {//write
                        if ((s_DevRxBuf[0]&BIT12) == 0)//check R/W bit
                        {
                            _WriteHandler2B(i3cs);
                        }
                        else
                        {
                            ERRLOG("\t[ERR]W bit and package length are not matched.(0x%08X)(L:%d)\n", s_DevRxBuf[0], __LINE__);
                        }
                    }
                }
                else
                {
                    if(pRespQ->b.LENGTH < 2)//read: -O0: cannot write data to TXRXD register before send FIFO out. So response NACK.
                                            //      -O3: can write data to TXRXD register before send FIFO out. So response correct data.
                    {
                        if (pRespQ->b.LENGTH != 0)
                        {
                            _ReadHandler(i3cs);
                        }
                        else
                        {
                            DBGLOG("Default read address pointer\n");
                        }
                    }
                    else
                    {//write
                        _WriteHandler(i3cs);
                    }
                }
                ret = 0;
            }
        }
        else
        {
            ret = 1;
        }
    }
    else
    {
    /*
        Note:
        The Slave controller NACKs all transfers once it has encountered an error until RESUME bit
        is set in the DEVICE_CTRL register from the Slave application and until the error status is cleared from the
        CCC_DEVICE_STATUS register by GETSTATUS CCC.
        For any other error status like underflow error or Master Early termination,
        the Slave application is expected to reset the TX FIFO and CMD FIFO before applying the resume in DEVICE_CTRL register.
    */
        DBGLOG("\t[I3CS%d]Error status 0x%x\n", (i3cs==I3CS0)?0:1, pRespQ->b.STATUS);
        if(i3cs->CCCDEVS & I3CS_CCCDEVS_PROTERR_Msk)//Protocol error: This bit is set when the slave controller encouters a Parity/CRC error during write data transfer.
        {
            /* Set MR52[0]: PAR_ERROR_STATUS */
            DevReg_SetParityErrStatus();
            WRNLOG("[WARN]Parity error\n");
        }
        Dev_RespErrorRecovery(i3cs, pRespQ->b.STATUS);
        ret = -1;
    }

    return ret;
}

/*
    judge only write and write then read cases.
    default address pointer mode has enable bit to let firmware knows to changes to normal read/write or default address pointer.
*/
/*
    Note:
    The Slave controller NACKs all transfers once it has encountered an error until RESUME bit
    is set in the DEVICE_CTRL register from the Slave application and until the error status is cleared from the
    CCC_DEVICE_STATUS register by GETSTATUS CCC.
    For any other error status like underflow error or Master Early termination,
    the Slave application is expected to reset the TX FIFO and CMD FIFO before applying the resume in DEVICE_CTRL register.
*/
static uint32_t I3CS_ParseIntStatus(I3CS_T * i3cs)
{
    volatile uint32_t u32IntSts, u32PresentSts;
    //int32_t     i32Ret = 0;

    /* PRESENT_STATE(0x54): CM_TFR_STS[13:8]:8-I2C WRITE transfer, 9-I2C READ transfer */

    u32IntSts = I3CS_GET_INTSTS(i3cs);

    // I3CS_INTSTS_RESPRDY_Msk
    if(u32IntSts&I3CS_INTSTS_RESPRDY_Msk) // auto cleared
    {
        //prepare read data
        s_DevRxBuf[0] = I3CS_GET_RXD(i3cs);
        if(I3CS_ProcessRespQueue(i3cs) >= 0)
            g_u32DevRespIntCnt = 1;
    }

    // I3CS_INTSTS_DAA_Msk
    if(u32IntSts&I3CS_INTSTS_DAA_Msk) // write 1 cleared
    {
        SPDH_EnableDEVSIR();

        /* Update MR18[5] NF_SEL bit as I3C basic protocol. */
        DevReg_EnableI3C();

        g_u32DeviceChangedToI3CMode = 1;

        DBGLOG("INT DYNAASTS (I3CS%d DA: 0x%02x)\n", (i3cs==I3CS0)?0:1, (uint32_t)I3CS_GET_I3CS_DA(i3cs));
        I3CS_CLEAR_DA_ASSIGNED_STATUS(i3cs);

        __NOP();
        /* To check if mode of device behind Hub and I3C slave is matched? */
        if (SPDH_GetDEVMode() == 0)
        {
            WRNLOG("[WARN] Mode of device behind Hub and I3C slave is not matched.(#%d)\n\n", __LINE__);
        }
    }

    if(u32IntSts & I3CS_INTSTS_READREQ_Msk)
    {
        DBGLOG("INT READ_REQUEST\n");
        I3CS_CLEAR_READ_REQUEST_STATUS(i3cs);
    }


    if(u32IntSts & I3CS_INTSTS_TFRERR_Msk)
    {
        DBGLOG("INT TRANSFER_ERR\n");
        I3CS_CLEAR_TRANSFER_ERR_STATUS(i3cs);
    }

    if(u32IntSts & I3CS_INTSTS_IBIUPD_Msk)
    {
        uint32_t u32IBICompleSts;
        u32IBICompleSts = i3cs->SIRRESP;
        DBGLOG("SLV_IBI_RESP: 0x%08X\n", i3cs->SIRRESP);
        if ((u32IBICompleSts & I3CS_SIRRESP_IBISTS_Msk) == I3CS_IBI_ACCEPTED)
        {
            DBGLOG("IBI accepted by the Master (ACK response received)\n");
            /* Clear pending status to 0 */
            i3cs->DEVCTL &= ~I3CS_DEVCTL_PENDINT_Msk;
            /* Clear MR48[7] : IBI_STATUS */
            DevReg_ClearIBIStatus();
        }
        else if ((u32IBICompleSts & I3CS_SIRRESP_IBISTS_Msk) == I3CS_IBI_MASTER_TERMINATE)
        {
            DBGLOG("Master Early Terminate (only for SIR with Data)\n");
        }
        else if ((u32IBICompleSts & I3CS_SIRRESP_IBISTS_Msk) == I3CS_IBI_NOT_ATTEMPTED)
        {
            DBGLOG("IBI Not Attempted\n");
        }
    }

    if(u32IntSts&I3CS_INTSTS_CCCUPD_Msk)// write 1 cleared
    {
        DBGLOG("INT CCCUPD (I3CS%d DA: 0x%02x)\n", (i3cs==I3CS0)?0:1, (uint32_t)I3CS_GET_I3CS_DA(i3cs));
        /* This interrupt is generated if any of the CCC registers are updated by I3C master through CCC commands. */
        if(i3cs->SLVEVNTS & I3CS_SLVEVNTS_SIREN_Msk)
        {
            /* SIR_EN(SLV_EVENT_STATUS[0]), Slave Interrupt Request Enable, this bit is set by ENEC */
            /* Set MR27[4]: IBI_ERROR_EN, In Band Error Interrupt Enable */
            DevReg_IBIEnable();
        }
        else
        {
            /* SIR_EN(SLV_EVENT_STATUS[0]), Slave Interrupt Request Enable, this bit is clear by DISEC */
            /* Clear MR27[4]: IBI_ERROR_EN, In Band Error Interrupt Enable */
            DevReg_IBIDisable();
        }
        I3CS_CLEAR_CCC_UPDATED_STATUS(i3cs);
    }

    return 0;
}

void LocalDev_SPDHIRQHandler(void)
{
    uint8_t u8StaticAddr, u8LID;
    uint32_t u32SpdhSts, u32HubDevCtrlCfg, u32StartOffset, u32LLSISycFuncCtrl;

    u32SpdhSts = SPDH_GetINTStatus();

    if(u32SpdhSts & SPDH_INTSTS_BUSRTOIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: BUS Reset Time-out Event\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_BUSRTOIF_Msk);

        /* Do chip reset by software, because Host sent bus reset to all device on the same bus. */
        //SYS_ResetChip();
    }

#if (SPDH_DETECT_POWER_DOWN == 1)
    if(u32SpdhSts & SPDH_INTSTS_PWRDTOIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Power Down Detect\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_PWRDTOIF_Msk);

        g_DetectedPowerDown = 1;
    }
#endif

    if(u32SpdhSts & SPDH_INTSTS_DSETHIDIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received SETHID CCC\n");

        u8LID = (I3CS1->DEVADDR & SPDH_DCTL_LID_Msk);
        u8StaticAddr = (uint8_t)(u8LID | SPDH_GetDEVHID());
        DBGLOG("u8StaticAddr: 0x%08X\n", u8StaticAddr);

        /* Change HID of I3C static address after received SETHID CCC command. */
        I3CS1->DEVADDR = (I3CS1->DEVADDR &~I3CS_DEVADDR_SA_Msk) | (I3CS_DEVADDR_SAVALID_Msk | u8StaticAddr);
        DBGLOG("I3C1->DEVADDR: 0x%08X\n", I3CS1->DEVADDR);
        
        SPDH_ClearINTFlag(SPDH_INTSTS_DSETHIDIF_Msk);
    }

    if(u32SpdhSts & SPDH_INTSTS_DEVPCIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: PEC Error Occurred\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DEVPCIF_Msk);
    }

    
    
    if(u32SpdhSts & SPDH_INTSTS_DDEVCTLIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received DEVCTRL CCC\n");
        u32HubDevCtrlCfg = SPDH_GetDEVStatus();
        if (u32HubDevCtrlCfg & SPDH_DSTS_PECSTS_Msk)
        {
            DBGLOG("Device DEVCTRL: PEC enabled.\n");
            /* Enable PEC function in Hub's device. */
            SPDH_EnableDEVCRC();
            /* Update MR18 register. */
            DevReg_PECEnable();
        }
        else
        {
            DBGLOG("Device DEVCTRL: PEC disabled.\n");
            /* Disable PEC function in Hub,s device. */
            SPDH_DisableDEVCRC();
            /* Update MR18 register. */
            DevReg_PECDisable();
        }
        if (u32HubDevCtrlCfg & SPDH_DSTS_PARDIS_Msk)
        {
            DBGLOG("Device DEVCTRL: Parity disabled.\n");
            /* Update MR18 register. */
            DevReg_ParityDisable();
        }
        else
        {
            DBGLOG("Device DEVCTRL: Parity enabled.\n");
            /* Update MR18 register. */
            DevReg_ParityEnable();
        }
        if (u32HubDevCtrlCfg & SPDH_DSTS_IBICLR_Msk)
        {
            DBGLOG("Device DEVCTRL:  Clear All Event and pending IBI.\n");
            DevReg_ClearAllEvent();
        }

        /* Check if byte 2 data for synchronous function is enabled. */
        /* if RegMod is 0 */
        if ((SPDH_GetDEVCTRL0()&SPDH_HDEVCTRL0_REGMOD_Msk) == 0)
        {
            /* Check StartOffset value */
            u32StartOffset = (SPDH_GetDEVCTRL0()&SPDH_HDEVCTRL0_STAOFSET_Msk)>>SPDH_HDEVCTRL0_STAOFSET_Pos;
            if (u32StartOffset < 3)
            {
                if (u32StartOffset == 0)
                {
                    u32LLSISycFuncCtrl = _GET_BYTE2(SPDH_GetDEVCTRL1());
                }
                else if (u32StartOffset == 1)
                {
                    u32LLSISycFuncCtrl = _GET_BYTE1(SPDH_GetDEVCTRL1());
                }
                else if (u32StartOffset == 2)
                {
                    u32LLSISycFuncCtrl = _GET_BYTE0(SPDH_GetDEVCTRL1());
                }
                /* Check if byte 2 data for synchronous function is enabled. */
                if (u32LLSISycFuncCtrl&BIT0)
                {
                    if (u32LLSISycFuncCtrl&BIT1)
                    {
                        /* Synchronous start to flash LED. */
                        DevReg_WriteReg(36, 1);
                    }
                    else
                    {
                        /* Synchronous stop to flash LED. */
                        DevReg_WriteReg(36, 0);
                    }
                }
            }
        }

        SPDH_ClearINTFlag(SPDH_INTSTS_DDEVCTLIF_Msk);
    }
    
    if(u32SpdhSts & SPDH_INTSTS_DDEVCAPIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received DEVCAP CCC\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DDEVCAPIF_Msk);
    }

    if(u32SpdhSts & SPDH_INTSTS_DEVIHDIF_Msk)
    {
        DBGLOG("\nSPDH_IRQ: Received IBI Header\n");
        SPDH_ClearINTFlag(SPDH_INTSTS_DEVIHDIF_Msk);

        /* if pending status is 0x1, then re-send the IBI request */

        if (SPDH_IsDEVINTStatus(SPDH_DSTS_PENDIBI_Msk))
        {
            DBGLOG("\nRe-send the IBI request\n");
            _DevSendIBIReq();
        }
    }
}

void I3CS1_IRQHandler(void)
{
    I3CS_ParseIntStatus(I3CS1);
}

int8_t LocalDev_Init(uint8_t u8DevAddr)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable SPD5 Hub clock. */
    CLK->APBCLK0 |= BIT26;

    /* Enable I3C1 clock for Hub. */
    CLK->APBCLK0 |= BIT25;

    /* Set LID. */
    SPDH_SetDEVLID(((u8DevAddr)&SPDH_DCTL_LID_Msk)>>SPDH_DCTL_LID_Pos);

    /* Disable Hot-join function before I3CS was enabled. */
    I3CS1->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;

    /* Disable SIR. */
    SPDH_DisableDEVSIR();

    I3CS_Open(I3CS1, u8DevAddr, 8);

    /* Enable chip's SPD5 Hub function */
    SYS->SPDHCTL |= SYS_SPDHCTL_SPDHEN_Msk;

    I3CS1->DBTHCTL = 0x3;

    /* configure Bus available time: JESD define is 1 us, 1/0.083 = 12 */
    I3CS1->BUSFAT = 12 << 16; //adjust for bus available time for IBI issue

	I3CS1->QUETHCTL  = ((I3CS_CFG_CMD_QUEUE_EMPTY_THLD-1) | ((I3CS_CFG_RESP_QUEUE_FULL_THLD-1)<<8));

    /* Enable Hub INT Status for local device */
    SPDH_EnableINT(SPDH_INTEN_BUSRTOEN_Msk|SPDH_INTEN_DDEVCTLEN_Msk|SPDH_INTEN_DDEVCAPEN_Msk| \
                    SPDH_INTEN_DSETHIDEN_Msk|SPDH_INTEN_DEVPCEN_Msk|SPDH_INTEN_DEVIHDEN_Msk);

    #if (SPDH_DETECT_POWER_DOWN == 1)
    SPDH_EnableINT(SPDH_INTEN_PWRDTOEN_Msk|SPDH_INTEN_WKUPEN_Msk);
    SPDH_SetPowerDownTimeout(1, 0x7F);
    #endif

    /* (221+1) *16384*13.8 = 50194022 ns = 50 ms */
    SPDH_SetBusResetTimeout(TRUE, 221);

    /* Enable I3CS1 INT Status */
    I3CS1->INTSTSEN = 0xFFFFFFFF;//All event was enabled


    /* Enable I3CS1 INT */
    I3CS1->INTEN |= (I3CS_INTEN_RESPQ_READY | I3CS_INTEN_CCC_UPDATED | I3CS_INTEN_DA_ASSIGNED |
                        I3CS_INTEN_TRANSFER_ERR | I3CS_INTEN_READ_REQUEST|I3CS_INTEN_IBI_UPDATED);
    NVIC_EnableIRQ(SPDH_IRQn);
    NVIC_EnableIRQ(I3CS1_IRQn);

    /* Enable I3CS1 controller */
    I3CS_Enable(I3CS1);

    /* Lock protected registers */
//    SYS_LockReg();

    return 0;
}

int8_t LocalDev_ResetInit(uint32_t u32I3cAddr)
{
    /* Disable Hot-join function before I3CS was enabled */
    I3CS1->SLVEVNTS &= ~BIT3;

    /* Disable SIR. */
    SPDH_DisableDEVSIR();

    /* Enable I3CS. */
    I3CS1->DEVCTL |= I3CS_DEVCTL_ENABLE_Msk|BIT27;

    I3CS1->DEVADDR = I3CS_DEVADDR_SAVALID_Msk | u32I3cAddr;

    I3CS1->DBTHCTL = 0x3;

    /* Enable I3CS1 INT Status */
    I3CS1->INTSTSEN = 0xFFFFFFFF;//All event was enabled
    I3CS1->INTEN |= (I3CS_INTEN_RESPQ_READY | I3CS_INTEN_CCC_UPDATED | I3CS_INTEN_DA_ASSIGNED |
                        I3CS_INTEN_TRANSFER_ERR | I3CS_INTEN_READ_REQUEST|I3CS_INTEN_RX_THLD);

    /* Update MRn register while changed back to I2C mode. */
    LocalDev_UpdateInterfaceSel(0);

    return 0;
}


/*  */
int8_t LocalDev_UpdateInterfaceSel(uint8_t u8InfSel)
{
    if (u8InfSel == 0)
    {
        /* Update MR18[5] NF_SEL bit as I2C basic protocol. */
        DevReg_EnableI2C();
    }
    else if (u8InfSel == 1)
    {
        /* Update MR18[5] NF_SEL bit as I3C basic protocol. */
        DevReg_EnableI3C();
    }
    else
    {
        ERRLOG("[ERR] No support this selection.\n");
        return (-1);
    }
    return 0;
}

int8_t LocalDev_CheckInterfaceSel(void)
{
    if (g_u32DeviceChangedToI3CMode)
    {
        /* Polling the Hub mode to check Host has sent RSTDAA command.(Because I3C no interrupt for RSADAA command) */
        if (I3CS_IS_DA_VALID(I3CS1) == 0)
        {
            g_u32DeviceChangedToI3CMode = 0;

            if(I3CS_GET_INTSTS(I3CS1)&I3CS_INTSTS_CCCUPD_Msk)
            {
                DBGLOG("INISTS CCCUPD (I3CS1 SA: 0x%02x)\n", (uint32_t)I3CS_GET_I2C_SA(I3CS1));
                I3CS1->INTSTS = I3CS_INTEN_CCCUPD_Msk;
            }

            /* Update MR18[5] INF_SEL bit as I2C basic protocol. */
            LocalDev_UpdateInterfaceSel(0);
            DBGLOG("I3C slave Back to I2C mode.\n\n");

            /* To check if mode of device behind Hub and I3C slave is matched? */
            if (SPDH_GetDEVMode() == 1)
            {
                WRNLOG("[WARN] Mode of device behide Hub and I3C slave is not matched.\n\n");
            }
        }
    }
    return 0;
}

static int8_t _DevSendIBIReq(void)
{
    DBGLOG("\nI3C device re-sends the IBI request\n");
    if (SPDH_IsDEVPECEnable())
    {
        /* PEC enabled case: */
        I3CS1->SIRDAT = g_au8DevReg[52]|g_au8DevReg[51];// IBI payload should be MR51 and MR52.
        I3CS1->SIR = 0x00030000|BIT0;
    }
    else
    {
        /* PEC disabled case: */
        I3CS1->SIRDAT = g_au8DevReg[52]|g_au8DevReg[51];// IBI payload should be MR51 and MR52.
        I3CS1->SIR = 0x00020000|BIT0;
    }

    /* Set pending status until IBI request has been accepted or cleared by host. */
    /* Set MR48[7] : IBI_STATUS */
    DevReg_SetIBIStatus();

    return 0;
}

int8_t LocalDev_CheckIBIReg(void)
{
    /* IBI_ERROR_EN */
    if (DevReg_IsIBIntEn(DEV_IBI_INTEN_ERROR))
    {
        if((DevReg_GetErrorStatus()&(DEV_ERROR_STATUS_PEC|DEV_ERROR_STATUS_PAR)))
        {
            /* MR52[1:0] = 1 generates an IBI to Host .*/
            _DevSendIBIReq();
        }
    }

    return 0;
}


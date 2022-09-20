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

#include "LocalDevReg.h"
#include "LocalDevFw.h"

volatile RESP_QUEUE_T     g_DevRespQue[I3CS_DEVICE_RESP_QUEUE_CNT] __attribute__((aligned(4)));
static volatile uint32_t  s_DevRxBuf[I3CS_DEVICE_RX_BUF_CNT];

uint32_t        g_u32DevRespIntCnt;
static volatile uint8_t   s_u8DevMRn;
static volatile uint8_t   s_u8DevCMD;
static volatile uint8_t   s_u8DevIdx;

volatile uint32_t g_u32DeviceChangedToI3CMode = 0;
uint32_t          g_u32DevPecValue;

extern volatile uint32_t g_u32FifoClr;
extern volatile uint8_t  g_au8DevReg[64];


static int8_t _DevSendIBIReq(void);

static int8_t _DevResetTxFifoOnly(I3CS_T * i3cs)
{
    DBGLOG("\nI3CS%d DEVCTL: 0x%08x\n", i3cs->DEVCTL, (i3cs == I3CS0)?0:1);
    i3cs->DEVCTL &= ~I3CS_DEVCTL_ENABLE_Msk;
    while(i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) {}

    i3cs->RSTCTL = BIT3|BIT1;//BIT3:TX_FIFO_RST, BIT1: CMD_QUEUE_RST
    DBGLOG("\nReset I3CS%d Tx/CMD FIFO ... ", (i3cs == I3CS0)?0:1);
    while(i3cs->RSTCTL != 0) {}
    DBGLOG("done\n");

    i3cs->DEVCTL |=  I3CS_DEVCTL_ENABLE_Msk;
    while((i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) == 0) {}
    DBGLOG("re-en(I3CS%d DEVCTL:0x%08x)\n", i3cs->DEVCTL, (i3cs == I3CS0)?0:1);

    WRNLOG("\nSet I3CS%d RESUME ...(#%d) \n", (i3cs == I3CS0)?0:1, __LINE__);
    i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
//    while((i3cs->DEVCTL&I3CS_CTL_RESUME_Msk) != 0) {}//wait for Host send GETSTATUS CCC, then RESUME bit is became to 0
//    printf("resume done\n");

	i3cs->QUETHCTL  = ((I3CS_CFG_CMD_QUEUE_EMPTY_THLD-1) | ((I3CS_CFG_RESP_QUEUE_FULL_THLD-1)<<8));

    return 0;
}

static int8_t _DevResetRxFifoOnly(I3CS_T * i3cs)
{
    DBGLOG("\nI3CS%d DEVCTL: 0x%08x\n", i3cs->DEVCTL, (i3cs == I3CS0)?0:1);
    i3cs->DEVCTL &= ~I3CS_DEVCTL_ENABLE_Msk;
    while(i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) {}

    i3cs->RSTCTL = BIT4;//BIT4:RX_FIFO_RST
    DBGLOG("\nReset I3CS%d Rx FIFO ... ", (i3cs == I3CS0)?0:1);
    while(i3cs->RSTCTL != 0) {}
    DBGLOG("done\n");

    i3cs->DEVCTL |=  I3CS_DEVCTL_ENABLE_Msk;
    while((i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) == 0) {}
    DBGLOG("re-en(I3CS%d DEVCTL:0x%08x)\n", i3cs->DEVCTL, (i3cs == I3CS0)?0:1);

    WRNLOG("\nSet I3CS%d RESUME ...(#%d) \n", (i3cs == I3CS0)?0:1, __LINE__);
    i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
//    while((i3cs->DEVCTL&I3CS_CTL_RESUME_Msk) != 0) {}//wait for Host send GETSTATUS CCC, then RESUME bit is became to 0
//    printf("resume done\n");

	i3cs->QUETHCTL  = ((I3CS_CFG_CMD_QUEUE_EMPTY_THLD-1) | ((I3CS_CFG_RESP_QUEUE_FULL_THLD-1)<<8));
    return 0;
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

    if (SPDH_IS_DEV_PEC_ENABLE())//if PEC enalbed in I3C mode
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
                    i3cs->CMDQUE = 0x00020028;//1 byte data only + 1 byte PEC, response T = 0 at byte 1, and TID is 5;
                    i3cs->TXRXDAT = g_au8DevReg[s_u8DevMRn];
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
    if (SPDH_IS_DEV_PEC_ENABLE())//if PEC enalbed in I3C mode
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

    do{
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
                if (I3CS_IS_DA_VALID(I3CS0) == 0)
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
                    else{//write
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

                    if (SPDH_IS_DEV_PEC_ENABLE())//if PEC enalbed in I3C mode
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
                                    ERRLOG("\t[ERR]R bit and package length are not matched.(0x%08X)(L:%d)_2\n", s_DevRxBuf[0], __LINE__);
                                }
                            }
                            else
                            {
                                DBGLOG("Default read address pointer\n");
                            }
                        }
                        else{//write
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
                        else{//write
                            _WriteHandler(i3cs);
                        }
                    }
                    //printf("\tRX resp done(len:%d)\n",pRespQ->b.LENGTH);
                    ret = 0;
                }
            }
            else
            {
                //printf("\tTX resp done\n");
                ret = 1;
            }
            //printf("payload length:%d\n", pRespQ->b.LENGTH);
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
            if ((u32RespErrSts <= 1)||(u32RespErrSts == 6))
            {
                /*  Error Recovery Flow: Overflow/Parity/CRC/Frame error, reset RX FIFO. */
                _DevResetRxFifoOnly(i3cs);
            }
//            else if (u32RespErrSts == 2)
//            {
//                /* parity error */
//                //_DevResetRxFifoOnly(i3c);
//            }
            else
            {
                /*  Error Recovery Flow: Master Early termination, reset TX FIFO. */
                _DevResetTxFifoOnly(i3cs);
            }

            ret = -1;
        }
    } while(0);

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

    if((u32IntSts&(I3CS_INTSTS_RXTH_Msk|I3CS_INTSTS_RESPRDY_Msk)) == (I3CS_INTSTS_RXTH_Msk|I3CS_INTSTS_RESPRDY_Msk)) // auto cleared
    {
        //printf("INT RXTHLD and RESPRDY(Read)\n");
        s_DevRxBuf[0] = I3CS_GET_RXD(i3cs);
        if(I3CS_ProcessRespQueue(i3cs) >= 0)
            g_u32DevRespIntCnt = 1; //g_RespINTFlag = 1;
    }
    // I3CS_INTSTS_RESPRDY_Msk
    else if(u32IntSts&I3CS_INTSTS_RESPRDY_Msk) // auto cleared
    {
        //prepare read data
        s_DevRxBuf[0] = I3CS_GET_RXD(i3cs);
        if(I3CS_ProcessRespQueue(i3cs) >= 0)
            g_u32DevRespIntCnt = 1; //g_RespINTFlag = 1;
    }

    // I3CS_INTSTS_DAA_Msk
    if(u32IntSts&I3CS_INTSTS_DAA_Msk) // write 1 cleared
    {
        /* Update MR18[5] NF_SEL bit as I3C basic protocol. */
        DevReg_EnableI3C();

        g_u32DeviceChangedToI3CMode = 1;

        DBGLOG("INT DYNAASTS (I3CS%d DA: 0x%02x)\n", (i3cs==I3CS0)?0:1, (uint32_t)I3CS_GET_I3CS_DA(i3cs));
        i3cs->INTSTS = I3CS_INTSTS_DAA_Msk;

        __NOP();
        /* To check if mode of device behind Hub and I3C slave is matched? */
        if (SPDH_GET_DEV_MODE() == 0)
        {
            WRNLOG("[WARN] Mode of device behind Hub and I3C slave is not matched.(#%d)\n\n", __LINE__);
        }
        //break;
    }

    if(u32IntSts&I3CS_INTSTS_IBIUPD_Msk)
    {
        uint32_t u32IBICompleSts;
        u32IBICompleSts = inpw(I3CS1_BASE+0x98);
        DBGLOG("SLV_IBI_RESP: 0x%08X\n", inpw(I3CS1_BASE+0x98));
        if ((u32IBICompleSts&7) == 1)
        {
            DBGLOG("IBI accepted by the Master (ACK response received)\n");
            /* Clear pending status to 0 */
            i3cs->DEVCTL &= ~I3CS_DEVCTL_PENDINT_Msk;
            /* Clear MR48[7] : IBI_STATUS */
            DevReg_ClearIBIStatus();
        }
        else if ((u32IBICompleSts&7) == 2)
        {
            DBGLOG("Master Early Terminate (only for SIR with Data)\n");
        }
        else if ((u32IBICompleSts&7) == 3)
        {
            DBGLOG("IBI Not Attempted\n");
        }
    }

    if(i3cs->CCCDEVS & BIT5)//Protocol error: This bit is set when the slave controller encouters a Parity/CRC error during write data transfer.
    {
        /* Set MR52[0]: PAR_ERROR_STATUS */
        DevReg_SetParityErrStatus();
        WRNLOG("[WARN]Parity error\n");
    }

    if(u32IntSts&I3CS_INTSTS_CCCUPD_Msk)// write 1 cleared
    {
        DBGLOG("INT CCCUPD (I3CS%d DA: 0x%02x)\n", (i3cs==I3CS0)?0:1, (uint32_t)I3CS_GET_I3CS_DA(i3cs));
        /* This interrupt is generated if any of the CCC registers are updated by I3C master through CCC commands. */
        if(i3cs->SLVEVNTS & BIT0)
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
        i3cs->INTSTS = I3CS_INTEN_CCCUPD_Msk;
    }

    if(i3cs->CCCDEVS & BIT9)//(Slave_BUSY == TRUE) // Halt state
    {
        i3cs->DEVCTL &= ~I3CS_DEVCTL_ENABLE_Msk;
        while(i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) {}

        i3cs->RSTCTL = 0x1E;
        DBGLOG("\n[SlvBusy]Reset I3C%d all FIFO ... ", (i3cs==I3CS0)?0:1);
        while(i3cs->RSTCTL != 0) {}
        DBGLOG("done\n");

        i3cs->DEVCTL |=  I3CS_DEVCTL_ENABLE_Msk;
        while((i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) == 0) {}

        WRNLOG("\nSet I3C%d RESUME ...(#%d) ", (i3cs==I3CS0)?0:1, __LINE__);
        i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
//            while((i3cs->DEVCTL&I3CS_CTL_RESUME_Msk) != 0) {}
//            printf("done\n");
    }

    u32PresentSts = ((i3cs->PRESENTS>>8)&0x1F);
    if(u32PresentSts == 6) // Halt state
    {
        i3cs->DEVCTL &= ~I3CS_DEVCTL_ENABLE_Msk;
        while(i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) {}

        i3cs->RSTCTL = 0x1E;
        DBGLOG("\n[TransferHalt]Reset I3C%d all FIFO ... ", (i3cs==I3CS0)?0:1);
        while(i3cs->RSTCTL != 0) {}
        DBGLOG("done\n");

        i3cs->DEVCTL |=  I3CS_DEVCTL_ENABLE_Msk;
        while((i3cs->DEVCTL&I3CS_DEVCTL_ENABLE_Msk) == 0) {}

        WRNLOG("\nSet I3CS%d RESUME ...(#%d) ", (i3cs==I3CS0)?0:1, __LINE__);
        i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
//            while((i3cs->DEVCTL&I3CS_CTL_RESUME_Msk) != 0) {}
//            printf("done\n");
    }

    return 0;
}

void LocalDev_SPDHIRQHandler(void)
{
    uint32_t u32SpdhSts, u32HubDevCtrlCfg, u32I3CAddr;

    u32SpdhSts = SPDH_GET_INT_STATUS();

    if(u32SpdhSts & SPDH_INTSTS_DDEVCTLIF_Msk)
    {
        DBGLOG("SPDH_IRQ: Device Received DEVCTRL CCC\n");
        u32HubDevCtrlCfg = SPDH_GET_HUB_STATUS();
        if (u32HubDevCtrlCfg & SPDH_DSTS_PECSTS_Msk)
        {
            DBGLOG("Device DEVCTRL: PEC enabled.\n");
            /* Enable PEC function in Hub's device. */
            SPDH_ENABLE_DEV_CRC();
            /* Update MR18 register. */
            DevReg_PECEnable();
           //printf("Enable PEC function in Hub's device\n");
        }
        else
        {
            DBGLOG("Device DEVCTRL: PEC disabled.\n");
            /* Disable PEC function in Hub,s device. */
            SPDH_DISABLE_DEV_CRC();
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
        SPDH_CLEAR_INT_FLAG(SPDH_INTSTS_DDEVCTLIF_Msk);
    }
    if(u32SpdhSts & SPDH_INTSTS_DDEVCAPIF_Msk)
    {
        DBGLOG("SPDH_IRQ: Device Receive DEVCAP CCC\n");
        SPDH_CLEAR_INT_FLAG(SPDH_INTSTS_DDEVCAPIF_Msk);
    }

    if (u32SpdhSts & SPDH_INTSTS_DEVIHDIF_Msk)
    {
        DBGLOG("\nDevice received IBI header\n");
        SPDH_CLEAR_INT_FLAG(SPDH_INTSTS_DEVIHDIF_Msk);

        /* if pending status is 0x1, then re-send the IBI request */
        
        if (SPDH_IS_DEV_INT_STATUS(SPDH_DSTS_PENDIBI_Msk))
        {
            DBGLOG("\nre-send the IBI request\n");
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

    /* Enable I3C Hub clock. */
    CLK->APBCLK0 |= BIT26;

    /* Enable I3C1 clock for Hub. */
    CLK->APBCLK0 |= BIT25;
    
    /* Set LID. */
    SPDH_SET_DEV_LID(((u8DevAddr)&SPDH_DCTL_LID_Msk)>>SPDH_DCTL_LID_Pos);

    /* Disable Hot-join function before I3CS was enabled. */
    I3CS1->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;

    /* Disable SIR. */
    SPDH_DISABLE_DEV_SIR();

    I3CS_Open(I3CS1, u8DevAddr, 8);

    /* Enable chip's SPD5 Hub function */
    SYS->SPDHCTL |= SYS_SPDHCTL_SPDHEN_Msk;

    I3CS1->DBTHCTL = 0x3;

    /* configure Bus available time: JESD define is 1 us, 1/0.083 = 12 */
    I3CS1->BUSFAT = 12 << 16; //adjust for bus available time for IBI issue

	I3CS1->QUETHCTL  = ((I3CS_CFG_CMD_QUEUE_EMPTY_THLD-1) | ((I3CS_CFG_RESP_QUEUE_FULL_THLD-1)<<8));

    /* Enable Hub INT Status */
    SPDH_ENABLE_INT(0x6FF); //Hub received DEVCAP and DEVCTRL CCC, and IBI header

    /* Enable I3CS1 INT Status */
    I3CS1->INTSTSEN = 0xFFFFFFFF;//All event was enabled


    /* Enable I3CS1 INT */
    I3CS1->INTEN |= (I3CS_INTEN_RESPQ_READY | I3CS_INTEN_CCC_UPDATED | I3CS_INTEN_DA_ASSIGNED |
                        I3CS_INTEN_TRANSFER_ERR | I3CS_INTEN_READ_REQUEST|I3CS_INTEN_RX_THLD);
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
    SPDH_DISABLE_DEV_SIR();

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
            if (SPDH_GET_DEV_MODE() == 1)
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
    if (SPDH_IS_DEV_PEC_ENABLE())
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


/***************************************************************************//**
 * @file     fmc_user.c
 * @version  V1.00 
 * @brief    FMC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "fmc_user.h"


int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    uint32_t u32Addr, Reg;
    uint32_t u32TimeOutCnt;

    for (u32Addr = addr_start; u32Addr < addr_end; data++) {
        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;

        if (u32Cmd == FMC_ISPCMD_PROGRAM) {
            FMC->ISPDAT = *data;
        }

        FMC->ISPTRG = 0x1;
        __ISB();

        /* Wait for ISP command done. */
        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while (FMC->ISPTRG & 0x1)
            if(--u32TimeOutCnt == 0) return -1;

        Reg = FMC->ISPCTL;

        if (Reg & FMC_ISPCTL_ISPFF_Msk) {
            FMC->ISPCTL = Reg;
            return -1;
        }

        if (u32Cmd == FMC_ISPCMD_READ) {
            *data = FMC->ISPDAT;
        }

        if (u32Cmd == FMC_ISPCMD_PAGE_ERASE) {
            u32Addr += FMC_FLASH_PAGE_SIZE;
        } else {
            u32Addr += 4;
        }
    }

    return 0;
}

void UpdateConfig(uint32_t *data, uint32_t *res)
{
    uint32_t u32Size = CONFIG_SIZE;
    FMC_ENABLE_CFG_UPDATE();
    FMC_Proc(FMC_ISPCMD_PAGE_ERASE, Config0, Config0 + 8, 0);
    FMC_Proc(FMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, data);

    if (res) {
        FMC_Proc(FMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    }

    FMC_DISABLE_CFG_UPDATE();
}

/**
 ******************************************************************************
 * @file    bms_nvm_port.c
 * @brief   SOC/SOH toolkit NVM 移植层：页 48/49，132 B 记录按 136 B 编程。
 *
 * M3 起由 App/detect/soc_est.c 管理：SocEst_Init -> BMS_NVM_Init，
 * SocEst_Task -> BMS_NVM_Task；进 LP 前 SocEst_PrepareForLp 立即落盘。
 ******************************************************************************
 */
#include "bms_nvm.h"

#include "board.h"
#include "bsp/flash_store.h"

#include <stddef.h>
#include <string.h>

#define BMS_NVM_SLOT_COUNT_IMPL  2u
#define BMS_NVM_PROGRAM_BYTES    136u

static uint32_t BmsNvm_SlotAddr(uint8_t slot)
{
    return BMS_NVM_FLASH_BASE + ((uint32_t)slot * BMS_NVM_FLASH_PAGE_SIZE);
}

uint8_t BMS_NVM_SlotCount(void)
{
    return BMS_NVM_SLOT_COUNT_IMPL;
}

uint8_t BMS_NVM_SlotRead(uint8_t slot, void *buf, uint32_t len)
{
    const uint8_t *src;

    if ((buf == NULL) || (slot >= BMS_NVM_SLOT_COUNT_IMPL) ||
        (len != BMS_NVM_BLOCK_LEN))
    {
        return 0u;
    }

    src = (const uint8_t *)BmsNvm_SlotAddr(slot);
    memcpy(buf, src, len);
    return 1u;
}

uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len)
{
    static uint8_t padded[BMS_NVM_PROGRAM_BYTES];
    uint32_t addr;

    if ((buf == NULL) || (slot >= BMS_NVM_SLOT_COUNT_IMPL) ||
        (len != BMS_NVM_BLOCK_LEN) ||
        ((uint32_t)&_image_end > BMS_NVM_FLASH_BASE))
    {
        return 0u;
    }

    memset(padded, 0xFF, sizeof(padded));
    memcpy(padded, buf, len);
    addr = BmsNvm_SlotAddr(slot);

    if (Fs_ErasePage(addr) != 0)
    {
        return 0u;
    }
    if (Fs_ProgramDwords(addr, padded, sizeof(padded)) != 0)
    {
        return 0u;
    }
    return 1u;
}

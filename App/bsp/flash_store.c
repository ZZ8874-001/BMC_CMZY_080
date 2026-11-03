/**
 ******************************************************************************
 * @file    flash_store.c
 * @brief   通用 Flash 擦/写原语（STM32L432KB，2 KB/页，单 bank）。
 ******************************************************************************
 */
#include "bsp/flash_store.h"

#include "stm32l4xx_hal.h"

#include <stddef.h>
#include <string.h>

int Fs_ErasePage(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0u;

    if (page_addr < FLASH_BASE || page_addr >= (FLASH_BASE + FLASH_SIZE))
    {
        return -1;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 3;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = (page_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages   = 1u;

    if ((HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) ||
        (page_error != 0xFFFFFFFFu))
    {
        (void)HAL_FLASH_Lock();
        return 1;
    }

    (void)HAL_FLASH_Lock();
    return 0;
}

int Fs_ProgramDwords(uint32_t addr, const uint8_t *src, uint32_t len)
{
    uint32_t offset;

    if ((src == NULL) || ((len & 7u) != 0u))
    {
        return -1;
    }
    if ((addr < FLASH_BASE) || ((addr + len) > (FLASH_BASE + FLASH_SIZE)))
    {
        return -1;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 3;
    }

    for (offset = 0u; offset < len; offset += 8u)
    {
        uint64_t dword;

        memcpy(&dword, src + offset, sizeof(dword));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              addr + offset, dword) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return 2;
        }
    }

    (void)HAL_FLASH_Lock();
    return 0;
}

/**
 ******************************************************************************
 * @file    state_store.c
 * @brief   L1 StateStore：页 50–51，48 B 记录，两页环形，CRC16-CCITT。
 *
 * 记录布局（小端）：
 *   off 0  magic u32      0x4C315354 ("L1ST")
 *   off 4  version u16
 *   off 6  flags u16
 *   off 8  seq u32
 *   off 12 battery_epoch u32
 *   off 16 cell_mv[3] u16
 *   off 22 pack_mv u16
 *   off 24 soc01 u16
 *   off 26 soh01 u16
 *   off 28 temp_dc i16
 *   off 30 reserved[16]
 *   off 46 crc16 u16（覆盖 off 0..45）
 *
 * 只允许初始化/主循环调用；不在 ISR 写 Flash。
 ******************************************************************************
 */
#include "app/state_store.h"

#include "board.h"
#include "app/faults.h"
#include "bsp/flash_store.h"

#include <stddef.h>

#define SS_MAGIC                0x4C315354UL
#define SS_VERSION              1u
#define SS_CRC_OFFSET           46u
#define SS_CRC_LEN              46u
#define SS_SLOT_COUNT           ((uint16_t)(STATE_STORE_FLASH_PAGES * \
                                             (BMS_NVM_FLASH_PAGE_SIZE / \
                                              STATE_STORE_RECORD_BYTES)))
#define SS_SLOTS_PER_PAGE       ((uint16_t)(BMS_NVM_FLASH_PAGE_SIZE / \
                                             STATE_STORE_RECORD_BYTES))
#define SS_BLANK_U32            0xFFFFFFFFUL

_Static_assert(STATE_STORE_RECORD_BYTES == 48u, "StateStore record must be 48 B");
_Static_assert((STATE_STORE_RECORD_BYTES % 8u) == 0u,
               "StateStore record must be 8-byte programmable");

static struct
{
    uint8_t valid;
    uint8_t err;
    uint32_t seq;
    uint16_t active_slot;
    uint16_t next_slot;
    state_store_view_t view;
} s_state;

static uint16_t Ss_Crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t bit;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x8000u) != 0u)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint32_t Ss_SlotAddr(uint16_t slot)
{
    uint16_t page = (uint16_t)(slot / SS_SLOTS_PER_PAGE);
    uint16_t index = (uint16_t)(slot % SS_SLOTS_PER_PAGE);

    return STATE_STORE_FLASH_BASE + ((uint32_t)page * BMS_NVM_FLASH_PAGE_SIZE) +
           ((uint32_t)index * STATE_STORE_RECORD_BYTES);
}

static uint32_t Ss_ReadU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t Ss_ReadU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void Ss_WriteU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void Ss_WriteU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint8_t Ss_SlotValid(const uint8_t *p)
{
    if ((Ss_ReadU32(p) != SS_MAGIC) ||
        (Ss_ReadU16(p + 4u) != SS_VERSION))
    {
        return 0u;
    }
    return (Ss_Crc16(p, SS_CRC_LEN) == Ss_ReadU16(p + SS_CRC_OFFSET)) ? 1u : 0u;
}

static void Ss_Pack(const state_store_view_t *view, uint32_t seq, uint8_t raw[48])
{
    uint16_t crc;
    uint8_t i;

    for (i = 0u; i < STATE_STORE_RECORD_BYTES; i++)
    {
        raw[i] = 0xFFu;
    }

    Ss_WriteU32(&raw[0], SS_MAGIC);
    Ss_WriteU16(&raw[4], SS_VERSION);
    Ss_WriteU16(&raw[6], view->flags);
    Ss_WriteU32(&raw[8], seq);
    Ss_WriteU32(&raw[12], view->battery_epoch);
    Ss_WriteU16(&raw[16], view->cell_mv[0]);
    Ss_WriteU16(&raw[18], view->cell_mv[1]);
    Ss_WriteU16(&raw[20], view->cell_mv[2]);
    Ss_WriteU16(&raw[22], view->pack_mv);
    Ss_WriteU16(&raw[24], view->soc01);
    Ss_WriteU16(&raw[26], view->soh01);
    Ss_WriteU16(&raw[28], (uint16_t)view->temp_dc);

    crc = Ss_Crc16(raw, SS_CRC_LEN);
    Ss_WriteU16(&raw[SS_CRC_OFFSET], crc);
}

static void Ss_Unpack(const uint8_t *raw, state_store_view_t *view, uint32_t *seq)
{
    view->flags = Ss_ReadU16(&raw[6]);
    *seq = Ss_ReadU32(&raw[8]);
    view->battery_epoch = Ss_ReadU32(&raw[12]);
    view->cell_mv[0] = Ss_ReadU16(&raw[16]);
    view->cell_mv[1] = Ss_ReadU16(&raw[18]);
    view->cell_mv[2] = Ss_ReadU16(&raw[20]);
    view->pack_mv = Ss_ReadU16(&raw[22]);
    view->soc01 = Ss_ReadU16(&raw[24]);
    view->soh01 = Ss_ReadU16(&raw[26]);
    view->temp_dc = (int16_t)Ss_ReadU16(&raw[28]);
    view->valid = 1u;
}

void StateStore_Init(void)
{
    uint16_t slot;
    uint16_t best_slot = 0u;
    uint32_t best_seq = 0u;
    uint8_t found = 0u;

    s_state.valid = 0u;
    s_state.err = 0u;
    s_state.seq = 0u;
    s_state.active_slot = 0u;
    s_state.next_slot = 0u;
    s_state.view.flags = 0u;
    s_state.view.seq = 0u;
    s_state.view.battery_epoch = 0u;
    s_state.view.cell_mv[0] = 0u;
    s_state.view.cell_mv[1] = 0u;
    s_state.view.cell_mv[2] = 0u;
    s_state.view.pack_mv = 0u;
    s_state.view.soc01 = 0u;
    s_state.view.soh01 = 0u;
    s_state.view.temp_dc = 0;
    s_state.view.valid = 0u;

    /* 固件不得侵入预留页。 */
    if ((uint32_t)&_image_end > STATE_STORE_FLASH_BASE)
    {
        s_state.err = 2u;
        Fault_SetSticky(FAULT_BIT_FLASH_STATE);
        return;
    }

    for (slot = 0u; slot < SS_SLOT_COUNT; slot++)
    {
        const uint8_t *p = (const uint8_t *)Ss_SlotAddr(slot);
        uint32_t seq;

        if (Ss_SlotValid(p) == 0u)
        {
            continue;
        }
        seq = Ss_ReadU32(p + 8u);
        if ((found == 0u) || (seq > best_seq))
        {
            best_seq = seq;
            best_slot = slot;
            found = 1u;
        }
    }

    if (found != 0u)
    {
        const uint8_t *p = (const uint8_t *)Ss_SlotAddr(best_slot);

        Ss_Unpack(p, &s_state.view, &s_state.seq);
        s_state.view.seq = s_state.seq;
        s_state.valid = 1u;
        s_state.active_slot = best_slot;
        s_state.next_slot = (uint16_t)((best_slot + 1u) % SS_SLOT_COUNT);
    }
}

uint8_t StateStore_IsValid(void)
{
    return s_state.valid;
}

void StateStore_GetView(state_store_view_t *out)
{
    if (out != NULL)
    {
        *out = s_state.view;
    }
}

uint32_t StateStore_GetSeq(void)
{
    return s_state.seq;
}

uint16_t StateStore_GetActiveSlot(void)
{
    return s_state.active_slot;
}

int StateStore_Commit(const state_store_view_t *view)
{
    uint8_t raw[STATE_STORE_RECORD_BYTES];
    uint16_t slot;
    uint32_t addr;
    uint32_t seq;

    if ((view == NULL) || (s_state.err == 2u))
    {
        return -1;
    }

    seq = s_state.valid ? (s_state.seq + 1u) : 1u;
    if (seq == SS_BLANK_U32)
    {
        seq = 1u;
    }

    slot = s_state.next_slot;
    addr = Ss_SlotAddr(slot);

    if ((slot % SS_SLOTS_PER_PAGE) == 0u)
    {
        if (Fs_ErasePage(addr) != 0)
        {
            s_state.err = 1u;
            Fault_SetSticky(FAULT_BIT_FLASH_STATE);
            return -1;
        }
    }

    Ss_Pack(view, seq, raw);
    if (Fs_ProgramDwords(addr, raw, STATE_STORE_RECORD_BYTES) != 0)
    {
        s_state.err = 1u;
        Fault_SetSticky(FAULT_BIT_FLASH_STATE);
        return -1;
    }

    s_state.view = *view;
    s_state.view.seq = seq;
    s_state.view.valid = 1u;
    s_state.seq = seq;
    s_state.valid = 1u;
    s_state.active_slot = slot;
    s_state.next_slot = (uint16_t)((slot + 1u) % SS_SLOT_COUNT);
    s_state.err = 0u;
    return 0;
}

int StateStore_ClearLatch(uint16_t flags)
{
    state_store_view_t view;

    if (s_state.valid == 0u)
    {
        return -1;
    }

    view = s_state.view;
    view.flags &= (uint16_t)~flags;
    return StateStore_Commit(&view);
}

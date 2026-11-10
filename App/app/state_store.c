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
 *   off 30 reserved[16] (v1 全 0xFF；v2 使用以下二级 CRC 格式)：
 *     +0  last_mark_epoch u32 (RTC 秒，非 uptime)
 *     +4  last_mark_id u32 (NORM/LP 同一递增标记 ID)
 *     +8  anomaly_count u8 (最多 4)
 *     +9  alarm_flags u8
 *     +10 estimate_flags u8
 *     +11 mark_quality u8
 *     +12/+13 v2 元数据标识 0xA2/0x5D
 *     +14 archive_crc16 u16 (仅覆盖 reserved[0..13])
 *   off 46 crc16 u16（覆盖 off 0..45，v1/v2 均保持原 CRC）
 *
 * 上电扫描兼容 v1 和 v2；每次写入为 v2，不改变 48 B 页布局。
 *
 * 只允许初始化/主循环调用；不在 ISR 写 Flash。
 ******************************************************************************
 */
#include "app/state_store.h"

#include "board.h"
#include "app/faults.h"
#include "bsp/flash_store.h"

#include <stddef.h>
#include <string.h>

#define SS_MAGIC                0x4C315354UL
#define SS_VERSION_V1           1u
#define SS_VERSION_V2           2u
#define SS_ARCHIVE_OFFSET       30u
#define SS_ARCHIVE_BYTES        16u
#define SS_ARCHIVE_CRC_OFFSET   44u
#define SS_ARCHIVE_CRC_LEN      14u
#define SS_ARCHIVE_TAG0         0xA2u
#define SS_ARCHIVE_TAG1         0x5Du
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
_Static_assert((SS_ARCHIVE_OFFSET + SS_ARCHIVE_BYTES) == SS_CRC_OFFSET,
               "StateStore v2 archive must fit reserved 16 B");

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

static uint32_t Ss_ReadU32(const uint8_t *p);

/* 断电/编程中断可能留下 CRC 无效的非空槽。跳过它，直到找到空槽；
 * 页首可直接作为下一写入点，因为写入前会整页擦除。 */
static uint16_t Ss_FindWritableSlot(uint16_t candidate)
{
    uint16_t offset;

    for (offset = 0u; offset < SS_SLOT_COUNT; offset++)
    {
        uint16_t slot = (uint16_t)((candidate + offset) % SS_SLOT_COUNT);
        const uint8_t *p = (const uint8_t *)Ss_SlotAddr(slot);

        if ((slot % SS_SLOTS_PER_PAGE) == 0u ||
            (Ss_ReadU32(p) == SS_BLANK_U32))
        {
            return slot;
        }
    }
    return candidate;
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
    uint16_t version;

    if (Ss_ReadU32(p) != SS_MAGIC)
    {
        return 0u;
    }
    version = Ss_ReadU16(p + 4u);
    if ((version != SS_VERSION_V1) && (version != SS_VERSION_V2))
    {
        return 0u;
    }
    if (Ss_Crc16(p, SS_CRC_LEN) != Ss_ReadU16(p + SS_CRC_OFFSET))
    {
        return 0u;
    }
    if (version == SS_VERSION_V2)
    {
        const uint8_t *archive = p + SS_ARCHIVE_OFFSET;

        if ((archive[12] != SS_ARCHIVE_TAG0) ||
            (archive[13] != SS_ARCHIVE_TAG1) ||
            (Ss_Crc16(archive, SS_ARCHIVE_CRC_LEN) !=
             Ss_ReadU16(archive + SS_ARCHIVE_CRC_LEN)) ||
            (archive[8] > 4u) ||
            ((archive[9] & (uint8_t)~(STATE_ARCHIVE_ALARM_PENDING |
                                     STATE_ARCHIVE_ALARM_LATCHED |
                                     STATE_ARCHIVE_SHUTDOWN_CONFIRMED)) != 0u) ||
            ((archive[10] & (uint8_t)~STATE_ARCHIVE_Q_ALL) != 0u) ||
            (archive[11] > STATE_ARCHIVE_MARK_CRITICAL))
        {
            return 0u;
        }
    }
    return 1u;
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
    Ss_WriteU16(&raw[4], SS_VERSION_V2);
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

    Ss_WriteU32(&raw[30], view->archive.last_mark_epoch);
    Ss_WriteU32(&raw[34], view->archive.last_mark_id);
    raw[38] = view->archive.anomaly_count;
    raw[39] = view->archive.alarm_flags;
    raw[40] = view->archive.estimate_flags;
    raw[41] = view->archive.mark_quality;
    raw[42] = SS_ARCHIVE_TAG0;
    raw[43] = SS_ARCHIVE_TAG1;
    Ss_WriteU16(&raw[SS_ARCHIVE_CRC_OFFSET],
                Ss_Crc16(&raw[SS_ARCHIVE_OFFSET], SS_ARCHIVE_CRC_LEN));

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
    memset(&view->archive, 0, sizeof(view->archive));
    if (Ss_ReadU16(&raw[4]) == SS_VERSION_V2)
    {
        view->archive.last_mark_epoch = Ss_ReadU32(&raw[30]);
        view->archive.last_mark_id = Ss_ReadU32(&raw[34]);
        view->archive.anomaly_count = raw[38];
        view->archive.alarm_flags = raw[39];
        view->archive.estimate_flags = raw[40];
        view->archive.mark_quality = raw[41];
    }
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
    memset(&s_state.view, 0, sizeof(s_state.view));

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

/* 内部提交保留 v1 兼容字段；Flash 成功且回读完全一致后才更新 RAM。
 * archive 必须来自当前 s_state.view 或由显式归档 API 派生，不接受外部
 * 已过时的 view->archive；这保证 SocEst 普通快照提交不会抹掉锁存。 */
static int Ss_CommitWithArchive(const state_store_view_t *view,
                                const state_store_archive_t *archive)
{
    state_store_view_t next;
    uint8_t raw[STATE_STORE_RECORD_BYTES];
    uint16_t slot;
    uint32_t addr;
    uint32_t seq;

    if ((view == NULL) || (archive == NULL) || (s_state.err == 2u))
    {
        return -1;
    }

    seq = s_state.valid ? (s_state.seq + 1u) : 1u;
    if (seq == SS_BLANK_U32)
    {
        seq = 1u;
    }
    if (archive->anomaly_count > 4u ||
        archive->mark_quality > STATE_ARCHIVE_MARK_CRITICAL ||
        (archive->estimate_flags & (uint8_t)~STATE_ARCHIVE_Q_ALL) != 0u ||
        (archive->alarm_flags &
         (uint8_t)~(STATE_ARCHIVE_ALARM_PENDING | STATE_ARCHIVE_ALARM_LATCHED |
                    STATE_ARCHIVE_SHUTDOWN_CONFIRMED)) != 0u)
    {
        return -1;
    }

    slot = Ss_FindWritableSlot(s_state.next_slot);
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

    next = *view;
    next.archive = *archive;
    Ss_Pack(&next, seq, raw);
    if ((Fs_ProgramDwords(addr, raw, STATE_STORE_RECORD_BYTES) != 0) ||
        (memcmp((const void *)addr, raw, STATE_STORE_RECORD_BYTES) != 0))
    {
        s_state.err = 1u;
        Fault_SetSticky(FAULT_BIT_FLASH_STATE);
        return -1;
    }

    s_state.view = next;
    s_state.view.seq = seq;
    s_state.view.valid = 1u;
    s_state.seq = seq;
    s_state.valid = 1u;
    s_state.active_slot = slot;
    s_state.next_slot = (uint16_t)((slot + 1u) % SS_SLOT_COUNT);
    s_state.err = 0u;
    return 0;
}

int StateStore_Commit(const state_store_view_t *view)
{
    /* 只允许专用归档接口修改计数/标记/报警；旧的 GetView→Commit
     * 流程可以自由更新电池快照，却不能回滚 v2 元数据。 */
    return Ss_CommitWithArchive(view, &s_state.view.archive);
}

int StateStore_CommitArchiveSnapshot(const state_store_view_t *view,
                                     uint32_t mark_epoch, uint32_t mark_id,
                                     uint8_t mark_quality, uint8_t estimate_flags)
{
    state_store_archive_t archive;
    state_store_view_t next;
    uint16_t missing;

    if ((view == NULL) || (mark_epoch == 0u) || (mark_id == 0u) ||
        (estimate_flags & (uint8_t)~STATE_ARCHIVE_Q_ALL) != 0u ||
        ((mark_quality != STATE_ARCHIVE_MARK_FULL) &&
         (mark_quality != STATE_ARCHIVE_MARK_DEGRADED) &&
         (mark_quality != STATE_ARCHIVE_MARK_CRITICAL)))
    {
        return -1;
    }
    if ((s_state.valid != 0u) && (s_state.view.archive.last_mark_id != 0u) &&
        (mark_id <= s_state.view.archive.last_mark_id))
    {
        return 1;  /* 同一 RTC 标记或更早的重放：不写 Flash，不动计数 */
    }

    missing = (uint16_t)(estimate_flags &
                         (STATE_ARCHIVE_Q_CELL1_MISSING |
                          STATE_ARCHIVE_Q_CELL2_MISSING |
                          STATE_ARCHIVE_Q_CELL3_MISSING |
                          STATE_ARCHIVE_Q_PACK_MISSING));
    if (((mark_quality == STATE_ARCHIVE_MARK_FULL) && (missing != 0u)) ||
        ((mark_quality == STATE_ARCHIVE_MARK_DEGRADED) &&
         ((missing == 0u) ||
          ((missing & (STATE_ARCHIVE_Q_CELL1_MISSING |
                       STATE_ARCHIVE_Q_CELL2_MISSING)) != 0u))) ||
        ((mark_quality == STATE_ARCHIVE_MARK_CRITICAL) &&
         ((missing & (STATE_ARCHIVE_Q_CELL1_MISSING |
                      STATE_ARCHIVE_Q_CELL2_MISSING)) == 0u)))
    {
        return -1;
    }

    /* 未实测的字段必须保持为 0；不允许估算值通过 v2 快照入口落盘。 */
    if (((missing & STATE_ARCHIVE_Q_CELL1_MISSING) && view->cell_mv[0] != 0u) ||
        ((missing & STATE_ARCHIVE_Q_CELL2_MISSING) && view->cell_mv[1] != 0u) ||
        ((missing & STATE_ARCHIVE_Q_CELL3_MISSING) && view->cell_mv[2] != 0u) ||
        ((missing & STATE_ARCHIVE_Q_PACK_MISSING) && view->pack_mv != 0u))
    {
        return -1;
    }

    next = *view;
    if (mark_quality != STATE_ARCHIVE_MARK_FULL)
    {
        next.flags |= STATE_FLAG_PACK_PARTIAL;
        next.flags &= (uint16_t)~STATE_FLAG_PACK_VALID;
    }
    else
    {
        next.flags &= (uint16_t)~STATE_FLAG_PACK_PARTIAL;
    }
    if ((missing & STATE_ARCHIVE_Q_PACK_MISSING) != 0u)
    {
        next.flags &= (uint16_t)~STATE_FLAG_PACK_VALID;
    }
    if ((estimate_flags & STATE_ARCHIVE_Q_SOC_ESTIMATED) != 0u)
    {
        next.flags |= STATE_FLAG_SOC_ESTIMATED;
        next.flags &= (uint16_t)~STATE_FLAG_SOC_VALID;
    }
    else
    {
        next.flags &= (uint16_t)~STATE_FLAG_SOC_ESTIMATED;
    }
    if ((estimate_flags & STATE_ARCHIVE_Q_SOC_INVALID) != 0u)
    {
        next.flags &= (uint16_t)~STATE_FLAG_SOC_VALID;
    }

    archive = s_state.view.archive;
    archive.last_mark_epoch = mark_epoch;
    archive.last_mark_id = mark_id;
    archive.mark_quality = mark_quality;
    archive.estimate_flags = estimate_flags;
    /* 报警/锁存/计数语义已随 bq_anomaly 框架删除；保留字段仅为格式兼容。 */
    archive.anomaly_count = 0u;
    archive.alarm_flags = 0u;
    return Ss_CommitWithArchive(&next, &archive);
}

int StateStore_CommitArchiveMark(uint32_t mark_epoch, uint32_t mark_id,
                                 uint8_t mark_quality, uint8_t estimate_flags)
{
    state_store_archive_t archive;

    /* LP 快标记只动 v2 元数据，不碰已有快照或 SOC 有效位。 */
    if ((mark_epoch == 0u) || (mark_id == 0u) ||
        ((mark_quality != STATE_ARCHIVE_MARK_FULL) &&
         (mark_quality != STATE_ARCHIVE_MARK_DEGRADED)) ||
        ((estimate_flags & (uint8_t)~STATE_ARCHIVE_Q_ALL) != 0u))
    {
        return -1;
    }
    if ((s_state.valid != 0u) && (s_state.view.archive.last_mark_id != 0u) &&
        (mark_id <= s_state.view.archive.last_mark_id))
    {
        return 1;
    }
    archive = s_state.view.archive;
    archive.last_mark_epoch = mark_epoch;
    archive.last_mark_id = mark_id;
    archive.mark_quality = mark_quality;
    archive.estimate_flags = estimate_flags;
    archive.anomaly_count = 0u;
    archive.alarm_flags = 0u;
    return Ss_CommitWithArchive(&s_state.view, &archive);
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

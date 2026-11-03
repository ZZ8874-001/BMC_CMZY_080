/**
 ******************************************************************************
 * @file    black_box.c
 * @brief   黑匣子 Flash 环形日志：12 页 × 85 条 × 24 B = 1020 条（≈21 天）。
 *
 * 记录布局（小端，手动逐字节打包）：
 *   off  0: seq        u32   全局自增序号（0xFFFFFFFF 表示空白槽）
 *   off  4: uptime_s   u32   累计通电运行秒（LP 期间不计）
 *   off  8: cell1_mv   u16
 *   off 10: cell2_mv   u16
 *   off 12: cell3_mv   u16
 *   off 14: pack_mv    u16
 *   off 16: cc_raw     i16
 *   off 18: ts1_raw    u16
 *   off 20: flags      u16   bit2:0 记录种类，bit5:3 唤醒原因
 *   off 22: crc16      u16   CRC-16-CCITT(0xFFFF 初值) 覆盖前 22 B
 *
 * 写入策略：仅在主循环、仅 NORM；跨页首槽先擦除整页（~25 ms 取指停摆）。
 * 断电安全：擦除后未写入的空白槽在下次上电时被跳过，seq 从上一个有效头接续。
 ******************************************************************************
 */
#include "app/black_box.h"

#include "board.h"
#include "bsp/bq_ic.h"
#include "bsp/flash_store.h"
#include "app/faults.h"

#define BB_SLOT_BYTES       24u
#define BB_RECORD_WORDS     (BB_RECORD_BYTES / 8u)   /* 3 个双字 */
#define BB_SLOT_COUNT       BB_RECORD_COUNT          /* 1020 */
#define BB_BLANK_U32        0xFFFFFFFFUL

/* 链接符号：.text 结束地址，用于断言固件不侵入黑匣子区。 */
extern uint32_t _etext;

typedef struct
{
    uint32_t seq;
    uint32_t uptime_s;
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    int16_t cc_raw;
    uint16_t ts1_raw;
    uint16_t flags;
    uint16_t crc16;
} bb_record_t;

static struct
{
    uint8_t valid_head;
    uint32_t head_seq;
    uint32_t uptime_base;
    uint16_t next_slot;         /* 0..1019 */
    uint32_t writes;
    uint32_t erase_count;
    uint32_t write_fail_count;
    uint32_t erase_fail_count;
    uint32_t skipped_bad_records;
    uint8_t err;
} s_bb;

/* ---------------- 工具 ---------------- */
static uint16_t Bb_Crc16(const uint8_t *data, uint32_t len)
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

static uint32_t Bb_SlotAddr(uint16_t slot)
{
    uint16_t page = (uint16_t)(slot / BB_RECORDS_PER_PAGE);
    uint16_t index = (uint16_t)(slot % BB_RECORDS_PER_PAGE);

    return BB_FLASH_BASE + ((uint32_t)page * BB_FLASH_PAGE_SIZE) +
           ((uint32_t)index * BB_SLOT_BYTES);
}

static void Bb_Pack(const bb_record_t *rec, uint8_t out[BB_SLOT_BYTES])
{
    uint16_t crc;

    out[0]  = (uint8_t)(rec->seq & 0xFFu);
    out[1]  = (uint8_t)((rec->seq >> 8) & 0xFFu);
    out[2]  = (uint8_t)((rec->seq >> 16) & 0xFFu);
    out[3]  = (uint8_t)((rec->seq >> 24) & 0xFFu);
    out[4]  = (uint8_t)(rec->uptime_s & 0xFFu);
    out[5]  = (uint8_t)((rec->uptime_s >> 8) & 0xFFu);
    out[6]  = (uint8_t)((rec->uptime_s >> 16) & 0xFFu);
    out[7]  = (uint8_t)((rec->uptime_s >> 24) & 0xFFu);

    out[8]  = (uint8_t)(rec->cell_mv[0] & 0xFFu);
    out[9]  = (uint8_t)((rec->cell_mv[0] >> 8) & 0xFFu);
    out[10] = (uint8_t)(rec->cell_mv[1] & 0xFFu);
    out[11] = (uint8_t)((rec->cell_mv[1] >> 8) & 0xFFu);
    out[12] = (uint8_t)(rec->cell_mv[2] & 0xFFu);
    out[13] = (uint8_t)((rec->cell_mv[2] >> 8) & 0xFFu);
    out[14] = (uint8_t)(rec->pack_mv & 0xFFu);
    out[15] = (uint8_t)((rec->pack_mv >> 8) & 0xFFu);
    out[16] = (uint8_t)((uint16_t)rec->cc_raw & 0xFFu);
    out[17] = (uint8_t)(((uint16_t)rec->cc_raw >> 8) & 0xFFu);
    out[18] = (uint8_t)(rec->ts1_raw & 0xFFu);
    out[19] = (uint8_t)((rec->ts1_raw >> 8) & 0xFFu);
    out[20] = (uint8_t)(rec->flags & 0xFFu);
    out[21] = (uint8_t)((rec->flags >> 8) & 0xFFu);

    crc = Bb_Crc16(out, 22u);
    out[22] = (uint8_t)(crc & 0xFFu);
    out[23] = (uint8_t)((crc >> 8) & 0xFFu);
}

static uint32_t Bb_ReadU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t Bb_ReadU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint8_t Bb_SlotValid(const uint8_t *p)
{
    uint16_t crc;

    if (Bb_ReadU32(p) == BB_BLANK_U32)
    {
        return 0u;
    }
    crc = Bb_Crc16(p, 22u);
    return (crc == Bb_ReadU16(p + 22)) ? 1u : 0u;
}

/* ---------------- 上电头查找 ---------------- */
void BlackBox_Init(void)
{
    uint16_t slot;
    uint32_t best_seq = 0u;
    uint16_t best_slot = 0u;
    uint8_t found = 0u;

    s_bb.valid_head = 0u;
    s_bb.head_seq = 0u;
    s_bb.uptime_base = 0u;
    s_bb.next_slot = 0u;
    s_bb.writes = 0u;
    s_bb.erase_count = 0u;
    s_bb.write_fail_count = 0u;
    s_bb.erase_fail_count = 0u;
    s_bb.skipped_bad_records = 0u;
    s_bb.err = 0u;

    /* 固件不得侵入黑匣子区，否则擦写会破坏代码。 */
    if ((uint32_t)&_etext >= BB_FLASH_BASE)
    {
        s_bb.err = 2u;
        Fault_SetSticky(FAULT_BIT_FLASH_LOG);
        return;
    }

    for (slot = 0u; slot < BB_SLOT_COUNT; slot++)
    {
        const uint8_t *p = (const uint8_t *)Bb_SlotAddr(slot);

        if (Bb_SlotValid(p) == 0u)
        {
            if (Bb_ReadU32(p) != BB_BLANK_U32)
            {
                s_bb.skipped_bad_records++;
            }
            continue;
        }

        {
            uint32_t seq = Bb_ReadU32(p);

            if ((found == 0u) || (seq > best_seq))
            {
                best_seq = seq;
                best_slot = slot;
                found = 1u;
            }
        }
    }

    if (found != 0u)
    {
        const uint8_t *p = (const uint8_t *)Bb_SlotAddr(best_slot);

        s_bb.valid_head = 1u;
        s_bb.head_seq = best_seq;
        s_bb.uptime_base = Bb_ReadU32(p + 4u);
        s_bb.next_slot = (uint16_t)((best_slot + 1u) % BB_SLOT_COUNT);
    }
    else
    {
        s_bb.next_slot = 0u;
    }
}

uint32_t BlackBox_GetUptimeBase(void)
{
    return s_bb.uptime_base;
}

uint8_t BlackBox_HasError(void)
{
    return s_bb.err;
}

/* ---------------- 写记录 ---------------- */
static int Bb_WriteRecord(uint32_t uptime_s, uint16_t flags)
{
    bq_snapshot_t snap;
    bb_record_t rec;
    uint8_t raw[BB_SLOT_BYTES];
    uint32_t addr;
    uint32_t seq;

    if (s_bb.err == 2u)
    {
        return -1;
    }

    BQ_GetSnapshot(&snap);

    /* 新记录紧接头序号；跳过 0xFFFFFFFF 空白哨兵。 */
    seq = s_bb.valid_head ? (s_bb.head_seq + 1u) : 1u;
    if (seq == BB_BLANK_U32)
    {
        seq = 1u;
    }

    addr = Bb_SlotAddr(s_bb.next_slot);

    /* 跨页首槽：先擦除本页（页内其他记录都是更老的记录）。 */
    if ((s_bb.next_slot % BB_RECORDS_PER_PAGE) == 0u)
    {
        if (Fs_ErasePage(addr) != 0)
        {
            s_bb.erase_fail_count++;
            s_bb.err = 1u;
            Fault_SetSticky(FAULT_BIT_FLASH_LOG);
            return -1;
        }
        s_bb.erase_count++;
    }

    rec.seq = seq;
    rec.uptime_s = uptime_s;
    rec.cell_mv[0] = snap.cell_mv[0];
    rec.cell_mv[1] = snap.cell_mv[1];
    rec.cell_mv[2] = snap.cell_mv[2];
    rec.pack_mv = snap.pack_mv;
    rec.cc_raw = snap.cc_raw;
    rec.ts1_raw = snap.ts1_raw;
    rec.flags = flags;
    rec.crc16 = 0u;

    Bb_Pack(&rec, raw);

    if (Fs_ProgramDwords(addr, raw, BB_RECORD_BYTES) != 0)
    {
        s_bb.write_fail_count++;
        s_bb.err = 1u;
        Fault_SetSticky(FAULT_BIT_FLASH_LOG);
        return -1;
    }

    s_bb.valid_head = 1u;
    s_bb.head_seq = seq;
    s_bb.uptime_base = uptime_s;
    s_bb.next_slot = (uint16_t)((s_bb.next_slot + 1u) % BB_SLOT_COUNT);
    s_bb.writes++;
    s_bb.err = 0u;
    return 0;
}

int BlackBox_WritePeriodic(uint32_t uptime_s)
{
    uint16_t flags = (uint16_t)(BB_KIND_PERIODIC & 0x07u);

    return Bb_WriteRecord(uptime_s, flags);
}

int BlackBox_WriteTransition(uint8_t transition_kind, uint8_t wake_cause,
                             uint32_t uptime_s)
{
    uint16_t flags;

    flags = (uint16_t)((transition_kind & 0x07u) |
                       ((uint16_t)(wake_cause & 0x07u) << 3));
    return Bb_WriteRecord(uptime_s, flags);
}

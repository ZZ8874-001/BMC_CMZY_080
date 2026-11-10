/**
 ******************************************************************************
 * @file    state_store.h
 * @brief   L1 跨掉电状态日志（页 50–51，48 B 记录，两页环形）。
 *
 * 仅保存“下次上电需要知道”的状态：换电/锁存/死存储标志、最后 SOC/SOH、
 * 最后稳定电芯/包压。初始化只读扫描；写入必须由主循环显式调用 Commit。
 ******************************************************************************
 */
#ifndef APP_STATE_STORE_H
#define APP_STATE_STORE_H

#include <stdint.h>

#define STATE_FLAG_EXPECTED_SWAP    (1u << 0)
#define STATE_FLAG_SWAP_ERROR       (1u << 1)
#define STATE_FLAG_CUTOFF_LATCH     (1u << 2)
#define STATE_FLAG_DEAD_STORAGE     (1u << 3)
#define STATE_FLAG_CHARGE_FULL      (1u << 4)
#define STATE_FLAG_SOC_VALID        (1u << 5)
#define STATE_FLAG_PACK_VALID       (1u << 6)
#define STATE_FLAG_SOC_ESTIMATED    (1u << 7) /* SOC 仅供显示/存档，禁止作保护输入 */
#define STATE_FLAG_PACK_PARTIAL     (1u << 8) /* 部分电压档案，PACK_VALID 必须清除 */

/* 48 B v2 记录的 reserved[16] 中存放跨复位归档状态；v1 读取时均为 0。 */
#define STATE_ARCHIVE_MARK_FULL       1u /* 实测成功标记，清四连异常计数 */
#define STATE_ARCHIVE_MARK_DEGRADED   2u /* 部分/缺失标记，计一次异常 */
#define STATE_ARCHIVE_MARK_CRITICAL   3u /* VC1/VC2 缺失，本标记直接按第四次处理 */
#define STATE_ARCHIVE_ALARM_PENDING   (1u << 0) /* 已实际报警；重启后从零重计时 */
#define STATE_ARCHIVE_ALARM_LATCHED   (1u << 1) /* 禁止输出/自动 LP，须安全确认才清 */
#define STATE_ARCHIVE_SHUTDOWN_CONFIRMED (1u << 2) /* 关管经 BQ 回读确认 */

/* 本次归档数据质量；真实电芯/包压缺失时必须存 0，估算值不得写入电压字段。 */
#define STATE_ARCHIVE_Q_CELL1_MISSING (1u << 0)
#define STATE_ARCHIVE_Q_CELL2_MISSING (1u << 1)
#define STATE_ARCHIVE_Q_CELL3_MISSING (1u << 2)
#define STATE_ARCHIVE_Q_PACK_MISSING  (1u << 3)
#define STATE_ARCHIVE_Q_SOC_ESTIMATED (1u << 4)
#define STATE_ARCHIVE_Q_SOC_INVALID   (1u << 5)
#define STATE_ARCHIVE_Q_ALL           0x3Fu

typedef struct
{
    uint32_t last_mark_epoch; /* RTC 绝对秒，非黑匣子 uptime */
    uint32_t last_mark_id;    /* 调用者提供的单调标记 ID，用于重复提交去重 */
    uint8_t anomaly_count;   /* 连续异常标记，饱和于 4 */
    uint8_t alarm_flags;     /* STATE_ARCHIVE_ALARM_* / SHUTDOWN_CONFIRMED */
    uint8_t estimate_flags;  /* STATE_ARCHIVE_Q_*：最近一次标记的质量 */
    uint8_t mark_quality;    /* STATE_ARCHIVE_MARK_*，0 表示尚无标记 */
} state_store_archive_t;

typedef struct
{
    uint16_t flags;
    uint32_t seq;
    uint32_t battery_epoch;
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    uint16_t soc01;
    uint16_t soh01;
    int16_t  temp_dc;
    uint8_t  valid;
    state_store_archive_t archive; /* v1 => 全 0；旧数据字段仍按原格式解析 */
} state_store_view_t;

void StateStore_Init(void);

uint8_t StateStore_IsValid(void);
void StateStore_GetView(state_store_view_t *out);
uint32_t StateStore_GetSeq(void);
uint16_t StateStore_GetActiveSlot(void);

/* 普通提交仅更新 view 的电池数据/flags；当前持久化的 archive 元数据始终
 * 由 StateStore 保留，避免 SocEst 的旧 view 副本抹去计数/报警/锁存。 */
int StateStore_Commit(const state_store_view_t *view);

/* LP 正常双帧的计数清除仅改 v2 元数据；不更新电芯、SOC、温度、
 * flags 或已有存档快照；只接受 FULL+estimate_flags=0。
 * RTC 30 min 标记 ID 须与 NORM 使用相同映射，返回 0 成功、1 已处理、-1 失败。 */
int StateStore_CommitArchiveMark(uint32_t mark_epoch, uint32_t mark_id,
                                 uint8_t mark_quality, uint8_t estimate_flags);

/* 同一 48 B 记录原子更新归档电池快照与标记计数；view->archive 由模块
 * 合并当前持久状态，忽略调用者的旧 archive 副本；返回语义同上。
 * 本接口禁止把估算电压写作实测电压，缺失项必须置 0 并在 flags 标出。 */
int StateStore_CommitArchiveSnapshot(const state_store_view_t *view,
                                     uint32_t mark_epoch, uint32_t mark_id,
                                     uint8_t mark_quality, uint8_t estimate_flags);

/* 归档任务不能在 ISR 调用；报警/锁存的持久提交失败必须由调用者
 * 维持 RAM 禁放行，Flash 无法保证掉电后的这次新锁存。 */
int StateStore_SetArchiveAlarm(uint8_t set_flags);
/* ClearArchiveAlarm 已删除：报警位只能由 ConfirmArchiveRecovery 或
 * SafeClearArchiveAlarm 按照各自的安全前置条件清除。 */

/* 上层确认连续 9 个新的实测好帧后、2 min 到期前调用：同一 Flash
 * 记录中清四连异常计数和 PENDING，不改变标记 ID，绝不清 LATCHED。
 * 锁存已置位时拒绝；返回 0/-1。 */
int StateStore_ConfirmArchiveRecovery(void);

/* 仅限人工安全解锁（独立核对实测好帧/保护/可信 CC/FET 回读后）：
 * 在一个 v2 记录中清所有报警、关断回读和四次计数，不改 mark ID。
 * 无持久化记录/写失败返回 -1，绝不能据此放行输出。 */
int StateStore_SafeClearArchiveAlarm(void);

/* 清除指定 flags 并提交当前记录；失败时内存中的 flags 也不改变。 */
int StateStore_ClearLatch(uint16_t flags);

#endif /* APP_STATE_STORE_H */

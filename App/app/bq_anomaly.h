/**
 ******************************************************************************
 * @file    bq_anomaly.h
 * @brief   BQ 局部电压异常的时限、NORM 档案与报警协调。
 *
 * 层：app。模块只消费 BQ/SOC 的证据并暴露事实、截止和归档请求；不写
 * FET、不代替 Protection 的 OCP/硬件保护，也不直接执行 STOP2/SHIP。
 * Task/ServicePersistence 只能从主循环调用，Flash 写入由 ServicePersistence
 * 通过 StateStore 完成。
 ******************************************************************************
 */
#ifndef APP_BQ_ANOMALY_H
#define APP_BQ_ANOMALY_H

#include <stdint.h>

#include "bsp/bq_ic.h"
#include "detect/soc_est.h"

typedef enum
{
    BQ_ANOMALY_BOOT_COLD = 0,       /* 冷启动/PIN/BOR/其它非 LP 唤醒 */
    BQ_ANOMALY_BOOT_LP_WAKE = 1     /* 已验证 LP WAKE token 的完整唤醒 */
} bq_anomaly_boot_t;

/* 上层提供已经由 BQ/Protection/策略层确认的事实。hardware_ok 不包含
 * 本模块的局部电压质量；它必须包含真实 OV/UV/SCD/OCD/XREADY/PVD 门控。 */
typedef struct
{
    uint8_t selftest_done;
    uint8_t selftest_passed;
    uint8_t hardware_ok;
    uint8_t link_ok;
    uint8_t comp_charging;
    uint8_t comp_valid;             /* 1=COMP2 已消抖、方向可信 */
    uint8_t output_on;             /* 实际 BQ FET 回读导通状态 */
    uint8_t adc_cc_on;             /* ADC_EN 与 CC_EN 已回读且持续开启 */
    uint8_t fet_off_verified;      /* 两 FET 已由 BQ 回读确认关闭 */
    uint8_t bq_idle;               /* BQ RUN/无 I2C 在飞，可做 Flash/清锁 */
    uint8_t protection_clear_ok;   /* Protection 已独立完成安全清锁核对 */
} bq_anomaly_inputs_t;

typedef struct
{
    uint32_t mark_epoch;           /* RTC 秒；不是 MCU uptime */
    uint32_t mark_id;              /* 跨 NORM/LP 的单调标记 ID */
    uint8_t mark_quality;          /* STATE_ARCHIVE_MARK_* */
    uint8_t estimate_flags;        /* STATE_ARCHIVE_Q_*；缺失电压必须写 0 */
    uint8_t bb_written;            /* 本次 RAM 归档已成功写黑匣子 */
    bq_snapshot_t bq;              /* FULL 是第九帧；降级值绝非保护输入 */
    soc_est_snapshot_t soc;        /* 与候选同源；仅显示/归档 */
} bq_anomaly_archive_t;

typedef struct
{
    uint8_t batch_quality;         /* bq_batch_class_t */
    uint8_t full_seen_this_boot;   /* 本次 MCU 启动是否见过新 FULL */
    uint8_t partial_active;
    uint8_t partial_cc_fresh;
    uint8_t discharge_window_ok;   /* 条件事实，不能直接当 FET 放行 */
    uint8_t discharge_timed_out;
    uint8_t boot_blue_hold;
    uint8_t alarm_active;          /* HOT/PB3 异常指示 */
    uint8_t hot_flash;
    uint8_t buzzer_active;         /* alarm 前 2 min 有效 */
    uint8_t latch_due;             /* 已到 2 min；上层应先请求两管关闭 */
    uint8_t output_inhibit_latched;
    uint8_t shutdown_confirmed;
    uint8_t auto_lp_blocked;
    uint8_t retry_due_off;         /* 仅输出关：允许 BQ_RequestPartialRetry */
    uint8_t recheck_due_on;        /* 输出开：只做额外复判，不重启 ADC */
    uint8_t archive_pending;
    uint8_t stop2_ship_trigger;    /* 5 min 观察量；本模块不执行动作 */
    uint8_t manual_clear_requested;
    uint8_t anomaly_mark_count;
    uint8_t full_streak;           /* 连续新 FULL；报警恢复/人工解锁用 */
    int32_t partial_current_ma;
    uint32_t partial_cc_sample_ms;
    uint32_t discharge_remaining_ms;
    uint32_t last_full_ms;
    uint32_t alarm_elapsed_ms;
    uint32_t last_mark_id;
} bq_anomaly_status_t;

#define BQ_ANOMALY_EVENT_ALARM_START   (1u << 0)
#define BQ_ANOMALY_EVENT_ALARM_CANCEL  (1u << 1)
#define BQ_ANOMALY_EVENT_LATCHED       (1u << 2)
#define BQ_ANOMALY_EVENT_MANUAL_CLEAR  (1u << 3)

/* StateStore_Init 必须已完成；任意 MCU 复位清本次真实好帧/放电资格，
 * StateStore 中的连续异常、报警和锁存保留。 */
void BqAnomaly_Init(bq_anomaly_boot_t origin, uint32_t now_ms);

/* 每轮 BQ_Poll、SOC 更新后由主循环调用。batch/CC 是同批证据；重复
 * batch_seq/published_seq 不重复计数。模块不实现 2 A OCP。 */
void BqAnomaly_Task(uint32_t now_ms, const bq_batch_quality_t *batch,
                    const bq_cc_sample_t *cc, const bq_snapshot_t *snap,
                    const soc_est_snapshot_t *soc,
                    const bq_anomaly_inputs_t *in);

/* 开始一个 NORM 30 min 标记窗口。返回 1=接受，0=重复/进行中/不安全。 */
uint8_t BqAnomaly_BeginNormMark(uint32_t mark_epoch, uint32_t mark_id,
                                uint32_t now_ms);
/* 返回当前候选；FULL 需九帧，降级候选在窗口满 60 s 后产生。 */
uint8_t BqAnomaly_GetArchiveRequest(bq_anomaly_archive_t *out);
/* 黑匣子成功后调用，StateStore 失败时后续只重试 StateStore。该影子不跨复位，
 * 两个 Flash 区非原子窗口可能留下重复黑匣子记录，不能假装事务原子。 */
void BqAnomaly_NoteBlackBoxWritten(uint32_t mark_id);
/* StateStore 返回 0 或 1（已去重）后调用；失败不得调用。 */
void BqAnomaly_ArchiveCommitted(uint32_t mark_id);
/* 正常 LP 两帧 FULL 标记由 LP/StateStore owner 提交后通知；不计入 NORM 九帧。 */
void BqAnomaly_NotifyLpFullMarkCommitted(uint32_t mark_id, uint32_t mark_epoch);

/* BQ_RequestPartialRetry() 返回后通知；started=1 表示已排队受控 ADC 重启。 */
void BqAnomaly_NotifyPartialRetry(uint32_t now_ms, uint8_t started);
/* 输出开时的 10 s 复判请求；清除后由下一次 Task 重新安排。 */
uint8_t BqAnomaly_TakeRecheckDue(void);

/* 只在主循环、BQ idle 时调用。返回 1=提交，0=无待写，-1=Flash 失败。
 * 失败维持 RAM fail-closed 状态。 */
int BqAnomaly_ServicePersistence(void);
/* 仅置 Ozone/人工请求；不会绕过 Protection 或 BQ 关管回读。 */
void BqAnomaly_RequestManualSafeClear(void);
void BqAnomaly_GetStatus(bq_anomaly_status_t *out);
uint8_t BqAnomaly_TakeEvent(void);

#endif /* APP_BQ_ANOMALY_H */

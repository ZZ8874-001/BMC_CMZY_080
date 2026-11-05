/**
 ******************************************************************************
 * @file    black_box.h
 * @brief   黑匣子 Flash 环形日志（页 52–63，1020 条 × 24 B）。
 *
 * 记录由主循环写入：30 min 一条周期记录 + LP 进入/唤醒转换记录。
 * 上电扫描取 seq 最大者为头，写入点紧随其后；跨页首槽前先擦除整页。
 * 记录格式见 black_box.c 顶部注释。
 ******************************************************************************
 */
#ifndef APP_BLACK_BOX_H
#define APP_BLACK_BOX_H

#include <stdint.h>

#include "bsp/bq_ic.h"

/* 记录种类（写入 flags 低 3 位）。 */
#define BB_KIND_PERIODIC    0u
#define BB_KIND_LP_ENTER    1u
#define BB_KIND_WAKE        2u   /* 兼容旧记录：wake 字段 0..3 */
#define BB_KIND_WAKE_MASK   3u   /* 新记录：wake 字段直接存 POWER_WAKE_* 位图 */
#define BB_KIND_EVENT       4u   /* 事件：flags[15:6] 为 event_code */

/* 事件码（只用于 BB_KIND_EVENT，10 bit 范围）。 */
#define BB_EVENT_WAKE_CHARGER   0x10u
#define BB_EVENT_WAKE_MONITOR   0x11u
#define BB_EVENT_CAL_BEGIN      0x12u
#define BB_EVENT_CAL_END        0x13u
#define BB_EVENT_LP_DIAG        0x14u
#define BB_EVENT_BQ_ADC_RESTART 0x15u
#define BB_EVENT_BQ_ADC_FAULT   0x16u
#define BB_EVENT_BQ_ADC_RECOVERED 0x17u

/* 旧记录的唤醒原因编码（flags 第 3–5 位）。 */
#define BB_WAKE_NONE        0u
#define BB_WAKE_RTC         1u
#define BB_WAKE_BUTTON      2u
#define BB_WAKE_BQ_ALERT    3u

#define BB_SHADOW_COUNT         12u  /* 覆盖一次 LP 进入的全部 stage 诊断 */

typedef struct
{
    uint32_t seq;
    uint32_t uptime_s;
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    int16_t  cc_raw;
    uint16_t ts1_raw;
    uint16_t flags;
    uint8_t  kind;
    uint8_t  wake_mask;    /* 解码为 POWER_WAKE_* 位图 */
    uint16_t event_code;
    uint8_t  valid;
} black_box_record_t;

/* RAM 影子 + LP 诊断字段。诊断字段只写 RAM，不擦写 Flash；Ozone 可直接
 * 展开 s_bb_shadow.diag_*，用于 LP 中无法连接调试器时观察唤醒路径。 */
typedef struct
{
    uint8_t count;
    black_box_record_t rec[BB_SHADOW_COUNT];  /* [0] 最新 */
    volatile uint32_t diag_lp_enter_count;
    volatile uint32_t diag_wut_count;
    volatile uint32_t diag_lp_loop_count;
    volatile uint32_t diag_button_irq_count;
    volatile uint32_t diag_wake_count;
    volatile uint8_t  diag_last_stage;
    volatile uint8_t  diag_flag_button;
    volatile uint8_t  diag_pb5_level;
    volatile uint8_t  diag_exti_imr;
    volatile uint8_t  diag_exti_pending;
    volatile uint8_t  diag_last_wake_cause;
} bb_shadow_t;

extern bb_shadow_t s_bb_shadow;

void BlackBox_Init(void);

/* 上电时头记录里的累计运行秒数（无有效头则为 0）。 */
uint32_t BlackBox_GetUptimeBase(void);

uint16_t BlackBox_GetHeadSlot(void);
uint32_t BlackBox_GetHeadSeq(void);

/* 读取头记录/指定槽；返回 0 成功，-1 参数/CRC/空白失败。 */
int BlackBox_ReadNewest(black_box_record_t *out);
int BlackBox_ReadSlot(uint16_t slot, black_box_record_t *out);

/* RAM 影子：上电时从 Flash 头记录向前加载最近 12 条；每次成功写记录
 * 前插更新。index 0 最新。LP 调试器连不上时，复位后在 NORM 读这里。 */
uint8_t BlackBox_GetShadowCount(void);
int BlackBox_ReadShadow(uint8_t index, black_box_record_t *out);

/* 返回 0 成功，-1 失败（失败同时置 sticky FAULT_FLASH_LOG）。
 * 记录只含电池快照 + 种类/唤醒原因；故障位图不入 Flash（24 B 布局已定）。 */
int BlackBox_WritePeriodic(uint32_t uptime_s);
/* LP 30 min 监测用：记录来自 HSI16 下捕获的 BQ 快照，而不是 BQ_GetSnapshot。 */
int BlackBox_WritePeriodicSnapshot(const bq_snapshot_t *snap, uint32_t uptime_s);
int BlackBox_WriteTransition(uint8_t transition_kind, uint8_t wake_cause,
                             uint32_t uptime_s);
int BlackBox_WriteWake(uint8_t power_wake_mask, uint32_t uptime_s);
int BlackBox_WriteEvent(uint16_t event_code, uint32_t uptime_s);

/* LP 按键诊断（只写一条/次 LP 按键尝试）：把计数器装进 24 B 记录，
 * 便于掉电/复位后从黑匣子影子或 Flash 头查看。字段映射见 black_box.c。 */
int BlackBox_WriteLpDiag(uint32_t uptime_s, uint32_t wut_count,
                         uint32_t button_irq_count, uint32_t loop_count,
                         uint8_t stage, uint8_t flags);

uint8_t BlackBox_HasError(void);

#endif /* APP_BLACK_BOX_H */

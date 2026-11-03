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

/* 记录种类（写入 flags 低 3 位）。 */
#define BB_KIND_PERIODIC    0u
#define BB_KIND_LP_ENTER    1u
#define BB_KIND_WAKE        2u

/* 唤醒原因（写入 flags 第 3–5 位）。 */
#define BB_WAKE_NONE        0u
#define BB_WAKE_RTC         1u
#define BB_WAKE_BUTTON      2u
#define BB_WAKE_BQ_ALERT    3u

void BlackBox_Init(void);

/* 上电时头记录里的累计运行秒数（无有效头则为 0）。 */
uint32_t BlackBox_GetUptimeBase(void);

/* 返回 0 成功，-1 失败（失败同时置 sticky FAULT_FLASH_LOG）。
 * 记录只含电池快照 + 种类/唤醒原因；故障位图不入 Flash（24 B 布局已定）。 */
int BlackBox_WritePeriodic(uint32_t uptime_s);
int BlackBox_WriteTransition(uint8_t transition_kind, uint8_t wake_cause,
                             uint32_t uptime_s);

uint8_t BlackBox_HasError(void);

#endif /* APP_BLACK_BOX_H */

/**
 ******************************************************************************
 * @file    rtc_time.h
 * @brief   RTC 绝对时间参考（LP 30 min 对齐与 IWDG 校准用）。
 *
 * 源由 LP_RTC_USE_LSE 选择：默认 LSI（相对时间调试），LSE 修好后切 1。
 * RTC 日历没有 wall-clock 含义；首次备份域复位后写 2001-01-01 00:00:00
 * 使其进入已初始化状态，之后只作为单调 epoch。所有读取都先等 RSF。
 ******************************************************************************
 */
#ifndef APP_RTC_TIME_H
#define APP_RTC_TIME_H

#include <stdint.h>

typedef struct
{
    uint32_t sec;      /* 自 2000-01-01 00:00:00 起的秒数 */
    uint16_t ms;       /* 当前秒内 0..999 ms */
} rtc_epoch_t;

/* 仅供 HAL_RTC_MspInit 调用：按 LP_RTC_USE_LSE 准备 LSE（必要时启动）
 * 或 LSI，返回 RCC_RTCCLKSOURCE_*；LSE 失败时清 LSEON 并回退 LSI。 */
uint32_t RtcTime_MspClockSource(void);

/* 严格按 LP_RTC_USE_LSE 对应的 RTC 源确认就绪、等待 RSF；日历未设置时写固定
 * 基准日期。返回 1=成功；LSE 模式回退 LSI 时返回 0（LP 被禁用）。 */
uint8_t RtcTime_Init(void);

/* 等待同步并读取当前 epoch；返回 1=成功。 */
uint8_t RtcTime_Read(rtc_epoch_t *out);

/* newer - older，单位 ms；用于 30 s 周期/boot 开销测量。 */
int32_t RtcTime_DiffMs(const rtc_epoch_t *newer, const rtc_epoch_t *older);

#endif /* APP_RTC_TIME_H */

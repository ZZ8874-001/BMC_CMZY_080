/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   IWDG1 (LSI) dead-man + LP 周期复位源。
 *
 * NORM 使用 /64、RLR 4095（~8.2 s）做死机保险；LP 轻量路径使用 RTC
 * 校准后的动态 timeout（30 s 基础）作为周期复位源。IWDG 不能停止，也不能
 * 产生早唤醒中断；每次 LP 周期结束就是一次 IWDG 系统复位。
 ******************************************************************************
 */
#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#include <stdint.h>

/* boot guard：/256、RLR 4095（~32.8 s @32 kHz），给轻量 LP boot 足够时间。 */
void Watchdog_Init(void);

/* NORM 死机保险：/64、RLR 4095（~8.2 s @32 kHz）。 */
void Watchdog_InitNorm(void);

void Watchdog_Feed(void);

/* 以 LSI 频率估计选择能覆盖 ms 的最小预分频，并写 PR/RLR。
 * actual_ms 返回四舍五入后的实际 timeout（可为 NULL）。返回 1=成功。 */
uint8_t Watchdog_SetTimeoutMs(uint32_t ms, uint32_t lsi_hz,
                              uint32_t *actual_ms);

#endif /* APP_WATCHDOG_H */

/**
 ******************************************************************************
 * @file    lp_cycle.h
 * @brief   IWDG 复位式 LP：boot 分类、轻量路径、30 s tick、30 min 记录。
 *
 * 只有两类 reset 进入轻量路径：
 *   - SOFTWARE + START token：NORM 空闲后由 PowerMode_RequestLp 触发；
 *   - IWDG + STOPPED token + RTC 间隔合理：STOP1 中 IWDG 超时。
 * 真正唤醒（PB5/充电/BQ ALERT/监测失败）只写 WAKE token + 软件复位，
 * 下一次 boot 走完整 App_Init，再由 LpCycle_ApplyWake 应用唤醒原因。
 ******************************************************************************
 */
#ifndef APP_LP_CYCLE_H
#define APP_LP_CYCLE_H

#include <stdint.h>

typedef enum
{
    LP_BOOT_NORMAL = 0,
    LP_BOOT_START,
    LP_BOOT_RESUME,
    LP_BOOT_WAKE,
    LP_BOOT_FAULT
} lp_boot_t;

/* 在 ResetCause_Init/RtcTime_Init 之后调用。 */
lp_boot_t LpCycle_ClassifyBoot(void);

/* START/RESUME：进入轻量 LP；正常情况下只会在下一次 reset 后返回（不返回）。 */
void LpCycle_Run(lp_boot_t boot);

/* NORM 自动 LP 入口：写 START token 并软件复位。1=已复位，0=RTC 未就绪。 */
uint8_t LpCycle_RequestStart(void);

/* 完整 App_Init 末尾调用：应用 PAUSED/FULL_WAKE 会话的唤醒原因。 */
void LpCycle_ApplyWake(void);

#endif /* APP_LP_CYCLE_H */

/**
 ******************************************************************************
 * @file    power_mode.h
 * @brief   NORM 电源/时钟 + IWDG 复位式 LP 的唤醒入口。
 *
 * 时钟唯一入口是 PowerMode_ClkSwitchToHsePll（冷启动与完整唤醒共用）；
 * App 其余代码不得调用生成的 SystemClock_Config（它只把系统开到 HSI16）。
 * LP 周期由 lp_cycle.c 的 IWDG 复位轻量路径负责。
 ******************************************************************************
 */
#ifndef APP_POWER_MODE_H
#define APP_POWER_MODE_H

#include <stdint.h>

typedef enum
{
    POWER_STATE_INIT = 0,
    POWER_STATE_NORM,
    POWER_STATE_DEGRADED   /* 80 MHz 切换失败：仅故障灯，不跑业务 */
} power_state_t;

#define POWER_WAKE_RTC          (1u << 0)
#define POWER_WAKE_BUTTON       (1u << 1)
#define POWER_WAKE_BQ_ALERT     (1u << 2)
#define POWER_WAKE_CHARGER      (1u << 3)   /* LP 内 LOAD_PRESENT+COMP2 判定为充电 */
#define POWER_WAKE_MONITOR      (1u << 4)   /* 30 min LP 维护唤醒（M2b-2 挂点） */

void PowerMode_Init(void);
/* LP 主时基门控：RTC 源/句柄就绪；LP_RTC_USE_LSE=0 要求 LSI，=1 要求 LSE。 */
uint8_t PowerMode_LpTimebaseReady(void);
void PowerMode_Loop(void);
void PowerMode_RequestLp(void);          /* 主循环自动空闲/调试器请求入口 */

/* IWDG 复位式 LP：完整 boot 之后注入 LP 唤醒原因/计数，或强制禁用自动 LP。 */
void PowerMode_NoteLpFullWake(uint8_t cause);
void PowerMode_ForceDisableAutoLp(void);

/* 轻量 LP boot 用：按编译期 POWER_LP_KEEP_SWD 设置 DBGMCU_CR.DBG_STOP。 */
void PowerMode_LpDbgStopApply(void);

/* 冷启动/唤醒共用：PA1=0 → OSC1 稳定 → HSE BYPASS → PLL 80 MHz。返回 1 成功。 */
uint8_t PowerMode_ClkSwitchToHsePll(void);

power_state_t PowerMode_GetState(void);
uint8_t PowerMode_IsDegraded(void);

#endif /* APP_POWER_MODE_H */

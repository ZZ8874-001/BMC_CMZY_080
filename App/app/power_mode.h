/**
 ******************************************************************************
 * @file    power_mode.h
 * @brief   NORM/LP 电源模式：HSE/PLL 时钟切换、STOP1 进入/唤醒、唤醒原因。
 *
 * 时钟唯一入口是 PowerMode_ClkSwitchToHsePll（冷启动与 LP 唤醒共用）；
 * App 其余代码不得调用生成的 SystemClock_Config（它只把系统开到 HSI16）。
 ******************************************************************************
 */
#ifndef APP_POWER_MODE_H
#define APP_POWER_MODE_H

#include <stdint.h>

typedef enum
{
    POWER_STATE_INIT = 0,
    POWER_STATE_NORM,
    POWER_STATE_ENTER_LP,
    POWER_STATE_LP,
    POWER_STATE_WAKING,
    POWER_STATE_DEGRADED   /* 80 MHz 切换失败：仅故障灯，不跑业务 */
} power_state_t;

#define POWER_WAKE_RTC          (1u << 0)
#define POWER_WAKE_BUTTON       (1u << 1)
#define POWER_WAKE_BQ_ALERT     (1u << 2)

void PowerMode_Init(void);
void PowerMode_Loop(void);
void PowerMode_EnterLp(void);

/* 冷启动/唤醒共用：PA1=0 → OSC1 稳定 → HSE BYPASS → PLL 80 MHz。返回 1 成功。 */
uint8_t PowerMode_ClkSwitchToHsePll(void);

power_state_t PowerMode_GetState(void);
uint8_t PowerMode_IsDegraded(void);
uint8_t PowerMode_GetWakeCause(void);

#endif /* APP_POWER_MODE_H */

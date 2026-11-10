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

/* RTC 30 min 标记交接：失败的 LP 监测（含纯电压异常）只交出原始 RTC
 * 标记，完整 NORM 成功归档后须以同一 epoch 确认，不能跳过或重复计数。
 * GetPendingMark 返回 1=有待处理标记，0=无；mark_epoch 即 mark_id。
 * IsPendingMarkLpArchived 返回 1=LP 黑匣子已写入，仅需补 StateStore。
 * LP 写黑匣子成功后必须 NotePendingMarkArchived，再提交 StateStore。
 * AckPendingMark 仅允许在 StateStore 已成功提交同 ID 归档标记之后调用，
 * 返回 1=确认并推进到下一标记，0=无匹配/RTC 不可用/会话写回校验失败；
 * 失败时仍需禁止自动 LP。 */
uint8_t LpCycle_GetPendingMark(uint32_t *mark_epoch);
uint8_t LpCycle_IsPendingMarkLpArchived(void);
/* 黑匣子写成功后立即调用；该位跨 IWDG/掉电保护，避免随后重启重复写。 */
uint8_t LpCycle_NotePendingMarkArchived(uint32_t mark_epoch);
uint8_t LpCycle_AckPendingMark(uint32_t mark_epoch);

/* 有效 LP WAKE token 中保存的入 LP 前运行累计秒（无 RTC/STOP 时间）。
 * 仅完整 WAKE 启动使用，App_Init 在 ApplyWake 前与黑匣子 uptime base 取大；
 * 旧 LP03 token 无 uptime 数据返回 0。 */
uint32_t LpCycle_GetUptimeBase(void);

/* 仅有效 LP WAKE：返回 LP_WAKE_* 原始位（0=非完整 LP 唤醒）。 */
uint8_t LpCycle_GetWakeCause(void);

#endif /* APP_LP_CYCLE_H */

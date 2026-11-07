/**
 ******************************************************************************
 * @file    lp_session.h
 * @brief   IWDG 复位式 LP 的跨复位会话（RTC backup registers）。
 *
 * BKP0R 保留给 ResetCause；BKP1..BKP10 存本会话；BKP11..BKP14 空闲；
 * BKP20..BKP23 保留给 HardFault 现场。所有数据带 magic/version/CRC，
 * 写序固定为 数据 → CRC → magic。
 ******************************************************************************
 */
#ifndef APP_LP_SESSION_H
#define APP_LP_SESSION_H

#include <stdint.h>

#define LP_SESSION_MAGIC        0x4C503033u  /* "LP03"（布局 v3） */
#define LP_SESSION_BKP_MAGIC    1u
#define LP_SESSION_BKP_CRC      2u
#define LP_SESSION_BKP_FIRST    3u

typedef enum
{
    LP_SESSION_IDLE = 0,
    LP_SESSION_START,
    LP_SESSION_PREP,
    LP_SESSION_STOPPED,
    LP_SESSION_PAUSED,
    LP_SESSION_FAULT
} lp_session_state_t;

/* lp_session_t.wake_cause */
#define LP_WAKE_NONE            0x00u
#define LP_WAKE_BUTTON          0x01u
#define LP_WAKE_CHARGER         0x02u
#define LP_WAKE_ALERT           0x04u
#define LP_WAKE_MONITOR         0x08u
#define LP_WAKE_FAULT           0x10u
#define LP_WAKE_SMBA            0x20u

/* lp_session_t.flags */
#define LP_FLAG_FULL_WAKE       0x04u
#define LP_FLAG_FAULT           0x20u

typedef struct
{
    uint8_t  state;              /* lp_session_state_t */
    uint8_t  anomaly_count;      /* 连续异常样本数 */
    uint8_t  flags;              /* LP_FLAG_* */
    uint32_t next_mark_epoch;    /* 下一个 30 min 点的 RTC 秒 */
    uint32_t lsi_hz_q16;         /* LSI 频率估计 Q16；0=未校准 */
    uint8_t  last_pr;            /* IWDG_PR 0..6 */
    uint16_t last_rlr;           /* IWDG_RLR 1..4095 */
    uint32_t last_arm_epoch;     /* 进 STOP1 前的 RTC 秒 */
    uint16_t last_arm_ms;        /* 进 STOP1 前秒内 ms */
    uint16_t boot_overhead_ms;   /* boot 到 ready 的估算 */
    uint8_t  wake_cause;         /* LP_WAKE_* */
    uint8_t  pb5_mode;           /* 0=EXTI5, 1=SMBA test */
    uint8_t  smba_result;        /* 0=未测,1=ALERT,2=无ALERT,3=仅EXTI */
    uint16_t seq;                /* 会话保存序号（最小诊断） */
} lp_session_t;

/* 读取并校验 BKP 会话；返回 1=有效。无效时把 *s 清零。 */
uint8_t LpSession_Load(lp_session_t *s);

/* 写回 BKP 会话（数据 → CRC → magic）。 */
void LpSession_Save(const lp_session_t *s);

/* 使 magic 失效；下一次 boot 走完整 App_Init。 */
void LpSession_Invalidate(void);

#endif /* APP_LP_SESSION_H */

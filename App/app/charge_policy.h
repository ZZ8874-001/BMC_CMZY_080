/**
 ******************************************************************************
 * @file    charge_policy.h
 * @brief   充放电策略层：CHARGE/DISCHARGE/CAL + FET 请求。
 *
 * 异常部分电压放电只允许 DSG，且必须通过 Protection 独立 CC 门控；
 * CHARGE/CAL 继续使用现有完整 BQ fresh、自检和保护条件。
 ******************************************************************************
 */
#ifndef APP_CHARGE_POLICY_H
#define APP_CHARGE_POLICY_H

#include <stdint.h>

typedef enum
{
    CHARGE_POLICY_IDLE = 0,
    CHARGE_POLICY_CHARGE,
    CHARGE_POLICY_DISCHARGE,
    CHARGE_POLICY_CAL,
    CHARGE_POLICY_FAULT
} charge_policy_state_t;

typedef enum
{
    CHARGE_POLICY_CAL_END_NONE = 0,
    CHARGE_POLICY_CAL_END_USER,
    CHARGE_POLICY_CAL_END_FAULT,
    CHARGE_POLICY_CAL_END_NOTREADY,
    CHARGE_POLICY_CAL_END_LP
} charge_policy_cal_end_t;

void ChargePolicy_Init(void);          /* 清零并 BQ_RequestFetMask(0) */
void ChargePolicy_OnPairHold3(uint32_t now_ms);
void ChargePolicy_OnLpWake(uint32_t now_ms);
void ChargePolicy_Task(uint32_t now_ms, uint8_t comp_charging);
/* 四连异常2分钟到期/持久锁存：策略唯一负责安全方向关 FET。 */
void ChargePolicy_SetArchiveLatch(uint8_t latched);

charge_policy_state_t ChargePolicy_GetState(void);
uint8_t ChargePolicy_GetRequestedMask(void);
uint8_t ChargePolicy_IsLpBlocked(void);
uint8_t ChargePolicy_IsCalibration(void);
charge_policy_cal_end_t ChargePolicy_GetCalEndReason(void);

#endif /* APP_CHARGE_POLICY_H */

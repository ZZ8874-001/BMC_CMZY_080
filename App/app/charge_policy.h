/**
 ******************************************************************************
 * @file    charge_policy.h
 * @brief   充放电策略层：CHARGE/DISCHARGE/CAL + FET 请求。
 *
 * 只从主循环调用；ISR 不调用。CAL 模式由调试器写 s_policy.cal_mode /
 * s_policy.cal_fet_mask 控制，用于长时间采集标定数据。
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

charge_policy_state_t ChargePolicy_GetState(void);
uint8_t ChargePolicy_GetRequestedMask(void);
uint8_t ChargePolicy_IsLpBlocked(void);
uint8_t ChargePolicy_IsCalibration(void);
charge_policy_cal_end_t ChargePolicy_GetCalEndReason(void);

#endif /* APP_CHARGE_POLICY_H */

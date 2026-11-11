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

/* 充电器存在/近满判据的输入。charge_current_ma 已按 BQ_CC_DISCHARGE_SIGN
 * 修正为“充电为正”；soc01 为 0..10000 的显示 SOC，只作策略启发，不是保护。
 * 同一电流在 DISCHARGE 进入后的前 10 s 还用 100/90 mA 判外部充电切换。 */
typedef struct
{
    int32_t  charge_current_ma;
    uint16_t soc01;
    uint8_t  current_valid;
    uint8_t  soc_valid;
} charge_policy_input_t;

void ChargePolicy_Init(void);          /* 清零并 BQ_RequestFetMask(0) */
void ChargePolicy_OnPairHold3(uint32_t now_ms);
void ChargePolicy_OnLpWake(uint32_t now_ms);
void ChargePolicy_Task(uint32_t now_ms, uint8_t comp_charging,
                       const charge_policy_input_t *in);

charge_policy_state_t ChargePolicy_GetState(void);
uint8_t ChargePolicy_GetRequestedMask(void);
uint8_t ChargePolicy_IsLpBlocked(void);
uint8_t ChargePolicy_IsCalibration(void);
uint8_t ChargePolicy_IsChargerPresent(void);
uint8_t ChargePolicy_GetBodyDiodeMode(void);
uint8_t ChargePolicy_GetChargeCurrentHold(void);
uint8_t ChargePolicy_GetDischargeChargeDetect(void);
uint32_t ChargePolicy_GetDischargeChargeDetectCount(void);
charge_policy_cal_end_t ChargePolicy_GetCalEndReason(void);

#endif /* APP_CHARGE_POLICY_H */

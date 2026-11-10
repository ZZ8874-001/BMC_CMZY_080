/**
 ******************************************************************************
 * @file    charge_policy.c
 * @brief   CHARGE/DISCHARGE/CAL 策略：FET 请求、充电器优先、自动 LP 阻止。
 *
 * 状态优先级：FAULT > CAL > CHARGE(充电器) > DISCHARGE(手势) > IDLE。
 * 所有 BQ 写操作都走影子请求；只有 CAL/FAULT 才在 BQ RUN 且空闲时尝试立即写。
 ******************************************************************************
 */
#include "app/charge_policy.h"

#include "app/power_mode.h"
#include "app/protection.h"
#include "app/self_test.h"
#include "app/state_machine.h"
#include "bsp/bq_ic.h"
#include "bsp/buzzer.h"

#define CHARGE_POLICY_CHARGER_LOW_MS 1000u

static struct
{
    charge_policy_state_t state;
    uint8_t requested_mask;
    uint8_t last_allow_mask;
    uint8_t user_discharge_enabled;
    uint8_t charger_present;
    uint8_t charger_low_active;
    uint32_t charger_low_since_ms;
    uint8_t lp_block;
    uint8_t cal_pending;
    uint8_t cal_active;
    volatile uint8_t cal_mode;       /* Ozone: 写 1 进入，写 0 退出 */
    volatile uint8_t cal_fet_mask;   /* Ozone: 0x00..0x03，先写 mask 再写 mode */
    charge_policy_cal_end_t cal_end_reason;

    uint32_t state_enter_ms;
    uint32_t transition_count;
    uint32_t charge_enter_count;
    uint32_t discharge_enter_count;
    uint32_t fault_off_count;
    uint32_t pair_toggle_count;
    uint32_t pair_reject_count;
    uint32_t cal_enter_count;
    uint32_t cal_abort_count;
    uint32_t apply_now_ok_count;
    uint32_t apply_now_fail_count;
} s_policy;

static uint8_t ChargePolicy_NormalReady(uint32_t now_ms)
{
    return (uint8_t)((SelfTest_IsDone() != 0u) &&
                     (SelfTest_IsPassed() != 0u) &&
                     (StateMachine_GetState() == SM_STATE_NORM) &&
                     (BQ_GetState() == BQ_ST_RUN) &&
                     (BQ_IsFresh(now_ms) != 0u) &&
                     (PowerMode_GetState() == POWER_STATE_NORM));
}

static uint8_t ChargePolicy_CalReady(uint32_t now_ms)
{
    return (uint8_t)((StateMachine_GetState() != SM_STATE_FAULT) &&
                     (BQ_GetState() == BQ_ST_RUN) &&
                     (BQ_IsFresh(now_ms) != 0u) &&
                     (PowerMode_GetState() == POWER_STATE_NORM));
}

static void ChargePolicy_RequestMask(uint8_t mask)
{
    /* 保护监督层是 FET 允许/禁止的唯一安全门；策略层仍是请求 owner。 */
    mask = (uint8_t)(mask & BQ_FET_MASK_ALL &
                     Protection_GetAllowedFetMask());
    /* 也检查 BQ 影子：LP 进入/监测可能绕过策略直接改过 request_fet_mask。 */
    if ((mask != s_policy.requested_mask) ||
        (BQ_GetFetMaskRequested() != mask))
    {
        BQ_RequestFetMask(mask);
        s_policy.requested_mask = mask;
    }
}

static void ChargePolicy_ApplyNow(uint8_t mask)
{
    mask = (uint8_t)(mask & BQ_FET_MASK_ALL &
                     Protection_GetAllowedFetMask());
    if ((BQ_GetState() == BQ_ST_RUN) && (BQ_IsIdle() != 0u))
    {
        if (BQ_ApplyFetMaskNow(mask) == 0)
        {
            s_policy.apply_now_ok_count++;
        }
        else
        {
            s_policy.apply_now_fail_count++;
        }
    }
}

static void ChargePolicy_SetState(charge_policy_state_t new_state,
                                  uint32_t now_ms, uint8_t announce)
{
    charge_policy_state_t old_state = s_policy.state;

    if (old_state == new_state)
    {
        return;
    }

    s_policy.state = new_state;
    s_policy.state_enter_ms = now_ms;
    s_policy.transition_count++;

    if (announce == 0u)
    {
        return;
    }

    if ((new_state == CHARGE_POLICY_CHARGE) ||
        (new_state == CHARGE_POLICY_DISCHARGE) ||
        (new_state == CHARGE_POLICY_CAL))
    {
        Buzzer_StartPattern(BUZZER_PATTERN_WAKE);
    }
    else if ((new_state == CHARGE_POLICY_IDLE) &&
             ((old_state == CHARGE_POLICY_DISCHARGE) ||
              (old_state == CHARGE_POLICY_CAL)))
    {
        Buzzer_StartPattern(BUZZER_PATTERN_LP_ENTER);
    }
}

void ChargePolicy_Init(void)
{
    s_policy.state = CHARGE_POLICY_IDLE;
    s_policy.requested_mask = BQ_FET_MASK_NONE;
    /* 保护层尚未 Init（读到 0）时，首次 Task 会立即收紧到 0，方向安全。 */
    s_policy.last_allow_mask = BQ_FET_MASK_ALL;
    s_policy.user_discharge_enabled = 0u;
    s_policy.charger_present = 0u;
    s_policy.charger_low_active = 0u;
    s_policy.charger_low_since_ms = 0u;
    s_policy.lp_block = 0u;
    s_policy.cal_pending = 0u;
    s_policy.cal_active = 0u;
    s_policy.cal_mode = 0u;
    s_policy.cal_fet_mask = 0u;
    s_policy.cal_end_reason = CHARGE_POLICY_CAL_END_NONE;

    s_policy.state_enter_ms = 0u;
    s_policy.transition_count = 0u;
    s_policy.charge_enter_count = 0u;
    s_policy.discharge_enter_count = 0u;
    s_policy.fault_off_count = 0u;
    s_policy.pair_toggle_count = 0u;
    s_policy.pair_reject_count = 0u;
    s_policy.cal_enter_count = 0u;
    s_policy.cal_abort_count = 0u;
    s_policy.apply_now_ok_count = 0u;
    s_policy.apply_now_fail_count = 0u;

    /* 不修改 BQ_DEFAULT_FET_MASK 宏；策略接管后 IDLE 明确请求 0。 */
    BQ_RequestFetMask(BQ_FET_MASK_NONE);
}

void ChargePolicy_OnPairHold3(uint32_t now_ms)
{
    if ((s_policy.state == CHARGE_POLICY_FAULT) ||
        (s_policy.state == CHARGE_POLICY_CAL) ||
        (s_policy.state == CHARGE_POLICY_CHARGE) ||
        (s_policy.cal_mode != 0u) ||
        (s_policy.cal_active != 0u) ||
        (s_policy.charger_present != 0u) ||
        (ChargePolicy_NormalReady(now_ms) == 0u))
    {
        s_policy.pair_reject_count++;
        return;
    }

    s_policy.user_discharge_enabled ^= 1u;
    s_policy.pair_toggle_count++;
}

void ChargePolicy_OnLpWake(uint32_t now_ms)
{
    s_policy.user_discharge_enabled = 0u;
    s_policy.charger_present = 0u;
    s_policy.charger_low_active = 0u;
    s_policy.charger_low_since_ms = 0u;
    s_policy.cal_pending = 0u;

    if ((s_policy.cal_mode != 0u) || (s_policy.cal_active != 0u))
    {
        s_policy.cal_mode = 0u;
        s_policy.cal_active = 0u;
        s_policy.cal_end_reason = CHARGE_POLICY_CAL_END_LP;
        s_policy.cal_abort_count++;
    }

    ChargePolicy_RequestMask(BQ_FET_MASK_NONE);
    ChargePolicy_ApplyNow(BQ_FET_MASK_NONE);
    ChargePolicy_SetState(CHARGE_POLICY_IDLE, now_ms, 0u);
    s_policy.lp_block = 0u;
}

void ChargePolicy_Task(uint32_t now_ms, uint8_t comp_charging)
{
    charge_policy_state_t old_state = s_policy.state;
    uint8_t charger_mask_changed = 0u;
    uint8_t allow_mask = Protection_GetAllowedFetMask();

    /* 允许掩码变化（M2 起可能是部分掩码，如只禁 CHG）：立即按新掩码收紧
     * 当前请求；恢复方向由本轮正常 RequestMask 重新放宽，再等 BQ run script。
     * 安全方向不能等 500 ms。 */
    if (allow_mask != s_policy.last_allow_mask)
    {
        uint8_t safe_mask = (uint8_t)(s_policy.requested_mask & allow_mask);
        ChargePolicy_RequestMask(safe_mask);
        ChargePolicy_ApplyNow(safe_mask);
        s_policy.last_allow_mask = allow_mask;
    }

    /* 部分故障/数据未就绪均先关输出；允许掩码可能是部分 DSG。 */
    if ((Protection_GetAllowedFetMask() == BQ_FET_MASK_NONE) &&
        ((s_policy.requested_mask != 0u) ||
         (BQ_GetFetMaskRequested() != 0u) ||
         (BQ_GetFetMaskApplied() != 0u)))
    {
        ChargePolicy_RequestMask(BQ_FET_MASK_NONE);
        ChargePolicy_ApplyNow(BQ_FET_MASK_NONE);
    }

    /* 充电器：上升沿立即算存在；下降沿连续 1 s 才撤销，防 CV 末端抖动。 */
    if (comp_charging != 0u)
    {
        s_policy.charger_present = 1u;
        s_policy.charger_low_active = 0u;
        s_policy.charger_low_since_ms = 0u;
    }
    else if (s_policy.charger_present != 0u)
    {
        if (s_policy.charger_low_active == 0u)
        {
            s_policy.charger_low_active = 1u;
            s_policy.charger_low_since_ms = now_ms;
        }
        else if ((now_ms - s_policy.charger_low_since_ms) >=
                 CHARGE_POLICY_CHARGER_LOW_MS)
        {
            s_policy.charger_present = 0u;
            s_policy.charger_low_active = 0u;
            s_policy.charger_low_since_ms = 0u;
        }
    }

    /* 关键故障最高优先级。 */
    if (StateMachine_GetState() == SM_STATE_FAULT)
    {
        if (old_state != CHARGE_POLICY_FAULT)
        {
            s_policy.fault_off_count++;
        }
        if ((s_policy.cal_mode != 0u) || (s_policy.cal_active != 0u))
        {
            s_policy.cal_mode = 0u;
            s_policy.cal_active = 0u;
            s_policy.cal_end_reason = CHARGE_POLICY_CAL_END_FAULT;
            s_policy.cal_abort_count++;
        }
        s_policy.user_discharge_enabled = 0u;
        s_policy.cal_pending = 0u;
        {
            uint8_t request_changed =
                (s_policy.requested_mask != BQ_FET_MASK_NONE) ? 1u : 0u;

            ChargePolicy_RequestMask(BQ_FET_MASK_NONE);
            if ((old_state != CHARGE_POLICY_FAULT) ||
                (request_changed != 0u))
            {
                ChargePolicy_ApplyNow(BQ_FET_MASK_NONE);
            }
        }
        ChargePolicy_SetState(CHARGE_POLICY_FAULT, now_ms, 0u);
        s_policy.lp_block = 1u;
        return;
    }

    /* 调试器 CAL 模式：故障之外最高优先级。 */
    if (s_policy.cal_mode != 0u)
    {
        if (ChargePolicy_CalReady(now_ms) != 0u)
        {
            uint8_t cal_mask = (uint8_t)(s_policy.cal_fet_mask & BQ_FET_MASK_ALL);

            s_policy.user_discharge_enabled = 0u;
            if (s_policy.cal_active == 0u)
            {
                s_policy.cal_active = 1u;
                s_policy.cal_enter_count++;
                s_policy.cal_end_reason = CHARGE_POLICY_CAL_END_NONE;
                charger_mask_changed = 1u;
            }
            if (cal_mask != s_policy.requested_mask)
            {
                charger_mask_changed = 1u;
            }

            s_policy.cal_pending = 0u;
            ChargePolicy_RequestMask(cal_mask);
            if (charger_mask_changed != 0u)
            {
                ChargePolicy_ApplyNow(cal_mask);
            }
            ChargePolicy_SetState(CHARGE_POLICY_CAL, now_ms, 1u);
            s_policy.lp_block = 1u;
            return;
        }

        /* 已经真正进入过 CAL，之后 BQ 不新鲜/非 RUN：自动结束，不自动恢复。 */
        if (s_policy.cal_active != 0u)
        {
            s_policy.cal_active = 0u;
            s_policy.cal_mode = 0u;
            s_policy.user_discharge_enabled = 0u;
            s_policy.cal_end_reason = CHARGE_POLICY_CAL_END_NOTREADY;
            s_policy.cal_abort_count++;
        }
        else
        {
            /* 调试器先写 cal_mode、BQ 还没就绪：保持 pending，阻止自动 LP。 */
            s_policy.cal_pending = 1u;
            s_policy.user_discharge_enabled = 0u;
            ChargePolicy_RequestMask(BQ_FET_MASK_NONE);
            ChargePolicy_SetState(CHARGE_POLICY_IDLE, now_ms, 0u);
            s_policy.lp_block = 1u;
            return;
        }
    }

    /* cal_mode 被清 0：如果会话还在，记录 USER 结束。 */
    if ((s_policy.cal_mode == 0u) && (s_policy.cal_active != 0u))
    {
        s_policy.cal_active = 0u;
        s_policy.cal_end_reason = CHARGE_POLICY_CAL_END_USER;
    }
    s_policy.cal_pending = 0u;

    /* 正常充放状态。 */
    if (ChargePolicy_NormalReady(now_ms) != 0u)
    {
        if (s_policy.charger_present != 0u)
        {
            s_policy.user_discharge_enabled = 0u;
            ChargePolicy_RequestMask(BQ_FET_MASK_ALL);
            if (s_policy.state != CHARGE_POLICY_CHARGE)
            {
                s_policy.charge_enter_count++;
            }
            ChargePolicy_SetState(CHARGE_POLICY_CHARGE, now_ms, 1u);
            s_policy.lp_block = 1u;
        }
        else if (s_policy.user_discharge_enabled != 0u)
        {
            ChargePolicy_RequestMask(BQ_FET_MASK_ALL);
            if (s_policy.state != CHARGE_POLICY_DISCHARGE)
            {
                s_policy.discharge_enter_count++;
            }
            ChargePolicy_SetState(CHARGE_POLICY_DISCHARGE, now_ms, 1u);
            s_policy.lp_block = 1u;
        }
        else
        {
            ChargePolicy_RequestMask(BQ_FET_MASK_NONE);
            ChargePolicy_SetState(CHARGE_POLICY_IDLE, now_ms, 1u);
            s_policy.lp_block = 0u;
        }
    }
    else
    {
        s_policy.user_discharge_enabled = 0u;
        ChargePolicy_RequestMask(BQ_FET_MASK_NONE);
        ChargePolicy_SetState(CHARGE_POLICY_IDLE, now_ms, 0u);
        s_policy.lp_block = 0u;
    }

    /* CAL 退出到 IDLE 时尽力立即关管；失败仍由影子在下一轮同步。 */
    if ((old_state == CHARGE_POLICY_CAL) &&
        (s_policy.state == CHARGE_POLICY_IDLE) &&
        (s_policy.requested_mask == BQ_FET_MASK_NONE))
    {
        ChargePolicy_ApplyNow(BQ_FET_MASK_NONE);
    }
}

charge_policy_state_t ChargePolicy_GetState(void)
{
    return s_policy.state;
}

uint8_t ChargePolicy_GetRequestedMask(void)
{
    return s_policy.requested_mask;
}

uint8_t ChargePolicy_IsLpBlocked(void)
{
    return s_policy.lp_block;
}

uint8_t ChargePolicy_IsCalibration(void)
{
    return (uint8_t)((s_policy.state == CHARGE_POLICY_CAL) ||
                     (s_policy.cal_mode != 0u));
}

charge_policy_cal_end_t ChargePolicy_GetCalEndReason(void)
{
    return s_policy.cal_end_reason;
}

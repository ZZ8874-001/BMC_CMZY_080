/**
 ******************************************************************************
 * @file    protection.c
 * @brief   BQ 保护监督层（M2：硬件锁存 + 软件 OCP + NTC 温度）。
 *
 * M1：评估 BQ 硬件 OV/UV/SCD/OCD 锁存、boot/LP 后 SCD/OCD 的安全清除、
 *     FET 允许掩码和一次性事件。
 * M2a：软件充电 5 A / 放电 30 A OCP，2 个新发布帧确认后锁存；只允许
 *      `Protection_RequestLatchClear()` 在安全条件下清除，不自动重试。
 * M2b：NTC 充放电温度门控，`PROT_TEMP_ENABLE=1` 且 TS1 标定后生效。
 *
 * FET 请求仍由 charge_policy 唯一持有；本模块只给允许掩码。
 ******************************************************************************
 */
#include "app/protection.h"

#include "board.h"
#include "app/faults.h"

#define PROT_CLEAR_CURRENT_MA       500
#define PROT_CLEAR_HOLD_MS          2000u
#define PROT_CLEAR_ALLOWED_MASK     (BQ_SYS_STAT_SCD | BQ_SYS_STAT_OCD)
#define PROT_SIGN_MISMATCH_FRAMES   3u

typedef struct
{
    protection_state_t state;
    uint16_t reasons;
    uint8_t boot_latched_mask;
    uint8_t allow_mask;
    uint8_t clear_requested;
    uint8_t clear_failed;
    uint8_t startup_capture_open;
    uint8_t clear_attempts;
    uint8_t last_clear_result;   /* 0=none, 1=success, 2=failed */
    uint8_t clear_block_reason;
    uint32_t safe_since_ms;
    uint32_t last_clear_ms;
    uint8_t event_flags;

    /* M2a 软件 OCP */
    volatile uint16_t ocp_chg_limit_ma;   /* 默认宏值；Ozone 可临时调低做台架 */
    volatile uint16_t ocp_dsg_limit_ma;
    uint8_t ocp_chg_latched;
    uint8_t ocp_dsg_latched;
    uint8_t ocp_chg_streak;
    uint8_t ocp_dsg_streak;
    uint32_t ocp_trip_count;
    uint32_t ocp_last_snap_ms;
    int32_t ocp_last_current_ma;
    uint32_t implausible_current_count;
    uint8_t sign_mismatch_streak;
    uint32_t sign_mismatch_count;

    /* M2b 温度 */
    uint8_t comp_charging_last;
    uint8_t temp_valid_last;
    int16_t temp_dc_last;
    uint8_t temp_chg_inhibit;
    uint8_t temp_dsg_inhibit;
    uint8_t temp_invalid;
    uint32_t temp_chg_trip_since_ms;
    uint32_t temp_dsg_trip_since_ms;
    uint32_t temp_invalid_since_ms;
    uint32_t temp_valid_since_ms;
} protection_ctx_t;

static protection_ctx_t s_protection;

static uint8_t Protection_TempAllowMask(void);

static uint8_t Protection_BqReady(void)
{
    return ((BQ_GetState() == BQ_ST_RUN) &&
            (BQ_IsRecovering() == 0u)) ? 1u : 0u;
}

static uint8_t Protection_DataReady(uint32_t now_ms, const bq_snapshot_t *snap)
{
    if (snap == NULL)
    {
        return 0u;
    }
    return ((snap->valid != 0u) && (snap->fresh != 0u) &&
            (snap->protection_ok != 0u) &&
            (BQ_IsFresh(now_ms) != 0u)) ? 1u : 0u;
}

static int32_t Protection_AbsCurrentMa(const bq_snapshot_t *snap)
{
    int32_t current_ma;

    if (snap == NULL)
    {
        return 0;
    }
    current_ma = snap->current_ma;
    return (current_ma < 0) ? -current_ma : current_ma;
}

/* board.h 的 BQ_CC_DISCHARGE_SIGN 是唯一符号来源：+1=放电为正，-1=充电为正。 */
static int32_t Protection_ChargeCurrentMa(int32_t current_ma)
{
    return (-current_ma) * BQ_CC_DISCHARGE_SIGN;
}

static int32_t Protection_DischargeCurrentMa(int32_t current_ma)
{
    return current_ma * BQ_CC_DISCHARGE_SIGN;
}

static uint8_t Protection_TempAllowMask(void)
{
    uint8_t allow = BQ_FET_MASK_ALL;

#if (PROT_TEMP_ENABLE != 0u)
    if (s_protection.temp_invalid != 0u)
    {
        return BQ_FET_MASK_NONE;
    }
    if (s_protection.temp_chg_inhibit != 0u)
    {
        allow &= (uint8_t)~BQ_FET_MASK_CHG;
    }
    if (s_protection.temp_dsg_inhibit != 0u)
    {
        allow &= (uint8_t)~BQ_FET_MASK_DSG;
    }
#endif
    return allow;
}

/* 每个“新发布帧”调用一次：CC 是 250 ms 积分值，2 帧≈0.5–1 s。 */
static void Protection_EvaluateOcp(const bq_snapshot_t *snap)
{
    int32_t charge_ma;
    int32_t discharge_ma;
    int32_t abs_ma;

    if (snap == NULL)
    {
        return;
    }

    abs_ma = Protection_AbsCurrentMa(snap);
    if (abs_ma > PROT_OCP_IMPLAUSIBLE_MA)
    {
        /* 正常持续电流远高于 SCD 44.7 A 已无意义；真短路由 BQ 硬件处理。
         * 这里只记录，防止单个 CC 异常值误锁存。 */
        s_protection.implausible_current_count++;
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_dsg_streak = 0u;
        return;
    }

    charge_ma = Protection_ChargeCurrentMa(snap->current_ma);
    discharge_ma = Protection_DischargeCurrentMa(snap->current_ma);

    /* COMP2 交叉检查只做诊断，不参与跳闸；符号错时 SOC 方向会反，也说明
     * SRP/SRN/CC 极性与 TI 约定不一致，必须在上板验证阶段确认。 */
    if (abs_ma >= PROT_OCP_SIGN_HINT_MA)
    {
        uint8_t mismatch = 0u;

        if ((s_protection.comp_charging_last != 0u) && (charge_ma < 0))
        {
            mismatch = 1u;
        }
        else if ((s_protection.comp_charging_last == 0u) &&
                 (discharge_ma < 0))
        {
            mismatch = 1u;
        }

        if (mismatch != 0u)
        {
            if (s_protection.sign_mismatch_streak < 0xFFu)
            {
                s_protection.sign_mismatch_streak++;
            }
            s_protection.sign_mismatch_count++;
        }
        else
        {
            s_protection.sign_mismatch_streak = 0u;
        }
    }
    else
    {
        s_protection.sign_mismatch_streak = 0u;
    }

    if ((s_protection.ocp_chg_latched != 0u) ||
        (s_protection.ocp_dsg_latched != 0u))
    {
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_dsg_streak = 0u;
        return;
    }

    /* 方向用 COMP2（独立于 CC 符号），幅值用 CC 积分值。这样即使 SRP/SRN
     * 或 CC 极性与 TI 约定相反，也不会把 5 A 放电误判成充电过流；CC 符号
     * 只用于 SOC 和 sign_mismatch 诊断。 */
    if ((s_protection.comp_charging_last != 0u) &&
        (abs_ma >= (int32_t)s_protection.ocp_chg_limit_ma))
    {
        if (s_protection.ocp_chg_streak < 0xFFu)
        {
            s_protection.ocp_chg_streak++;
        }
        s_protection.ocp_dsg_streak = 0u;
    }
    else if ((s_protection.comp_charging_last == 0u) &&
             (abs_ma >= (int32_t)s_protection.ocp_dsg_limit_ma))
    {
        if (s_protection.ocp_dsg_streak < 0xFFu)
        {
            s_protection.ocp_dsg_streak++;
        }
        s_protection.ocp_chg_streak = 0u;
    }
    else
    {
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_dsg_streak = 0u;
    }

    if ((s_protection.ocp_chg_streak >= PROT_OCP_CONFIRM_FRAMES) &&
        (s_protection.ocp_chg_latched == 0u))
    {
        s_protection.ocp_chg_latched = 1u;
        s_protection.ocp_last_current_ma = snap->current_ma;
        s_protection.ocp_trip_count++;
        s_protection.clear_requested = 0u;   /* 新的锁存必须重新人工请求 */
        s_protection.event_flags |= PROT_EVENT_SW_CHG_OCP;
    }
    else if ((s_protection.ocp_dsg_streak >= PROT_OCP_CONFIRM_FRAMES) &&
             (s_protection.ocp_dsg_latched == 0u))
    {
        s_protection.ocp_dsg_latched = 1u;
        s_protection.ocp_last_current_ma = snap->current_ma;
        s_protection.ocp_trip_count++;
        s_protection.clear_requested = 0u;   /* 新的锁存必须重新人工请求 */
        s_protection.event_flags |= PROT_EVENT_SW_DSG_OCP;
    }
}

static void Protection_UpdateTemperature(uint32_t now_ms, uint8_t temp_valid,
                                         int16_t temp_dc)
{
    uint8_t chg_trip;
    uint8_t dsg_trip;

    s_protection.temp_valid_last = (temp_valid != 0u) ? 1u : 0u;
    s_protection.temp_dc_last = temp_dc;

    if (temp_valid == 0u)
    {
        s_protection.temp_valid_since_ms = 0u;
        s_protection.temp_chg_trip_since_ms = 0u;
        s_protection.temp_dsg_trip_since_ms = 0u;
        if (s_protection.temp_invalid == 0u)
        {
            if (s_protection.temp_invalid_since_ms == 0u)
            {
                s_protection.temp_invalid_since_ms = now_ms;
            }
            else if ((now_ms - s_protection.temp_invalid_since_ms) >=
                     PROT_TEMP_DWELL_MS)
            {
                s_protection.temp_invalid = 1u;
                s_protection.temp_chg_inhibit = 1u;
                s_protection.temp_dsg_inhibit = 1u;
                s_protection.event_flags |= PROT_EVENT_TEMP_INHIBIT;
            }
        }
        return;
    }

    s_protection.temp_invalid_since_ms = 0u;

    /* 无效传感器恢复：连续有效 2 s 才真正放行，避免 TS1 抖动反复开管。 */
    if (s_protection.temp_invalid != 0u)
    {
        if (s_protection.temp_valid_since_ms == 0u)
        {
            s_protection.temp_valid_since_ms = now_ms;
        }
        else if ((now_ms - s_protection.temp_valid_since_ms) >=
                 PROT_TEMP_DWELL_MS)
        {
            s_protection.temp_invalid = 0u;
            s_protection.event_flags |= PROT_EVENT_TEMP_RECOVER;
        }
    }
    else
    {
        s_protection.temp_valid_since_ms = 0u;
    }

    /* 充电方向：触发侧 2 s 去抖，恢复侧 2 °C 回差。 */
    if (s_protection.temp_chg_inhibit != 0u)
    {
        if ((temp_dc >= PROT_TEMP_CHG_REC_MIN_DC) &&
            (temp_dc <= PROT_TEMP_CHG_REC_MAX_DC))
        {
            s_protection.temp_chg_inhibit = 0u;
            s_protection.temp_chg_trip_since_ms = 0u;
            s_protection.event_flags |= PROT_EVENT_TEMP_RECOVER;
        }
        else
        {
            s_protection.temp_chg_trip_since_ms = 0u;
        }
    }
    else
    {
        chg_trip = (uint8_t)(((temp_dc < PROT_TEMP_CHG_MIN_DC) ||
                              (temp_dc > PROT_TEMP_CHG_MAX_DC)) ? 1u : 0u);
        if (chg_trip == 0u)
        {
            s_protection.temp_chg_trip_since_ms = 0u;
        }
        else if (s_protection.temp_chg_trip_since_ms == 0u)
        {
            s_protection.temp_chg_trip_since_ms = now_ms;
        }
        else if ((now_ms - s_protection.temp_chg_trip_since_ms) >=
                 PROT_TEMP_DWELL_MS)
        {
            s_protection.temp_chg_inhibit = 1u;
            s_protection.temp_chg_trip_since_ms = 0u;
            s_protection.event_flags |= PROT_EVENT_TEMP_INHIBIT;
        }
    }

    /* 放电方向。 */
    if (s_protection.temp_dsg_inhibit != 0u)
    {
        if ((temp_dc >= PROT_TEMP_DSG_REC_MIN_DC) &&
            (temp_dc <= PROT_TEMP_DSG_REC_MAX_DC))
        {
            s_protection.temp_dsg_inhibit = 0u;
            s_protection.temp_dsg_trip_since_ms = 0u;
            s_protection.event_flags |= PROT_EVENT_TEMP_RECOVER;
        }
        else
        {
            s_protection.temp_dsg_trip_since_ms = 0u;
        }
    }
    else
    {
        dsg_trip = (uint8_t)(((temp_dc < PROT_TEMP_DSG_MIN_DC) ||
                              (temp_dc > PROT_TEMP_DSG_MAX_DC)) ? 1u : 0u);
        if (dsg_trip == 0u)
        {
            s_protection.temp_dsg_trip_since_ms = 0u;
        }
        else if (s_protection.temp_dsg_trip_since_ms == 0u)
        {
            s_protection.temp_dsg_trip_since_ms = now_ms;
        }
        else if ((now_ms - s_protection.temp_dsg_trip_since_ms) >=
                 PROT_TEMP_DWELL_MS)
        {
            s_protection.temp_dsg_inhibit = 1u;
            s_protection.temp_dsg_trip_since_ms = 0u;
            s_protection.event_flags |= PROT_EVENT_TEMP_INHIBIT;
        }
    }
}

void Protection_Init(void)
{
    s_protection.state = PROTECTION_INIT;
    s_protection.reasons = 0u;
    s_protection.boot_latched_mask = 0u;
    s_protection.allow_mask = 0u;
    s_protection.clear_requested = 0u;
    s_protection.clear_failed = 0u;
    s_protection.startup_capture_open = 1u;
    s_protection.clear_attempts = 0u;
    s_protection.last_clear_result = 0u;
    s_protection.clear_block_reason = 0u;
    s_protection.safe_since_ms = 0u;
    s_protection.last_clear_ms = 0u;
    s_protection.event_flags = 0u;

    s_protection.ocp_chg_limit_ma = PROT_OCP_CHG_MA;
    s_protection.ocp_dsg_limit_ma = PROT_OCP_DSG_MA;
    s_protection.ocp_chg_latched = 0u;
    s_protection.ocp_dsg_latched = 0u;
    s_protection.ocp_chg_streak = 0u;
    s_protection.ocp_dsg_streak = 0u;
    s_protection.ocp_trip_count = 0u;
    s_protection.ocp_last_snap_ms = 0u;
    s_protection.ocp_last_current_ma = 0;
    s_protection.implausible_current_count = 0u;
    s_protection.sign_mismatch_streak = 0u;
    s_protection.sign_mismatch_count = 0u;

    s_protection.comp_charging_last = 0u;
    s_protection.temp_valid_last = 0u;
    s_protection.temp_dc_last = 0;
    s_protection.temp_chg_inhibit = 0u;
    s_protection.temp_dsg_inhibit = 0u;
    s_protection.temp_invalid = 1u;   /* 首次有效温度持续 2 s 后才放行 */
    s_protection.temp_chg_trip_since_ms = 0u;
    s_protection.temp_dsg_trip_since_ms = 0u;
    s_protection.temp_invalid_since_ms = 0u;
    s_protection.temp_valid_since_ms = 0u;

    Fault_ClearLive(FAULT_BIT_PROTECTION);
    Fault_ClearLive(FAULT_BIT_TEMP);
}

void Protection_Task(uint32_t now_ms, const bq_snapshot_t *snap,
                     uint8_t comp_charging,
                     uint8_t temp_valid, int16_t temp_dc)
{
    uint16_t reasons = 0u;
    uint8_t bq_ready;
    uint8_t data_ready;
    uint8_t sys_stat = (snap != NULL) ? snap->sys_stat : 0u;
    uint8_t latch = (uint8_t)(sys_stat & BQ_SYS_STAT_PROT_MASK);
    uint8_t boot_mask = (uint8_t)(BQ_GetBootProtectionLatchMask() &
                                  PROT_CLEAR_ALLOWED_MASK);
    uint8_t boot_active = 0u;
    uint8_t ocp_latched;
    uint8_t allow = BQ_FET_MASK_NONE;
    protection_state_t next_state;

    s_protection.comp_charging_last = (comp_charging != 0u) ? 1u : 0u;
    Protection_UpdateTemperature(now_ms, temp_valid, temp_dc);

    bq_ready = Protection_BqReady();
    data_ready = Protection_DataReady(now_ms, snap);

    /* 首次拿到有效帧时补齐 boot 锁存：若 BQ 驱动的 pre_sys_stat 捕获或
     * 配置期间新锁存没有被 boot_prot_latch_mask 捕获，这里允许一次启动窗口
     * 内的 SCD/OCD 进入安全清除流程；窗口关闭后运行期锁存不再自动清。 */
    if (s_protection.startup_capture_open != 0u)
    {
        if (data_ready != 0u)
        {
            if (boot_mask == 0u)
            {
                boot_mask = (uint8_t)((snap->pre_sys_stat | latch) &
                                      PROT_CLEAR_ALLOWED_MASK);
            }
            s_protection.startup_capture_open = 0u;
        }
    }
    boot_active = (uint8_t)(boot_mask & latch);

    /* 软件 OCP 只在新的发布帧上累计；BQ 硬件锁存期间 FET 已关，不叠加。 */
    if ((data_ready != 0u) && (latch == 0u) && (snap != NULL) &&
        (snap->last_update_ms != s_protection.ocp_last_snap_ms))
    {
        s_protection.ocp_last_snap_ms = snap->last_update_ms;
        Protection_EvaluateOcp(snap);
    }

    ocp_latched = (uint8_t)((s_protection.ocp_chg_latched != 0u) ||
                            (s_protection.ocp_dsg_latched != 0u));

    if ((latch & BQ_SYS_STAT_OV) != 0u)
    {
        reasons |= PROT_RSN_BQ_OV;
    }
    if ((latch & BQ_SYS_STAT_UV) != 0u)
    {
        reasons |= PROT_RSN_BQ_UV;
    }
    if ((latch & BQ_SYS_STAT_SCD) != 0u)
    {
        reasons |= PROT_RSN_BQ_SCD;
    }
    if ((latch & BQ_SYS_STAT_OCD) != 0u)
    {
        reasons |= PROT_RSN_BQ_OCD;
    }
    if ((sys_stat & BQ_SYS_STAT_XREADY) != 0u)
    {
        reasons |= PROT_RSN_BQ_XREADY;
    }
    if (boot_active != 0u)
    {
        reasons |= PROT_RSN_BOOT_LATCH;
    }
    if (s_protection.clear_failed != 0u)
    {
        reasons |= PROT_RSN_CLEAR_FAILED;
    }
    if (s_protection.ocp_chg_latched != 0u)
    {
        reasons |= PROT_RSN_SW_CHG_OCP;
    }
    if (s_protection.ocp_dsg_latched != 0u)
    {
        reasons |= PROT_RSN_SW_DSG_OCP;
    }
    if (s_protection.sign_mismatch_streak >= PROT_SIGN_MISMATCH_FRAMES)
    {
        reasons |= PROT_RSN_CURRENT_SIGN;
    }
    if (s_protection.temp_chg_inhibit != 0u)
    {
        reasons |= PROT_RSN_TEMP_CHG;
    }
    if (s_protection.temp_dsg_inhibit != 0u)
    {
        reasons |= PROT_RSN_TEMP_DSG;
    }
    if (s_protection.temp_invalid != 0u)
    {
        reasons |= PROT_RSN_TEMP_INVALID;
    }
    if ((bq_ready == 0u) || (data_ready == 0u) || (snap == NULL))
    {
        reasons |= PROT_RSN_BQ_NOT_READY;
    }

    if ((latch != 0u) || (ocp_latched != 0u))
    {
        next_state = ((boot_active != 0u) &&
                      (s_protection.clear_failed == 0u) &&
                      (ocp_latched == 0u))
                     ? PROTECTION_RECOVERY : PROTECTION_LATCHED;
        allow = BQ_FET_MASK_NONE;
    }
    else if (s_protection.clear_failed != 0u)
    {
        next_state = PROTECTION_LATCHED;
        allow = BQ_FET_MASK_NONE;
    }
    else if ((bq_ready == 0u) || (data_ready == 0u) || (snap == NULL))
    {
        next_state = (s_protection.state == PROTECTION_INIT)
                     ? PROTECTION_INIT : PROTECTION_RECOVERY;
        allow = BQ_FET_MASK_NONE;
    }
    else
    {
        next_state = PROTECTION_NORMAL;
        allow = Protection_TempAllowMask();
    }

    if ((next_state != s_protection.state) &&
        ((next_state == PROTECTION_RECOVERY) ||
         (next_state == PROTECTION_LATCHED)))
    {
        s_protection.event_flags |= (next_state == PROTECTION_LATCHED)
                                    ? PROT_EVENT_LATCHED : PROT_EVENT_TRIP;
    }
    if (next_state == PROTECTION_NORMAL)
    {
        s_protection.safe_since_ms = 0u;
    }

    s_protection.state = next_state;
    s_protection.reasons = reasons;
    s_protection.boot_latched_mask = boot_mask;
    s_protection.allow_mask = allow;

    if ((latch != 0u) || (s_protection.clear_failed != 0u) ||
        (ocp_latched != 0u))
    {
        Fault_SetLive(FAULT_BIT_PROTECTION);
    }
    else
    {
        Fault_ClearLive(FAULT_BIT_PROTECTION);
    }

    if (s_protection.temp_invalid != 0u)
    {
        Fault_SetLive(FAULT_BIT_TEMP);
    }
    else
    {
        Fault_ClearLive(FAULT_BIT_TEMP);
    }
}

void Protection_Service(uint32_t now_ms, const bq_snapshot_t *snap)
{
    uint8_t bq_mask = 0u;
    uint8_t stat;
    uint8_t want_bq = 0u;
    uint8_t want_sw;

    want_sw = (uint8_t)(((s_protection.ocp_chg_latched != 0u) ||
                         (s_protection.ocp_dsg_latched != 0u)) &&
                        (s_protection.clear_requested != 0u));

    if ((snap == NULL) || (Protection_BqReady() == 0u) ||
        (Protection_DataReady(now_ms, snap) == 0u))
    {
        s_protection.safe_since_ms = 0u;
        s_protection.clear_block_reason = PROT_CLEAR_BLOCK_READY;
        return;
    }

    /* BQ 每 500 ms 有一次短暂 run script；这只是“暂时不能写 I2C”，
     * 不是安全条件失效。不能因此把 2 s 安全计时清掉，否则永远到不了清除。 */
    if (BQ_IsIdle() == 0u)
    {
        s_protection.clear_block_reason = PROT_CLEAR_BLOCK_READY;
        return;
    }

    stat = snap->sys_stat;

    if (s_protection.state == PROTECTION_RECOVERY)
    {
        /* 用本地 boot mask（含启动窗口兜底），不要只看 BQ 驱动的原始字段。 */
        bq_mask = (uint8_t)(s_protection.boot_latched_mask &
                            PROT_CLEAR_ALLOWED_MASK &
                            stat);
    }
    if (s_protection.clear_requested != 0u)
    {
        bq_mask |= (uint8_t)(stat & PROT_CLEAR_ALLOWED_MASK);
    }
    want_bq = (bq_mask != 0u) ? 1u : 0u;

    if ((want_bq == 0u) && (want_sw == 0u))
    {
        s_protection.safe_since_ms = 0u;
        s_protection.clear_requested = 0u;
        s_protection.clear_block_reason = 0u;
        return;
    }

    /* 保护清锁存绝不能在 FET 可能导通或电流不明确时进行。 */
    if ((BQ_GetFetMaskApplied() != 0u) ||
        (BQ_GetFetMaskRequested() != 0u))
    {
        s_protection.safe_since_ms = 0u;
        s_protection.clear_block_reason = PROT_CLEAR_BLOCK_FET;
        return;
    }
    if (Protection_AbsCurrentMa(snap) > PROT_CLEAR_CURRENT_MA)
    {
        s_protection.safe_since_ms = 0u;
        s_protection.clear_block_reason = PROT_CLEAR_BLOCK_CURRENT;
        return;
    }
    /* 充电 OCP：充电器必须在物理上移除，不能只靠电流很小。 */
    if ((s_protection.ocp_chg_latched != 0u) &&
        (s_protection.comp_charging_last != 0u))
    {
        s_protection.safe_since_ms = 0u;
        s_protection.clear_block_reason = PROT_CLEAR_BLOCK_CHARGER;
        return;
    }

    s_protection.clear_block_reason = 0u;

    if (s_protection.safe_since_ms == 0u)
    {
        s_protection.safe_since_ms = now_ms;
        return;
    }
    if ((now_ms - s_protection.safe_since_ms) < PROT_CLEAR_HOLD_MS)
    {
        return;
    }

    s_protection.clear_attempts++;
    s_protection.last_clear_ms = now_ms;

    if (want_bq != 0u)
    {
        if (BQ_ClearProtectionVerified(bq_mask, &stat) != 0)
        {
            s_protection.last_clear_result = 2u;
            s_protection.safe_since_ms = 0u;
            s_protection.clear_requested = 0u;
            s_protection.clear_failed = 1u;
            s_protection.event_flags |= PROT_EVENT_CLEAR_FAILED;
            s_protection.state = PROTECTION_LATCHED;
            s_protection.allow_mask = BQ_FET_MASK_NONE;
            return;
        }
    }

    if (want_sw != 0u)
    {
        s_protection.ocp_chg_latched = 0u;
        s_protection.ocp_dsg_latched = 0u;
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_dsg_streak = 0u;
    }

    s_protection.last_clear_result = 1u;
    s_protection.safe_since_ms = 0u;
    s_protection.clear_requested = 0u;
    s_protection.clear_failed = 0u;
    s_protection.event_flags |= PROT_EVENT_CLEARED;

    if ((stat & BQ_SYS_STAT_PROT_MASK) == 0u)
    {
        s_protection.state = PROTECTION_NORMAL;
        s_protection.allow_mask = Protection_TempAllowMask();
        s_protection.reasons &= (uint16_t)~(PROT_RSN_BQ_OV |
                                            PROT_RSN_BQ_UV |
                                            PROT_RSN_BQ_SCD |
                                            PROT_RSN_BQ_OCD |
                                            PROT_RSN_BOOT_LATCH |
                                            PROT_RSN_SW_CHG_OCP |
                                            PROT_RSN_SW_DSG_OCP);
    }
}

uint8_t Protection_GetAllowedFetMask(void)
{
    return s_protection.allow_mask;
}

uint8_t Protection_IsLpBlocked(void)
{
    return (s_protection.state != PROTECTION_NORMAL) ? 1u : 0u;
}

protection_state_t Protection_GetState(void)
{
    return s_protection.state;
}

uint16_t Protection_GetReasons(void)
{
    return s_protection.reasons;
}

uint8_t Protection_GetBootLatchMask(void)
{
    return s_protection.boot_latched_mask;
}

void Protection_GetSnapshot(protection_snapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->state = s_protection.state;
    out->reasons = s_protection.reasons;
    out->allow_mask = s_protection.allow_mask;
    out->boot_latched_mask = s_protection.boot_latched_mask;
    out->clear_requested = s_protection.clear_requested;
    out->clear_failed = s_protection.clear_failed;
    out->clear_block_reason = s_protection.clear_block_reason;
    out->last_clear_result = s_protection.last_clear_result;
    out->ocp_chg_latched = s_protection.ocp_chg_latched;
    out->ocp_dsg_latched = s_protection.ocp_dsg_latched;
    out->ocp_chg_streak = s_protection.ocp_chg_streak;
    out->ocp_dsg_streak = s_protection.ocp_dsg_streak;
    out->ocp_chg_limit_ma = s_protection.ocp_chg_limit_ma;
    out->ocp_dsg_limit_ma = s_protection.ocp_dsg_limit_ma;
    out->ocp_last_current_ma = s_protection.ocp_last_current_ma;
    out->ocp_trip_count = s_protection.ocp_trip_count;
    out->ocp_last_snap_ms = s_protection.ocp_last_snap_ms;
    out->implausible_current_count = s_protection.implausible_current_count;
    out->sign_mismatch_streak = s_protection.sign_mismatch_streak;
    out->sign_mismatch_count = s_protection.sign_mismatch_count;
    out->temp_chg_inhibit = s_protection.temp_chg_inhibit;
    out->temp_dsg_inhibit = s_protection.temp_dsg_inhibit;
    out->temp_invalid = s_protection.temp_invalid;
    out->temp_valid_last = s_protection.temp_valid_last;
    out->temp_dc_last = s_protection.temp_dc_last;
    out->comp_charging_last = s_protection.comp_charging_last;
}

void Protection_RequestLatchClear(void)
{
    /* 显式人工请求；仍然必须经过 Service 的安全条件。 */
    s_protection.clear_requested = 1u;
    s_protection.clear_failed = 0u;
}

uint8_t Protection_TakeEvent(void)
{
    uint8_t events = s_protection.event_flags;

    s_protection.event_flags = 0u;
    return events;
}

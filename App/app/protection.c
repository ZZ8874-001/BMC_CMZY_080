/**
 ******************************************************************************
 * @file    protection.c
 * @brief   BQ 保护监督层（M2：硬件锁存 + 软件 OCP + NTC 温度）。
 *
 * M1：评估 BQ 硬件 OV/UV/SCD/OCD 锁存、boot/LP 后 SCD/OCD 的安全清除、
 *     FET 允许掩码和一次性事件。
 * M2a：软件充电 5.5 A / 放电 33 A OCP，2 个新发布帧确认后锁存；只允许
 *      `Protection_RequestLatchClear()` 在安全条件下清除，不自动重试。
 * M2b：NTC 包温度门控，4–54 °C，充放电都收紧；TS1 无效时也双禁。
 *
 * FET 请求仍由 charge_policy 唯一持有；本模块只给允许掩码。
 ******************************************************************************
 */
#include "app/protection.h"

#include "board.h"
#include "app/faults.h"
#include "bsp/buzzer.h"

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
    uint16_t event_flags;

    /* M2a 软件 OCP */
    volatile uint16_t ocp_chg_limit_ma;   /* 默认宏值；Ozone 可临时调低做台架 */
    volatile uint16_t ocp_dsg_limit_ma;
    uint8_t ocp_chg_latched;
    uint8_t ocp_dsg_latched;
    uint8_t ocp_chg_streak;
    uint8_t ocp_dsg_streak;
    uint32_t ocp_trip_count;
    uint32_t ocp_last_snap_ms;
    uint32_t ocp_chg_since_ms;
    uint32_t ocp_dsg_since_ms;
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

    /* 低压分级 */
    uint8_t  cell_uv_active;
    uint8_t  cell_uv_recover_streak;
    uint8_t  ref_cell2_only;
    uint8_t  low_batt_dsg_allowed;
    uint16_t cell_uv_min_mv;
    uint32_t cell_last_update_ms;
    uint32_t cell_uv_count;

    uint8_t  soc_early_latched;
    uint8_t  soc_early_streak;
    uint8_t  soc_early_recover_streak;
    uint16_t soc_last_soc01;
    uint32_t soc_last_update_ms;
    uint32_t soc_early_count;
    uint8_t  soc_valid_last;

    uint8_t  charge_full;
    uint8_t  charge_full_clear_streak;
    uint32_t charge_full_since_ms;
    uint32_t charge_full_count;
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
            (BQ_IsFresh(now_ms) != 0u) &&
            (BQ_GetRefCellMv(snap) != 0u)) ? 1u : 0u;
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
        s_protection.ocp_chg_since_ms = 0u;
        s_protection.ocp_dsg_since_ms = 0u;
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
        s_protection.ocp_chg_since_ms = 0u;
        s_protection.ocp_dsg_since_ms = 0u;
        return;
    }

    /* 方向用 COMP2（独立于 CC 符号），幅值用 CC 积分值。这样即使 SRP/SRN
     * 或 CC 极性与 TI 约定相反，也不会把充电/放电过流误判；CC 符号只用于
     * SOC 和 sign_mismatch 诊断。阈值/时间：充电 5.5 A/1 s、放电 33 A/500 ms，
     * 且至少 2 个新发布帧。 */
    if ((s_protection.comp_charging_last != 0u) &&
        (abs_ma >= (int32_t)s_protection.ocp_chg_limit_ma))
    {
        if (s_protection.ocp_chg_streak == 0u)
        {
            s_protection.ocp_chg_since_ms = snap->last_update_ms;
        }
        if (s_protection.ocp_chg_streak < 0xFFu)
        {
            s_protection.ocp_chg_streak++;
        }
        s_protection.ocp_dsg_streak = 0u;
        s_protection.ocp_dsg_since_ms = 0u;
    }
    else if ((s_protection.comp_charging_last == 0u) &&
             (abs_ma >= (int32_t)s_protection.ocp_dsg_limit_ma))
    {
        if (s_protection.ocp_dsg_streak == 0u)
        {
            s_protection.ocp_dsg_since_ms = snap->last_update_ms;
        }
        if (s_protection.ocp_dsg_streak < 0xFFu)
        {
            s_protection.ocp_dsg_streak++;
        }
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_chg_since_ms = 0u;
    }
    else
    {
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_dsg_streak = 0u;
        s_protection.ocp_chg_since_ms = 0u;
        s_protection.ocp_dsg_since_ms = 0u;
    }

    if ((s_protection.ocp_chg_streak >= PROT_OCP_CONFIRM_FRAMES) &&
        ((snap->last_update_ms - s_protection.ocp_chg_since_ms) >=
         PROT_OCP_CHG_MS) &&
        (s_protection.ocp_chg_latched == 0u))
    {
        s_protection.ocp_chg_latched = 1u;
        s_protection.ocp_last_current_ma = snap->current_ma;
        s_protection.ocp_trip_count++;
        s_protection.clear_requested = 0u;   /* 新的锁存必须重新人工请求 */
        s_protection.event_flags |= PROT_EVENT_SW_CHG_OCP;
    }
    else if ((s_protection.ocp_dsg_streak >= PROT_OCP_CONFIRM_FRAMES) &&
             ((snap->last_update_ms - s_protection.ocp_dsg_since_ms) >=
              PROT_OCP_DSG_MS) &&
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

static uint16_t Protection_RefCellMv(const bq_snapshot_t *snap)
{
    /* 正常=三节最低值；cell3/pack 不可信时=VC2；参考节无效返回 0。 */
    return BQ_GetRefCellMv(snap);
}

static uint8_t Protection_SocWarnValid(const soc_est_snapshot_t *soc)
{
    return (uint8_t)((soc != NULL) &&
                     (soc->frame_valid != 0u) &&
                     (soc->quality == SOC_EST_QUALITY_REAL) &&
                     (soc->cal_valid != 0u) &&
                     (soc->temp_valid != 0u) &&
                     (soc->soc_estimated == 0u));
}

/* 低电状态普通情况下只保留 CHG；只有 ChargePolicy 已确认 >200 mA 的
 * DSG boost 时，才不把 cell_uv/低电锁存的 DSG 裁掉。硬故障 allow 已经是
 * NONE，boost 不会重新打开。 */
static uint8_t Protection_ApplyLowVoltageAllow(uint8_t allow,
                                               uint8_t dsg_boost)
{
    if (dsg_boost == 0u)
    {
        if (s_protection.cell_uv_active != 0u)
        {
            allow &= BQ_FET_MASK_CHG;
        }
        if (s_protection.soc_early_latched != 0u)
        {
            allow &= BQ_FET_MASK_CHG;
        }
    }
    return allow;
}

static void Protection_UpdateLowVoltage(uint32_t now_ms,
                                        const bq_snapshot_t *snap,
                                        const soc_est_snapshot_t *soc,
                                        uint8_t data_ready)
{
    uint16_t ref_mv;
    uint8_t soc_usable;
    uint8_t low_now;
    uint8_t low_clear;

    (void)now_ms;

    if (soc != NULL)
    {
        s_protection.soc_last_update_ms = soc->last_update_ms;
        s_protection.soc_last_soc01 = (uint16_t)((soc->soc01 < 0) ? 0 :
                                      ((soc->soc01 > 10000) ? 10000 :
                                       soc->soc01));
        s_protection.soc_valid_last =
            (Protection_SocWarnValid(soc) != 0u) ? 1u : 0u;
    }
    else
    {
        s_protection.soc_valid_last = 0u;
    }

    if ((snap == NULL) || (data_ready == 0u) ||
        (snap->last_update_ms == s_protection.cell_last_update_ms))
    {
        return;
    }
    s_protection.cell_last_update_ms = snap->last_update_ms;
    ref_mv = Protection_RefCellMv(snap);
    s_protection.cell_uv_min_mv = ref_mv;

    /* 2.8 V 单节硬保护：只依赖有效 BQ 帧，不依赖 SOC。 */
    if (s_protection.cell_uv_active == 0u)
    {
        if (ref_mv < PROT_CELL_UV_MV)
        {
            s_protection.cell_uv_active = 1u;
            s_protection.cell_uv_recover_streak = 0u;
            s_protection.cell_uv_count++;
            s_protection.event_flags |= PROT_EVENT_CELL_UV;
        }
    }
    else if (ref_mv >= PROT_CELL_UV_RECOVER_MV)
    {
        if (s_protection.cell_uv_recover_streak < 0xFFu)
        {
            s_protection.cell_uv_recover_streak++;
        }
        if (s_protection.cell_uv_recover_streak >=
            PROT_CELL_UV_RECOVER_FRAMES)
        {
            s_protection.cell_uv_active = 0u;
            s_protection.cell_uv_recover_streak = 0u;
            s_protection.event_flags |= PROT_EVENT_CELL_UV_CLEARED;
        }
    }
    else
    {
        s_protection.cell_uv_recover_streak = 0u;
    }

    /* 低电状态：电压条件独立于 SOC；SOC 只在标定/温度/真实帧有效时参与。 */
    if (ref_mv == 0u)
    {
        s_protection.soc_early_streak = 0u;
        s_protection.soc_early_recover_streak = 0u;
        return;
    }

    soc_usable = Protection_SocWarnValid(soc);
    low_now = (uint8_t)((ref_mv < PROT_EARLY_CELL_MV) ? 1u : 0u);
    low_clear = (uint8_t)((ref_mv >= PROT_EARLY_CELL_MV) ? 1u : 0u);
    if (soc_usable != 0u)
    {
        if (soc->soc01 < (int32_t)PROT_EARLY_SOC_01)
        {
            low_now = 1u;
        }
        if (soc->soc01 < (int32_t)PROT_EARLY_RECOVER_SOC_01)
        {
            low_clear = 0u;
        }
    }

    if (s_protection.soc_early_latched == 0u)
    {
        if (low_now != 0u)
        {
            if (s_protection.soc_early_streak < 0xFFu)
            {
                s_protection.soc_early_streak++;
            }
            if (s_protection.soc_early_streak >=
                PROT_EARLY_CONFIRM_FRAMES)
            {
                s_protection.soc_early_latched = 1u;
                s_protection.soc_early_streak = 0u;
                s_protection.soc_early_recover_streak = 0u;
                s_protection.soc_early_count++;
                s_protection.event_flags |= PROT_EVENT_EARLY_SOC_CUTOFF;
                Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
            }
        }
        else
        {
            s_protection.soc_early_streak = 0u;
        }
    }
    else
    {
        if (low_clear != 0u)
        {
            if (s_protection.soc_early_recover_streak < 0xFFu)
            {
                s_protection.soc_early_recover_streak++;
            }
            if (s_protection.soc_early_recover_streak >=
                PROT_EARLY_RECOVER_FRAMES)
            {
                s_protection.soc_early_latched = 0u;
                s_protection.soc_early_streak = 0u;
                s_protection.soc_early_recover_streak = 0u;
                s_protection.event_flags |=
                    PROT_EVENT_EARLY_SOC_CUTOFF_CLEARED;
            }
        }
        else
        {
            s_protection.soc_early_recover_streak = 0u;
        }
    }
}

/* 充满只做提示：不改变 allow、不关 CHG；BQ OV 仍是最终硬件保护。 */
static void Protection_UpdateFull(uint32_t now_ms,
                                  const bq_snapshot_t *snap,
                                  const soc_est_snapshot_t *soc,
                                  uint8_t data_ready,
                                  uint8_t comp_charging,
                                  charge_policy_state_t policy_state)
{
    uint16_t min_mv;
    int32_t charge_ma;
    uint8_t candidate = 0u;

    if ((snap == NULL) || (data_ready == 0u) ||
        (comp_charging == 0u) || (policy_state != CHARGE_POLICY_CHARGE))
    {
        s_protection.charge_full_since_ms = 0u;
        if (s_protection.charge_full != 0u)
        {
            s_protection.charge_full = 0u;
            s_protection.charge_full_clear_streak = 0u;
            s_protection.event_flags |= PROT_EVENT_CHARGE_FULL_CLEARED;
        }
        return;
    }

    min_mv = Protection_RefCellMv(snap);
    charge_ma = Protection_ChargeCurrentMa(snap->current_ma);

    if (((min_mv >= PROT_FULL_CELL_MV) ||
         ((Protection_SocWarnValid(soc) != 0u) &&
          (soc->soc01 >= (int32_t)PROT_FULL_SOC01))) &&
        (charge_ma >= -20) && (charge_ma <= PROT_FULL_TAPER_MA))
    {
        candidate = 1u;
    }

    if (candidate != 0u)
    {
        s_protection.charge_full_clear_streak = 0u;
        if (s_protection.charge_full == 0u)
        {
            if (s_protection.charge_full_since_ms == 0u)
            {
                s_protection.charge_full_since_ms = now_ms;
            }
            else if ((now_ms - s_protection.charge_full_since_ms) >=
                     PROT_FULL_TAPER_MS)
            {
                s_protection.charge_full = 1u;
                s_protection.charge_full_count++;
                s_protection.event_flags |= PROT_EVENT_CHARGE_FULL;
            }
        }
    }
    else
    {
        s_protection.charge_full_since_ms = 0u;
        if (s_protection.charge_full != 0u)
        {
            if (charge_ma > PROT_FULL_EXIT_MA)
            {
                if (s_protection.charge_full_clear_streak < 0xFFu)
                {
                    s_protection.charge_full_clear_streak++;
                }
                if (s_protection.charge_full_clear_streak >=
                    PROT_FULL_CLEAR_FRAMES)
                {
                    s_protection.charge_full = 0u;
                    s_protection.charge_full_clear_streak = 0u;
                    s_protection.event_flags |=
                        PROT_EVENT_CHARGE_FULL_CLEARED;
                }
            }
            else
            {
                s_protection.charge_full_clear_streak = 0u;
            }
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
    s_protection.ocp_chg_since_ms = 0u;
    s_protection.ocp_dsg_since_ms = 0u;
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

    s_protection.cell_uv_active = 0u;
    s_protection.cell_uv_recover_streak = 0u;
    s_protection.ref_cell2_only = 0u;
    s_protection.low_batt_dsg_allowed = 0u;
    s_protection.cell_uv_min_mv = 0u;
    s_protection.cell_last_update_ms = 0u;
    s_protection.cell_uv_count = 0u;

    s_protection.soc_early_latched = 0u;
    s_protection.soc_early_streak = 0u;
    s_protection.soc_early_recover_streak = 0u;
    s_protection.soc_last_soc01 = 0u;
    s_protection.soc_last_update_ms = 0u;
    s_protection.soc_early_count = 0u;
    s_protection.soc_valid_last = 0u;

    s_protection.charge_full = 0u;
    s_protection.charge_full_clear_streak = 0u;
    s_protection.charge_full_since_ms = 0u;
    s_protection.charge_full_count = 0u;

    /* 复位按换电处理：不从 StateStore 恢复旧的 CUTOFF_LATCH。低电状态
     * 由首个有效 BQ 帧的实时参考节/SOC 重新建立。 */
    Fault_ClearLive(FAULT_BIT_PROTECTION);
    Fault_ClearLive(FAULT_BIT_TEMP);
}

void Protection_Task(uint32_t now_ms, const bq_snapshot_t *snap,
                     const soc_est_snapshot_t *soc,
                     uint8_t comp_charging, uint8_t charger_present,
                     charge_policy_state_t policy_state,
                     uint8_t low_batt_dsg_on)
{
    uint16_t reasons = 0u;
    uint8_t bq_ready;
    uint8_t data_ready;
    uint8_t low_batt_boost = 0u;
    uint8_t sys_stat = (snap != NULL) ? snap->sys_stat : 0u;
    uint8_t latch = (uint8_t)(sys_stat & BQ_SYS_STAT_PROT_MASK);
    uint8_t hard_latch = (uint8_t)(latch &
                                   (BQ_SYS_STAT_OV | BQ_SYS_STAT_SCD |
                                    BQ_SYS_STAT_OCD));
    uint8_t uv_only;
    uint8_t boot_mask = (uint8_t)(BQ_GetBootProtectionLatchMask() &
                                  PROT_CLEAR_ALLOWED_MASK);
    uint8_t boot_active = 0u;
    uint8_t ocp_latched;
    uint8_t allow = BQ_FET_MASK_NONE;
    protection_state_t next_state;

    s_protection.comp_charging_last = (comp_charging != 0u) ? 1u : 0u;
    s_protection.ref_cell2_only =
        ((snap != NULL) && (snap->ref_cell2_only != 0u)) ? 1u : 0u;
    Protection_UpdateTemperature(now_ms,
                                 (soc != NULL) ? soc->temp_valid : 0u,
                                 (soc != NULL) ? soc->temp_dc : 0);

    bq_ready = Protection_BqReady();
    data_ready = Protection_DataReady(now_ms, snap);
    uv_only = (uint8_t)(((latch & BQ_SYS_STAT_UV) != 0u) &&
                        (hard_latch == 0u) &&
                        ((sys_stat & BQ_SYS_STAT_XREADY) == 0u) &&
                        (s_protection.ocp_chg_latched == 0u) &&
                        (s_protection.ocp_dsg_latched == 0u) &&
                        (s_protection.clear_failed == 0u));

    Protection_UpdateLowVoltage(now_ms, snap, soc, data_ready);

    /* ChargePolicy 已在低电状态内完成 200/120 mA 回差；这里只消费它的
     * 结果，避免在 120–200 mA 之间提前裁掉 DSG。CHG 必须仍允许。 */
    if ((policy_state == CHARGE_POLICY_LOW_BATTERY_CHARGE) &&
        (low_batt_dsg_on != 0u) && (snap != NULL) &&
        (data_ready != 0u) &&
        (Protection_RefCellMv(snap) >= PROT_CELL_UV_MV) &&
        ((Protection_TempAllowMask() & BQ_FET_MASK_CHG) != 0u))
    {
        low_batt_boost = 1u;
    }
    s_protection.low_batt_dsg_allowed = low_batt_boost;

    /* 数据中断或 BQ 已锁存时不能把旧的 OCP 计时带入下一段连续过流。 */
    if ((data_ready == 0u) || (latch != 0u))
    {
        s_protection.ocp_chg_streak = 0u;
        s_protection.ocp_dsg_streak = 0u;
        s_protection.ocp_chg_since_ms = 0u;
        s_protection.ocp_dsg_since_ms = 0u;
    }

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

    Protection_UpdateFull(now_ms, snap, soc, data_ready, comp_charging,
                          policy_state);

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
    if (s_protection.cell_uv_active != 0u)
    {
        reasons |= PROT_RSN_CELL_UV;
    }
    if (s_protection.soc_early_latched != 0u)
    {
        reasons |= PROT_RSN_SOC_CUTOFF;
    }
    if ((bq_ready == 0u) || (data_ready == 0u) || (snap == NULL))
    {
        reasons |= PROT_RSN_BQ_NOT_READY;
    }

    if ((hard_latch != 0u) || (ocp_latched != 0u))
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
        if (uv_only != 0u)
        {
            /* BQ UV 单独存在时只关 DSG；CHG 用于充电恢复。 */
            allow &= BQ_FET_MASK_CHG;
        }
    }

    allow = Protection_ApplyLowVoltageAllow(allow, low_batt_boost);

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

    if ((hard_latch != 0u) || (s_protection.clear_failed != 0u) ||
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
        s_protection.allow_mask =
            Protection_ApplyLowVoltageAllow(Protection_TempAllowMask(),
                                            s_protection.low_batt_dsg_allowed);
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
    out->ocp_chg_since_ms = s_protection.ocp_chg_since_ms;
    out->ocp_dsg_since_ms = s_protection.ocp_dsg_since_ms;
    out->implausible_current_count = s_protection.implausible_current_count;
    out->sign_mismatch_streak = s_protection.sign_mismatch_streak;
    out->sign_mismatch_count = s_protection.sign_mismatch_count;
    out->temp_chg_inhibit = s_protection.temp_chg_inhibit;
    out->temp_dsg_inhibit = s_protection.temp_dsg_inhibit;
    out->temp_invalid = s_protection.temp_invalid;
    out->temp_valid_last = s_protection.temp_valid_last;
    out->temp_dc_last = s_protection.temp_dc_last;
    out->comp_charging_last = s_protection.comp_charging_last;
    out->soc01 = s_protection.soc_last_soc01;
    out->min_cell_mv = s_protection.cell_uv_min_mv;
    out->soc_valid = s_protection.soc_valid_last;
    out->ref_cell2_only = s_protection.ref_cell2_only;
    out->cell_uv_active = s_protection.cell_uv_active;
    out->low_batt_dsg_allowed = s_protection.low_batt_dsg_allowed;
    out->cell_uv_streak = s_protection.cell_uv_recover_streak;
    out->early_soc_latched = s_protection.soc_early_latched;
    out->early_soc_streak = s_protection.soc_early_streak;
    out->early_soc_recover_streak = s_protection.soc_early_recover_streak;
    out->charge_full = s_protection.charge_full;
    out->charge_full_since_ms = s_protection.charge_full_since_ms;
    out->cell_uv_count = s_protection.cell_uv_count;
    out->early_soc_count = s_protection.soc_early_count;
    out->charge_full_count = s_protection.charge_full_count;
}

uint8_t Protection_IsCellUvActive(void)
{
    return s_protection.cell_uv_active;
}

uint8_t Protection_IsLowBattery(void)
{
    return (uint8_t)((s_protection.soc_early_latched != 0u) ||
                     (s_protection.cell_uv_active != 0u));
}

uint8_t Protection_IsSocCutoffLatched(void)
{
    return s_protection.soc_early_latched;
}

uint8_t Protection_GetChargeFull(void)
{
    return s_protection.charge_full;
}

void Protection_RestoreSocCutoff(uint8_t latched)
{
    s_protection.soc_early_latched = (latched != 0u) ? 1u : 0u;
    s_protection.soc_early_streak = 0u;
    s_protection.soc_early_recover_streak = 0u;
}

void Protection_RequestLatchClear(void)
{
    /* 显式人工请求；仍然必须经过 Service 的安全条件。 */
    s_protection.clear_requested = 1u;
    s_protection.clear_failed = 0u;
}

uint16_t Protection_TakeEvent(void)
{
    uint16_t events = s_protection.event_flags;

    s_protection.event_flags = 0u;
    return events;
}

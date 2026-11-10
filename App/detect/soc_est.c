/**
 ******************************************************************************
 * @file    soc_est.c
 * @brief   BQ76920 快照 → li-ion-soc-soh-toolkit SOC/SOH 适配。
 *
 * 处理链（每帧 BQ 数据）：
 *   min(cell_mv) + current_ma + TS1 温度
 *     → BMS_CAL_Update(temp)
 *     → SOC_AhUpdate(cell_mv, current_ma)
 *     → SOH_Update(cell_mv, current_ma, temp)
 *
 * TS1 温度：MF52（10 kΩ @25°C，B=3950）经 10 kΩ 上拉到 5 V，
 * 按 R-T 表线性插值。NMC 标定前 SOC_EST_CAL_VALID 固定 0，结果只用于
 * 显示/记录，不能单独作为保护判据。
 ******************************************************************************
 */
#include "detect/soc_est.h"

#include "bms_algo.h"
#include "app/state_store.h"

#include <stddef.h>

#define SOC_EST_CELL_MIN_MV     1000
#define SOC_EST_CELL_MAX_MV     5000
#define SOC_EST_PACK_SUM_TOL_MV 300
#define SOC_EST_CC_FRESH_MS     1000u
/* The toolkit's OCV resync interval is internal; wait one extra real sample
 * after it before clearing the sticky estimated provenance. */
#define SOC_EST_REAL_RESYNC_MS (SOC_IDLE_MS + \
                                (SOC_QUIET_DEBOUNCE + 1u) * BMS_UPDATE_PERIOD_MS)

typedef struct
{
    int16_t  temp_dc;
    uint32_t r_ohm;
} ntc_point_t;

/* MF52（10 kΩ @25°C，B=3950）R-T 点；温度升序、电阻降序。
 * 由 R(T)=10k×exp(B×(1/T-1/298.15)) 计算。 */
static const ntc_point_t s_ntc_mf52[] =
{
    {-600, 1969964u},
    {-500, 858613u},
    {-400, 401860u},
    {-300, 200204u},
    {-200, 105385u},
    {-100,  58246u},
    {   0,  33621u},
    { 100,  20175u},
    { 200,  12535u},
    { 250,  10000u},
    { 300,   8037u},
    { 400,   5301u},
    { 500,   3588u},
    { 600,   2486u},
    { 700,   1760u},
    { 800,   1270u},
    { 900,    934u},
    {1000,    698u},
    {1100,    529u},
    {1200,    407u}
};

#define SOC_EST_NTC_POINTS  (sizeof(s_ntc_mf52) / sizeof(s_ntc_mf52[0]))

static struct
{
    uint8_t  frame_valid;
    uint8_t  temp_valid;
    uint8_t  cal_valid;
    soc_est_quality_t quality;
    uint8_t  soc_estimated;   /* partial 后直到真实静置 OCV 重同步仍非校准 SOC */
    uint8_t  ekf_reset_pending;
    uint8_t  real_quiet_count;
    uint8_t  sample_seen;
    uint8_t  nvm_ok;
    int32_t  soc01;
    int32_t  soh01;
    int16_t  temp_dc;
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    uint16_t ts1_raw;
    uint16_t ts1_mv;
    int32_t  current_ma;
    uint32_t last_update_ms;
    uint32_t sample_ms;
    uint32_t source_batch_seq;
    uint32_t last_cc_seq;
    uint32_t real_quiet_since_ms;
    uint32_t update_count;
    uint32_t nvm_save_count;
    uint32_t state_commit_count;
} s_soc_est;

static uint16_t SocEst_Ts1Mv(const bq_snapshot_t *snap)
{
    int32_t microvolts;
    int32_t millivolts;

    if (snap == NULL)
    {
        return 0u;
    }
    microvolts = (int32_t)(365 + (int32_t)snap->adc_gain) *
                 (int32_t)snap->ts1_raw;
    millivolts = (microvolts / 1000) + (int32_t)snap->adc_offset;
    if (millivolts < 0)
    {
        millivolts = 0;
    }
    if (millivolts > 65535)
    {
        millivolts = 65535;
    }
    return (uint16_t)millivolts;
}

static uint8_t SocEst_Ts1ToDeciC(const bq_snapshot_t *snap, int16_t *temp_dc)
{
    uint16_t mv;
    uint32_t r_ohm;
    uint32_t denom;
    uint8_t i;

    if ((snap == NULL) || (temp_dc == NULL) ||
        (snap->ts1_raw == 0u) || (snap->ts1_raw >= 0x3FFFu))
    {
        return 0u;
    }

    mv = SocEst_Ts1Mv(snap);
    if ((mv < 20u) || (mv >= SOC_EST_TS1_PULL_MV))
    {
        return 0u;
    }

    /* 外部/内部上拉近似为 SOC_EST_TS1_PULL_OHM，得到 NTC 电阻。 */
    denom = (uint32_t)SOC_EST_TS1_PULL_MV - (uint32_t)mv;
    if (denom == 0u)
    {
        return 0u;
    }
    r_ohm = ((uint32_t)SOC_EST_TS1_PULL_OHM * (uint32_t)mv) / denom;

    if (r_ohm >= s_ntc_mf52[0].r_ohm)
    {
        *temp_dc = s_ntc_mf52[0].temp_dc;
        return 1u;
    }
    for (i = 1u; i < SOC_EST_NTC_POINTS; i++)
    {
        if (r_ohm >= s_ntc_mf52[i].r_ohm)
        {
            int32_t r_hi = (int32_t)s_ntc_mf52[i - 1u].r_ohm;
            int32_t r_lo = (int32_t)s_ntc_mf52[i].r_ohm;
            int32_t t_hi = s_ntc_mf52[i - 1u].temp_dc;
            int32_t t_lo = s_ntc_mf52[i].temp_dc;
            int32_t num = (r_hi - (int32_t)r_ohm) * (t_lo - t_hi);

            *temp_dc = (int16_t)(t_hi + (num / (r_hi - r_lo)));
            return 1u;
        }
    }

    *temp_dc = s_ntc_mf52[SOC_EST_NTC_POINTS - 1u].temp_dc;
    return 1u;
}

void SocEst_Init(void)
{
    s_soc_est.frame_valid = 0u;
    s_soc_est.temp_valid = 0u;
    s_soc_est.cal_valid = SOC_EST_CAL_VALID;
    s_soc_est.quality = SOC_EST_QUALITY_NONE;
    s_soc_est.soc_estimated = 0u;
    s_soc_est.ekf_reset_pending = 0u;
    s_soc_est.real_quiet_count = 0u;
    s_soc_est.sample_seen = 0u;
    s_soc_est.nvm_ok = 1u;
    s_soc_est.soc01 = 0;
    s_soc_est.soh01 = 10000;
    s_soc_est.temp_dc = 250;
    s_soc_est.cell_mv[0] = 0u;
    s_soc_est.cell_mv[1] = 0u;
    s_soc_est.cell_mv[2] = 0u;
    s_soc_est.pack_mv = 0u;
    s_soc_est.ts1_raw = 0u;
    s_soc_est.ts1_mv = 0u;
    s_soc_est.current_ma = 0;
    s_soc_est.last_update_ms = 0u;
    s_soc_est.sample_ms = 0u;
    s_soc_est.source_batch_seq = 0u;
    s_soc_est.last_cc_seq = 0u;
    s_soc_est.real_quiet_since_ms = 0u;
    s_soc_est.update_count = 0u;
    s_soc_est.nvm_save_count = 0u;
    s_soc_est.state_commit_count = 0u;

    /* 顺序固定：标定表 -> SOH 基线 -> 读回 NVM 学习结果。 */
    BMS_CAL_Init();
    SOH_Init();
    if (BMS_NVM_Init() == 0u)
    {
        s_soc_est.nvm_ok = 0u;
    }
    s_soc_est.soh01 = SOH_GetPercent01();
}

void SocEst_OnSnapshot(const bq_snapshot_t *snap, uint32_t now_ms)
{
    uint16_t min_mv;
    int16_t temp_dc = 250;
    uint8_t frame_ok = 1u;
    uint8_t temp_ok;
    uint8_t i;
    bq_batch_quality_t batch;

    (void)now_ms;
    if ((snap == NULL) || (snap->fresh == 0u) || (snap->valid == 0u) ||
        (snap->last_update_ms == s_soc_est.last_update_ms))
    {
        return;
    }
    s_soc_est.last_update_ms = snap->last_update_ms;
    s_soc_est.sample_ms = snap->last_update_ms;
    s_soc_est.source_batch_seq = 0u;
    s_soc_est.quality = SOC_EST_QUALITY_NONE;

    for (i = 0u; i < 3u; i++)
    {
        if ((snap->cell_mv[i] < SOC_EST_CELL_MIN_MV) ||
            (snap->cell_mv[i] > SOC_EST_CELL_MAX_MV))
        {
            frame_ok = 0u;
        }
    }
    if ((snap->current_ma < -50000) || (snap->current_ma > 50000))
    {
        frame_ok = 0u;
    }

    /* 3S 接法错误/悬空通道时 BAT 不等于三节之和；这种帧不要喂 SOC，
     * 否则会把错误的 cell3/包压积分进去。 */
    {
        int32_t sum_mv = (int32_t)snap->cell_mv[0] +
                         (int32_t)snap->cell_mv[1] +
                         (int32_t)snap->cell_mv[2];
        int32_t delta_mv = (int32_t)snap->pack_mv - sum_mv;

        if ((delta_mv > SOC_EST_PACK_SUM_TOL_MV) ||
            (delta_mv < -SOC_EST_PACK_SUM_TOL_MV))
        {
            frame_ok = 0u;
        }
    }

    temp_ok = SocEst_Ts1ToDeciC(snap, &temp_dc);
    if (temp_ok == 0u)
    {
        temp_dc = 250;      /* 温度无效时 SOC 仍按 25°C 占位，SOH 不学习 */
    }
    /* 板级合理温度窗：-40..100°C。超出（典型是 TS1 开路被 5 V 上拉、
     * 或 NTC 短路）判无效，避免把异常 NTC 读数当成真实温度。 */
    if ((temp_dc < -400) || (temp_dc > 1000))
    {
        temp_ok = 0u;
        temp_dc = 250;
    }

    s_soc_est.frame_valid = frame_ok;
    s_soc_est.temp_valid = temp_ok;
    s_soc_est.temp_dc = temp_dc;
    s_soc_est.cell_mv[0] = snap->cell_mv[0];
    s_soc_est.cell_mv[1] = snap->cell_mv[1];
    s_soc_est.cell_mv[2] = snap->cell_mv[2];
    s_soc_est.pack_mv = snap->pack_mv;
    s_soc_est.ts1_raw = snap->ts1_raw;
    s_soc_est.ts1_mv = SocEst_Ts1Mv(snap);
    s_soc_est.current_ma = snap->current_ma;

    if (frame_ok == 0u)
    {
        return;             /* 坏帧不喂算法，SOC 保持上次值 */
    }

    min_mv = snap->cell_mv[0];
    if (snap->cell_mv[1] < min_mv)
    {
        min_mv = snap->cell_mv[1];
    }
    if (snap->cell_mv[2] < min_mv)
    {
        min_mv = snap->cell_mv[2];
    }

    BMS_CAL_Update(temp_dc);
    if (s_soc_est.ekf_reset_pending != 0u)
    {
        /* Re-seed only the EKF; retain the Ah-integral SOC base. */
        SOC_EkfReset();
        s_soc_est.ekf_reset_pending = 0u;
    }
    SOC_AhUpdate((int32_t)min_mv,
                 (int32_t)(SOC_EST_CURRENT_SIGN * snap->current_ma));
    if (temp_ok != 0u)
    {
        SOH_Update((int32_t)min_mv,
                   (int32_t)(SOC_EST_CURRENT_SIGN * snap->current_ma),
                   (int32_t)temp_dc);
    }

    s_soc_est.soc01 = SOC_GetPercent01();
    s_soc_est.soh01 = SOH_GetPercent01();
    s_soc_est.frame_valid = 1u;
    s_soc_est.quality = SOC_EST_QUALITY_REAL;
    BQ_GetBatchQuality(&batch);
    if ((batch.quality == BQ_BATCH_FULL_REAL_OK) &&
        (batch.published_ms == snap->last_update_ms))
    {
        s_soc_est.source_batch_seq = batch.batch_seq;
    }
    if ((SOC_EST_CURRENT_SIGN * snap->current_ma) < SOC_IDLE_CUR_MA &&
        (SOC_EST_CURRENT_SIGN * snap->current_ma) > -SOC_IDLE_CUR_MA)
    {
        if (s_soc_est.real_quiet_count == 0u)
        {
            s_soc_est.real_quiet_since_ms = snap->last_update_ms;
        }
        if (s_soc_est.real_quiet_count < 255u)
        {
            s_soc_est.real_quiet_count++;
        }
    }
    else
    {
        s_soc_est.real_quiet_count = 0u;
        s_soc_est.real_quiet_since_ms = 0u;
    }
    if ((s_soc_est.soc_estimated != 0u) &&
        (s_soc_est.real_quiet_count >= (SOC_QUIET_DEBOUNCE + 1u)) &&
        ((uint32_t)(snap->last_update_ms - s_soc_est.real_quiet_since_ms) >=
         SOC_EST_REAL_RESYNC_MS))
    {
        s_soc_est.soc_estimated = 0u;
    }
    s_soc_est.update_count++;
}

void SocEst_OnPartialSample(const bq_batch_quality_t *batch,
                            const bq_cc_sample_t *cc)
{
    const uint16_t required_fields = (uint16_t)(BQ_FIELD_VC1 | BQ_FIELD_VC2 |
        BQ_FIELD_VC3 | BQ_FIELD_VC4 | BQ_FIELD_CC |
        BQ_FIELD_SYS_STAT | BQ_FIELD_CTRL1 | BQ_FIELD_CTRL2 |
        BQ_FIELD_PROTECTION);
    uint32_t now_ms;
    uint32_t age_ms;
    uint32_t delta_ms;
    uint16_t min_mv;

    if ((batch == NULL) || (cc == NULL) || (batch->complete == 0u) ||
        (batch->quality != BQ_BATCH_PARTIAL_VC5_PACK) ||
        (batch->batch_seq == 0u) || (cc->valid == 0u) ||
        (cc->sample_seq == 0u) || (cc->batch_seq != batch->batch_seq) ||
        (batch->partial_confirm_count < 2u) ||
        ((batch->field_ok_mask & required_fields) != required_fields) ||
        ((batch->crc_ok_mask & required_fields) != required_fields) ||
        (batch->raw_vc3 > 128u) || (batch->raw_vc4 > 128u) ||
        (batch->cell_mv[0] < SOC_EST_CELL_MIN_MV) ||
        (batch->cell_mv[0] > SOC_EST_CELL_MAX_MV) ||
        (batch->cell_mv[1] < SOC_EST_CELL_MIN_MV) ||
        (batch->cell_mv[1] > SOC_EST_CELL_MAX_MV) ||
        ((batch->estimated_mask & (BQ_EST_CELL3 | BQ_EST_PACK)) == 0u) ||
        (((batch->estimated_mask & BQ_EST_CELL3) == 0u) &&
         ((batch->cell_mv[2] < SOC_EST_CELL_MIN_MV) ||
          (batch->cell_mv[2] > SOC_EST_CELL_MAX_MV))) ||
        (((batch->estimated_mask & BQ_EST_PACK) == 0u) &&
         ((batch->pack_mv < 3000u) || (batch->pack_mv > 15000u))) ||
        (cc->current_ma < -50000) || (cc->current_ma > 50000) ||
        (cc->sample_seq == s_soc_est.last_cc_seq))
    {
        return;
    }

    now_ms = BMS_NowMs();
    age_ms = now_ms - cc->sample_ms;
    if ((age_ms > SOC_EST_CC_FRESH_MS) || (age_ms > 0x7FFFFFFFUL))
    {
        return;
    }
    if (s_soc_est.sample_seen != 0u)
    {
        delta_ms = cc->sample_ms - s_soc_est.sample_ms;
        if ((delta_ms == 0u) || (delta_ms > 0x7FFFFFFFUL))
        {
            return;
        }
    }

    min_mv = (batch->cell_mv[0] < batch->cell_mv[1]) ?
             batch->cell_mv[0] : batch->cell_mv[1];
    /* There is no verified TS1 in this API. Keep the last calibration table
     * selection for SOC_AhUpdate, but do not expose it as fresh temperature or
     * pass the partial sample to SOH_Update. */
    BMS_CAL_Update(BMS_CAL_GetTempDc());
    SOC_AhUpdate((int32_t)min_mv,
                 (int32_t)(SOC_EST_CURRENT_SIGN * cc->current_ma));

    s_soc_est.soc01 = SOC_GetPercent01();
    s_soc_est.soh01 = SOH_GetPercent01();
    s_soc_est.quality = SOC_EST_QUALITY_ESTIMATED;
    s_soc_est.soc_estimated = 1u;
    s_soc_est.ekf_reset_pending = 1u;
    s_soc_est.frame_valid = 0u;
    s_soc_est.temp_valid = 0u;
    s_soc_est.temp_dc = 0;
    s_soc_est.cell_mv[0] = batch->cell_mv[0];
    s_soc_est.cell_mv[1] = batch->cell_mv[1];
    s_soc_est.cell_mv[2] = 0u;
    s_soc_est.pack_mv = 0u;
    s_soc_est.ts1_raw = 0u;
    s_soc_est.ts1_mv = 0u;
    s_soc_est.current_ma = cc->current_ma;
    s_soc_est.sample_ms = cc->sample_ms;
    s_soc_est.source_batch_seq = batch->batch_seq;
    s_soc_est.last_cc_seq = cc->sample_seq;
    s_soc_est.sample_seen = 1u;
    s_soc_est.real_quiet_count = 0u;
    s_soc_est.real_quiet_since_ms = 0u;
    s_soc_est.update_count++;
}

void SocEst_InvalidateInput(void)
{
    s_soc_est.quality = SOC_EST_QUALITY_NONE;
    s_soc_est.frame_valid = 0u;
    s_soc_est.temp_valid = 0u;
    s_soc_est.temp_dc = 0;
    s_soc_est.cell_mv[0] = 0u;
    s_soc_est.cell_mv[1] = 0u;
    s_soc_est.cell_mv[2] = 0u;
    s_soc_est.pack_mv = 0u;
    s_soc_est.ts1_raw = 0u;
    s_soc_est.ts1_mv = 0u;
    s_soc_est.current_ma = 0;
    s_soc_est.sample_ms = 0u;
    s_soc_est.source_batch_seq = 0u;
    s_soc_est.real_quiet_count = 0u;
    s_soc_est.real_quiet_since_ms = 0u;
}

void SocEst_Task(void)
{
    BMS_NVM_Task();
    s_soc_est.nvm_save_count = BMS_NVM_GetSaveCount();
}

void SocEst_PrepareForLp(void)
{
    if (BMS_NVM_IsDirty() != 0u)
    {
        (void)BMS_NVM_Save();
    }
}

int SocEst_FillStateStoreView(const soc_est_snapshot_t *captured,
                              const uint16_t measured_cell_mv[3],
                              uint16_t measured_pack_mv, uint16_t archive_flags,
                              state_store_view_t *out)
{
    uint8_t pack_missing;
    uint8_t partial;
    uint8_t soc_estimated;
    uint8_t i;

    if ((captured == NULL) || (measured_cell_mv == NULL) || (out == NULL) ||
        ((archive_flags & (uint16_t)~STATE_ARCHIVE_Q_ALL) != 0u) ||
        ((captured->quality == SOC_EST_QUALITY_NONE) &&
         ((archive_flags & STATE_ARCHIVE_Q_SOC_INVALID) == 0u)))
    {
        return -1;
    }

    partial = (captured->quality == SOC_EST_QUALITY_ESTIMATED) ? 1u : 0u;
    pack_missing = ((archive_flags & STATE_ARCHIVE_Q_PACK_MISSING) != 0u) ? 1u : 0u;
    soc_estimated = (uint8_t)((partial != 0u) ||
                              (captured->soc_estimated != 0u) ||
                              ((archive_flags & STATE_ARCHIVE_Q_SOC_ESTIMATED) != 0u));

    if ((partial != 0u) &&
        ((archive_flags & STATE_ARCHIVE_Q_SOC_ESTIMATED) == 0u))
    {
        return -1;
    }
    if ((partial != 0u) &&
        ((archive_flags & (STATE_ARCHIVE_Q_CELL3_MISSING |
                           STATE_ARCHIVE_Q_PACK_MISSING)) == 0u))
    {
        return -1;
    }
    if (((archive_flags & STATE_ARCHIVE_Q_CELL1_MISSING) != 0u) &&
        (measured_cell_mv[0] != 0u))
    {
        return -1;
    }
    if (((archive_flags & STATE_ARCHIVE_Q_CELL2_MISSING) != 0u) &&
        (measured_cell_mv[1] != 0u))
    {
        return -1;
    }
    if ((archive_flags & STATE_ARCHIVE_Q_CELL3_MISSING) != 0u &&
        (measured_cell_mv[2] != 0u))
    {
        return -1;
    }
    if ((pack_missing != 0u) && (measured_pack_mv != 0u))
    {
        return -1;
    }
    if ((partial == 0u) &&
        (captured->quality != SOC_EST_QUALITY_REAL) &&
        ((archive_flags & STATE_ARCHIVE_Q_SOC_INVALID) == 0u))
    {
        return -1;
    }
    if ((captured->quality == SOC_EST_QUALITY_REAL) &&
        (captured->frame_valid != 0u) &&
        (captured->sample_ms != captured->last_update_ms))
    {
        return -1;
    }
    for (i = 0u; i < 3u; i++)
    {
        if ((measured_cell_mv[i] < SOC_EST_CELL_MIN_MV) ||
            (measured_cell_mv[i] > SOC_EST_CELL_MAX_MV))
        {
            if (!(((i == 0u) &&
                   ((archive_flags & STATE_ARCHIVE_Q_CELL1_MISSING) != 0u)) ||
                  ((i == 1u) &&
                   ((archive_flags & STATE_ARCHIVE_Q_CELL2_MISSING) != 0u)) ||
                  ((i == 2u) &&
                   ((archive_flags & STATE_ARCHIVE_Q_CELL3_MISSING) != 0u))) ||
                (measured_cell_mv[i] != 0u))
            {
                return -1;
            }
        }
    }
    if (pack_missing == 0u &&
        ((measured_pack_mv < 3000u) || (measured_pack_mv > 15000u)))
    {
        return -1;
    }

    StateStore_GetView(out);
    if (StateStore_IsValid() == 0u)
    {
        out->flags = 0u;
        out->battery_epoch = 0u;
    }

    out->cell_mv[0] = measured_cell_mv[0];
    out->cell_mv[1] = measured_cell_mv[1];
    out->cell_mv[2] = measured_cell_mv[2];
    out->pack_mv = measured_pack_mv;
    out->soc01 = (uint16_t)((captured->soc01 < 0) ? 0 :
                            ((captured->soc01 > 10000) ? 10000 :
                             captured->soc01));
    out->soh01 = (uint16_t)((captured->soh01 < 0) ? 0 :
                            ((captured->soh01 > 10000) ? 10000 :
                             captured->soh01));
    out->temp_dc = captured->temp_valid ? captured->temp_dc : 0;

    out->flags &= (uint16_t)~(STATE_FLAG_SOC_VALID | STATE_FLAG_PACK_VALID |
                              STATE_FLAG_SOC_ESTIMATED | STATE_FLAG_PACK_PARTIAL);
    if ((soc_estimated != 0u) ||
        ((archive_flags & STATE_ARCHIVE_Q_SOC_INVALID) != 0u))
    {
        out->flags |= STATE_FLAG_SOC_ESTIMATED;
    }
    if (soc_estimated == 0u && captured->cal_valid != 0u &&
        captured->frame_valid != 0u && captured->temp_valid != 0u)
    {
        out->flags |= STATE_FLAG_SOC_VALID;
    }
    if (pack_missing == 0u)
    {
        /* The caller supplied and range-checked an actual pack measurement.
         * A partial cell3 does not invalidate an independently measured pack. */
        out->flags |= STATE_FLAG_PACK_VALID;
    }
    if ((archive_flags & (STATE_ARCHIVE_Q_CELL3_MISSING |
                          STATE_ARCHIVE_Q_PACK_MISSING)) != 0u)
    {
        out->flags |= STATE_FLAG_PACK_PARTIAL;
    }
    return 0;
}

int SocEst_CommitStateStore(void)
{
    state_store_view_t view;

    StateStore_GetView(&view);
    if (StateStore_IsValid() == 0u)
    {
        view.flags = 0u;
        view.battery_epoch = 0u;
    }

    view.cell_mv[0] = s_soc_est.cell_mv[0];
    view.cell_mv[1] = s_soc_est.cell_mv[1];
    view.cell_mv[2] = s_soc_est.cell_mv[2];
    view.pack_mv = s_soc_est.pack_mv;
    view.soc01 = (uint16_t)((s_soc_est.soc01 < 0) ? 0 :
                            ((s_soc_est.soc01 > 10000) ? 10000 :
                             s_soc_est.soc01));
    view.soh01 = (uint16_t)((s_soc_est.soh01 < 0) ? 0 :
                            ((s_soc_est.soh01 > 10000) ? 10000 :
                             s_soc_est.soh01));
    view.temp_dc = s_soc_est.temp_valid ? s_soc_est.temp_dc : 0;

    view.flags &= (uint16_t)~(STATE_FLAG_SOC_VALID | STATE_FLAG_PACK_VALID |
                              STATE_FLAG_SOC_ESTIMATED | STATE_FLAG_PACK_PARTIAL);
    if ((s_soc_est.cal_valid != 0u) && (s_soc_est.frame_valid != 0u) &&
        (s_soc_est.temp_valid != 0u) && (s_soc_est.soc_estimated == 0u))
    {
        view.flags |= STATE_FLAG_SOC_VALID;
    }
    if ((s_soc_est.frame_valid != 0u) &&
        (s_soc_est.quality == SOC_EST_QUALITY_REAL))
    {
        view.flags |= STATE_FLAG_PACK_VALID;
    }
    if (s_soc_est.soc_estimated != 0u ||
        (s_soc_est.quality == SOC_EST_QUALITY_ESTIMATED))
    {
        view.flags |= STATE_FLAG_SOC_ESTIMATED;
    }
    if (s_soc_est.quality == SOC_EST_QUALITY_ESTIMATED)
    {
        view.flags |= STATE_FLAG_PACK_PARTIAL;
    }

    if (StateStore_Commit(&view) == 0)
    {
        s_soc_est.state_commit_count++;
        return 0;
    }
    return -1;
}

void SocEst_ResetForNewPack(void)
{
    SOH_Init();
    SOC_SetCapacityMAh(SOC_CAPACITY_NOMINAL_MAH);
    SOC_ForceSetPercent01(5000);
    BMS_NVM_MarkDirty();
}

uint8_t SocEst_IsValid(void)
{
    return (uint8_t)((s_soc_est.cal_valid != 0u) &&
                     (s_soc_est.frame_valid != 0u) &&
                     (s_soc_est.temp_valid != 0u) &&
                     (s_soc_est.soc_estimated == 0u));
}

uint8_t SocEst_IsTempValid(void)
{
    return s_soc_est.temp_valid;
}

int32_t SocEst_GetSoc01(void)
{
    return s_soc_est.soc01;
}

uint8_t SocEst_GetSocPercent(void)
{
    return (uint8_t)((s_soc_est.soc01 + 50) / 100);
}

int32_t SocEst_GetSoh01(void)
{
    return s_soc_est.soh01;
}

int16_t SocEst_GetTempDc(void)
{
    return s_soc_est.temp_dc;
}

void SocEst_GetSnapshot(soc_est_snapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->frame_valid = s_soc_est.frame_valid;
    out->temp_valid = s_soc_est.temp_valid;
    out->cal_valid = s_soc_est.cal_valid;
    out->quality = s_soc_est.quality;
    out->soc_estimated = s_soc_est.soc_estimated;
    out->soc01 = s_soc_est.soc01;
    out->soh01 = s_soc_est.soh01;
    out->temp_dc = s_soc_est.temp_dc;
    out->cell_mv[0] = s_soc_est.cell_mv[0];
    out->cell_mv[1] = s_soc_est.cell_mv[1];
    out->cell_mv[2] = s_soc_est.cell_mv[2];
    out->pack_mv = s_soc_est.pack_mv;
    out->ts1_raw = s_soc_est.ts1_raw;
    out->ts1_mv = s_soc_est.ts1_mv;
    out->current_ma = s_soc_est.current_ma;
    out->update_count = s_soc_est.update_count;
    out->last_update_ms = s_soc_est.last_update_ms;
    out->sample_ms = s_soc_est.sample_ms;
    out->source_batch_seq = s_soc_est.source_batch_seq;
}

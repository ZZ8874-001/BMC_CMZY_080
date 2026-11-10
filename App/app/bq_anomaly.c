/**
 ******************************************************************************
 * @file    bq_anomaly.c
 * @brief   BQ 局部电压异常的时限、档案候选和报警协调。
 *
 * 本模块不写 FET，不改变 BQ/Protection 的安全判据，也不实现 OCP。它只消费
 * BQ 已分类的同批证据，提供给上层的条件事实、截止和归档请求。Flash 事务
 * 仍在主循环中，由 ServicePersistence() 调 StateStore；黑匣子由编排层写入。
 ******************************************************************************
 */
#include "app/bq_anomaly.h"

#include "app/state_store.h"
#include "app/faults.h"
#include "board.h"

#include <stddef.h>

#define BQ_ANOMALY_PARTIAL_RETRY_MS       10000u
#define BQ_ANOMALY_DISCHARGE_WINDOW_MS    20000u
#define BQ_ANOMALY_SELFTEST_ALARM_MS      60000u
#define BQ_ANOMALY_BOOT_BLUE_HOLD_MS      2000u
#define BQ_ANOMALY_MARK_WAIT_MS           60000u
#define BQ_ANOMALY_ARCHIVE_WINDOW_MS      1800000u
#define BQ_ANOMALY_ALARM_MS               120000u
#define BQ_ANOMALY_SHIP_TRIGGER_MS        300000u
#define BQ_ANOMALY_FULL_REQUIRED          9u
#define BQ_ANOMALY_CLEAR_CURRENT_MA       500
#define BQ_ANOMALY_CC_MAX_MS              1000u
#define BQ_ANOMALY_MARK_ANOMALY_LIMIT     4u

/* 该模块只由主循环调用，所有字段保持为普通标量，便于 Ozone 展开。 */
static struct
{
    bq_anomaly_boot_t boot_origin;
    uint8_t initialized;
    uint8_t last_batch_quality;
    uint8_t full_seen;
    uint8_t partial_active;
    uint8_t partial_cc_seen;
    uint8_t partial_retry_pending;
    uint8_t recheck_pending;
    uint8_t discharge_timed_out;
    uint8_t alarm_active;
    uint8_t alarm_latch_armed;
    uint8_t alarm_persist_pending;
    uint8_t alarm_cancel_pending;
    uint8_t latch_persist_pending;
    uint8_t latched;
    uint8_t shutdown_confirmed;
    uint8_t shutdown_persist_pending;
    uint8_t manual_clear_requested;
    uint8_t stop2_ship_trigger;
    uint8_t archive_active;
    uint8_t archive_ready;
    uint8_t archive_bb_written;
    uint8_t archive_is_critical;
    uint8_t archive_partial_valid;
    uint8_t premark_partial_valid;
    uint8_t certified_full_valid;
    uint8_t archive_quality;
    uint8_t archive_flags;
    uint8_t archive_mark_count;
    uint8_t archive_full_streak;
    uint8_t full_streak;
    uint32_t certified_full_ms;
    bq_snapshot_t certified_full_snapshot;
    soc_est_snapshot_t certified_full_soc;
    bq_snapshot_t archive_candidate_snapshot;
    soc_est_snapshot_t archive_candidate_soc;
    uint8_t selftest_alarm_raised;
    uint8_t event_pending;
    uint32_t last_batch_seq;
    uint32_t last_published_seq;
    uint32_t last_cc_sample_seq;
    uint32_t last_cc_sample_ms;
    int32_t last_cc_current_ma;
    uint32_t last_full_ms;
    uint32_t partial_since_ms;
    uint32_t partial_retry_due_ms;
    uint32_t recheck_due_ms;
    uint32_t blue_hold_until_ms;
    uint32_t selftest_start_ms;
    uint32_t alarm_start_ms;
    uint32_t mark_started_ms;
    uint32_t mark_epoch;
    uint32_t mark_id;
    uint32_t now_ms;
    uint32_t last_mark_id;
    uint32_t last_mark_epoch;
    bq_snapshot_t last_full_snapshot;
    soc_est_snapshot_t last_full_soc;
    bq_snapshot_t last_partial_snapshot;
    soc_est_snapshot_t last_partial_soc;
    bq_anomaly_inputs_t last_inputs;
} s_bq_anomaly;

static void BqAnomaly_PrepareArchive(uint32_t now_ms);

static uint8_t BqAnomaly_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1u : 0u;
}

static uint32_t BqAnomaly_Age(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

static void BqAnomaly_SetEvent(uint8_t event)
{
    s_bq_anomaly.event_pending |= event;
}

static void BqAnomaly_LoadPersistent(void)
{
    state_store_view_t view;

    view.flags = 0u;
    view.seq = 0u;
    view.battery_epoch = 0u;
    view.cell_mv[0] = 0u;
    view.cell_mv[1] = 0u;
    view.cell_mv[2] = 0u;
    view.pack_mv = 0u;
    view.soc01 = 0u;
    view.soh01 = 0u;
    view.temp_dc = 0;
    view.valid = 0u;
    view.archive.last_mark_epoch = 0u;
    view.archive.last_mark_id = 0u;
    view.archive.anomaly_count = 0u;
    view.archive.alarm_flags = 0u;
    view.archive.estimate_flags = 0u;
    view.archive.mark_quality = 0u;

    StateStore_GetView(&view);
    s_bq_anomaly.archive_mark_count = view.archive.anomaly_count;
    if (s_bq_anomaly.archive_mark_count > BQ_ANOMALY_MARK_ANOMALY_LIMIT)
    {
        s_bq_anomaly.archive_mark_count = BQ_ANOMALY_MARK_ANOMALY_LIMIT;
    }
    s_bq_anomaly.last_mark_epoch = view.archive.last_mark_epoch;
    s_bq_anomaly.last_mark_id = view.archive.last_mark_id;

    if ((StateStore_IsValid() == 0u) &&
        ((Fault_GetAll() & FAULT_BIT_FLASH_STATE) != 0u))
    {
        s_bq_anomaly.latched = 1u;
        s_bq_anomaly.alarm_active = 1u;
        s_bq_anomaly.alarm_latch_armed = 1u;
    }

    if ((view.archive.alarm_flags & STATE_ARCHIVE_ALARM_PENDING) != 0u)
    {
        s_bq_anomaly.alarm_active = 1u;
        s_bq_anomaly.alarm_latch_armed =
            (s_bq_anomaly.archive_mark_count >= BQ_ANOMALY_MARK_ANOMALY_LIMIT) ? 1u : 0u;
        s_bq_anomaly.alarm_start_ms = 0u;
        s_bq_anomaly.alarm_persist_pending = 0u;
    }
    if ((view.archive.alarm_flags & STATE_ARCHIVE_ALARM_LATCHED) != 0u)
    {
        s_bq_anomaly.latched = 1u;
        s_bq_anomaly.shutdown_confirmed =
            ((view.archive.alarm_flags & STATE_ARCHIVE_SHUTDOWN_CONFIRMED) != 0u) ? 1u : 0u;
    }
}

void BqAnomaly_Init(bq_anomaly_boot_t origin, uint32_t now_ms)
{
    s_bq_anomaly.boot_origin = origin;
    s_bq_anomaly.initialized = 1u;
    s_bq_anomaly.full_seen = 0u;
    s_bq_anomaly.partial_active = 0u;
    s_bq_anomaly.partial_cc_seen = 0u;
    s_bq_anomaly.partial_retry_pending = 0u;
    s_bq_anomaly.recheck_pending = 0u;
    s_bq_anomaly.discharge_timed_out = 0u;
    s_bq_anomaly.alarm_active = 0u;
    s_bq_anomaly.alarm_latch_armed = 0u;
    s_bq_anomaly.alarm_persist_pending = 0u;
    s_bq_anomaly.alarm_cancel_pending = 0u;
    s_bq_anomaly.latch_persist_pending = 0u;
    s_bq_anomaly.latched = 0u;
    s_bq_anomaly.shutdown_confirmed = 0u;
    s_bq_anomaly.shutdown_persist_pending = 0u;
    s_bq_anomaly.manual_clear_requested = 0u;
    s_bq_anomaly.stop2_ship_trigger = 0u;
    s_bq_anomaly.archive_active = 0u;
    s_bq_anomaly.archive_ready = 0u;
    s_bq_anomaly.archive_bb_written = 0u;
    s_bq_anomaly.archive_is_critical = 0u;
    s_bq_anomaly.archive_partial_valid = 0u;
    s_bq_anomaly.premark_partial_valid = 0u;
    s_bq_anomaly.certified_full_valid = 0u;
    s_bq_anomaly.certified_full_ms = 0u;
    s_bq_anomaly.archive_quality = 0u;
    s_bq_anomaly.archive_flags = 0u;
    s_bq_anomaly.archive_mark_count = 0u;
    s_bq_anomaly.archive_full_streak = 0u;
    s_bq_anomaly.full_streak = 0u;
    s_bq_anomaly.last_batch_quality = (uint8_t)BQ_BATCH_INVALID;
    s_bq_anomaly.selftest_alarm_raised = 0u;
    s_bq_anomaly.event_pending = 0u;
    s_bq_anomaly.last_batch_seq = 0u;
    s_bq_anomaly.last_published_seq = 0u;
    s_bq_anomaly.last_cc_sample_seq = 0u;
    s_bq_anomaly.last_cc_sample_ms = 0u;
    s_bq_anomaly.last_cc_current_ma = 0;
    s_bq_anomaly.last_full_ms = 0u;
    s_bq_anomaly.partial_since_ms = 0u;
    s_bq_anomaly.partial_retry_due_ms = now_ms + BQ_ANOMALY_PARTIAL_RETRY_MS;
    s_bq_anomaly.recheck_due_ms = now_ms + BQ_ANOMALY_PARTIAL_RETRY_MS;
    s_bq_anomaly.blue_hold_until_ms = now_ms + BQ_ANOMALY_BOOT_BLUE_HOLD_MS;
    s_bq_anomaly.selftest_start_ms = now_ms;
    s_bq_anomaly.alarm_start_ms = 0u;
    s_bq_anomaly.mark_started_ms = 0u;
    s_bq_anomaly.mark_epoch = 0u;
    s_bq_anomaly.mark_id = 0u;
    s_bq_anomaly.now_ms = now_ms;
    s_bq_anomaly.last_mark_epoch = 0u;
    s_bq_anomaly.last_mark_id = 0u;
    s_bq_anomaly.last_full_snapshot.fresh = 0u;
    s_bq_anomaly.last_full_snapshot.valid = 0u;
    s_bq_anomaly.last_full_soc.quality = SOC_EST_QUALITY_NONE;
    s_bq_anomaly.last_partial_snapshot.fresh = 0u;
    s_bq_anomaly.last_partial_snapshot.valid = 0u;
    s_bq_anomaly.last_partial_soc.quality = SOC_EST_QUALITY_NONE;
    s_bq_anomaly.last_inputs.selftest_done = 0u;
    s_bq_anomaly.last_inputs.selftest_passed = 0u;
    s_bq_anomaly.last_inputs.hardware_ok = 0u;
    s_bq_anomaly.last_inputs.link_ok = 0u;
    s_bq_anomaly.last_inputs.comp_charging = 0u;
    s_bq_anomaly.last_inputs.comp_valid = 0u;
    s_bq_anomaly.last_inputs.output_on = 0u;
    s_bq_anomaly.last_inputs.adc_cc_on = 0u;
    s_bq_anomaly.last_inputs.fet_off_verified = 0u;
    s_bq_anomaly.last_inputs.bq_idle = 0u;
    s_bq_anomaly.last_inputs.protection_clear_ok = 0u;

    /* 任意 MCU 复位清本次真实好帧与放电资格；StateStore 的四连状态保留。 */
    BqAnomaly_LoadPersistent();
}

static void BqAnomaly_ProcessFull(uint32_t now_ms,
                                  const bq_batch_quality_t *batch,
                                  const bq_snapshot_t *snap,
                                  const soc_est_snapshot_t *soc)
{
    uint8_t is_new_publish;

    if ((batch == NULL) || (snap == NULL))
    {
        return;
    }
    is_new_publish = (batch->quality == (uint8_t)BQ_BATCH_FULL_REAL_OK) &&
                     (batch->published_seq != 0u) &&
                     (batch->published_seq != s_bq_anomaly.last_published_seq);
    if (is_new_publish == 0u)
    {
        return;
    }

    s_bq_anomaly.last_published_seq = batch->published_seq;
    s_bq_anomaly.full_seen = 1u;
    s_bq_anomaly.premark_partial_valid = 0u;
    s_bq_anomaly.partial_active = 0u;
    s_bq_anomaly.partial_cc_seen = 0u;
    s_bq_anomaly.partial_retry_pending = 0u;
    s_bq_anomaly.recheck_pending = 0u;
    s_bq_anomaly.discharge_timed_out = 0u;
    s_bq_anomaly.last_full_ms = now_ms;
    s_bq_anomaly.last_full_snapshot = *snap;
    if (soc != NULL && soc->quality == SOC_EST_QUALITY_REAL &&
        soc->last_update_ms == snap->last_update_ms &&
        soc->source_batch_seq == batch->batch_seq)
    {
        s_bq_anomaly.last_full_soc = *soc;
    }
    else
    {
        s_bq_anomaly.last_full_soc.quality = SOC_EST_QUALITY_NONE;
    }
    if (s_bq_anomaly.full_streak < BQ_ANOMALY_FULL_REQUIRED)
    {
        s_bq_anomaly.full_streak++;
    }
    if ((s_bq_anomaly.last_full_soc.quality != SOC_EST_QUALITY_REAL) ||
        (s_bq_anomaly.last_full_snapshot.fresh == 0u) ||
        (s_bq_anomaly.last_full_snapshot.valid == 0u))
    {
        s_bq_anomaly.full_streak = 0u;
        if (s_bq_anomaly.archive_ready == 0u)
        {
            s_bq_anomaly.archive_full_streak = 0u;
        }
    }
    if (s_bq_anomaly.full_streak >= BQ_ANOMALY_FULL_REQUIRED &&
        s_bq_anomaly.last_full_soc.quality == SOC_EST_QUALITY_REAL &&
        s_bq_anomaly.archive_active == 0u)
    {
        s_bq_anomaly.certified_full_snapshot = s_bq_anomaly.last_full_snapshot;
        s_bq_anomaly.certified_full_soc = s_bq_anomaly.last_full_soc;
        s_bq_anomaly.certified_full_ms = now_ms;
        s_bq_anomaly.certified_full_valid = 1u;
    }

    if (s_bq_anomaly.full_streak >= BQ_ANOMALY_FULL_REQUIRED &&
        s_bq_anomaly.certified_full_valid != 0u &&
        s_bq_anomaly.alarm_active == 0u &&
        s_bq_anomaly.archive_mark_count != 0u &&
        s_bq_anomaly.latched == 0u)
    {
        s_bq_anomaly.alarm_cancel_pending = 1u;
    }
    if (s_bq_anomaly.archive_active != 0u && s_bq_anomaly.archive_ready == 0u &&
        (s_bq_anomaly.certified_full_valid == 0u ||
         s_bq_anomaly.certified_full_ms > s_bq_anomaly.mark_started_ms) &&
        s_bq_anomaly.last_full_soc.quality == SOC_EST_QUALITY_REAL &&
        s_bq_anomaly.archive_full_streak < BQ_ANOMALY_FULL_REQUIRED)
    {
        s_bq_anomaly.archive_full_streak++;
        if (s_bq_anomaly.archive_full_streak >= BQ_ANOMALY_FULL_REQUIRED)
        {
            s_bq_anomaly.archive_quality = STATE_ARCHIVE_MARK_FULL;
            s_bq_anomaly.archive_flags =
                (s_bq_anomaly.last_full_soc.soc_estimated != 0u) ?
                STATE_ARCHIVE_Q_SOC_ESTIMATED : 0u;
            s_bq_anomaly.archive_ready = 1u;
            s_bq_anomaly.archive_candidate_snapshot = s_bq_anomaly.last_full_snapshot;
            s_bq_anomaly.archive_candidate_soc = s_bq_anomaly.last_full_soc;
            s_bq_anomaly.certified_full_valid = 1u;
            s_bq_anomaly.certified_full_snapshot = s_bq_anomaly.last_full_snapshot;
            s_bq_anomaly.certified_full_soc = s_bq_anomaly.last_full_soc;
            s_bq_anomaly.certified_full_ms = now_ms;
        }
    }

    if (s_bq_anomaly.alarm_active != 0u && s_bq_anomaly.latched == 0u &&
        s_bq_anomaly.full_streak >= BQ_ANOMALY_FULL_REQUIRED &&
        s_bq_anomaly.certified_full_valid != 0u)
    {
        /* StateStore 还未原子清除四连报警；此时只能提出恢复请求，
         * 不能先熄 HOT/放行下一阶段。 */
        s_bq_anomaly.alarm_cancel_pending = 1u;
    }
}

static void BqAnomaly_ProcessPartial(uint32_t now_ms,
                                     const bq_batch_quality_t *batch,
                                     const bq_cc_sample_t *cc,
                                     const bq_snapshot_t *snap,
                                     const soc_est_snapshot_t *soc)
{
    if ((batch == NULL) || (snap == NULL) ||
        (batch->quality != (uint8_t)BQ_BATCH_PARTIAL_VC5_PACK))
    {
        return;
    }
    /* 调用方仅在新批次进入此分支；不可从旧发布帧推断局部电压。 */

    if (s_bq_anomaly.partial_active == 0u)
    {
        s_bq_anomaly.partial_since_ms = now_ms;
        s_bq_anomaly.partial_retry_due_ms = now_ms + BQ_ANOMALY_PARTIAL_RETRY_MS;
        s_bq_anomaly.recheck_due_ms = now_ms + BQ_ANOMALY_PARTIAL_RETRY_MS;
    }
    s_bq_anomaly.partial_active = 1u;
    s_bq_anomaly.full_streak = 0u;
    if (s_bq_anomaly.archive_ready == 0u)
    {
        s_bq_anomaly.archive_full_streak = 0u;
        s_bq_anomaly.archive_partial_valid = 0u;
    }
    s_bq_anomaly.last_partial_snapshot = *snap;
    s_bq_anomaly.last_partial_snapshot.cell_mv[0] = batch->cell_mv[0];
    s_bq_anomaly.last_partial_snapshot.cell_mv[1] = batch->cell_mv[1];
    s_bq_anomaly.last_partial_snapshot.cell_mv[2] = batch->cell_mv[2];
    s_bq_anomaly.last_partial_snapshot.pack_mv = batch->pack_mv;
    s_bq_anomaly.last_partial_snapshot.raw_cell[0] = batch->raw_cell[0];
    s_bq_anomaly.last_partial_snapshot.raw_cell[1] = batch->raw_cell[1];
    s_bq_anomaly.last_partial_snapshot.raw_cell[2] = batch->raw_cell[2];
    s_bq_anomaly.last_partial_snapshot.raw_pack = batch->raw_pack;
    s_bq_anomaly.last_partial_snapshot.cc_raw = ((cc != NULL) && cc->valid != 0u &&
                                                 cc->batch_seq == batch->batch_seq) ?
                                                cc->cc_raw : 0;
    s_bq_anomaly.last_partial_snapshot.ts1_raw = 0u;
    s_bq_anomaly.last_partial_snapshot.current_ma = ((cc != NULL) && cc->valid != 0u &&
                                                      cc->batch_seq == batch->batch_seq) ?
                                                     cc->current_ma : 0;
    s_bq_anomaly.last_partial_snapshot.fresh = 0u;
    s_bq_anomaly.last_partial_snapshot.valid = 0u;
    if ((soc != NULL) && (soc->quality == SOC_EST_QUALITY_ESTIMATED) &&
        (soc->source_batch_seq == batch->batch_seq) &&
        (soc->sample_ms == ((cc != NULL) ? cc->sample_ms : batch->batch_ms)))
    {
        s_bq_anomaly.last_partial_soc = *soc;
        s_bq_anomaly.archive_partial_valid = 1u;
        s_bq_anomaly.premark_partial_valid = 1u;
    }
    else if (s_bq_anomaly.archive_ready == 0u)
    {
        s_bq_anomaly.last_partial_soc.quality = SOC_EST_QUALITY_NONE;
        s_bq_anomaly.archive_partial_valid = 0u;
        s_bq_anomaly.premark_partial_valid = 0u;
    }
    if ((cc != NULL) && (cc->valid != 0u) &&
        (cc->batch_seq == batch->batch_seq) &&
        (cc->sample_ms == batch->batch_ms))
    {
        s_bq_anomaly.partial_cc_seen = 1u;
    }
    else
    {
        s_bq_anomaly.partial_cc_seen = 0u;
    }
}

void BqAnomaly_Task(uint32_t now_ms, const bq_batch_quality_t *batch,
                    const bq_cc_sample_t *cc, const bq_snapshot_t *snap,
                    const soc_est_snapshot_t *soc, const bq_anomaly_inputs_t *in)
{
    uint8_t batch_is_new = 0u;

    if (s_bq_anomaly.initialized == 0u)
    {
        return;
    }
    s_bq_anomaly.now_ms = now_ms;
    if (in != NULL)
    {
        s_bq_anomaly.last_inputs = *in;
    }
    if ((batch != NULL) && (batch->batch_seq != 0u) &&
        (batch->batch_seq != s_bq_anomaly.last_batch_seq))
    {
        s_bq_anomaly.last_batch_quality = batch->quality;
        s_bq_anomaly.last_batch_seq = batch->batch_seq;
        batch_is_new = 1u;
    }

    if (batch_is_new != 0u)
    {
        if (batch->quality == (uint8_t)BQ_BATCH_FULL_REAL_OK)
        {
            BqAnomaly_ProcessFull(now_ms, batch, snap, soc);
        }
        else if (batch->quality == (uint8_t)BQ_BATCH_PARTIAL_VC5_PACK)
        {
            BqAnomaly_ProcessPartial(now_ms, batch, cc, snap, soc);
        }
        else
        {
            s_bq_anomaly.partial_active = 0u;
            s_bq_anomaly.partial_cc_seen = 0u;
            s_bq_anomaly.partial_retry_pending = 0u;
            s_bq_anomaly.recheck_pending = 0u;
            s_bq_anomaly.full_streak = 0u;
            if (s_bq_anomaly.archive_ready == 0u)
            {
                s_bq_anomaly.archive_full_streak = 0u;
                s_bq_anomaly.archive_partial_valid = 0u;
            }
            /* 已认证的九帧候选不因后续坏帧丢失；未经认证的两节数据
             * 一旦变成 INVALID，不可在 60 s 后冒充本次归档候选。 */
            s_bq_anomaly.premark_partial_valid = 0u;
            if (s_bq_anomaly.alarm_active == 0u)
            {
                s_bq_anomaly.alarm_cancel_pending = 0u;
            }
        }
    }

    if ((batch == NULL) || (batch->complete == 0u) ||
        ((now_ms - batch->batch_ms) > BQ_ANOMALY_CC_MAX_MS) ||
        ((Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                            FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                            FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                            FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                            FAULT_BIT_POWER | FAULT_BIT_CLK)) != 0u))
    {
        s_bq_anomaly.partial_active = 0u;
        s_bq_anomaly.partial_cc_seen = 0u;
        s_bq_anomaly.partial_retry_pending = 0u;
        s_bq_anomaly.recheck_pending = 0u;
    }
    if ((cc != NULL) && (batch != NULL) && (cc->valid != 0u) &&
        (cc->sample_seq != s_bq_anomaly.last_cc_sample_seq))
    {
        s_bq_anomaly.last_cc_sample_seq = cc->sample_seq;
        s_bq_anomaly.last_cc_sample_ms = cc->sample_ms;
        s_bq_anomaly.last_cc_current_ma = cc->current_ma;
        if (s_bq_anomaly.partial_active != 0u &&
            cc->batch_seq == s_bq_anomaly.last_batch_seq &&
            cc->sample_ms == batch->batch_ms)
        {
            s_bq_anomaly.partial_cc_seen = 1u;
        }
    }

    if (s_bq_anomaly.full_seen != 0u &&
        s_bq_anomaly.discharge_timed_out == 0u &&
        s_bq_anomaly.last_inputs.output_on != 0u &&
        s_bq_anomaly.last_inputs.comp_charging == 0u &&
        BqAnomaly_Age(now_ms, s_bq_anomaly.last_full_ms) >=
        BQ_ANOMALY_DISCHARGE_WINDOW_MS)
    {
        s_bq_anomaly.discharge_timed_out = 1u;
        if (s_bq_anomaly.alarm_active == 0u)
        {
            s_bq_anomaly.alarm_active = 1u;
            s_bq_anomaly.alarm_latch_armed = 0u;
            s_bq_anomaly.alarm_start_ms = now_ms;
            s_bq_anomaly.alarm_persist_pending = 1u;
            BqAnomaly_SetEvent(BQ_ANOMALY_EVENT_ALARM_START);
        }
    }
    if (s_bq_anomaly.partial_active != 0u &&
        s_bq_anomaly.last_inputs.output_on == 0u &&
        s_bq_anomaly.partial_retry_pending == 0u &&
        BqAnomaly_TimeReached(now_ms, s_bq_anomaly.partial_retry_due_ms) != 0u)
    {
        s_bq_anomaly.partial_retry_pending = 1u;
    }
    if (s_bq_anomaly.partial_active != 0u &&
        s_bq_anomaly.last_inputs.output_on != 0u &&
        BqAnomaly_TimeReached(now_ms, s_bq_anomaly.recheck_due_ms) != 0u)
    {
        s_bq_anomaly.recheck_pending = 1u;
        s_bq_anomaly.recheck_due_ms = now_ms + BQ_ANOMALY_PARTIAL_RETRY_MS;
    }

    BqAnomaly_PrepareArchive(now_ms);

    if ((s_bq_anomaly.boot_origin == BQ_ANOMALY_BOOT_COLD) &&
        s_bq_anomaly.alarm_active == 0u &&
        s_bq_anomaly.full_seen == 0u &&
        s_bq_anomaly.partial_active != 0u &&
        s_bq_anomaly.last_inputs.hardware_ok != 0u &&
        s_bq_anomaly.last_inputs.link_ok != 0u &&
        s_bq_anomaly.selftest_alarm_raised == 0u &&
        BqAnomaly_Age(now_ms, s_bq_anomaly.selftest_start_ms) >=
        BQ_ANOMALY_SELFTEST_ALARM_MS)
    {
        s_bq_anomaly.alarm_active = 1u;
        s_bq_anomaly.alarm_latch_armed = 0u;
        s_bq_anomaly.alarm_start_ms = now_ms;
        s_bq_anomaly.alarm_persist_pending = 1u;
        s_bq_anomaly.selftest_alarm_raised = 1u;
        BqAnomaly_SetEvent(BQ_ANOMALY_EVENT_ALARM_START);
    }

    if (s_bq_anomaly.alarm_active != 0u)
    {
        if (s_bq_anomaly.alarm_start_ms == 0u)
        {
            s_bq_anomaly.alarm_start_ms = now_ms;
        }
        if (s_bq_anomaly.alarm_latch_armed != 0u &&
            BqAnomaly_Age(now_ms, s_bq_anomaly.alarm_start_ms) >=
            BQ_ANOMALY_ALARM_MS)
        {
            if (s_bq_anomaly.latched == 0u)
            {
                s_bq_anomaly.latched = 1u;
                s_bq_anomaly.latch_persist_pending = 1u;
                BqAnomaly_SetEvent(BQ_ANOMALY_EVENT_LATCHED);
            }
        }
        if (BqAnomaly_Age(now_ms, s_bq_anomaly.alarm_start_ms) >=
            BQ_ANOMALY_SHIP_TRIGGER_MS)
        {
            s_bq_anomaly.stop2_ship_trigger = 1u;
        }
    }
}

uint8_t BqAnomaly_BeginNormMark(uint32_t mark_epoch, uint32_t mark_id,
                                uint32_t now_ms)
{
    if (s_bq_anomaly.initialized == 0u || mark_id == 0u ||
        s_bq_anomaly.archive_active != 0u ||
        mark_id <= s_bq_anomaly.last_mark_id)
    {
        return 0u;
    }
    s_bq_anomaly.archive_active = 1u;
    s_bq_anomaly.archive_ready = 0u;
    s_bq_anomaly.archive_bb_written = 0u;
    s_bq_anomaly.archive_is_critical = 0u;
    s_bq_anomaly.archive_partial_valid = s_bq_anomaly.premark_partial_valid;
    s_bq_anomaly.archive_full_streak = 0u;
    s_bq_anomaly.archive_quality = 0u;
    s_bq_anomaly.archive_flags = 0u;
    s_bq_anomaly.mark_epoch = mark_epoch;
    s_bq_anomaly.mark_id = mark_id;
    s_bq_anomaly.mark_started_ms = now_ms;
    if (s_bq_anomaly.certified_full_valid != 0u &&
        (now_ms - s_bq_anomaly.certified_full_ms) <= BQ_ANOMALY_ARCHIVE_WINDOW_MS &&
        s_bq_anomaly.certified_full_snapshot.valid != 0u &&
        s_bq_anomaly.certified_full_soc.quality == SOC_EST_QUALITY_REAL)
    {
        s_bq_anomaly.archive_quality = STATE_ARCHIVE_MARK_FULL;
        s_bq_anomaly.archive_ready = 1u;
        s_bq_anomaly.archive_candidate_snapshot = s_bq_anomaly.certified_full_snapshot;
        s_bq_anomaly.archive_candidate_soc = s_bq_anomaly.certified_full_soc;
        s_bq_anomaly.archive_flags =
            (s_bq_anomaly.certified_full_soc.soc_estimated != 0u) ?
            STATE_ARCHIVE_Q_SOC_ESTIMATED : 0u;
    }
    return 1u;
}

static void BqAnomaly_PrepareArchive(uint32_t now_ms)
{
    uint8_t full_ok;
    uint8_t partial_ok;
    uint8_t flags = 0u;

    if (s_bq_anomaly.archive_active == 0u ||
        s_bq_anomaly.archive_ready != 0u)
    {
        return;
    }
    full_ok = (s_bq_anomaly.archive_full_streak >= BQ_ANOMALY_FULL_REQUIRED) ? 1u : 0u;
    if (full_ok != 0u)
    {
        s_bq_anomaly.archive_quality = STATE_ARCHIVE_MARK_FULL;
        s_bq_anomaly.archive_candidate_snapshot = s_bq_anomaly.certified_full_snapshot;
        s_bq_anomaly.archive_candidate_soc = s_bq_anomaly.certified_full_soc;
        s_bq_anomaly.archive_flags =
            (s_bq_anomaly.certified_full_soc.soc_estimated != 0u) ?
            STATE_ARCHIVE_Q_SOC_ESTIMATED : 0u;
        s_bq_anomaly.archive_ready = 1u;
        return;
    }
    if ((s_bq_anomaly.certified_full_valid != 0u) &&
        (s_bq_anomaly.certified_full_ms <= s_bq_anomaly.mark_started_ms) &&
        ((s_bq_anomaly.mark_started_ms - s_bq_anomaly.certified_full_ms) <=
         BQ_ANOMALY_ARCHIVE_WINDOW_MS) &&
        (s_bq_anomaly.certified_full_snapshot.valid != 0u) &&
        (s_bq_anomaly.certified_full_snapshot.fresh != 0u) &&
        (s_bq_anomaly.certified_full_soc.quality == SOC_EST_QUALITY_REAL))
    {
        s_bq_anomaly.archive_quality = STATE_ARCHIVE_MARK_FULL;
        s_bq_anomaly.archive_candidate_snapshot = s_bq_anomaly.certified_full_snapshot;
        s_bq_anomaly.archive_candidate_soc = s_bq_anomaly.certified_full_soc;
        s_bq_anomaly.archive_flags =
            (s_bq_anomaly.certified_full_soc.soc_estimated != 0u) ?
            STATE_ARCHIVE_Q_SOC_ESTIMATED : 0u;
        s_bq_anomaly.archive_ready = 1u;
        return;
    }
    if (BqAnomaly_Age(now_ms, s_bq_anomaly.mark_started_ms) <
        BQ_ANOMALY_MARK_WAIT_MS)
    {
        return;
    }

    partial_ok = (s_bq_anomaly.archive_partial_valid != 0u &&
                  s_bq_anomaly.last_partial_snapshot.cell_mv[0] != 0u &&
                  s_bq_anomaly.last_partial_snapshot.cell_mv[1] != 0u) ? 1u : 0u;
    if (partial_ok != 0u)
    {
        /* 降级归档两节为本批实测，第三节和包压均按本次缺失置 0。 */
        flags |= STATE_ARCHIVE_Q_CELL3_MISSING | STATE_ARCHIVE_Q_PACK_MISSING;
        s_bq_anomaly.last_partial_snapshot.cell_mv[2] = 0u;
        s_bq_anomaly.last_partial_snapshot.pack_mv = 0u;
        flags |= STATE_ARCHIVE_Q_SOC_ESTIMATED;
    }
    else
    {
        flags = (uint8_t)(STATE_ARCHIVE_Q_CELL1_MISSING |
                          STATE_ARCHIVE_Q_CELL2_MISSING |
                          STATE_ARCHIVE_Q_CELL3_MISSING |
                          STATE_ARCHIVE_Q_PACK_MISSING |
                          STATE_ARCHIVE_Q_SOC_INVALID);
        s_bq_anomaly.archive_quality = STATE_ARCHIVE_MARK_CRITICAL;
        s_bq_anomaly.archive_is_critical = 1u;
        s_bq_anomaly.last_partial_snapshot.cell_mv[0] = 0u;
        s_bq_anomaly.last_partial_snapshot.cell_mv[1] = 0u;
        s_bq_anomaly.last_partial_snapshot.cell_mv[2] = 0u;
        s_bq_anomaly.last_partial_snapshot.pack_mv = 0u;
        s_bq_anomaly.last_partial_snapshot.cc_raw = 0;
        s_bq_anomaly.last_partial_snapshot.ts1_raw = 0u;
        s_bq_anomaly.last_partial_snapshot.fresh = 0u;
        s_bq_anomaly.last_partial_snapshot.valid = 0u;
        s_bq_anomaly.last_partial_soc.quality = SOC_EST_QUALITY_NONE;
    }
    if (partial_ok != 0u)
    {
        s_bq_anomaly.archive_quality = STATE_ARCHIVE_MARK_DEGRADED;
        s_bq_anomaly.archive_is_critical = 0u;
    }
    s_bq_anomaly.archive_flags = flags;
    s_bq_anomaly.archive_ready = 1u;
}

uint8_t BqAnomaly_GetArchiveRequest(bq_anomaly_archive_t *out)
{
    if (out == NULL || s_bq_anomaly.archive_active == 0u)
    {
        return 0u;
    }
    /* The parent calls this after Task with its current HAL ms unavailable here;
     * archive readiness is prepared on each Task using the mark start time. */
    if (s_bq_anomaly.archive_ready == 0u)
    {
        return 0u;
    }
    out->mark_epoch = s_bq_anomaly.mark_epoch;
    out->mark_id = s_bq_anomaly.mark_id;
    out->mark_quality = s_bq_anomaly.archive_quality;
    out->estimate_flags = s_bq_anomaly.archive_flags;
    out->bb_written = s_bq_anomaly.archive_bb_written;
    if (s_bq_anomaly.archive_quality == STATE_ARCHIVE_MARK_FULL)
    {
        out->bq = s_bq_anomaly.archive_candidate_snapshot;
        out->soc = s_bq_anomaly.archive_candidate_soc;
    }
    else
    {
        out->bq = s_bq_anomaly.last_partial_snapshot;
        out->soc = s_bq_anomaly.last_partial_soc;
        if ((out->estimate_flags & STATE_ARCHIVE_Q_CELL1_MISSING) != 0u)
        {
            out->bq.cell_mv[0] = 0u;
        }
        if ((out->estimate_flags & STATE_ARCHIVE_Q_CELL2_MISSING) != 0u)
        {
            out->bq.cell_mv[1] = 0u;
        }
        if ((out->estimate_flags & STATE_ARCHIVE_Q_CELL3_MISSING) != 0u)
        {
            out->bq.cell_mv[2] = 0u;
        }
        if ((out->estimate_flags & STATE_ARCHIVE_Q_PACK_MISSING) != 0u)
        {
            out->bq.pack_mv = 0u;
        }
        out->bq.valid = 0u;
        out->bq.fresh = 0u;
    }
    return 1u;
}

void BqAnomaly_NoteBlackBoxWritten(uint32_t mark_id)
{
    if (s_bq_anomaly.archive_active != 0u && mark_id == s_bq_anomaly.mark_id)
    {
        s_bq_anomaly.archive_bb_written = 1u;
    }
}

void BqAnomaly_ArchiveCommitted(uint32_t mark_id)
{
    if (s_bq_anomaly.archive_active == 0u || mark_id != s_bq_anomaly.mark_id)
    {
        return;
    }
    s_bq_anomaly.last_mark_id = mark_id;
    s_bq_anomaly.last_mark_epoch = s_bq_anomaly.mark_epoch;
    if (s_bq_anomaly.archive_quality == STATE_ARCHIVE_MARK_FULL)
    {
        s_bq_anomaly.archive_mark_count = 0u;
        if (s_bq_anomaly.alarm_active == 0u)
        {
            s_bq_anomaly.full_streak = 0u;
            s_bq_anomaly.alarm_cancel_pending = 0u;
        }
    }
    else if (s_bq_anomaly.archive_quality == STATE_ARCHIVE_MARK_CRITICAL)
    {
        s_bq_anomaly.archive_mark_count = BQ_ANOMALY_MARK_ANOMALY_LIMIT;
    }
    else if (s_bq_anomaly.archive_mark_count < BQ_ANOMALY_MARK_ANOMALY_LIMIT)
    {
        s_bq_anomaly.archive_mark_count++;
    }
    if ((s_bq_anomaly.archive_is_critical != 0u ||
         s_bq_anomaly.archive_mark_count >= BQ_ANOMALY_MARK_ANOMALY_LIMIT) &&
        s_bq_anomaly.alarm_active == 0u)
    {
        s_bq_anomaly.alarm_active = 1u;
        s_bq_anomaly.alarm_latch_armed = 1u;
        s_bq_anomaly.alarm_start_ms = s_bq_anomaly.now_ms;
        s_bq_anomaly.alarm_persist_pending = 1u;
        BqAnomaly_SetEvent(BQ_ANOMALY_EVENT_ALARM_START);
    }
    else if ((s_bq_anomaly.archive_is_critical != 0u ||
              s_bq_anomaly.archive_mark_count >= BQ_ANOMALY_MARK_ANOMALY_LIMIT) &&
             s_bq_anomaly.alarm_latch_armed == 0u &&
             s_bq_anomaly.latched == 0u)
    {
        /* 先前 20 s/启动报警不计作“四个档案点”两分钟倒计时。 */
        s_bq_anomaly.alarm_latch_armed = 1u;
        s_bq_anomaly.alarm_start_ms = s_bq_anomaly.now_ms;
    }
    s_bq_anomaly.archive_active = 0u;
    s_bq_anomaly.archive_ready = 0u;
    s_bq_anomaly.archive_bb_written = 0u;
    s_bq_anomaly.archive_is_critical = 0u;
    s_bq_anomaly.archive_full_streak = 0u;
    s_bq_anomaly.premark_partial_valid = 0u;
    s_bq_anomaly.certified_full_valid = 0u;
}

void BqAnomaly_NotifyLpFullMarkCommitted(uint32_t mark_id, uint32_t mark_epoch)
{
    if ((mark_id == 0u) || (mark_id == s_bq_anomaly.last_mark_id))
    {
        return;
    }
    s_bq_anomaly.last_mark_id = mark_id;
    s_bq_anomaly.last_mark_epoch = mark_epoch;
    s_bq_anomaly.archive_mark_count = 0u;
    s_bq_anomaly.archive_full_streak = 0u;
    s_bq_anomaly.certified_full_valid = 0u;
    s_bq_anomaly.full_streak = 0u;
    if (s_bq_anomaly.alarm_active == 0u)
    {
        s_bq_anomaly.alarm_cancel_pending = 0u;
    }
}

void BqAnomaly_NotifyPartialRetry(uint32_t now_ms, uint8_t started)
{
    s_bq_anomaly.partial_retry_pending = 0u;
    s_bq_anomaly.partial_retry_due_ms = now_ms + BQ_ANOMALY_PARTIAL_RETRY_MS;
    if (started != 0u)
    {
        s_bq_anomaly.partial_active = 0u;
        s_bq_anomaly.partial_since_ms = 0u;
        s_bq_anomaly.partial_cc_seen = 0u;
        s_bq_anomaly.recheck_pending = 0u;
    }
}

uint8_t BqAnomaly_TakeRecheckDue(void)
{
    uint8_t pending = s_bq_anomaly.recheck_pending;
    s_bq_anomaly.recheck_pending = 0u;
    return pending;
}

int BqAnomaly_ServicePersistence(void)
{
    int result;

    if (s_bq_anomaly.last_inputs.bq_idle == 0u)
    {
        return 0;
    }
    if ((Fault_GetAll() & FAULT_BIT_FLASH_STATE) != 0u)
    {
        s_bq_anomaly.latched = 1u;
        return -1;
    }
    if (s_bq_anomaly.alarm_cancel_pending != 0u &&
        s_bq_anomaly.latched == 0u &&
        s_bq_anomaly.full_streak >= BQ_ANOMALY_FULL_REQUIRED &&
        s_bq_anomaly.certified_full_valid != 0u)
    {
        result = StateStore_ConfirmArchiveRecovery();
        if (result == 0)
        {
            uint8_t was_alarming = s_bq_anomaly.alarm_active;

            s_bq_anomaly.alarm_cancel_pending = 0u;
            s_bq_anomaly.alarm_active = 0u;
            s_bq_anomaly.alarm_latch_armed = 0u;
            s_bq_anomaly.alarm_start_ms = 0u;
            s_bq_anomaly.stop2_ship_trigger = 0u;
            s_bq_anomaly.alarm_persist_pending = 0u;
            s_bq_anomaly.archive_mark_count = 0u;
            if (was_alarming != 0u)
            {
                BqAnomaly_SetEvent(BQ_ANOMALY_EVENT_ALARM_CANCEL);
            }
            return 1;
        }
        return -1;
    }
    if (s_bq_anomaly.manual_clear_requested != 0u &&
        s_bq_anomaly.latched != 0u &&
        s_bq_anomaly.latch_persist_pending == 0u &&
        s_bq_anomaly.shutdown_confirmed != 0u &&
        s_bq_anomaly.last_inputs.protection_clear_ok != 0u &&
        s_bq_anomaly.last_inputs.fet_off_verified != 0u &&
        s_bq_anomaly.last_inputs.hardware_ok != 0u &&
        s_bq_anomaly.last_inputs.comp_valid != 0u &&
        s_bq_anomaly.last_inputs.comp_charging == 0u &&
        s_bq_anomaly.last_inputs.adc_cc_on != 0u &&
        s_bq_anomaly.last_inputs.link_ok != 0u &&
        s_bq_anomaly.last_inputs.selftest_passed != 0u &&
        s_bq_anomaly.last_cc_sample_seq != 0u &&
        BqAnomaly_Age(s_bq_anomaly.now_ms, s_bq_anomaly.last_cc_sample_ms) <=
            BQ_ANOMALY_CC_MAX_MS &&
        s_bq_anomaly.last_cc_current_ma >= -BQ_ANOMALY_CLEAR_CURRENT_MA &&
        s_bq_anomaly.last_cc_current_ma <= BQ_ANOMALY_CLEAR_CURRENT_MA &&
        s_bq_anomaly.full_streak >= BQ_ANOMALY_FULL_REQUIRED &&
        s_bq_anomaly.certified_full_valid != 0u)
    {
        result = StateStore_SafeClearArchiveAlarm();
        if (result == 0)
        {
            s_bq_anomaly.manual_clear_requested = 0u;
            s_bq_anomaly.latched = 0u;
            s_bq_anomaly.shutdown_confirmed = 0u;
            s_bq_anomaly.shutdown_persist_pending = 0u;
            s_bq_anomaly.alarm_active = 0u;
            s_bq_anomaly.alarm_latch_armed = 0u;
            s_bq_anomaly.latch_persist_pending = 0u;
            s_bq_anomaly.alarm_persist_pending = 0u;
            s_bq_anomaly.archive_mark_count = 0u;
            BqAnomaly_SetEvent(BQ_ANOMALY_EVENT_MANUAL_CLEAR);
            return 1;
        }
        return -1;
    }
    if (s_bq_anomaly.latch_persist_pending != 0u)
    {
        result = StateStore_SetArchiveAlarm((uint8_t)(STATE_ARCHIVE_ALARM_PENDING |
                                                      STATE_ARCHIVE_ALARM_LATCHED));
        if (result == 0)
        {
            s_bq_anomaly.latch_persist_pending = 0u;
            s_bq_anomaly.alarm_persist_pending = 0u;
            return 1;
        }
        return -1;
    }
    if ((s_bq_anomaly.latched != 0u) &&
        (s_bq_anomaly.shutdown_confirmed == 0u) &&
        (s_bq_anomaly.last_inputs.fet_off_verified != 0u))
    {
        s_bq_anomaly.shutdown_persist_pending = 1u;
    }
    if (s_bq_anomaly.shutdown_persist_pending != 0u)
    {
        result = StateStore_SetArchiveAlarm(STATE_ARCHIVE_SHUTDOWN_CONFIRMED);
        if (result == 0)
        {
            s_bq_anomaly.shutdown_confirmed = 1u;
            s_bq_anomaly.shutdown_persist_pending = 0u;
            return 1;
        }
        return -1;
    }
    if (s_bq_anomaly.alarm_persist_pending != 0u)
    {
        result = StateStore_SetArchiveAlarm(STATE_ARCHIVE_ALARM_PENDING);
        if (result == 0)
        {
            s_bq_anomaly.alarm_persist_pending = 0u;
            return 1;
        }
        return -1;
    }
    return 0;
}

void BqAnomaly_RequestManualSafeClear(void)
{
    s_bq_anomaly.manual_clear_requested = 1u;
}

void BqAnomaly_GetStatus(bq_anomaly_status_t *out)
{
    uint32_t now_ms = s_bq_anomaly.now_ms;

    if (out == NULL)
    {
        return;
    }
    out->batch_quality = s_bq_anomaly.last_batch_quality;
    out->full_seen_this_boot = s_bq_anomaly.full_seen;
    out->partial_active = s_bq_anomaly.partial_active;
    out->discharge_window_ok = (s_bq_anomaly.full_seen != 0u &&
                                s_bq_anomaly.partial_active != 0u &&
                                s_bq_anomaly.discharge_timed_out == 0u &&
                                s_bq_anomaly.partial_cc_seen != 0u &&
                                BqAnomaly_Age(now_ms, s_bq_anomaly.last_cc_sample_ms) <=
                                BQ_ANOMALY_CC_MAX_MS) ? 1u : 0u;
    out->discharge_timed_out = s_bq_anomaly.discharge_timed_out;
    out->partial_cc_fresh = (s_bq_anomaly.partial_cc_seen != 0u &&
                             BqAnomaly_Age(now_ms, s_bq_anomaly.last_cc_sample_ms) <=
                             BQ_ANOMALY_CC_MAX_MS) ? 1u : 0u;
    out->partial_current_ma = s_bq_anomaly.last_cc_current_ma;
    out->partial_cc_sample_ms = s_bq_anomaly.last_cc_sample_ms;
    out->discharge_remaining_ms = (s_bq_anomaly.full_seen != 0u &&
                                    s_bq_anomaly.discharge_timed_out == 0u &&
                                    BqAnomaly_Age(now_ms, s_bq_anomaly.last_full_ms) <
                                    BQ_ANOMALY_DISCHARGE_WINDOW_MS) ?
                                   (BQ_ANOMALY_DISCHARGE_WINDOW_MS -
                                    (now_ms - s_bq_anomaly.last_full_ms)) : 0u;
    out->boot_blue_hold = ((s_bq_anomaly.boot_origin == BQ_ANOMALY_BOOT_COLD) &&
                           (s_bq_anomaly.alarm_active == 0u) &&
                           (s_bq_anomaly.full_seen == 0u ||
                            BqAnomaly_TimeReached(now_ms,
                                s_bq_anomaly.blue_hold_until_ms) == 0u)) ? 1u : 0u;
    out->alarm_active = s_bq_anomaly.alarm_active;
    out->hot_flash = (s_bq_anomaly.partial_active != 0u ||
                      s_bq_anomaly.alarm_active != 0u) ? 1u : 0u;
    out->latch_due = (s_bq_anomaly.alarm_active != 0u &&
                      s_bq_anomaly.alarm_latch_armed != 0u &&
                      s_bq_anomaly.latched == 0u &&
                      s_bq_anomaly.alarm_start_ms != 0u &&
                      BqAnomaly_Age(now_ms, s_bq_anomaly.alarm_start_ms) >=
                      BQ_ANOMALY_ALARM_MS) ? 1u : 0u;
    out->buzzer_active = (s_bq_anomaly.alarm_active != 0u &&
                          s_bq_anomaly.alarm_start_ms != 0u &&
                          BqAnomaly_Age(now_ms, s_bq_anomaly.alarm_start_ms) <
                          BQ_ANOMALY_ALARM_MS) ? 1u : 0u;
    out->output_inhibit_latched = s_bq_anomaly.latched;
    out->shutdown_confirmed = s_bq_anomaly.shutdown_confirmed;
    out->auto_lp_blocked = (s_bq_anomaly.latched != 0u ||
                            s_bq_anomaly.alarm_active != 0u ||
                            s_bq_anomaly.archive_active != 0u ||
                            s_bq_anomaly.alarm_persist_pending != 0u ||
                            s_bq_anomaly.latch_persist_pending != 0u ||
                            s_bq_anomaly.shutdown_persist_pending != 0u ||
                            (Fault_GetAll() & FAULT_BIT_FLASH_STATE) != 0u) ? 1u : 0u;
    out->retry_due_off = s_bq_anomaly.partial_retry_pending;
    out->recheck_due_on = s_bq_anomaly.recheck_pending;
    out->archive_pending = s_bq_anomaly.archive_active;
    out->stop2_ship_trigger = s_bq_anomaly.stop2_ship_trigger;
    out->manual_clear_requested = s_bq_anomaly.manual_clear_requested;
    out->anomaly_mark_count = s_bq_anomaly.archive_mark_count;
    out->full_streak = s_bq_anomaly.full_streak;
    out->last_full_ms = s_bq_anomaly.last_full_ms;
    out->alarm_elapsed_ms = s_bq_anomaly.alarm_active != 0u ?
                            (now_ms - s_bq_anomaly.alarm_start_ms) : 0u;
    out->last_mark_id = s_bq_anomaly.last_mark_id;
}

uint8_t BqAnomaly_TakeEvent(void)
{
    uint8_t event = s_bq_anomaly.event_pending;
    s_bq_anomaly.event_pending = 0u;
    return event;
}

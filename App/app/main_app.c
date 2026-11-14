/**
 ******************************************************************************
 * @file    main_app.c
 * @brief   应用编排：App_Init / App_Loop / 250 Hz TIM2 时基。
 *
 * 初始化顺序（无时间预算，允许保守阻塞）：
 *   复位原因 → 故障图 → 电源模式（HSI16 → HSE/PLL 80 MHz；RTC 源由 LP_RTC_USE_LSE 选择）
 *   → 黑匣子 → 5 V 上电 → 日志 → BQ → WS2812 → 状态灯 → 蜂鸣器
 *   → 自检 → 状态机 → COMP2 → TIM2（最后启动）。
 *
 * ISR 约束：TIM2 中断只读 COMP2、推进灯相位、累计秒数并置调度标志；
 * 所有通信/Flash 动作留在主循环。
 ******************************************************************************
 */
#include "app/main_app.h"

#include "board.h"
#include "app/black_box.h"
#include "app/lp_cycle.h"
#include "app/lp_session.h"
#include "app/rtc_time.h"
#include "app/cal_stream.h"
#include "app/charge_policy.h"
#include "app/faults.h"
#include "app/led_status.h"
#include "app/power_mode.h"
#include "app/protection.h"
#include "app/reset_cause.h"
#include "app/self_test.h"
#include "app/state_machine.h"
#include "app/state_store.h"
#include "app/watchdog.h"
#include "bsp/bq_ic.h"
#include "bsp/button.h"
#include "bsp/buzzer.h"
#include "bsp/ws2812_uart.h"
#include "comm/app_log.h"
#include "comm/app_telemetry.h"
#include "detect/gesture.h"
#include "detect/soc_est.h"

#include <string.h>

#define APP_TIM2_PRESCALER          319u    /* 80 MHz / 320 = 250 kHz */
#define APP_TIM2_AUTORELOAD         999u    /* 250 kHz / 1000 = 250 Hz */
#define APP_TIM2_IRQ_PRIORITY       2u
#define APP_COMP_DEBOUNCE_SAMPLES   32u     /* 32 × 4 ms = 128 ms */
#define APP_500MS_TICKS             (APP_BQ_READ_PERIOD_MS / APP_TICK_MS)
#define APP_1S_TICKS                (1000u / APP_TICK_MS)
#define APP_30MIN_TICKS             (APP_BB_PERIOD_MS / APP_TICK_MS)
#define APP_DIAG_LOG_PERIOD_MS      10000u
#define APP_DEGRADED_BLINK_MS       250u
#define APP_5V_STARTUP_DELAY_MS     20u
#define APP_IDLE_LP_MS              10000u   /* 无操作/无充电 10 s 自动进 LP */
#define APP_BOOT_VOLTAGE_HOLD_MS    2000u

/* VC5/BAT 异常恢复：同值重写 SYS_CTRL2（假开关）。 */
#define APP_FAKE_SWITCH_RETRY_MS        2500u
/* 门控跳过用短重试：利用 500 ms BQ 轮询之间的空闲窗口；实际动作仍 >=2.5 s。 */
#define APP_FAKE_SWITCH_GATE_RETRY_MS   50u
#define APP_FAKE_SWITCH_LP_LEAD_MS      3000u

#define APP_FAKE_SWITCH_RESULT_NONE     0u
#define APP_FAKE_SWITCH_RESULT_OK       1u
#define APP_FAKE_SWITCH_RESULT_FAIL     2u
#define APP_FAKE_SWITCH_RESULT_BUSY     3u
#define APP_FAKE_SWITCH_RESULT_GATED    4u
#define APP_FAKE_SWITCH_RESULT_LP       5u

/* s_app.auto_lp_block：自动 LP 被哪一项条件挡住（Ozone 直接看）。 */
#define APP_LP_BLOCK_CHARGING       (1u << 0)
#define APP_LP_BLOCK_BUTTON         (1u << 1)
#define APP_LP_BLOCK_TEST           (1u << 2)   /* 自检未完成/失败 */
#define APP_LP_BLOCK_STATE          (1u << 3)   /* 状态机非 NORM */
#define APP_LP_BLOCK_BQ             (1u << 4)   /* BQ 不新鲜 */
#define APP_LP_BLOCK_TIMEBASE       (1u << 5)   /* RTC 时基（LP_RTC_USE_LSE 选中源）不可用 */
#define APP_LP_BLOCK_POWER          (1u << 6)   /* 电源模式非 NORM */
#define APP_LP_BLOCK_POLICY         (1u << 7)   /* CHARGE/DISCHARGE/CAL 或 CAL pending */
#define APP_LP_BLOCK_PROTECTION     (1u << 8)   /* 保护监督层尚未恢复 */
#define APP_LP_BLOCK_FLASH_MARK     (1u << 9)   /* 30 min 归档/LP 标记尚未完成 */

static struct
{
    /* 时基（TIM2 中断写） */
    volatile uint32_t uptime_s;
    uint32_t uptime_base_s;

    /* 调度标志（中断置位，主循环消费） */
    volatile uint8_t flag_bq_read_request;
    volatile uint8_t flag_diag_request;
    volatile uint8_t flag_blackbox_request;

    /* 中断内分频器 */
    uint16_t prescale_500ms;
    uint16_t prescale_1s;
    uint32_t prescale_30min;

    /* COMP2 方向消抖（中断写） */
    volatile uint8_t comparator_charging;   /* 1 = 充电（COMP_BTTN > COMP_PACK） */
    uint8_t comparator_last_sample;
    uint8_t comparator_match_count;

    /* 运行时/调试可见状态 */
    uint32_t diag_log_ms;
    uint32_t last_activity_ms;      /* 自动 LP 的活动计时基准 */
    uint16_t auto_lp_block;         /* 自动 LP 阻止原因位图（[DIAG] 使用） */

    /* 降级模式（80 MHz 切换失败）灯闪 */
    uint32_t degraded_blink_ms;
    uint8_t degraded_blink_on;

    /* 30 min 普通运行标记与双 Flash 提交的 RAM 重试缓存。 */
    uint8_t norm_mark_active;
    uint8_t archive_bb_written;
    uint8_t archive_cache_valid;
    uint32_t archive_mark_id;
    bq_snapshot_t archive_snap;
    soc_est_snapshot_t archive_soc;
    uint8_t archive_cell3_valid;
    uint8_t archive_pack_valid;

    /* LP/NORM 标记交接。 */
    uint32_t pending_lp_mark_epoch;
    uint8_t pending_lp_mark_active;
    uint8_t pending_lp_mark_lp_archived;

    /* VC5/BAT 异常 -> 同值重写 SYS_CTRL2（RAM only）。 */
    uint32_t fake_switch_seen_batch_seq;
    uint32_t fake_switch_next_ms;
    uint32_t fake_switch_attempt_count;
    uint32_t fake_switch_ok_count;
    uint32_t fake_switch_fail_count;
    uint32_t fake_switch_skip_busy_count;
    uint32_t fake_switch_skip_gate_count;
    uint8_t fake_switch_anomaly;
    uint8_t fake_switch_last_result;
    uint8_t fake_switch_last_mask;
} s_app;

/* ---------------- 内部工具 ---------------- */

static void App_TickTimerInit(void)
{
    /* CubeMX 只生成了占位配置（PSC=0/ARR=max），时基参数由 App 覆写。 */
    htim2.Instance->PSC = APP_TIM2_PRESCALER;
    htim2.Instance->ARR = APP_TIM2_AUTORELOAD;
    htim2.Instance->CNT = 0u;
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    HAL_NVIC_SetPriority(TIM2_IRQn, APP_TIM2_IRQ_PRIORITY, 0u);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void App_TickStart(void)
{
    htim2.Instance->CNT = 0u;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    (void)HAL_TIM_Base_Start_IT(&htim2);
}

uint32_t App_GetUptimeSeconds(void)
{
    return s_app.uptime_base_s + s_app.uptime_s;
}

uint16_t App_GetFaultFlags(void)
{
    return Fault_GetAll();
}

static void App_OnTick250Hz(void)
{
    uint8_t sample;


    /* COMP2：32 个连续一致样本才更新方向（128 ms ≈ 5τ，τ≈24 ms）。 */
    sample = (HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH) ? 1u : 0u;
    if (sample == s_app.comparator_last_sample)
    {
        if (s_app.comparator_match_count < 0xFFu)
        {
            s_app.comparator_match_count++;
        }
    }
    else
    {
        s_app.comparator_last_sample = sample;
        s_app.comparator_match_count = 0u;
    }
    if ((s_app.comparator_match_count >= APP_COMP_DEBOUNCE_SAMPLES) &&
        (s_app.comparator_charging != sample))
    {
        s_app.comparator_charging = sample;
    }

    LedStatus_Tick();

    s_app.prescale_500ms++;
    if (s_app.prescale_500ms >= APP_500MS_TICKS)
    {
        s_app.prescale_500ms = 0u;
        s_app.flag_bq_read_request = 1u;
    }

    s_app.prescale_1s++;
    if (s_app.prescale_1s >= APP_1S_TICKS)
    {
        s_app.prescale_1s = 0u;
        s_app.uptime_s++;
        s_app.flag_diag_request = 1u;
    }

    s_app.prescale_30min++;
    if (s_app.prescale_30min >= APP_30MIN_TICKS)
    {
        s_app.prescale_30min = 0u;
        s_app.flag_blackbox_request = 1u;
    }
}

static void App_UpdateLedMode(const soc_est_snapshot_t *soc_snap)
{
    led_flow_t flow;
    charge_policy_state_t policy_state;
    uint32_t now_ms = HAL_GetTick();
    uint8_t soc_display_valid = 0u;

    if (soc_snap != NULL)
    {
        soc_display_valid =
            ((soc_snap->frame_valid != 0u) &&
             (soc_snap->quality == SOC_EST_QUALITY_REAL)) ? 1u : 0u;
        LedStatus_SetSoc(soc_snap->soc01, soc_display_valid);
    }

    if (StateMachine_GetState() == SM_STATE_FAULT)
    {
        LedStatus_SetMode(LED_MODE_FAULT, LED_FLOW_FAULT);
        return;
    }

    if ((SelfTest_IsDone() != 0u) && (SelfTest_IsPassed() != 0u) &&
        ((now_ms - SelfTest_GetCompletionMs()) < APP_BOOT_VOLTAGE_HOLD_MS))
    {
        LedStatus_SetMode(LED_MODE_BOOT, LED_FLOW_STANDBY);
        return;
    }

    policy_state = ChargePolicy_GetState();
    if (policy_state == CHARGE_POLICY_CAL)
    {
        /* CAL 允许绕过自检失败；BQ 就绪性由策略层保证。 */
        LedStatus_SetMode(LED_MODE_NORM, LED_FLOW_CAL);
        return;
    }

    if ((SelfTest_IsDone() != 0u) && (SelfTest_IsPassed() == 0u) &&
        ((SelfTest_GetFailMask() & (uint16_t)~SELFTEST_FAIL_BQ_DATA) != 0u ||
         (Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                            FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                            FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                            FAULT_BIT_POWER | FAULT_BIT_CLK)) != 0u))
    {
        LedStatus_SetMode(LED_MODE_FAULT, LED_FLOW_FAULT);
        return;
    }

    if ((Protection_IsLowBattery() != 0u) ||
        (ChargePolicy_GetState() == CHARGE_POLICY_LOW_BATTERY_CHARGE))
    {
        LedStatus_SetMode(LED_MODE_NORM, LED_FLOW_LOW_BATT);
        return;
    }

    switch (StateMachine_GetState())
    {
    case SM_STATE_FAULT:
        LedStatus_SetMode(LED_MODE_FAULT, LED_FLOW_FAULT);
        break;

    case SM_STATE_BOOT:
        LedStatus_SetMode(LED_MODE_BOOT, LED_FLOW_STANDBY);
        break;

    case SM_STATE_NORM:
    default:
        if (policy_state == CHARGE_POLICY_CHARGE)
        {
            flow = (Protection_GetChargeFull() != 0u) ?
                   LED_FLOW_CHARGE_FULL : LED_FLOW_CHARGING;
        }
        else if (policy_state == CHARGE_POLICY_DISCHARGE)
        {
            flow = LED_FLOW_DISCHARGING;
        }
        else
        {
            flow = LED_FLOW_STANDBY;
        }
        LedStatus_SetMode(LED_MODE_NORM, flow);
        break;
    }
}

static void App_BqAdcEventTask(void)
{
    uint8_t events;

    /* Flash 写只能在主循环、BQ 空闲且不处于恢复回合时进行；
     * 事件位保留在 bq_ic，等到可写窗口再落盘。 */
    if ((BQ_IsIdle() == 0u) || (BQ_GetState() != BQ_ST_RUN) ||
        (BQ_IsRecovering() != 0u) ||
        (PowerMode_GetState() != POWER_STATE_NORM))
    {
        return;
    }

    events = BQ_TakeAdcEvent();
    if (events == 0u)
    {
        return;
    }

    if ((events & BQ_ADC_EVENT_RESTART) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_BQ_ADC_RESTART,
                                  App_GetUptimeSeconds());
    }
    if ((events & BQ_ADC_EVENT_FAULT) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_BQ_ADC_FAULT,
                                  App_GetUptimeSeconds());
    }
    if ((events & BQ_ADC_EVENT_RECOVERED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_BQ_ADC_RECOVERED,
                                  App_GetUptimeSeconds());
    }
}

static void App_ProtectionEventTask(void)
{
    uint16_t events;
    protection_snapshot_t prot;

    /* 保护事件落盘同样只在主循环、BQ 空闲/RUN 时进行。 */
    if ((BQ_IsIdle() == 0u) || (BQ_GetState() != BQ_ST_RUN) ||
        (BQ_IsRecovering() != 0u) ||
        (PowerMode_GetState() != POWER_STATE_NORM))
    {
        return;
    }

    events = Protection_TakeEvent();
    if (events == 0u)
    {
        return;
    }

    if ((events & PROT_EVENT_TRIP) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_TRIP,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CLEARED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CLEARED,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CLEAR_FAILED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CLEAR_FAILED,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_LATCHED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_LATCHED,
                                  App_GetUptimeSeconds());
        Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
    }
    if ((events & PROT_EVENT_SW_CHG_OCP) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_SW_CHG_OCP,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_SW_DSG_OCP) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_SW_DSG_OCP,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_TEMP_INHIBIT) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_TEMP_INHIBIT,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_TEMP_RECOVER) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_TEMP_RECOVER,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_EARLY_SOC_CUTOFF) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_EARLY_SOC_CUTOFF,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_EARLY_SOC_CUTOFF_CLEARED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_EARLY_SOC_CLEARED,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CELL_UV) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CELL_UV,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CELL_UV_CLEARED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CELL_UV_CLEARED,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CHARGE_FULL) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CHARGE_FULL,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CHARGE_FULL_CLEARED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CHARGE_FULL_CLEARED,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CELL_OV) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CELL_OV,
                                  App_GetUptimeSeconds());
    }
    if ((events & PROT_EVENT_CELL_OV_CLEARED) != 0u)
    {
        (void)BlackBox_WriteEvent(BB_EVENT_PROT_CELL_OV_CLEARED,
                                  App_GetUptimeSeconds());
    }

    Protection_GetSnapshot(&prot);
    App_Log_Printf("[PROT] ev=%04X state=%u reason=%04X allow=%02X boot=%02X "
                   "ocp=%u/%u st=%u/%u I=%ld T=%d tv=%u inh=%u/%u inv=%u clr=%u\r\n",
                   (unsigned)events,
                   (unsigned)prot.state,
                   (unsigned)prot.reasons,
                   (unsigned)prot.allow_mask,
                   (unsigned)prot.boot_latched_mask,
                   (unsigned)prot.ocp_chg_latched,
                   (unsigned)prot.ocp_dsg_latched,
                   (unsigned)prot.ocp_chg_streak,
                   (unsigned)prot.ocp_dsg_streak,
                   (long)prot.ocp_last_current_ma,
                   (int)prot.temp_dc_last,
                   (unsigned)prot.temp_valid_last,
                   (unsigned)prot.temp_chg_inhibit,
                   (unsigned)prot.temp_dsg_inhibit,
                   (unsigned)prot.temp_invalid,
                   (unsigned)prot.clear_block_reason);
}

/* 30 min 归档质量位：cell3/pack 用真实可信值，缺失时置 0 并标记。 */
static uint16_t App_ArchiveFlags(const bq_snapshot_t *snap,
                                 const soc_est_snapshot_t *soc)
{
    uint16_t flags = 0u;

    if ((snap->log_cell3_valid == 0u) || (snap->log_pack_valid == 0u))
    {
        flags |= BB_PERIODIC_Q_PARTIAL;
    }
    if (snap->log_cell3_valid == 0u)
    {
        flags |= BB_PERIODIC_Q_CELL3_MISSING;
    }
    if (snap->log_pack_valid == 0u)
    {
        flags |= BB_PERIODIC_Q_PACK_MISSING;
    }
    if ((soc == NULL) || (soc->quality == SOC_EST_QUALITY_NONE))
    {
        flags |= BB_PERIODIC_Q_CC_INVALID;
    }
    if ((soc == NULL) || (soc->temp_valid == 0u))
    {
        flags |= BB_PERIODIC_Q_TS1_INVALID;
    }
    return flags;
}

static int App_CommitArchiveSnapshot(uint32_t mark_epoch, uint32_t mark_id)
{
    state_store_view_t view;
    const bq_snapshot_t *snap = &s_app.archive_snap;
    const soc_est_snapshot_t *soc = &s_app.archive_soc;
    uint8_t estimate_flags = 0u;
    uint8_t mark_quality;

    if (soc->quality == SOC_EST_QUALITY_NONE)
    {
        return -1;
    }
    if (StateStore_IsValid() != 0u)
    {
        StateStore_GetView(&view);
    }
    else
    {
        memset(&view, 0, sizeof(view));
    }

    view.cell_mv[0] = snap->cell_mv[0];
    view.cell_mv[1] = snap->cell_mv[1];
    view.cell_mv[2] = s_app.archive_cell3_valid ? snap->log_cell3_mv : 0u;
    view.pack_mv = s_app.archive_pack_valid ? snap->log_pack_mv : 0u;
    view.soc01 = (uint16_t)((soc->soc01 < 0) ? 0 :
                            ((soc->soc01 > 10000) ? 10000 : soc->soc01));
    view.soh01 = (uint16_t)((soc->soh01 < 0) ? 0 :
                            ((soc->soh01 > 10000) ? 10000 : soc->soh01));
    view.temp_dc = (soc->temp_valid != 0u) ? soc->temp_dc : 0;

    view.flags &= (uint16_t)~(STATE_FLAG_SOC_VALID | STATE_FLAG_PACK_VALID |
                              STATE_FLAG_SOC_ESTIMATED | STATE_FLAG_PACK_PARTIAL);
    if ((soc->frame_valid != 0u) && (soc->temp_valid != 0u) &&
        (soc->cal_valid != 0u) && (soc->quality == SOC_EST_QUALITY_REAL))
    {
        view.flags |= STATE_FLAG_SOC_VALID;
    }
    if (s_app.archive_pack_valid != 0u)
    {
        view.flags |= STATE_FLAG_PACK_VALID;
    }
    else
    {
        view.flags |= STATE_FLAG_PACK_PARTIAL;
        estimate_flags |= STATE_ARCHIVE_Q_PACK_MISSING;
    }
    if (s_app.archive_cell3_valid == 0u)
    {
        estimate_flags |= STATE_ARCHIVE_Q_CELL3_MISSING;
    }
    mark_quality = (estimate_flags == 0u) ? STATE_ARCHIVE_MARK_FULL :
                                            STATE_ARCHIVE_MARK_DEGRADED;
    return StateStore_CommitArchiveSnapshot(&view, mark_epoch, mark_id,
                                             mark_quality, estimate_flags);
}

static void App_ArchiveTask(uint32_t now_ms)
{
    state_store_view_t stored;
    rtc_epoch_t rtc;
    uint32_t mark_epoch = 0u;
    uint16_t bb_flags;
    int result;

    (void)now_ms;

    if (s_app.pending_lp_mark_active == 0u &&
        LpCycle_GetPendingMark(&mark_epoch) != 0u)
    {
        s_app.pending_lp_mark_epoch = mark_epoch;
        s_app.pending_lp_mark_active = 1u;
        s_app.pending_lp_mark_lp_archived = LpCycle_IsPendingMarkLpArchived();
    }

    if ((s_app.pending_lp_mark_active != 0u) && (s_app.norm_mark_active == 0u))
    {
        s_app.archive_mark_id = s_app.pending_lp_mark_epoch;
        s_app.norm_mark_active = 1u;
        s_app.archive_bb_written = s_app.pending_lp_mark_lp_archived;
        s_app.archive_cache_valid = 0u;
    }
    else if ((s_app.pending_lp_mark_active == 0u) &&
             (s_app.flag_blackbox_request != 0u) &&
             (s_app.norm_mark_active == 0u))
    {
        if (RtcTime_Read(&rtc) == 0u || rtc.sec == 0u)
        {
            return;
        }
        StateStore_GetView(&stored);
        if ((StateStore_IsValid() != 0u) &&
            (stored.archive.last_mark_id >= rtc.sec))
        {
            s_app.flag_blackbox_request = 0u;
            return;
        }
        s_app.archive_mark_id = rtc.sec;
        s_app.norm_mark_active = 1u;
        s_app.archive_bb_written = 0u;
        s_app.archive_cache_valid = 0u;
        s_app.flag_blackbox_request = 0u;
    }

    if ((s_app.norm_mark_active == 0u) ||
        (BQ_IsIdle() == 0u) || (BQ_GetState() != BQ_ST_RUN))
    {
        return;
    }

    if (s_app.archive_cache_valid == 0u)
    {
        BQ_GetSnapshot(&s_app.archive_snap);
        SocEst_GetSnapshot(&s_app.archive_soc);
        if ((s_app.archive_snap.fresh == 0u) ||
            (s_app.archive_snap.valid == 0u) ||
            (s_app.archive_soc.quality == SOC_EST_QUALITY_NONE))
        {
            return;
        }
        s_app.archive_cell3_valid = s_app.archive_snap.log_cell3_valid;
        s_app.archive_pack_valid = s_app.archive_snap.log_pack_valid;
        s_app.archive_cache_valid = 1u;
    }

    if (s_app.archive_bb_written == 0u)
    {
        bq_snapshot_t log = s_app.archive_snap;
        log.cell_mv[2] = s_app.archive_cell3_valid ?
                         s_app.archive_snap.log_cell3_mv : 0u;
        log.pack_mv = s_app.archive_pack_valid ?
                      s_app.archive_snap.log_pack_mv : 0u;
        bb_flags = App_ArchiveFlags(&s_app.archive_snap, &s_app.archive_soc);
        result = BlackBox_WritePeriodicQualified(&log,
                    App_GetUptimeSeconds(), bb_flags);
        if (result != 0)
        {
            return;
        }
        s_app.archive_bb_written = 1u;
        if (s_app.pending_lp_mark_active != 0u)
        {
            if (LpCycle_NotePendingMarkArchived(s_app.archive_mark_id) == 0u)
            {
                return;
            }
            s_app.pending_lp_mark_lp_archived = 1u;
        }
    }

    result = App_CommitArchiveSnapshot(s_app.archive_mark_id,
                                       s_app.archive_mark_id);
    if (result < 0)
    {
        return;
    }
    if (s_app.pending_lp_mark_active != 0u)
    {
        if (LpCycle_AckPendingMark(s_app.pending_lp_mark_epoch) != 0u)
        {
            s_app.pending_lp_mark_active = 0u;
            s_app.pending_lp_mark_lp_archived = 0u;
        }
    }
    s_app.norm_mark_active = 0u;
    s_app.archive_bb_written = 0u;
    s_app.archive_cache_valid = 0u;
    s_app.archive_mark_id = 0u;
}

static uint8_t App_FakeSwitchLpImminent(uint32_t now_ms)
{
    /* 充电/按键/策略/保护本身会阻止 LP，不构成“LP 临近”。 */
    if ((s_app.comparator_charging != 0u) ||
        (Button_IsPressed() != 0u) ||
        (ChargePolicy_IsLpBlocked() != 0u) ||
        (Protection_IsLpBlocked() != 0u))
    {
        return 0u;
    }
    return ((now_ms - s_app.last_activity_ms) >=
            (APP_IDLE_LP_MS - APP_FAKE_SWITCH_LP_LEAD_MS)) ? 1u : 0u;
}

static uint8_t App_FakeSwitchGatesReady(uint32_t now_ms)
{
    if ((SelfTest_IsDone() == 0u) || (SelfTest_IsPassed() == 0u))
    {
        return 0u;
    }
    if (StateMachine_GetState() != SM_STATE_NORM)
    {
        return 0u;
    }
    if ((BQ_GetState() != BQ_ST_RUN) || (BQ_IsFresh(now_ms) == 0u))
    {
        return 0u;
    }
    if ((BQ_IsRecovering() != 0u) || (BQ_GetAdcRecoveryFailed() != 0u))
    {
        return 0u;
    }
    if ((PowerMode_GetState() != POWER_STATE_NORM) ||
        (Protection_GetState() != PROTECTION_NORMAL) ||
        (Protection_IsLpBlocked() != 0u))
    {
        return 0u;
    }
    if ((ChargePolicy_GetState() == CHARGE_POLICY_CAL) ||
        (ChargePolicy_GetState() == CHARGE_POLICY_FAULT) ||
        (ChargePolicy_IsCalibration() != 0u))
    {
        return 0u;
    }
    if (Button_IsPressed() != 0u)
    {
        return 0u;
    }
    if ((s_app.pending_lp_mark_active != 0u) ||
        (s_app.norm_mark_active != 0u) ||
        (LpCycle_GetPendingMark(NULL) != 0u))
    {
        return 0u;
    }
    if ((BQ_GetFetMaskApplied() &
         (uint8_t)~Protection_GetAllowedFetMask()) != 0u)
    {
        return 0u;
    }
    /* 故障期间也必须保留假开关动作：不再因 BQ 退避、request/applied
     * 不一致或 BQ 已清掉 CHG/DSG 位而跳过；异常批一到就尝试同值重写。
     * 同值重写本身不会强制打开被 BQ 硬件保护关掉的 FET。 */
    return 1u;
}

/* 最低优先级：只在新完整批次上判断真实 VC5/BAT 是否异常；异常时播放与
 * 当前输出同方向的提示音，并同值重写 SYS_CTRL2。动作后 2.5 s 内不重复。 */
static void App_FakeSwitchTask(uint32_t now_ms)
{
    bq_batch_quality_t batch;
    uint8_t mask;
    int rc;

    BQ_GetBatchQuality(&batch);
    if ((batch.complete == 0u) || (batch.batch_seq == 0u) ||
        (batch.batch_seq == s_app.fake_switch_seen_batch_seq))
    {
        return;
    }

    /* VC2 不可信的批不能作为触发依据。 */
    if ((batch.field_ok_mask & BQ_FIELD_VC2) != BQ_FIELD_VC2)
    {
        s_app.fake_switch_seen_batch_seq = batch.batch_seq;
        return;
    }

    if (batch.ref_cell2_only == 0u)
    {
        s_app.fake_switch_seen_batch_seq = batch.batch_seq;
        s_app.fake_switch_anomaly = 0u;
        s_app.fake_switch_next_ms = 0u;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_NONE;
        return;
    }

    /* 异常批先不标记 seen：若因 BQ 脚本/同步/蜂鸣器/LP 门控跳过，
     * 下一圈可以继续用同一批，在实际动作或恢复正常后再消费该批。 */
    s_app.fake_switch_anomaly = 1u;
    if ((int32_t)(now_ms - s_app.fake_switch_next_ms) < 0)
    {
        return;
    }

    if (App_FakeSwitchLpImminent(now_ms) != 0u)
    {
        s_app.fake_switch_skip_gate_count++;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_LP;
        s_app.fake_switch_next_ms = now_ms + APP_FAKE_SWITCH_GATE_RETRY_MS;
        return;
    }
    if (App_FakeSwitchGatesReady(now_ms) == 0u)
    {
        s_app.fake_switch_skip_gate_count++;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_GATED;
        s_app.fake_switch_next_ms = now_ms + APP_FAKE_SWITCH_GATE_RETRY_MS;
        return;
    }
    if (Buzzer_IsActive() != 0u)
    {
        s_app.fake_switch_skip_busy_count++;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_BUSY;
        s_app.fake_switch_next_ms = now_ms + APP_FAKE_SWITCH_GATE_RETRY_MS;
        return;
    }
    if (BQ_IsSameStateWriteReady() == 0u)
    {
        s_app.fake_switch_skip_gate_count++;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_GATED;
        s_app.fake_switch_next_ms = now_ms + APP_FAKE_SWITCH_GATE_RETRY_MS;
        return;
    }

    mask = (uint8_t)(BQ_GetFetMaskApplied() & BQ_FET_MASK_ALL);
    s_app.fake_switch_last_mask = mask;
    s_app.fake_switch_seen_batch_seq = batch.batch_seq;
    Buzzer_StartPattern((mask != BQ_FET_MASK_NONE) ?
                        BUZZER_PATTERN_WAKE : BUZZER_PATTERN_LP_ENTER);
    rc = BQ_ApplyFetMaskIfUnchanged(mask);
    s_app.fake_switch_attempt_count++;
    if (rc == 0)
    {
        s_app.fake_switch_ok_count++;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_OK;
    }
    else
    {
        s_app.fake_switch_fail_count++;
        s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_FAIL;
    }
    s_app.fake_switch_next_ms = now_ms + APP_FAKE_SWITCH_RETRY_MS;
}

static void App_HandleSelfTestCompletion(void)
{
    uint8_t event = SelfTest_TakeEvent();

    if (event == SELFTEST_EVENT_NONE)
    {
        return;
    }

    s_app.last_activity_ms = HAL_GetTick();   /* 自检完成后再给 5 s 空闲窗口 */
    if (event == SELFTEST_EVENT_PASS)
    {
        Buzzer_StartPattern(BUZZER_PATTERN_BOOT_OK);
        App_Log_Printf("[TEST] pass\r\n");
    }
    else if (event == SELFTEST_EVENT_FAIL)
    {
        if ((SelfTest_GetFailMask() & (uint16_t)~SELFTEST_FAIL_BQ_DATA) != 0u ||
            (Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                               FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                               FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                               FAULT_BIT_POWER | FAULT_BIT_CLK)) != 0u)
        {
            Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
        }
        App_Log_Printf("[TEST] fail=0x%04X\r\n", SelfTest_GetFailMask());
    }
    else
    {
        /* BQ 动态项恢复：更新 LED/策略阻止状态即可，不再响故障音。 */
        App_Log_Printf("[TEST] recovered\r\n");
    }
}

static void App_DiagTask(uint32_t now_ms)
{
    bq_snapshot_t snapshot;
    protection_snapshot_t prot;

    if (BQ_IsFresh(now_ms) != 0u)
    {
        Fault_ClearLive(FAULT_BIT_BQ_STALE);
    }
    else
    {
        Fault_SetLive(FAULT_BIT_BQ_STALE);
    }

    if ((now_ms - s_app.diag_log_ms) < APP_DIAG_LOG_PERIOD_MS)
    {
        return;
    }
    s_app.diag_log_ms = now_ms;

    BQ_GetSnapshot(&snapshot);
    {
        soc_est_snapshot_t soc;
        SocEst_GetSnapshot(&soc);
        App_Log_Printf("[SOC] soc=%ld soh=%ld T=%d TSR=%u TSV=%u "
                       "fv=%u tv=%u cv=%u ref=%u ref2=%u upd=%lu\r\n",
                       (long)soc.soc01, (long)soc.soh01, (int)soc.temp_dc,
                       (unsigned)soc.ts1_raw, (unsigned)soc.ts1_mv,
                       (unsigned)soc.frame_valid, (unsigned)soc.temp_valid,
                       (unsigned)soc.cal_valid, (unsigned)soc.ref_cell_mv,
                       (unsigned)soc.ref_cell2_only,
                       (unsigned long)soc.update_count);
    }
    App_Log_Printf("[FAULT] all=%04X live=%04X sticky=%04X test=%u/%u "
                   "fail=%04X sm=%u pvd=%lu\r\n",
                   (unsigned)Fault_GetAll(),
                   (unsigned)Fault_GetLive(),
                   (unsigned)Fault_GetSticky(),
                   (unsigned)SelfTest_IsDone(),
                   (unsigned)SelfTest_IsPassed(),
                   (unsigned)SelfTest_GetFailMask(),
                   (unsigned)StateMachine_GetState(),
                   (unsigned long)Fault_GetPvdIrqCount());
    App_Log_Printf("[DIAG] sm=%u flow=%u V=%u/%u/%u P=%u CC=%d I=%ld TS=%u "
                   "ref=%u ref2=%u flt=%04X bq=%u lpblk=%04X up=%lu\r\n",
                   (unsigned)StateMachine_GetState(),
                   (unsigned)StateMachine_GetFlow(),
                   (unsigned)snapshot.cell_mv[0],
                   (unsigned)snapshot.cell_mv[1],
                   (unsigned)snapshot.cell_mv[2],
                   (unsigned)snapshot.pack_mv,
                   (int)snapshot.cc_raw,
                   (long)snapshot.current_ma,
                   (unsigned)snapshot.ts1_raw,
                   (unsigned)BQ_GetRefCellMv(&snapshot),
                   (unsigned)snapshot.ref_cell2_only,
                   (unsigned)Fault_GetAll(),
                   (unsigned)BQ_GetState(),
                   (unsigned)s_app.auto_lp_block,
                   (unsigned long)App_GetUptimeSeconds());

    App_Log_Printf("[BQ] pre=%02X/%02X ss=%02X bal=%02X/%02X post=%02X/%02X "
                   "ss=%02X off=%02X/%02X rst=%lu a=%u r=%u f=%u\r\n",
                   (unsigned)snapshot.pre_ctrl1,
                   (unsigned)snapshot.pre_ctrl2,
                   (unsigned)snapshot.pre_sys_stat,
                   (unsigned)snapshot.pre_cellbal1,
                   (unsigned)snapshot.post_cellbal1,
                   (unsigned)snapshot.post_ctrl1,
                   (unsigned)snapshot.post_ctrl2,
                   (unsigned)snapshot.post_sys_stat,
                   (unsigned)snapshot.adc_off_ctrl1,
                   (unsigned)snapshot.adc_off_ctrl2,
                   (unsigned long)snapshot.adc_restart_count,
                   (unsigned)snapshot.adc_restart_attempts,
                   (unsigned)snapshot.adc_restart_reason,
                   (unsigned)snapshot.adc_recovery_failed);
    App_Log_Printf("[BQ] rawC3=%u rawP=%u logC3=%u/%u logP=%u/%u "
                   "ss=%02X rawss=%02X ovt=%u uvt=%u c12=%u/%u c12v=%u\r\n",
                   (unsigned)snapshot.raw_cell[2],
                   (unsigned)snapshot.raw_pack,
                   (unsigned)snapshot.log_cell3_mv,
                   (unsigned)snapshot.log_cell3_valid,
                   (unsigned)snapshot.log_pack_mv,
                   (unsigned)snapshot.log_pack_valid,
                   (unsigned)snapshot.sys_stat,
                   (unsigned)snapshot.raw_sys_stat,
                   (unsigned)snapshot.ov_trip_mv,
                   (unsigned)snapshot.uv_trip_mv,
                   (unsigned)snapshot.cell12_meas_mv[0],
                   (unsigned)snapshot.cell12_meas_mv[1],
                   (unsigned)snapshot.cell12_meas_valid);
    App_Log_Printf("[BQ] ok=%lu rej=%lu stk=%u fcon=%u/%lu vc3=%u vc4=%u "
                   "xr=%u fv=%u/%u\r\n",
                   (unsigned long)snapshot.frame_ok_count,
                   (unsigned long)snapshot.frame_reject_count,
                   (unsigned)snapshot.reject_streak,
                   (unsigned)snapshot.frame_confirm_count,
                   (unsigned long)snapshot.frame_confirm_reset_count,
                   (unsigned)snapshot.raw_vc3,
                   (unsigned)snapshot.raw_vc4,
                   (unsigned)snapshot.xready_count,
                   (unsigned)snapshot.fresh,
                   (unsigned)snapshot.valid);
    App_Log_Printf("[BQ] last=%u V=%u/%u/%u/%u vc=%u/%u ss=%02X lm=%lu\r\n",
                   (unsigned)snapshot.last_reject_reason,
                   (unsigned)snapshot.last_reject_cell_mv[0],
                   (unsigned)snapshot.last_reject_cell_mv[1],
                   (unsigned)snapshot.last_reject_cell_mv[2],
                   (unsigned)snapshot.last_reject_pack_mv,
                   (unsigned)snapshot.last_reject_vc3_raw,
                   (unsigned)snapshot.last_reject_vc4_raw,
                   (unsigned)snapshot.last_reject_sys_stat,
                   (unsigned long)snapshot.last_reject_ms);
    Protection_GetSnapshot(&prot);
    App_Log_Printf("[PROT] state=%u reason=%04X allow=%02X boot=%02X "
                   "ocp=%u/%u st=%u/%u n=%lu I=%ld T=%d tv=%u inh=%u/%u "
                   "inv=%u uvref=%u ref2=%u lb=%u boost=%u clr=%u "
                   "vb=%u hold=%02X swov=%u ovref=%u\r\n",
                   (unsigned)prot.state,
                   (unsigned)prot.reasons,
                   (unsigned)prot.allow_mask,
                   (unsigned)prot.boot_latched_mask,
                   (unsigned)prot.ocp_chg_latched,
                   (unsigned)prot.ocp_dsg_latched,
                   (unsigned)prot.ocp_chg_streak,
                   (unsigned)prot.ocp_dsg_streak,
                   (unsigned long)prot.ocp_trip_count,
                   (long)prot.ocp_last_current_ma,
                   (int)prot.temp_dc_last,
                   (unsigned)prot.temp_valid_last,
                   (unsigned)prot.temp_chg_inhibit,
                   (unsigned)prot.temp_dsg_inhibit,
                   (unsigned)prot.temp_invalid,
                   (unsigned)prot.min_cell_mv,
                   (unsigned)prot.ref_cell2_only,
                   (unsigned)((prot.early_soc_latched != 0u) ||
                              (prot.cell_uv_active != 0u)),
                   (unsigned)prot.low_batt_dsg_allowed,
                   (unsigned)prot.clear_block_reason,
                   (unsigned)prot.voltage_backoff,
                   (unsigned)prot.bq_fet_hold_mask,
                   (unsigned)prot.cell_ov_active,
                   (unsigned)prot.cell_ov_ref_mv);

    App_Log_Printf("[BQ] oor=%u rs=%u sw=%u ep=%u va=%u gp=%u ref2=%u "
                   "vd=%lu rl=%lu vb=%u vbp=%u vbf=%u vbc=%u vbn=%u ovt=%u uvt=%u\r\n",
                   (unsigned)snapshot.oor_streak,
                   (unsigned)snapshot.resample_count,
                   (unsigned)snapshot.strict_window,
                   (unsigned)snapshot.recovery_episode,
                   (unsigned)snapshot.verdict_active,
                   (unsigned)snapshot.good_publish_streak,
                   (unsigned)snapshot.ref_cell2_only,
                   (unsigned long)snapshot.verdict_deadline_ms,
                   (unsigned long)snapshot.adc_restart_last_ms,
                   (unsigned)snapshot.voltage_backoff,
                   (unsigned)snapshot.voltage_backoff_pending,
                   (unsigned)snapshot.voltage_backoff_failed,
                   (unsigned)snapshot.voltage_backoff_clear_failed,
                   (unsigned)snapshot.voltage_backoff_normal_streak,
                   (unsigned)snapshot.ov_trip_mv,
                   (unsigned)snapshot.uv_trip_mv);

    App_Log_Printf("[POL] st=%u req=%u app=%u chg=%u dis=%u cal=%u "
                   "cp=%u bd=%u ih=%u cmp=%u I=%ld soc=%u rsn=%u "
                   "lb=%u dsg=%u ref=%u dd=%u dc=%lu\r\n",
                   (unsigned)ChargePolicy_GetState(),
                   (unsigned)ChargePolicy_GetRequestedMask(),
                   (unsigned)BQ_GetFetMaskApplied(),
                   (unsigned)(((ChargePolicy_GetState() ==
                                CHARGE_POLICY_CHARGE) ||
                               (ChargePolicy_GetState() ==
                                CHARGE_POLICY_LOW_BATTERY_CHARGE)) ? 1u : 0u),
                   (unsigned)((ChargePolicy_GetState() ==
                               CHARGE_POLICY_DISCHARGE) ? 1u : 0u),
                   (unsigned)ChargePolicy_IsCalibration(),
                   (unsigned)ChargePolicy_IsChargerPresent(),
                   (unsigned)ChargePolicy_GetBodyDiodeMode(),
                   (unsigned)ChargePolicy_GetChargeCurrentHold(),
                   (unsigned)s_app.comparator_charging,
                   (long)snapshot.current_ma,
                   (unsigned)SocEst_GetSocPercent(),
                   (unsigned)ChargePolicy_GetCalEndReason(),
                   (unsigned)Protection_IsLowBattery(),
                   (unsigned)ChargePolicy_GetLowBattDsgOn(),
                   (unsigned)BQ_GetRefCellMv(&snapshot),
                   (unsigned)ChargePolicy_GetDischargeChargeDetect(),
                   (unsigned long)ChargePolicy_GetDischargeChargeDetectCount());
    App_Log_Printf("[VSW] an=%u n=%lu ok=%lu fail=%lu busy=%lu gate=%lu "
                   "res=%u mask=%02X next=%lu\r\n",
                   (unsigned)s_app.fake_switch_anomaly,
                   (unsigned long)s_app.fake_switch_attempt_count,
                   (unsigned long)s_app.fake_switch_ok_count,
                   (unsigned long)s_app.fake_switch_fail_count,
                   (unsigned long)s_app.fake_switch_skip_busy_count,
                   (unsigned long)s_app.fake_switch_skip_gate_count,
                   (unsigned)s_app.fake_switch_last_result,
                   (unsigned)s_app.fake_switch_last_mask,
                   (unsigned long)s_app.fake_switch_next_ms);
}

static void App_DegradedLoop(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - s_app.degraded_blink_ms) < APP_DEGRADED_BLINK_MS)
    {
        return;
    }
    s_app.degraded_blink_ms = now_ms;
    s_app.degraded_blink_on ^= 1u;

    HAL_GPIO_WritePin(IND_ROHT_GPIO_Port, IND_ROHT_Pin,
                      (s_app.degraded_blink_on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_RESET);
}

/* ---------------- 对外入口 ---------------- */

void App_Boot(void)
{
    lp_boot_t boot;

    ResetCause_Init();

    /* 复位后先在 App 层把 PA1 拉高（关 PACK_SC/HSE 前端）并确保 5 V 关，
     * 避免 MX_GPIO_Init 默认低电平在 LP 复位 boot 期间产生前端/5 V 毛刺。
     * 完整 NORM boot 会在 PowerMode_Init 里按需要重新拉低 PA1 启动 HSE。 */
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_RESET);

    (void)RtcTime_Init();
    boot = LpCycle_ClassifyBoot();

    if ((boot == LP_BOOT_START) || (boot == LP_BOOT_RESUME))
    {
        /* IWDG 轻量 LP 路径；正常情况下只会在下一次复位后返回（不返回）。 */
        LpCycle_Run(boot);
    }

    App_Init();

    if (boot == LP_BOOT_FAULT)
    {
        PowerMode_ForceDisableAutoLp();
        LpSession_Invalidate();
    }
}

void App_Init(void)
{
    /* MX_IWDG_Init() (CubeMX) starts IWDG with its generated /4 prescaler,
     * which is only ~0.5 s at LSI and would expire during App_Init.  Reconfigure
     * to the ~8 s dead-man immediately, before any blocking init work. */
    Watchdog_InitNorm();

    s_app.uptime_s = 0u;
    s_app.uptime_base_s = 0u;
    s_app.flag_bq_read_request = 0u;
    s_app.flag_diag_request = 0u;
    s_app.flag_blackbox_request = 0u;
    s_app.prescale_500ms = 0u;
    s_app.prescale_1s = 0u;
    s_app.prescale_30min = 0u;
    s_app.comparator_charging = 0u;
    s_app.comparator_last_sample = 0u;
    s_app.comparator_match_count = 0u;
    s_app.diag_log_ms = 0u;
    s_app.last_activity_ms = 0u;
    s_app.auto_lp_block = 0u;
    s_app.degraded_blink_ms = 0u;
    s_app.degraded_blink_on = 0u;
    s_app.norm_mark_active = 0u;
    s_app.archive_bb_written = 0u;
    s_app.archive_cache_valid = 0u;
    s_app.archive_mark_id = 0u;
    memset(&s_app.archive_snap, 0, sizeof(s_app.archive_snap));
    memset(&s_app.archive_soc, 0, sizeof(s_app.archive_soc));
    s_app.archive_cell3_valid = 0u;
    s_app.archive_pack_valid = 0u;
    s_app.pending_lp_mark_epoch = 0u;
    s_app.pending_lp_mark_active = 0u;
    s_app.pending_lp_mark_lp_archived = 0u;
    s_app.fake_switch_seen_batch_seq = 0u;
    s_app.fake_switch_next_ms = 0u;
    s_app.fake_switch_attempt_count = 0u;
    s_app.fake_switch_ok_count = 0u;
    s_app.fake_switch_fail_count = 0u;
    s_app.fake_switch_skip_busy_count = 0u;
    s_app.fake_switch_skip_gate_count = 0u;
    s_app.fake_switch_anomaly = 0u;
    s_app.fake_switch_last_result = APP_FAKE_SWITCH_RESULT_NONE;
    s_app.fake_switch_last_mask = 0u;

    Fault_Init();
    PowerMode_Init();          /* 16 MHz → 80 MHz；核对 RTC 源/LSE 健康 */
    Watchdog_Feed();

    BlackBox_Init();
    StateStore_Init();
    s_app.uptime_base_s = BlackBox_GetUptimeBase();
    if (LpCycle_GetUptimeBase() > s_app.uptime_base_s)
    {
        s_app.uptime_base_s = LpCycle_GetUptimeBase();
    }

    if (PowerMode_IsDegraded() != 0u)
    {
        /* 时钟降级：只留故障灯闪烁，通信/BQ 全部不启动。 */
        HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_RESET);
        return;
    }

    /* LP 以 RTC 为绝对时间参考（LP_RTC_USE_LSE 选 LSE，默认 LSI 调试），
     * IWDG 复位作为周期 tick。时基未就绪时 auto-LP 由
     * PowerMode_LpTimebaseReady() 阻止。 */
    Watchdog_Feed();

    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(APP_5V_STARTUP_DELAY_MS);

    App_Log_Init();
    App_Log_Printf("[BOOT] app init, reset=0x%02X\r\n",
                   (unsigned)ResetCause_GetLatched());

    BQ_Init();
    Watchdog_Feed();
    SocEst_Init();
    Watchdog_Feed();
    /* Ws2812_HwInit 按 80 MHz 重设 USART2SEL=PCLK1 并在 UE=0 时写 BRR。 */
    Ws2812_Init();
    LedStatus_Init();
    Buzzer_Init();
    Button_Init();
    Gesture_Init();

    SelfTest_Init();
    SelfTest_Request();
    StateMachine_Init();
    ChargePolicy_Init();
    Protection_Init();
    CalStream_Init();
    App_Telemetry_Init();

    (void)HAL_COMP_Start(&hcomp2);

    s_app.diag_log_ms = HAL_GetTick();
    App_TickTimerInit();
    App_TickStart();           /* ★ 时基最后启动 */
    Watchdog_Feed();

    s_app.last_activity_ms = HAL_GetTick();
    LpCycle_ApplyWake();
}

void App_Loop(void)
{
    uint32_t now_ms;
    uint8_t had_activity = 0u;
    button_event_t button_ev;
    gesture_event_t gesture_ev;
    bq_snapshot_t bq_snap;
    soc_est_snapshot_t soc_snap;
    charge_policy_input_t policy_in;

    Watchdog_Feed();
    Fault_PowerRecoverTask(HAL_GetTick());

    if (PowerMode_IsDegraded() != 0u)
    {
        App_DegradedLoop();
        return;
    }

    /* 1) 自动/调试器 LP 请求：写 BKP START token 后软件复位，本函数不再
     * 返回 LP；真正唤醒由下一次完整 boot 的 LpCycle_ApplyWake() 处理。 */
    PowerMode_Loop();

    now_ms = HAL_GetTick();

    /* 按键/手势：任何事件都算活动；按下期间不自动进 LP。 */
    Button_Poll(now_ms);
    while (Button_GetEvent(&button_ev) != 0u)
    {
        had_activity = 1u;
        Gesture_OnButtonEvent(&button_ev);
    }
    Gesture_Poll(now_ms);
    while (Gesture_GetEvent(&gesture_ev) != 0u)
    {
        had_activity = 1u;
        if (gesture_ev == GESTURE_PAIR_HOLD3)
        {
            ChargePolicy_OnPairHold3(now_ms);
        }
    }
    if ((had_activity != 0u) || (Button_IsPressed() != 0u))
    {
        s_app.last_activity_ms = now_ms;
    }

    /* 2) BQ 事务推进 + 500 ms 周期读。 */
    if (s_app.flag_bq_read_request != 0u)
    {
        s_app.flag_bq_read_request = 0u;
        BQ_RequestRead();
    }
    BQ_Poll();
    BQ_GetSnapshot(&bq_snap);
    if ((bq_snap.fresh != 0u) && (bq_snap.valid != 0u) &&
        (BQ_IsFresh(now_ms) != 0u))
    {
        SocEst_OnSnapshot(&bq_snap, now_ms);
    }
    else
    {
        SocEst_InvalidateInput();
    }
    SocEst_Task();
    App_BqAdcEventTask();

    SelfTest_Poll(now_ms);
    App_HandleSelfTestCompletion();

    /* 4) 1 s 诊断（BQ 新鲜度 → FAULT_BQ_STALE + 10 s 日志）。 */
    if (s_app.flag_diag_request != 0u)
    {
        s_app.flag_diag_request = 0u;
        /* 先发 1 Hz D 帧，再进入 10 s 诊断突发。 */
        App_Telemetry_Task(now_ms, &bq_snap);
        App_DiagTask(now_ms);
    }

    /* 同一轮只取一次 SOC 快照，供 Protection、策略、LED 共用。 */
    SocEst_GetSnapshot(&soc_snap);

    /* 保护监督层必须先于状态机。 */
    Protection_Task(now_ms, &bq_snap, &soc_snap,
                    s_app.comparator_charging,
                    ChargePolicy_IsChargerPresent(),
                    ChargePolicy_GetState(),
                    ChargePolicy_GetLowBattDsgOn());
    /* BQ OV/UV 退避：Protection 已按 VC2 更新 allow，再执行阈值切换；
     * 失败时 BQ 驱动会保持 FET hold，下一轮 Protection fail-closed。 */
    (void)BQ_ServiceVoltageBackoff();
    StateMachine_Update(BQ_IsFresh(now_ms), s_app.comparator_charging,
                        Fault_GetAll());
    policy_in.charge_current_ma =
        (-(int32_t)bq_snap.current_ma) * BQ_CC_DISCHARGE_SIGN;
    policy_in.current_valid =
        ((bq_snap.valid != 0u) && (bq_snap.fresh != 0u) &&
         (BQ_IsFresh(now_ms) != 0u)) ? 1u : 0u;
    policy_in.soc01 = (uint16_t)((soc_snap.soc01 < 0) ? 0 :
                       ((soc_snap.soc01 > 10000) ? 10000 : soc_snap.soc01));
    /* 只要求真实 BQ 帧，不要求 NMC 标定；SOC 仅作近满策略启发，不是保护输入。 */
    policy_in.soc_valid =
        ((soc_snap.frame_valid != 0u) &&
         (soc_snap.quality == SOC_EST_QUALITY_REAL)) ? 1u : 0u;
    policy_in.ref_cell_mv = BQ_GetRefCellMv(&bq_snap);
    if ((BQ_IsVoltageBackoffActive() != 0u) ||
        (BQ_IsVoltageBackoffPending() != 0u) ||
        (BQ_IsVoltageBackoffFailed() != 0u))
    {
        /* VC5/BAT 不可信时低电/DSG 策略也必须只使用 VC2。 */
        if ((bq_snap.cell_mv[1] >= 1000u) && (bq_snap.cell_mv[1] <= 5000u))
        {
            policy_in.ref_cell_mv = bq_snap.cell_mv[1];
        }
        else
        {
            policy_in.ref_cell_mv = 0u;
        }
    }
    policy_in.ref_valid = (policy_in.ref_cell_mv != 0u) ? 1u : 0u;
    policy_in.current_ms = bq_snap.last_update_ms;
    ChargePolicy_Task(now_ms, s_app.comparator_charging, &policy_in);
    Protection_Service(now_ms, &bq_snap);
    App_ProtectionEventTask();

    CalStream_Task(now_ms, App_GetUptimeSeconds(), &bq_snap);
    App_UpdateLedMode(&soc_snap);

    /* 先处理 LP 待标记和 NORM 30 min 标记，再判断能否进 LP；否则
     * 标记到达的这一圈可能抢先复位，造成档案窗口漏处理。 */
    App_ArchiveTask(now_ms);
    App_FakeSwitchTask(now_ms);   /* 最低优先级：所有其它服务之后 */

    /* 自动 LP：逐项记录阻止原因，5 s 无活动且无阻止项时请求 LP。 */
    {
        uint16_t block = 0u;

        if (s_app.comparator_charging != 0u)
        {
            block |= APP_LP_BLOCK_CHARGING;
            s_app.last_activity_ms = now_ms;
        }
        if (Button_IsPressed() != 0u)
        {
            block |= APP_LP_BLOCK_BUTTON;
        }
        if (((SelfTest_IsDone() == 0u) || (SelfTest_IsPassed() == 0u)) &&
            !((ChargePolicy_GetState() ==
               CHARGE_POLICY_LOW_BATTERY_CHARGE) &&
              (ChargePolicy_IsLpBlocked() == 0u) &&
              ((SelfTest_GetFailMask() &
                (uint16_t)~SELFTEST_FAIL_BQ_DATA) == 0u) &&
              ((Fault_GetAll() &
                (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                 FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                 FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                 FAULT_BIT_POWER | FAULT_BIT_CLK | FAULT_BIT_TEMP |
                 FAULT_BIT_FLASH_LOG | FAULT_BIT_FLASH_STATE |
                 FAULT_BIT_TIMEBASE)) == 0u)))
        {
            /* 低电且没有充电活动时，仅 BQ_DATA 自检失败不能阻止休眠；
             * 真实通信/硬件/温度/Flash 故障仍按原逻辑阻止。 */
            block |= APP_LP_BLOCK_TEST;
        }
        if (StateMachine_GetState() != SM_STATE_NORM)
        {
            block |= APP_LP_BLOCK_STATE;
        }
        if ((BQ_IsFresh(now_ms) == 0u) || (BQ_IsRecovering() != 0u) ||
            (BQ_GetAdcRecoveryFailed() != 0u))
        {
            block |= APP_LP_BLOCK_BQ;
        }
        if (PowerMode_LpTimebaseReady() == 0u)
        {
            block |= APP_LP_BLOCK_TIMEBASE;
        }
        if (PowerMode_GetState() != POWER_STATE_NORM)
        {
            block |= APP_LP_BLOCK_POWER;
        }
        if (ChargePolicy_IsLpBlocked() != 0u)
        {
            block |= APP_LP_BLOCK_POLICY;
            s_app.last_activity_ms = now_ms;
        }
        if (Protection_IsLpBlocked() != 0u)
        {
            block |= APP_LP_BLOCK_PROTECTION;
        }

        if ((s_app.pending_lp_mark_active != 0u) ||
            (s_app.norm_mark_active != 0u) ||
            (LpCycle_GetPendingMark(NULL) != 0u))
        {
            block |= APP_LP_BLOCK_FLASH_MARK;
        }
        if ((StateMachine_GetState() != SM_STATE_NORM) ||
            (s_app.pending_lp_mark_active != 0u) ||
            (s_app.norm_mark_active != 0u))
        {
            s_app.last_activity_ms = now_ms;
        }
        s_app.auto_lp_block = block;
        if ((block == 0u) &&
            ((now_ms - s_app.last_activity_ms) >= APP_IDLE_LP_MS))
        {
            PowerMode_RequestLp();
            s_app.last_activity_ms = now_ms;
        }
    }

    /* 6) 蜂鸣器 / RGB / 日志。 */
    Buzzer_Poll();
    LedStatus_Flush();

    App_Log_Poll();
    App_Log_FlushSmall((ChargePolicy_IsCalibration() != 0u) ? 32u : 4u);
}

/* ---------------- 中断 / weak 回调 ---------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        App_OnTick250Hz();
    }
}

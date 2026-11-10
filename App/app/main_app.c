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
#include "app/bq_anomaly.h"
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
#define APP_IDLE_LP_MS              5000u   /* 无操作/无充电 5 s 自动进 LP */
#define APP_BOOT_VOLTAGE_HOLD_MS    2000u

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
#define APP_LP_BLOCK_VOLTAGE        (1u << 9)   /* 电压异常/归档/报警尚未收口 */

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

    /* 最近一批 BQ 同批质量证据与电流转换。 */
    bq_batch_quality_t batch_quality;
    bq_cc_sample_t cc_sample;
    uint32_t last_soc_input_batch_seq;
    uint32_t full_gate_seq;
    uint32_t full_gate_ms;
    uint8_t full_gate_passed_this_boot;

    /* 30 min 普通运行标记与跨双 Flash 提交的 RAM 重试缓存。 */
    uint8_t norm_mark_active;
    uint8_t archive_cache_valid;
    uint8_t archive_bb_written;
    uint32_t archive_mark_id;
    bq_anomaly_archive_t archive_cache;

    /* LP/NORM 标记交接。 */
    uint32_t pending_lp_mark_epoch;
    uint8_t pending_lp_mark_active;
    uint8_t pending_lp_mark_lp_archived;

    /* 异常模块输入上下文与状态缓存。 */
    bq_anomaly_inputs_t anomaly_inputs;
    bq_anomaly_status_t anomaly_status;
    uint8_t anomaly_buzzer_owned;
    uint8_t last_voltage_events;
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

static void App_UpdateLedMode(void)
{
    led_flow_t flow;
    charge_policy_state_t policy_state;
    uint32_t now_ms = HAL_GetTick();

    if (StateMachine_GetState() == SM_STATE_FAULT)
    {
        LedStatus_SetMode(LED_MODE_FAULT, LED_FLOW_FAULT);
        return;
    }

    if ((s_app.anomaly_status.boot_blue_hold != 0u ||
         (s_app.anomaly_status.full_seen_this_boot != 0u &&
          SelfTest_IsDone() != 0u &&
          (now_ms - SelfTest_GetCompletionMs()) < APP_BOOT_VOLTAGE_HOLD_MS)) &&
        (s_app.anomaly_status.alarm_active == 0u) &&
        ((SelfTest_GetFailMask() & (uint16_t)~SELFTEST_FAIL_BQ_DATA) == 0u))
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
        (s_app.anomaly_status.partial_active == 0u) &&
        (s_app.anomaly_status.alarm_active == 0u) &&
        ((SelfTest_GetFailMask() & (uint16_t)~SELFTEST_FAIL_BQ_DATA) != 0u ||
         (Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                            FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                            FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                            FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                            FAULT_BIT_POWER | FAULT_BIT_CLK)) != 0u))
    {
        LedStatus_SetMode(LED_MODE_FAULT, LED_FLOW_FAULT);
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

    case SM_STATE_BQ_ANOMALY:
        flow = (policy_state == CHARGE_POLICY_DISCHARGE) ?
               LED_FLOW_DISCHARGING : LED_FLOW_STANDBY;
        LedStatus_SetMode(LED_MODE_NORM, flow);
        break;

    case SM_STATE_NORM:
    default:
        if (policy_state == CHARGE_POLICY_CHARGE)
        {
            flow = LED_FLOW_CHARGING;
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
    uint8_t events;
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

    Protection_GetSnapshot(&prot);
    App_Log_Printf("[PROT] ev=%02X state=%u reason=%04X allow=%02X boot=%02X "
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

static void App_AnomalyBuildInputs(const bq_batch_quality_t *batch)
{
    const uint16_t ctrl_fields = (uint16_t)(BQ_FIELD_CTRL1 | BQ_FIELD_CTRL2 |
                                             BQ_FIELD_PROTECTION);
    uint16_t faults = Fault_GetAll();
    uint8_t verified_ctrl = 0u;
    uint8_t fet_mask = BQ_FET_MASK_ALL;

    if ((batch != NULL) && (batch->complete != 0u) &&
        ((batch->field_ok_mask & ctrl_fields) == ctrl_fields))
    {
        verified_ctrl = 1u;
        fet_mask = (uint8_t)(batch->ctrl2 & BQ_FET_MASK_ALL);
    }

    s_app.anomaly_inputs.selftest_done = SelfTest_IsDone();
    s_app.anomaly_inputs.selftest_passed = SelfTest_IsPassed();
    s_app.anomaly_inputs.hardware_ok =
        ((faults & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                    FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                    FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                    FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_POWER |
                    FAULT_BIT_CLK | FAULT_BIT_PROTECTION)) == 0u &&
         (batch != NULL) && (batch->complete != 0u) &&
         ((batch->field_ok_mask & (BQ_FIELD_SYS_STAT | BQ_FIELD_PROTECTION)) ==
          (BQ_FIELD_SYS_STAT | BQ_FIELD_PROTECTION)) &&
         ((batch->sys_stat & (BQ_SYS_STAT_PROT_MASK | BQ_SYS_STAT_XREADY)) == 0u))
        ? 1u : 0u;
    s_app.anomaly_inputs.link_ok =
        ((BQ_GetState() == BQ_ST_RUN) && (BQ_IsRecovering() == 0u) &&
         ((faults & FAULT_BIT_BQ_COMM) == 0u)) ? 1u : 0u;
    s_app.anomaly_inputs.comp_charging = s_app.comparator_charging;
    s_app.anomaly_inputs.comp_valid =
        ((HAL_COMP_GetState(&hcomp2) == HAL_COMP_STATE_BUSY) &&
         (s_app.comparator_match_count >= APP_COMP_DEBOUNCE_SAMPLES)) ? 1u : 0u;
    s_app.anomaly_inputs.adc_cc_on =
        ((verified_ctrl != 0u) &&
         ((batch->ctrl1 & BQ_SYS_CTRL1_ADC_EN) != 0u) &&
         ((batch->ctrl2 & BQ_SYS_CTRL2_CC_EN) != 0u)) ? 1u : 0u;
    s_app.anomaly_inputs.fet_off_verified =
        ((verified_ctrl != 0u) && (fet_mask == BQ_FET_MASK_NONE) &&
         (BQ_GetFetMaskRequested() == BQ_FET_MASK_NONE) &&
         (BQ_GetFetMaskApplied() == BQ_FET_MASK_NONE)) ? 1u : 0u;
    s_app.anomaly_inputs.output_on =
        ((verified_ctrl != 0u) ? (fet_mask != BQ_FET_MASK_NONE) :
         ((BQ_GetFetMaskRequested() != BQ_FET_MASK_NONE) ||
          (BQ_GetFetMaskApplied() != BQ_FET_MASK_NONE))) ? 1u : 0u;
    s_app.anomaly_inputs.bq_idle =
        ((BQ_GetState() == BQ_ST_RUN) && (BQ_IsIdle() != 0u)) ? 1u : 0u;
    s_app.anomaly_inputs.protection_clear_ok =
        ((Protection_GetState() == PROTECTION_NORMAL) &&
         (s_app.anomaly_inputs.link_ok != 0u) &&
         (s_app.anomaly_inputs.hardware_ok != 0u) &&
         ((faults & (FAULT_BIT_PROTECTION | FAULT_BIT_TEMP |
                     FAULT_BIT_FLASH_STATE)) == 0u)) ? 1u : 0u;
}

static void App_AnomalyEventTask(void)
{
    uint8_t events = BqAnomaly_TakeEvent();

    s_app.last_voltage_events |= events;
    if ((s_app.last_voltage_events == 0u) ||
        (BQ_IsIdle() == 0u) || (BQ_GetState() != BQ_ST_RUN) ||
        (PowerMode_GetState() != POWER_STATE_NORM))
    {
        return;
    }

    events = s_app.last_voltage_events;
    if ((events & BQ_ANOMALY_EVENT_ALARM_START) != 0u)
    {
        if (BlackBox_WriteEvent(BB_EVENT_BQ_VOLTAGE_ALARM,
                                App_GetUptimeSeconds()) != 0)
        {
            return;
        }
        s_app.last_voltage_events &= (uint8_t)~BQ_ANOMALY_EVENT_ALARM_START;
    }
    if ((events & BQ_ANOMALY_EVENT_ALARM_CANCEL) != 0u)
    {
        if (BlackBox_WriteEvent(BB_EVENT_BQ_VOLTAGE_CANCEL,
                                App_GetUptimeSeconds()) != 0)
        {
            return;
        }
        s_app.last_voltage_events &= (uint8_t)~BQ_ANOMALY_EVENT_ALARM_CANCEL;
    }
    if ((events & BQ_ANOMALY_EVENT_LATCHED) != 0u)
    {
        if (BlackBox_WriteEvent(BB_EVENT_BQ_VOLTAGE_LATCHED,
                                App_GetUptimeSeconds()) != 0)
        {
            return;
        }
        s_app.last_voltage_events &= (uint8_t)~BQ_ANOMALY_EVENT_LATCHED;
    }
    if ((events & BQ_ANOMALY_EVENT_MANUAL_CLEAR) != 0u)
    {
        if (BlackBox_WriteEvent(BB_EVENT_BQ_VOLTAGE_MANUAL_CLEAR,
                                App_GetUptimeSeconds()) == 0)
        {
            s_app.last_voltage_events &= (uint8_t)~BQ_ANOMALY_EVENT_MANUAL_CLEAR;
        }
    }
}

static uint16_t App_ArchiveBlackBoxFlags(const bq_anomaly_archive_t *archive)
{
    uint16_t flags = 0u;

    if (archive->mark_quality != STATE_ARCHIVE_MARK_FULL)
    {
        flags |= BB_PERIODIC_Q_PARTIAL;
    }
    if ((archive->estimate_flags & STATE_ARCHIVE_Q_CELL1_MISSING) != 0u)
    {
        flags |= BB_PERIODIC_Q_CELL1_MISSING;
    }
    if ((archive->estimate_flags & STATE_ARCHIVE_Q_CELL2_MISSING) != 0u)
    {
        flags |= BB_PERIODIC_Q_CELL2_MISSING;
    }
    if ((archive->estimate_flags & STATE_ARCHIVE_Q_CELL3_MISSING) != 0u)
    {
        flags |= BB_PERIODIC_Q_CELL3_MISSING;
    }
    if ((archive->estimate_flags & STATE_ARCHIVE_Q_PACK_MISSING) != 0u)
    {
        flags |= BB_PERIODIC_Q_PACK_MISSING;
    }
    if ((archive->estimate_flags & STATE_ARCHIVE_Q_SOC_ESTIMATED) != 0u)
    {
        flags |= BB_PERIODIC_Q_SOC_ESTIMATED;
    }
    if ((archive->estimate_flags & STATE_ARCHIVE_Q_SOC_INVALID) != 0u)
    {
        flags |= BB_PERIODIC_Q_SOC_INVALID;
    }
    if ((archive->mark_quality != STATE_ARCHIVE_MARK_FULL) &&
        (archive->soc.quality == SOC_EST_QUALITY_NONE))
    {
        flags |= BB_PERIODIC_Q_CC_INVALID | BB_PERIODIC_Q_TS1_INVALID;
    }
    else if (archive->mark_quality != STATE_ARCHIVE_MARK_FULL)
    {
        flags |= BB_PERIODIC_Q_TS1_INVALID;
    }
    return flags;
}

static int App_CommitArchiveSnapshot(const bq_anomaly_archive_t *archive)
{
    state_store_view_t view;
    uint16_t cells[3];
    uint16_t missing = (uint16_t)(archive->estimate_flags &
        (STATE_ARCHIVE_Q_CELL1_MISSING | STATE_ARCHIVE_Q_CELL2_MISSING |
         STATE_ARCHIVE_Q_CELL3_MISSING | STATE_ARCHIVE_Q_PACK_MISSING));
    uint16_t cells_flags = archive->estimate_flags;
    int result;

    cells[0] = archive->bq.cell_mv[0];
    cells[1] = archive->bq.cell_mv[1];
    cells[2] = archive->bq.cell_mv[2];
    if ((missing & STATE_ARCHIVE_Q_CELL1_MISSING) != 0u) cells[0] = 0u;
    if ((missing & STATE_ARCHIVE_Q_CELL2_MISSING) != 0u) cells[1] = 0u;
    if ((missing & STATE_ARCHIVE_Q_CELL3_MISSING) != 0u) cells[2] = 0u;

    if (archive->soc.quality != SOC_EST_QUALITY_NONE)
    {
        result = SocEst_FillStateStoreView(&archive->soc, cells,
                    ((missing & STATE_ARCHIVE_Q_PACK_MISSING) != 0u) ?
                    0u : archive->bq.pack_mv,
                    cells_flags, &view);
        if (result != 0)
        {
            return -1;
        }
    }
    else if (archive->mark_quality == STATE_ARCHIVE_MARK_CRITICAL)
    {
        StateStore_GetView(&view);
        if (StateStore_IsValid() == 0u)
        {
            memset(&view, 0, sizeof(view));
        }
        view.cell_mv[0] = cells[0];
        view.cell_mv[1] = cells[1];
        view.cell_mv[2] = cells[2];
        view.pack_mv = ((missing & STATE_ARCHIVE_Q_PACK_MISSING) != 0u) ?
                       0u : archive->bq.pack_mv;
        view.flags &= (uint16_t)~(STATE_FLAG_SOC_VALID | STATE_FLAG_PACK_VALID);
        view.flags |= STATE_FLAG_PACK_PARTIAL;
        view.flags &= (uint16_t)~STATE_FLAG_SOC_ESTIMATED;
        cells_flags |= STATE_ARCHIVE_Q_SOC_INVALID;
    }
    else
    {
        return -1;
    }

    return StateStore_CommitArchiveSnapshot(&view, archive->mark_epoch,
                    archive->mark_id, archive->mark_quality, (uint8_t)cells_flags);
}

static void App_ArchiveTask(uint32_t now_ms)
{
    bq_anomaly_archive_t request;
    state_store_view_t stored;
    rtc_epoch_t rtc;
    uint32_t mark_epoch = 0u;
    uint16_t bb_flags;
    int result;

    if (s_app.pending_lp_mark_active == 0u &&
        LpCycle_GetPendingMark(&mark_epoch) != 0u)
    {
        s_app.pending_lp_mark_epoch = mark_epoch;
        s_app.pending_lp_mark_active = 1u;
        s_app.pending_lp_mark_lp_archived = LpCycle_IsPendingMarkLpArchived();
    }

    if (s_app.pending_lp_mark_active != 0u)
    {
        StateStore_GetView(&stored);
        if ((StateStore_IsValid() != 0u) &&
            (stored.archive.last_mark_id == s_app.pending_lp_mark_epoch))
        {
            if (stored.archive.mark_quality == STATE_ARCHIVE_MARK_FULL)
            {
                BqAnomaly_NotifyLpFullMarkCommitted(
                    s_app.pending_lp_mark_epoch, s_app.pending_lp_mark_epoch);
            }
            if (LpCycle_AckPendingMark(s_app.pending_lp_mark_epoch) != 0u)
            {
                s_app.pending_lp_mark_active = 0u;
                s_app.pending_lp_mark_lp_archived = 0u;
            }
        }
    }

    if ((s_app.pending_lp_mark_active != 0u) &&
        (s_app.norm_mark_active == 0u))
    {
        if (s_app.pending_lp_mark_lp_archived != 0u)
        {
            result = StateStore_CommitArchiveMark(s_app.pending_lp_mark_epoch,
                          s_app.pending_lp_mark_epoch, STATE_ARCHIVE_MARK_FULL, 0u);
            if (result >= 0 &&
                LpCycle_AckPendingMark(s_app.pending_lp_mark_epoch) != 0u)
            {
                BqAnomaly_NotifyLpFullMarkCommitted(
                    s_app.pending_lp_mark_epoch, s_app.pending_lp_mark_epoch);
                s_app.pending_lp_mark_active = 0u;
                s_app.pending_lp_mark_lp_archived = 0u;
            }
            return;
        }
        if ((LpCycle_GetWakeCause() & LP_WAKE_FAULT) != 0u)
        {
            /* LP 的 I²C/真实保护故障不能伪装成电压异常归档。 */
            return;
        }
        if (BqAnomaly_BeginNormMark(s_app.pending_lp_mark_epoch,
                                    s_app.pending_lp_mark_epoch, now_ms) != 0u)
        {
            s_app.norm_mark_active = 1u;
        }
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
        if (BqAnomaly_BeginNormMark(rtc.sec, rtc.sec, now_ms) != 0u)
        {
            s_app.norm_mark_active = 1u;
            s_app.flag_blackbox_request = 0u;
        }
    }

    if (BqAnomaly_GetArchiveRequest(&request) == 0u)
    {
        return;
    }
    if ((s_app.archive_cache_valid == 0u) ||
        (s_app.archive_mark_id != request.mark_id))
    {
        s_app.archive_cache = request;
        s_app.archive_cache_valid = 1u;
        s_app.archive_mark_id = request.mark_id;
        s_app.archive_bb_written = request.bb_written;
    }

    if ((s_app.archive_bb_written == 0u) &&
        (BQ_IsIdle() != 0u) && (BQ_GetState() == BQ_ST_RUN))
    {
        /* 两个 Flash 区不能跨区原子提交，成功后仅在本次启动缓存 mark_id。 */
        bb_flags = App_ArchiveBlackBoxFlags(&s_app.archive_cache);
        result = BlackBox_WritePeriodicQualified(&s_app.archive_cache.bq,
                    App_GetUptimeSeconds(), bb_flags);
        if (result != 0)
        {
            return;
        }
        s_app.archive_bb_written = 1u;
        BqAnomaly_NoteBlackBoxWritten(s_app.archive_mark_id);
        if ((s_app.pending_lp_mark_active != 0u) &&
            (s_app.pending_lp_mark_epoch == s_app.archive_mark_id))
        {
            if (LpCycle_NotePendingMarkArchived(s_app.archive_mark_id) == 0u)
            {
                return;
            }
            s_app.pending_lp_mark_lp_archived = 1u;
        }
    }

    if ((s_app.archive_bb_written != 0u) &&
        (s_app.pending_lp_mark_active != 0u) &&
        (s_app.pending_lp_mark_epoch == s_app.archive_mark_id) &&
        (LpCycle_IsPendingMarkLpArchived() == 0u))
    {
        if (LpCycle_NotePendingMarkArchived(s_app.archive_mark_id) == 0u)
        {
            return;
        }
        s_app.pending_lp_mark_lp_archived = 1u;
    }
    if ((s_app.archive_bb_written == 0u) ||
        (BQ_IsIdle() == 0u) || (BQ_GetState() != BQ_ST_RUN))
    {
        return;
    }

    result = App_CommitArchiveSnapshot(&s_app.archive_cache);
    if (result < 0)
    {
        return;
    }

    BqAnomaly_ArchiveCommitted(s_app.archive_mark_id);
    if ((s_app.pending_lp_mark_active != 0u) &&
        (s_app.pending_lp_mark_epoch == s_app.archive_mark_id))
    {
        if (LpCycle_AckPendingMark(s_app.pending_lp_mark_epoch) != 0u)
        {
            s_app.pending_lp_mark_active = 0u;
            s_app.pending_lp_mark_lp_archived = 0u;
        }
    }
    s_app.norm_mark_active = 0u;
    s_app.archive_cache_valid = 0u;
    s_app.archive_bb_written = 0u;
    s_app.archive_mark_id = 0u;
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
        if (s_app.anomaly_status.partial_active == 0u &&
            s_app.anomaly_status.alarm_active == 0u &&
            ((SelfTest_GetFailMask() & (uint16_t)~SELFTEST_FAIL_BQ_DATA) != 0u ||
             (Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                                FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                                FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                                FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                                FAULT_BIT_POWER | FAULT_BIT_CLK)) != 0u))
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
                       "fv=%u tv=%u cv=%u upd=%lu\r\n",
                       (long)soc.soc01, (long)soc.soh01, (int)soc.temp_dc,
                       (unsigned)soc.ts1_raw, (unsigned)soc.ts1_mv,
                       (unsigned)soc.frame_valid, (unsigned)soc.temp_valid,
                       (unsigned)soc.cal_valid, (unsigned long)soc.update_count);
    }
    App_Log_Printf("[DIAG] sm=%u flow=%u V=%u/%u/%u P=%u CC=%d I=%ld TS=%u "
                   "flt=%04X bq=%u lpblk=%04X up=%lu\r\n",
                   (unsigned)StateMachine_GetState(),
                   (unsigned)StateMachine_GetFlow(),
                   (unsigned)snapshot.cell_mv[0],
                   (unsigned)snapshot.cell_mv[1],
                   (unsigned)snapshot.cell_mv[2],
                   (unsigned)snapshot.pack_mv,
                   (int)snapshot.cc_raw,
                   (long)snapshot.current_ma,
                   (unsigned)snapshot.ts1_raw,
                   (unsigned)Fault_GetAll(),
                   (unsigned)BQ_GetState(),
                   (unsigned)s_app.auto_lp_block,
                   (unsigned long)App_GetUptimeSeconds());

    App_Log_Printf("[BQ] pre=%02X/%02X ss=%02X bal=%02X post=%02X/%02X "
                   "ss=%02X off=%02X/%02X rst=%lu a=%u r=%u f=%u\r\n",
                   (unsigned)snapshot.pre_ctrl1,
                   (unsigned)snapshot.pre_ctrl2,
                   (unsigned)snapshot.pre_sys_stat,
                   (unsigned)snapshot.pre_cellbal1,
                   (unsigned)snapshot.post_ctrl1,
                   (unsigned)snapshot.post_ctrl2,
                   (unsigned)snapshot.post_sys_stat,
                   (unsigned)snapshot.adc_off_ctrl1,
                   (unsigned)snapshot.adc_off_ctrl2,
                   (unsigned long)snapshot.adc_restart_count,
                   (unsigned)snapshot.adc_restart_attempts,
                   (unsigned)snapshot.adc_restart_reason,
                   (unsigned)snapshot.adc_recovery_failed);
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
                   "inv=%u clr=%u\r\n",
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
                   (unsigned)prot.clear_block_reason);

    App_Log_Printf("[BQ] oor=%u rs=%u sw=%u ep=%u va=%u gp=%u "
                   "vd=%lu rl=%lu\r\n",
                   (unsigned)snapshot.oor_streak,
                   (unsigned)snapshot.resample_count,
                   (unsigned)snapshot.strict_window,
                   (unsigned)snapshot.recovery_episode,
                   (unsigned)snapshot.verdict_active,
                   (unsigned)snapshot.good_publish_streak,
                   (unsigned long)snapshot.verdict_deadline_ms,
                   (unsigned long)snapshot.adc_restart_last_ms);

    App_Log_Printf("[BQAN] q=%u b=%lu p=%lu cc=%lu/%u hot=%u alarm=%u "
                   "hold=%u retry=%u marks=%u full9=%u latch=%u lock=%u ship=%u\r\n",
                   (unsigned)s_app.batch_quality.quality,
                   (unsigned long)s_app.batch_quality.batch_seq,
                   (unsigned long)s_app.batch_quality.published_seq,
                   (unsigned long)s_app.cc_sample.sample_seq,
                   (unsigned)s_app.cc_sample.valid,
                   (unsigned)s_app.anomaly_status.hot_flash,
                   (unsigned)s_app.anomaly_status.alarm_active,
                   (unsigned)s_app.anomaly_status.boot_blue_hold,
                   (unsigned)s_app.anomaly_status.retry_due_off,
                   (unsigned)s_app.anomaly_status.anomaly_mark_count,
                   (unsigned)s_app.anomaly_status.full_streak,
                   (unsigned)s_app.anomaly_status.output_inhibit_latched,
                   (unsigned)s_app.anomaly_status.shutdown_confirmed,
                   (unsigned)s_app.anomaly_status.stop2_ship_trigger);

    App_Log_Printf("[POL] st=%u req=%u app=%u chg=%u dis=%u cal=%u rsn=%u\r\n",
                   (unsigned)ChargePolicy_GetState(),
                   (unsigned)ChargePolicy_GetRequestedMask(),
                   (unsigned)BQ_GetFetMaskApplied(),
                   (unsigned)((ChargePolicy_GetState() ==
                               CHARGE_POLICY_CHARGE) ? 1u : 0u),
                   (unsigned)((ChargePolicy_GetState() ==
                               CHARGE_POLICY_DISCHARGE) ? 1u : 0u),
                   (unsigned)ChargePolicy_IsCalibration(),
                   (unsigned)ChargePolicy_GetCalEndReason());
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
    s_app.last_soc_input_batch_seq = 0u;
    s_app.full_gate_seq = 0u;
    s_app.full_gate_ms = 0u;
    s_app.full_gate_passed_this_boot = 0u;
    s_app.norm_mark_active = 0u;
    s_app.archive_cache_valid = 0u;
    s_app.archive_bb_written = 0u;
    s_app.archive_mark_id = 0u;
    s_app.pending_lp_mark_epoch = 0u;
    s_app.pending_lp_mark_active = 0u;
    s_app.pending_lp_mark_lp_archived = 0u;
    s_app.last_voltage_events = 0u;
    s_app.anomaly_buzzer_owned = 0u;
    memset(&s_app.batch_quality, 0, sizeof(s_app.batch_quality));
    memset(&s_app.cc_sample, 0, sizeof(s_app.cc_sample));
    memset(&s_app.anomaly_status, 0, sizeof(s_app.anomaly_status));
    memset(&s_app.anomaly_inputs, 0, sizeof(s_app.anomaly_inputs));

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
    BqAnomaly_Init((LpCycle_GetWakeCause() != 0u) ?
                   BQ_ANOMALY_BOOT_LP_WAKE : BQ_ANOMALY_BOOT_COLD,
                   HAL_GetTick());
    CalStream_Init();

    (void)HAL_COMP_Start(&hcomp2);

    s_app.diag_log_ms = HAL_GetTick();
    App_TickTimerInit();
    App_TickStart();           /* ★ 时基最后启动 */
    Watchdog_Feed();

    s_app.last_activity_ms = HAL_GetTick();
    LpCycle_ApplyWake();
    BqAnomaly_GetStatus(&s_app.anomaly_status);
    ChargePolicy_SetArchiveLatch(s_app.anomaly_status.output_inhibit_latched);
}

void App_Loop(void)
{
    uint32_t now_ms;
    uint8_t had_activity = 0u;
    button_event_t button_ev;
    gesture_event_t gesture_ev;
    bq_snapshot_t bq_snap;
    soc_est_snapshot_t soc_snap;
    protection_partial_evidence_t evidence;

    Watchdog_Feed();

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
    BQ_GetBatchQuality(&s_app.batch_quality);
    BQ_GetCcSample(&s_app.cc_sample);
    if ((s_app.batch_quality.quality == (uint8_t)BQ_BATCH_FULL_REAL_OK) &&
        (s_app.batch_quality.published_seq != 0u) &&
        (s_app.batch_quality.published_ms == bq_snap.last_update_ms) &&
        (s_app.batch_quality.batch_seq != s_app.last_soc_input_batch_seq))
    {
        SocEst_OnSnapshot(&bq_snap, now_ms);
        s_app.last_soc_input_batch_seq = s_app.batch_quality.batch_seq;
    }
    else if ((s_app.batch_quality.quality == (uint8_t)BQ_BATCH_PARTIAL_VC5_PACK) &&
             (s_app.batch_quality.complete != 0u) &&
             (s_app.cc_sample.valid != 0u) &&
             (s_app.cc_sample.batch_seq == s_app.batch_quality.batch_seq) &&
             (BQ_IsCcFresh(now_ms) != 0u) &&
             (s_app.batch_quality.batch_seq != s_app.last_soc_input_batch_seq))
    {
        SocEst_OnPartialSample(&s_app.batch_quality, &s_app.cc_sample);
        s_app.last_soc_input_batch_seq = s_app.batch_quality.batch_seq;
    }
    else if ((s_app.batch_quality.quality == (uint8_t)BQ_BATCH_INVALID ||
              s_app.batch_quality.quality == (uint8_t)BQ_BATCH_PARTIAL_CANDIDATE ||
              ((s_app.batch_quality.quality == (uint8_t)BQ_BATCH_PARTIAL_VC5_PACK) &&
               (s_app.cc_sample.valid == 0u ||
                s_app.cc_sample.batch_seq != s_app.batch_quality.batch_seq ||
                BQ_IsCcFresh(now_ms) == 0u))) &&
             (s_app.batch_quality.batch_seq != s_app.last_soc_input_batch_seq))
    {
        SocEst_InvalidateInput();
        s_app.last_soc_input_batch_seq = s_app.batch_quality.batch_seq;
    }
    SocEst_Task();
    SocEst_GetSnapshot(&soc_snap);
    App_BqAdcEventTask();

    /* 动态 BQ 失败只有严格 PARTIAL 可以延缓；真正 FULL 才可通过自检。 */
    SelfTest_SetVoltageAnomaly(
        (s_app.batch_quality.quality == (uint8_t)BQ_BATCH_PARTIAL_VC5_PACK) ? 1u : 0u,
        (s_app.batch_quality.quality == (uint8_t)BQ_BATCH_FULL_REAL_OK &&
         s_app.batch_quality.published_ms == bq_snap.last_update_ms) ? 0u : 1u);
    SelfTest_Poll(now_ms);
    App_HandleSelfTestCompletion();

    /* 4) 1 s 诊断（BQ 新鲜度 → FAULT_BQ_STALE + 10 s 日志）。 */
    if (s_app.flag_diag_request != 0u)
    {
        s_app.flag_diag_request = 0u;
        App_DiagTask(now_ms);
    }

    if ((s_app.batch_quality.quality == (uint8_t)BQ_BATCH_FULL_REAL_OK) &&
        (s_app.batch_quality.published_seq != 0u) &&
        (s_app.batch_quality.published_seq != s_app.full_gate_seq) &&
        (s_app.batch_quality.published_ms == bq_snap.last_update_ms) &&
        (SelfTest_IsPassed() != 0u) &&
        (Protection_GetState() == PROTECTION_NORMAL) &&
        (s_app.anomaly_status.output_inhibit_latched == 0u) &&
        ((Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                            FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                            FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                            FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                            FAULT_BIT_POWER | FAULT_BIT_CLK)) == 0u))
    {
        s_app.full_gate_seq = s_app.batch_quality.published_seq;
        s_app.full_gate_ms = s_app.batch_quality.published_ms;
        s_app.full_gate_passed_this_boot = 1u;
    }
    memset(&evidence, 0, sizeof(evidence));
    evidence.batch = s_app.batch_quality;
    evidence.cc = s_app.cc_sample;
    evidence.partial_scope_open =
        (s_app.batch_quality.quality == (uint8_t)BQ_BATCH_PARTIAL_VC5_PACK) ? 1u : 0u;
    evidence.comp_direction_valid =
        ((HAL_COMP_GetState(&hcomp2) == HAL_COMP_STATE_BUSY) &&
         (s_app.comparator_match_count >= APP_COMP_DEBOUNCE_SAMPLES)) ? 1u : 0u;
    evidence.comp_charging = s_app.comparator_charging;
    evidence.full_gate_passed_this_boot = s_app.full_gate_passed_this_boot;
    evidence.full_gate_ms = s_app.full_gate_ms;
    evidence.full_gate_seq = s_app.full_gate_seq;
    Protection_SetPartialEvidence(&evidence);

    /* 保护监督层必须先于状态机；不能让估算电压满足原 fresh 判据。 */
    Protection_Task(now_ms, &bq_snap, s_app.comparator_charging,
                    SocEst_IsTempValid(), SocEst_GetTempDc());
    StateMachine_SetPartialAnomaly(Protection_IsPartialAnomaly());
    StateMachine_Update(BQ_IsFresh(now_ms), s_app.comparator_charging,
                        Fault_GetAll());
    ChargePolicy_SetArchiveLatch(s_app.anomaly_status.output_inhibit_latched);
    ChargePolicy_Task(now_ms, s_app.comparator_charging);
    Protection_Service(now_ms, &bq_snap);
    App_ProtectionEventTask();

    App_AnomalyBuildInputs(&s_app.batch_quality);
    BqAnomaly_Task(now_ms, &s_app.batch_quality, &s_app.cc_sample,
                   &bq_snap, &soc_snap, &s_app.anomaly_inputs);
    BqAnomaly_GetStatus(&s_app.anomaly_status);
    if ((s_app.anomaly_status.latch_due != 0u) ||
        (s_app.anomaly_status.output_inhibit_latched != 0u))
    {
        ChargePolicy_SetArchiveLatch(1u);
        ChargePolicy_Task(now_ms, s_app.comparator_charging);
    }
    if ((s_app.anomaly_status.retry_due_off != 0u) &&
        (s_app.anomaly_inputs.output_on == 0u))
    {
        uint8_t started = BQ_RequestPartialRetry();
        BqAnomaly_NotifyPartialRetry(now_ms, started);
    }
    if (BqAnomaly_TakeRecheckDue() != 0u)
    {
        BQ_RequestRead();
    }
    if (s_app.anomaly_inputs.bq_idle != 0u)
    {
        (void)BqAnomaly_ServicePersistence();
    }
    BqAnomaly_GetStatus(&s_app.anomaly_status);
    App_AnomalyEventTask();
    if ((s_app.anomaly_status.buzzer_active != 0u) &&
        (StateMachine_GetState() != SM_STATE_FAULT) &&
        (s_app.anomaly_buzzer_owned == 0u))
    {
        Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
        s_app.anomaly_buzzer_owned = 1u;
    }
    else if ((s_app.anomaly_status.buzzer_active != 0u) &&
             (StateMachine_GetState() != SM_STATE_FAULT) &&
             (s_app.anomaly_buzzer_owned != 0u) &&
             (Buzzer_IsActive() == 0u))
    {
        Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
    }
    if ((s_app.anomaly_status.buzzer_active == 0u) &&
        (s_app.anomaly_buzzer_owned != 0u))
    {
        if (StateMachine_GetState() != SM_STATE_FAULT)
        {
            Buzzer_Stop();
        }
        s_app.anomaly_buzzer_owned = 0u;
    }
    CalStream_Task(now_ms, App_GetUptimeSeconds(), &bq_snap);
    LedStatus_SetVoltageAnomaly(s_app.anomaly_status.partial_active,
                                s_app.anomaly_status.alarm_active);
    App_UpdateLedMode();

    /* 先处理 LP 待标记和 NORM 30 min 标记，再判断能否进 LP；否则
     * 标记到达的这一圈可能抢先复位，造成档案窗口漏处理。 */
    App_ArchiveTask(now_ms);

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
        if ((SelfTest_IsDone() == 0u) || (SelfTest_IsPassed() == 0u))
        {
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

        if ((s_app.anomaly_status.auto_lp_blocked != 0u) ||
            (s_app.pending_lp_mark_active != 0u) ||
            (s_app.norm_mark_active != 0u) ||
            (LpCycle_GetPendingMark(NULL) != 0u))
        {
            block |= APP_LP_BLOCK_VOLTAGE;
        }
        /* 异常 LP 仅能在已有真好帧完成自检且没有任何真实故障时进入。
         * 无好帧的冷启动/监测失败不能通过清阻止位进入轻量路径。 */
        if ((s_app.anomaly_status.partial_active != 0u) &&
            (s_app.anomaly_status.full_seen_this_boot != 0u) &&
            (s_app.anomaly_inputs.adc_cc_on != 0u) &&
            (s_app.anomaly_inputs.output_on == 0u) &&
            (SelfTest_IsPassed() != 0u) &&
            (s_app.anomaly_inputs.hardware_ok != 0u) &&
            (s_app.anomaly_inputs.link_ok != 0u) &&
            (s_app.anomaly_status.auto_lp_blocked == 0u) &&
            (s_app.pending_lp_mark_active == 0u) &&
            (s_app.norm_mark_active == 0u) &&
            (StateMachine_GetState() == SM_STATE_BQ_ANOMALY) &&
            (ChargePolicy_GetState() == CHARGE_POLICY_IDLE) &&
            ((Fault_GetAll() & (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY |
                                FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                                FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |
                                FAULT_BIT_BQ_ADC_STUCK | FAULT_BIT_PROTECTION |
                                FAULT_BIT_POWER | FAULT_BIT_CLK)) == 0u))
        {
            block &= (uint16_t)~(APP_LP_BLOCK_STATE | APP_LP_BLOCK_BQ |
                                 APP_LP_BLOCK_PROTECTION);
        }

        if ((StateMachine_GetState() != SM_STATE_NORM &&
             StateMachine_GetState() != SM_STATE_BQ_ANOMALY) ||
            (s_app.anomaly_status.auto_lp_blocked != 0u) ||
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

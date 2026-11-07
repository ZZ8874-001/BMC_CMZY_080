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

/* s_app.auto_lp_block：自动 LP 被哪一项条件挡住（Ozone 直接看）。 */
#define APP_LP_BLOCK_CHARGING       (1u << 0)
#define APP_LP_BLOCK_BUTTON         (1u << 1)
#define APP_LP_BLOCK_TEST           (1u << 2)   /* 自检未完成/失败 */
#define APP_LP_BLOCK_STATE          (1u << 3)   /* 状态机非 NORM */
#define APP_LP_BLOCK_BQ             (1u << 4)   /* BQ 不新鲜 */
#define APP_LP_BLOCK_TIMEBASE       (1u << 5)   /* RTC 时基（LP_RTC_USE_LSE 选中源）不可用 */
#define APP_LP_BLOCK_POWER          (1u << 6)   /* 电源模式非 NORM */
#define APP_LP_BLOCK_POLICY         (1u << 7)   /* CHARGE/DISCHARGE/CAL 或 CAL pending */

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
    uint8_t auto_lp_block;          /* 自动 LP 阻止原因位图（[DIAG] 使用） */

    /* 降级模式（80 MHz 切换失败）灯闪 */
    uint32_t degraded_blink_ms;
    uint8_t degraded_blink_on;
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

    if (StateMachine_GetState() == SM_STATE_FAULT)
    {
        LedStatus_SetMode(LED_MODE_FAULT, LED_FLOW_FAULT);
        return;
    }

    policy_state = ChargePolicy_GetState();
    if (policy_state == CHARGE_POLICY_CAL)
    {
        /* CAL 允许绕过自检失败；BQ 就绪性由策略层保证。 */
        LedStatus_SetMode(LED_MODE_NORM, LED_FLOW_CAL);
        return;
    }

    if ((SelfTest_IsDone() != 0u) && (SelfTest_IsPassed() == 0u))
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
        Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
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
                   "flt=%04X bq=%u lpblk=%02X up=%lu\r\n",
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

    Fault_Init();
    PowerMode_Init();          /* 16 MHz → 80 MHz；核对 RTC 源/LSE 健康 */
    Watchdog_Feed();

    BlackBox_Init();
    StateStore_Init();
    s_app.uptime_base_s = BlackBox_GetUptimeBase();

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
    CalStream_Init();

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
    SocEst_OnSnapshot(&bq_snap, now_ms);
    SocEst_Task();
    App_BqAdcEventTask();

    /* 3) 自检与状态分类。 */
    SelfTest_Poll(now_ms);
    App_HandleSelfTestCompletion();

    /* 4) 1 s 诊断（BQ 新鲜度 → FAULT_BQ_STALE + 10 s 日志）。 */
    if (s_app.flag_diag_request != 0u)
    {
        s_app.flag_diag_request = 0u;
        App_DiagTask(now_ms);
    }

    StateMachine_Update(BQ_IsFresh(now_ms), s_app.comparator_charging,
                        Fault_GetAll());
    ChargePolicy_Task(now_ms, s_app.comparator_charging);
    CalStream_Task(now_ms, App_GetUptimeSeconds(), &bq_snap);
    App_UpdateLedMode();

    /* 4b) 自动 LP：逐项记录阻止原因，5 s 无活动且无阻止项时请求 LP。
     * 请求后重置计时，避免准备失败时每圈重试。 */
    {
        uint8_t block = 0u;

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

        s_app.auto_lp_block = block;
        if ((block == 0u) &&
            ((now_ms - s_app.last_activity_ms) >= APP_IDLE_LP_MS))
        {
            PowerMode_RequestLp();
            s_app.last_activity_ms = now_ms;
        }
    }

    /* 5) 黑匣子 30 min 记录。 */
    if (s_app.flag_blackbox_request != 0u)
    {
        s_app.flag_blackbox_request = 0u;
        (void)BlackBox_WritePeriodic(App_GetUptimeSeconds());
        (void)SocEst_CommitStateStore();
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

/**
 ******************************************************************************
 * @file    main_app.c
 * @brief   应用编排：App_Init / App_Loop / 250 Hz TIM2 时基。
 *
 * 初始化顺序（无时间预算，允许保守阻塞）：
 *   复位原因 → 故障图 → 电源模式（HSI16 → HSE/PLL 80 MHz + LSE/RTC）
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
#include "app/faults.h"
#include "app/led_status.h"
#include "app/power_mode.h"
#include "app/reset_cause.h"
#include "app/self_test.h"
#include "app/state_machine.h"
#include "bsp/bq_ic.h"
#include "bsp/buzzer.h"
#include "bsp/ws2812_uart.h"
#include "comm/app_log.h"

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

static struct
{
    /* 时基（TIM2 中断写） */
    volatile uint32_t tick_250hz_count;
    volatile uint32_t uptime_s;
    uint32_t uptime_base_s;
    volatile uint8_t tick_running;

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
    uint8_t boot_announced;
    uint32_t diag_log_ms;
    uint32_t loop_count;

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
    s_app.tick_running = 1u;
}

void App_TickStop(void)
{
    (void)HAL_TIM_Base_Stop_IT(&htim2);
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    s_app.tick_running = 0u;
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

    s_app.tick_250hz_count++;

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
        if (StateMachine_GetFlow() == SM_FLOW_CHARGING)
        {
            flow = LED_FLOW_CHARGING;
        }
        else if (StateMachine_GetFlow() == SM_FLOW_DISCHARGING)
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

static void App_HandleSelfTestCompletion(void)
{
    if ((SelfTest_IsDone() == 0u) || (s_app.boot_announced != 0u))
    {
        return;
    }

    s_app.boot_announced = 1u;
    if (SelfTest_IsPassed() != 0u)
    {
        Buzzer_StartPattern(BUZZER_PATTERN_BOOT_OK);
        App_Log_Printf("[TEST] pass\r\n");
    }
    else
    {
        Buzzer_StartPattern(BUZZER_PATTERN_FAULT);
        App_Log_Printf("[TEST] fail=0x%04X\r\n", SelfTest_GetFailMask());
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
    App_Log_Printf("[DIAG] sm=%u flow=%u V=%u/%u/%u P=%u CC=%d I=%ld TS=%u "
                   "flt=%04X bq=%u up=%lu\r\n",
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
                   (unsigned long)App_GetUptimeSeconds());
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

void App_Init(void)
{
    s_app.tick_250hz_count = 0u;
    s_app.uptime_s = 0u;
    s_app.uptime_base_s = 0u;
    s_app.tick_running = 0u;
    s_app.flag_bq_read_request = 0u;
    s_app.flag_diag_request = 0u;
    s_app.flag_blackbox_request = 0u;
    s_app.prescale_500ms = 0u;
    s_app.prescale_1s = 0u;
    s_app.prescale_30min = 0u;
    s_app.comparator_charging = 0u;
    s_app.comparator_last_sample = 0u;
    s_app.comparator_match_count = 0u;
    s_app.boot_announced = 0u;
    s_app.diag_log_ms = 0u;
    s_app.loop_count = 0u;
    s_app.degraded_blink_ms = 0u;
    s_app.degraded_blink_on = 0u;

    ResetCause_Init();
    Fault_Init();
    PowerMode_Init();          /* 16 MHz → 80 MHz；LSE/RTC 初始化 */

    BlackBox_Init();
    s_app.uptime_base_s = BlackBox_GetUptimeBase();

    if (PowerMode_IsDegraded() != 0u)
    {
        /* 时钟降级：只留故障灯闪烁，通信/BQ 全部不启动。 */
        HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_RESET);
        return;
    }

    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(APP_5V_STARTUP_DELAY_MS);

    App_Log_Init();
    App_Log_Printf("[BOOT] app init, reset=0x%02X\r\n",
                   (unsigned)ResetCause_GetLatched());

    BQ_Init();
    /* Ws2812_HwInit 按 80 MHz 重设 USART2SEL=PCLK1 并在 UE=0 时写 BRR。 */
    Ws2812_Init();
    LedStatus_Init();
    Buzzer_Init();

    SelfTest_Init();
    SelfTest_Request();
    StateMachine_Init();

    (void)HAL_COMP_Start(&hcomp2);

    s_app.diag_log_ms = HAL_GetTick();
    App_TickTimerInit();
    App_TickStart();           /* ★ 时基最后启动 */
}

void App_Loop(void)
{
    uint32_t now_ms;

    if (PowerMode_IsDegraded() != 0u)
    {
        App_DegradedLoop();
        return;
    }

    now_ms = HAL_GetTick();

    /* 1) LP 请求（调试器写 s_power.request_lp=1）——阻塞到唤醒返回。 */
    PowerMode_Loop();

    /* 2) BQ 事务推进 + 500 ms 周期读。 */
    if (s_app.flag_bq_read_request != 0u)
    {
        s_app.flag_bq_read_request = 0u;
        BQ_RequestRead();
    }
    BQ_Poll();

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
    App_UpdateLedMode();

    /* 5) 黑匣子 30 min 记录。 */
    if (s_app.flag_blackbox_request != 0u)
    {
        s_app.flag_blackbox_request = 0u;
        (void)BlackBox_WritePeriodic(App_GetUptimeSeconds());
    }

    /* 6) 蜂鸣器 / RGB / 日志。 */
    Buzzer_Poll();
    LedStatus_Flush();

    App_Log_Poll();
    App_Log_FlushSmall(4u);

    s_app.loop_count++;
}

/* ---------------- 中断 / weak 回调 ---------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        App_OnTick250Hz();
    }
}

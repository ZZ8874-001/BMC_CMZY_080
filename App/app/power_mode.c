/**
 ******************************************************************************
 * @file    power_mode.c
 * @brief   NORM ⇄ LP（STOP1）电源模式与 HSE/PLL 时钟切换。
 *
 * 时钟：
 *   - 复位后运行 HSI16（CubeMX 生成），本模块在 App_Init 早期切到 HSE+PLL 80 MHz；
 *   - 唯一入口 PowerMode_ClkSwitchToHsePll，LP 唤醒后复用；
 *   - PA1（HSE_DIS/CHGSC_DIS）低 = OSC1 使能 + PACK_SC 前端使能。
 *
 * LP 进入顺序（不可打乱）：
 *   排空通信 → BQ 关 ADC/TEMP、CC_EN=0/清状态 → 黑匣子转换记录 → 停外设 → 灭灯 →
 *   布防 RTC 20 s → 清挂起 → SYSCLK 切 MSI 4 MHz → 关 PLL → 关 HSE →
 *   PA1=1/PB0=0 → WFI(STOP1)。
 *   唤醒后：MSI 下先修 SysTick → PA1=0 → HSE/PLL 80 MHz → PB0=1 → 重建外设。
 *
 * 唤醒原因：RTC（WUT 中断）、PB5 按下（EXTI5 中断）、PA8 BQ ALERT（EXTI 事件，
 * 无中断标志）——三者都清过挂起，因此“无 RTC/按键标志”即判定为 BQ ALERT。
 ******************************************************************************
 */
#include "app/power_mode.h"

#include "board.h"
#include "app/main_app.h"
#include "app/black_box.h"
#include "app/faults.h"
#include "comm/app_log.h"
#include "bsp/bq_ic.h"
#include "bsp/buzzer.h"
#include "bsp/ws2812_uart.h"

#define POWER_CLK_RETRY             3u
#define POWER_RTC_WUT_SECONDS       20u
#define POWER_DRAIN_TIMEOUT_MS      100u

static struct
{
    power_state_t state;
    volatile uint8_t request_lp;     /* 调试器写 1 请求进入 LP（主循环消费） */
    volatile uint8_t wake_cause;
    volatile uint8_t flag_rtc;
    volatile uint8_t flag_button;
    uint8_t lse_ok;
    uint8_t degraded;
    uint8_t clock_ok;
    uint32_t lp_enter_count;
    uint32_t wake_count;
    uint32_t wake_rtc_count;
    uint32_t wake_button_count;
    uint32_t wake_alert_count;
    uint32_t clock_fail_count;
    uint32_t lp_prepare_fail_count;
    uint32_t wake_clock_fail_count;
} s_power;

uint8_t PowerMode_ClkSwitchToHsePll(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    uint8_t attempt;

    /* OSC1 tri-state 使能（低有效），与 PACK_SC 前端同一网络。 */
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_RESET);

    for (attempt = 0u; attempt < POWER_CLK_RETRY; attempt++)
    {
        /* OSC1 起振余量：冷启动 16 MHz 下 30 ms；唤醒时 SysTick 已修到 1 ms。 */
        HAL_Delay(30u);

        osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        osc.HSEState = RCC_HSE_BYPASS;
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        osc.PLL.PLLM = 3u;
        osc.PLL.PLLN = 40u;
        osc.PLL.PLLP = RCC_PLLP_DIV7;
        osc.PLL.PLLQ = RCC_PLLQ_DIV2;
        osc.PLL.PLLR = RCC_PLLR_DIV2;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        {
            s_power.clock_fail_count++;
            continue;
        }

        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        {
            s_power.clock_fail_count++;
            continue;
        }

        clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
        clk.APB1CLKDivider = RCC_HCLK_DIV1;
        clk.APB2CLKDivider = RCC_HCLK_DIV1;
        if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK)
        {
            s_power.clock_fail_count++;
            continue;
        }

        s_power.clock_ok = 1u;
        return 1u;
    }

    s_power.clock_ok = 0u;
    return 0u;
}

void PowerMode_Init(void)
{
    s_power.state = POWER_STATE_INIT;
    s_power.request_lp = 0u;
    s_power.wake_cause = 0u;
    s_power.flag_rtc = 0u;
    s_power.flag_button = 0u;
    s_power.lse_ok = 0u;
    s_power.degraded = 0u;
    s_power.clock_ok = 0u;
    s_power.lp_enter_count = 0u;
    s_power.wake_count = 0u;
    s_power.wake_rtc_count = 0u;
    s_power.wake_button_count = 0u;
    s_power.wake_alert_count = 0u;
    s_power.clock_fail_count = 0u;
    s_power.lp_prepare_fail_count = 0u;
    s_power.wake_clock_fail_count = 0u;

    __HAL_RCC_PWR_CLK_ENABLE();

    /* LSE/RTC 由生成的 MX_RTC_Init（IOC 中的 RTC 条目）初始化，这里只核对状态。 */
    s_power.lse_ok = ((__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u) &&
                      (HAL_RTC_GetState(&hrtc) == HAL_RTC_STATE_READY)) ? 1u : 0u;
    if (s_power.lse_ok == 0u)
    {
        Fault_SetSticky(FAULT_BIT_LSE);
    }

    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

    /* PB5 按下唤醒兜底（IOC 已为下降沿，这里幂等覆写）。 */
    EXTI->RTSR1 &= (uint32_t)~EXTI_RTSR1_RT5;
    EXTI->FTSR1 |= EXTI_FTSR1_FT5;
    EXTI->PR1 = EXTI_PR1_PIF5;

    if (PowerMode_ClkSwitchToHsePll() == 0u)
    {
        s_power.degraded = 1u;
        s_power.state = POWER_STATE_DEGRADED;
        Fault_SetSticky(FAULT_BIT_CLK);
        return;
    }

    s_power.degraded = 0u;
    s_power.state = POWER_STATE_NORM;
}

static void PowerMode_StopPeripherals(void)
{
    App_TickStop();
    App_Log_DeInitForLp();
    Ws2812_DeInitForLp();
    BQ_DeInitForLp();
    (void)HAL_COMP_Stop(&hcomp2);
    Buzzer_DeInitForLp();
    Fault_PvdDisable();

    HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_ROHT_GPIO_Port, IND_ROHT_Pin, GPIO_PIN_RESET);
}

static void PowerMode_RestorePeripherals(void)
{
    /* 顺序敏感：BQ 先于 WS2812 初始化（两者都会重配 DMA/NVIC 与 hi2c1 链接）。 */
    App_Log_ReInit();
    BQ_ReInitAfterLp();
    /* 唤醒已回到 80 MHz：Ws2812_ReInit 重设 USART2SEL=PCLK1 与 BRR。 */
    Ws2812_ReInit();
    Buzzer_ReInit();
    (void)HAL_COMP_Start(&hcomp2);
    Fault_PvdEnable();
    App_TickStart();
}

void PowerMode_EnterLp(void)
{
    uint32_t drain_start;
    uint8_t cause = 0u;

    if (s_power.state != POWER_STATE_NORM)
    {
        return;
    }
    s_power.state = POWER_STATE_ENTER_LP;

    /* 1) 排空在飞通信（最多 100 ms）。 */
    drain_start = HAL_GetTick();
    while (((Ws2812_IsBusy() != false) || (App_Log_TxBusy() != false) ||
            (BQ_IsIdle() == 0u)) &&
           ((HAL_GetTick() - drain_start) < POWER_DRAIN_TIMEOUT_MS))
    {
        App_Log_Poll();
        BQ_Poll();
    }
    /* 超时未排空也继续进入 LP；在飞事务由各外设的 LP 停机路径中止。 */

    /* 2) BQ：关 ADC/TEMP（SYS_CTRL1=0x00，LP 期间无电压/电流/温度检测）+
     *    CC_EN=0（保持两只 MOS 导通）+ 清 SYS_STAT，防 CC_READY 唤醒风暴。 */
    if (BQ_PrepareForLp() != 0)
    {
        s_power.lp_prepare_fail_count++;
        Fault_SetSticky(FAULT_BIT_BQ_COMM);
    }

    /* 3) 黑匣子转换记录（此时仍在 NORM，Flash 可写）。 */
    (void)BlackBox_WriteTransition(BB_KIND_LP_ENTER, BB_WAKE_NONE,
                                   App_GetUptimeSeconds());

    /* 4) 布防 RTC 20 s 自动唤醒（LSE 常开）。布防失败则不进入 LP：
     *    否则没有定时唤醒源，设备可能一睡不起。 */
    s_power.flag_rtc = 0u;
    s_power.flag_button = 0u;
    s_power.wake_cause = 0u;
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 1u, 0u);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, POWER_RTC_WUT_SECONDS,
                                    RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
    {
        s_power.lse_ok = 0u;
        s_power.request_lp = 0u;
        s_power.state = POWER_STATE_NORM;
        Fault_SetSticky(FAULT_BIT_LSE);
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
        HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
        return;
    }

    /* 5) 停外设、灭灯。 */
    PowerMode_StopPeripherals();

    /* 6) 清挂起唤醒源，保证唤醒后判定只可能是 RTC/PB5/PA8。 */
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    EXTI->PR1 = (uint32_t)(EXTI_PR1_PIF5 | EXTI_PR1_PIF8);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

    /* 7) 确定唤醒后时钟：MSI 4 MHz（复位默认范围）+ STOPWUCK=MSI。 */
    __HAL_RCC_MSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == 0u)
    {
    }
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

    /* 8) SYSCLK → MSI；此时仍允许 4 WS，切完再降电压等级。 */
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_MSI);
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_MSI)
    {
    }
    SystemCoreClock = 4000000u;
    (void)HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* 9) 关 PLL、关 HSE（必须先于抬 PA1；HSEBYP 仅在 HSEON=0 时可写）。 */
    __HAL_RCC_PLL_DISABLE();
    while ((RCC->CR & RCC_CR_PLLRDY) != 0u)
    {
    }
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);   /* 清 HSEON 与 HSEBYP，唤醒后才能重配 */
    while ((RCC->CR & RCC_CR_HSERDY) != 0u)
    {
    }

    /* 10) 关 OSC1/PACK_SC、关 5 V，进入 STOP1。 */
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_RESET);

    s_power.state = POWER_STATE_LP;
    s_power.lp_enter_count++;

    HAL_SuspendTick();
    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    /* ================= 唤醒点 ================= */
    s_power.state = POWER_STATE_WAKING;

    if (s_power.flag_rtc != 0u)
    {
        cause |= POWER_WAKE_RTC;
        s_power.wake_rtc_count++;
    }
    if (s_power.flag_button != 0u)
    {
        cause |= POWER_WAKE_BUTTON;
        s_power.wake_button_count++;
    }
    if ((cause == 0u) || ((EXTI->PR1 & EXTI_PR1_PIF8) != 0u))
    {
        cause |= POWER_WAKE_BQ_ALERT;
        s_power.wake_alert_count++;
    }
    EXTI->PR1 = (uint32_t)(EXTI_PR1_PIF5 | EXTI_PR1_PIF8);
    s_power.flag_rtc = 0u;
    s_power.flag_button = 0u;
    s_power.wake_cause = cause;
    s_power.wake_count++;

    /* 唤醒后时钟是 MSI 4 MHz，先把 SystemCoreClock/SysTick 修到 1 ms。 */
    SystemCoreClock = 4000000u;
    (void)HAL_InitTick(TICK_INT_PRIORITY);

    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

    /* 恢复 80 MHz；失败则降级（仅故障灯，不跑业务）。 */
    if (PowerMode_ClkSwitchToHsePll() == 0u)
    {
        s_power.degraded = 1u;
        s_power.wake_clock_fail_count++;
        s_power.request_lp = 0u;
        s_power.state = POWER_STATE_DEGRADED;
        Fault_SetSticky(FAULT_BIT_CLK);
        return;
    }

    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(2u);   /* 5 V 软启动余量 */

    PowerMode_RestorePeripherals();

    (void)BlackBox_WriteTransition(BB_KIND_WAKE, s_power.wake_cause,
                                   App_GetUptimeSeconds());

    s_power.request_lp = 0u;
    s_power.state = POWER_STATE_NORM;
}

void PowerMode_Loop(void)
{
    if (s_power.degraded != 0u)
    {
        return;
    }

    if ((s_power.request_lp != 0u) && (s_power.state == POWER_STATE_NORM))
    {
        PowerMode_EnterLp();
    }
}

power_state_t PowerMode_GetState(void)
{
    return s_power.state;
}

uint8_t PowerMode_IsDegraded(void)
{
    return s_power.degraded;
}

uint8_t PowerMode_GetWakeCause(void)
{
    return s_power.wake_cause;
}

/* ---------------- 中断 / weak 回调（power_mode 唯一 owner） ---------------- */
void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_ptr)
{
    if (hrtc_ptr->Instance == RTC)
    {
        s_power.flag_rtc = 1u;
        __HAL_RTC_WAKEUPTIMER_DISABLE_IT(&hrtc, RTC_IT_WUT);
    }
}

/* 生成代码的 EXTI9_5_IRQHandler 只派发 PB5（WAKE_EXTI5）。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == WAKE_EXTI5_Pin)
    {
        s_power.flag_button = 1u;
    }
}

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
 * LP 是复位式的：NORM 空闲后 PowerMode_RequestLp() 只置 request_lp；
 * PowerMode_Loop() 调 LpCycle_RequestStart() 写 BKP START token 并软件复位。
 * 轻量 LP 路径（lp_cycle.c）读 RTC epoch、做 BQ sniff/充电/PB5/PA8
 * 检查，30 min 点做 BQ 监测+黑匣子记录，配置校准后的 IWDG timeout 后进
 * STOP1。IWDG 超时复位；真正唤醒（PB5/充电/ALERT/监测失败）写 WAKE
 * token 再软件复位到完整 App_Init。RTC 只作绝对时间参考，LPTIM 不参与。
 * RTC 源由 LP_RTC_USE_LSE 选择：默认 LSI（相对时间调试），LSE 修好后切 1；
 * LSE 模式失败时回退 LSI 但 LP 保持禁用。
 ******************************************************************************
 */
#include "app/power_mode.h"

#include "board.h"
#include "app/faults.h"
#include "app/lp_cycle.h"
#include "app/reset_cause.h"

/* LP/STOP1 是否保持 SWD 调试连接（DBGMCU_CR_DBG_STOP）。
 * 0 = STOP1 关 SWD（默认，最低功耗）；1 = 保持调试时钟。
 * 复位式 LP 的轻量 boot 不跑 PowerMode_Init，因此该选项只在编译期生效：
 * make POWER_LP_KEEP_SWD=0/1；改变后必须 make clean。 */
#ifndef POWER_LP_KEEP_SWD
#define POWER_LP_KEEP_SWD           0u
#endif

#define POWER_CLK_RETRY             3u
#define POWER_LSE_RECOVER_MS        100u   /* LSERDY 连续正常多久才清 LSE 故障 */

static struct
{
    power_state_t state;
    volatile uint8_t request_lp;     /* 调试器写 1 请求进入 LP（主循环消费） */
    volatile uint8_t wake_cause;
    uint8_t boot_reset_cause;        /* ResetCause_GetLatched()（最小诊断） */
    uint8_t lse_ok;
    uint8_t degraded;
    uint32_t lse_healthy_since_ms;
    uint32_t wake_count;
    volatile uint8_t lp_disable_auto; /* 1=禁止自动 LP */
} s_power;

static void PowerMode_DbgStopEnable(void);

/* 时钟切换失败/重试前强制回到 HSI16，并同步 SystemCoreClock + SysTick。
 * HAL_RCC_ClockConfig 可能在 MODIFY_REG(SW) 之后才因 SWS 超时返回
 * HAL_TIMEOUT；此时硬件已经切到 PLL，但 HAL 还没更新 SystemCoreClock/
 * SysTick。若不先回落，后续 HAL_Delay/HAL_GetTick 会按旧 16 MHz 的 tick
 * 运行，30 ms 等延时全部失真。 */
static void PowerMode_ClkForceHsi16(void)
{
    __HAL_RCC_HSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == 0u)
    {
    }
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_HSI)
    {
    }
    SystemCoreClock = 16000000u;
    (void)HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
    (void)HAL_InitTick(TICK_INT_PRIORITY);
}

#if (LP_RTC_USE_LSE != 0u)
/* LSE 本身是否可用；只在 LSE 模式用于故障诊断。 */
static uint8_t PowerMode_LseReady(void)
{
    return ((__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u) &&
            ((RCC->BDCR & RCC_BDCR_LSEON) != 0u)) ? 1u : 0u;
}
#endif

/* RTC 时基严格按 LP_RTC_USE_LSE 对应的源判定：LSE 模式不接受 LSI 回退，
 * LSI 模式不接受残留 LSE。RTCEN 由 RtcTime_Init() 修复并校验。 */
static uint8_t PowerMode_RtcClockReady(void)
{
    return (((RCC->BDCR & RCC_BDCR_RTCSEL) == LP_RTC_EXPECT_RTCSEL) &&
            (__HAL_RCC_GET_FLAG(LP_RTC_EXPECT_RDY_FLAG) != 0u) &&
            (HAL_RTC_GetState(&hrtc) == HAL_RTC_STATE_READY)) ? 1u : 0u;
}

/* auto-LP / LP 入口 / 自检的唯一时基门控。 */
uint8_t PowerMode_LpTimebaseReady(void)
{
    return PowerMode_RtcClockReady();
}

/* LSE 故障恢复（仅 LP_RTC_USE_LSE=1）。LSI 模式是空函数：硬件 LSE 缺失
 * 不是故障，LP 直接用 LSI 相对时基。LSE 模式不做运行中 RTCSEL 热切换；
 * LSE 恢复后仍需复位才能让 RTC 重新选 LSE。 */
static void PowerMode_LseHealthTask(void)
{
#if (LP_RTC_USE_LSE != 0u)
    uint32_t now;

    if (PowerMode_LseReady() == 0u)
    {
        s_power.lse_ok = 0u;
        s_power.lse_healthy_since_ms = 0u;
        Fault_SetLive(FAULT_BIT_LSE);
        Fault_SetLive(FAULT_BIT_TIMEBASE);
        return;
    }

    now = HAL_GetTick();
    if ((s_power.lse_ok == 0u) ||
        ((Fault_GetAll() & FAULT_BIT_LSE) != 0u))
    {
        if (s_power.lse_healthy_since_ms == 0u)
        {
            s_power.lse_healthy_since_ms = now;
            return;
        }
        if ((now - s_power.lse_healthy_since_ms) < POWER_LSE_RECOVER_MS)
        {
            return;
        }

        s_power.lse_ok = 1u;
        s_power.lse_healthy_since_ms = 0u;
        Fault_ClearLive(FAULT_BIT_LSE);
        Fault_ClearSticky(FAULT_BIT_LSE);
        if (PowerMode_RtcClockReady() != 0u)
        {
            Fault_ClearLive(FAULT_BIT_TIMEBASE);
        }
        else
        {
            /* LSE 已恢复但 RTC 仍是 LSI 回退；需要复位重新选源。 */
            Fault_SetLive(FAULT_BIT_TIMEBASE);
        }
    }
    else
    {
        s_power.lse_healthy_since_ms = 0u;
    }
#endif
}

/* 冷启动/唤醒共用：PA1=0 → OSC1 稳定 → HSE BYPASS → PLL 80 MHz。
 * 返回 1=成功；失败后回到 HSI16 并进入降级。 */
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

        /* HAL_RCC_OscConfig 会等 HSERDY/PLLRDY 后才返回 HAL_OK。 */
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
            PowerMode_ClkForceHsi16();
            continue;
        }

        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        {
            PowerMode_ClkForceHsi16();
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
            PowerMode_ClkForceHsi16();
            continue;
        }

        return 1u;
    }

    PowerMode_ClkForceHsi16();   /* 降级模式也必须让 HAL tick 与 HSI16 一致 */
    return 0u;
}

void PowerMode_Init(void)
{
    s_power.state = POWER_STATE_INIT;
    s_power.request_lp = 0u;
    s_power.wake_cause = 0u;
    s_power.boot_reset_cause = ResetCause_GetLatched();
    s_power.lse_ok = 0u;
    s_power.degraded = 0u;
    s_power.lse_healthy_since_ms = 0u;
    s_power.wake_count = 0u;
    s_power.lp_disable_auto = 0u;

    /* 只有真正的 IWDG 故障复位（非 LP tick，LpCycle_ClassifyBoot 已判为
     * FAULT/NORMAL）才禁用本 boot 的自动 LP，避免 reset loop。
     * 有效 LP tick 不会执行 App_Init/PowerMode_Init；软件唤醒复位也不置位。 */
    if ((ResetCause_GetLatched() & RESET_CAUSE_WATCHDOG) != 0u)
    {
        s_power.lp_disable_auto = 1u;
    }

    __HAL_RCC_PWR_CLK_ENABLE();
    /* PWR and the RTC APB interface must remain clocked across STOP1: the
     * first post-WFI action is a PWR->CR1 backup-access write, and the RTC
     * WKUP ISR needs RTCAPB.  Sleep works without these SMEN bits because
     * APB1 stays on; STOP1 does not. */
    RCC->APB1ENR1 |= (RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN);
    RCC->APB1SMENR1 |= (RCC_APB1SMENR1_PWRSMEN |
                        RCC_APB1SMENR1_RTCAPBSMEN);
    (void)RCC->APB1ENR1;
    HAL_PWR_EnableBkUpAccess();

    /* RTC 源由 MX_RTC_Init/RtcTime_MspClockSource 按 LP_RTC_USE_LSE 选择。
     * LSI 模式下硬件 LSE 缺失不是故障；LSE 模式缺失才置 LSE/TIMEBASE。 */
#if (LP_RTC_USE_LSE != 0u)
    s_power.lse_ok = PowerMode_LseReady();
    if (s_power.lse_ok == 0u)
    {
        Fault_SetLive(FAULT_BIT_LSE);
    }
#else
    s_power.lse_ok = 0u;
    Fault_ClearLive(FAULT_BIT_LSE);
    Fault_ClearSticky(FAULT_BIT_LSE);
#endif
    if (PowerMode_LpTimebaseReady() != 0u)
    {
        Fault_ClearLive(FAULT_BIT_TIMEBASE);
    }
    else
    {
        Fault_SetLive(FAULT_BIT_TIMEBASE);
    }

    if ((RTC->CR & RTC_CR_WUTE) != 0u)
    {
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    }
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    EXTI->PR1 = EXTI_PR1_PIF20;   /* 清上电/复位遗留的 WUT 挂起 */

    /* PB5 在 CubeMX 中保持 GPIO_Input；轻量 LP 路径由 Button_WakeInit()
     * 显式配置 EXTI5/内部上拉/NVIC，生产唤醒不依赖 CubeMX 生成 EXTI。 */
    PowerMode_DbgStopEnable();

    if (PowerMode_ClkSwitchToHsePll() == 0u)
    {
        s_power.degraded = 1u;
        s_power.state = POWER_STATE_DEGRADED;
        Fault_SetSticky(FAULT_BIT_CLK);
        return;
    }

    s_power.degraded = 0u;
    s_power.state = POWER_STATE_NORM;

    /* LPTIM1 不再是 LP 唤醒源。保持停止并屏蔽其 EXTI/NVIC，防止旧配置
     * 在 NORM/STOP 中产生中断或唤醒。LP 周期由 IWDG 复位触发。 */
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
    HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);
    EXTI->IMR2 &= (uint32_t)~EXTI_IMR2_IM32;
    EXTI->EMR2 &= (uint32_t)~EXTI_EMR2_EM32;
    EXTI->PR2 = (1u << 0);   /* EXTI line 32 (LPTIM1) pending */
    CLEAR_BIT(LPTIM1->CR, LPTIM_CR_ENABLE);
}

/* DBG_STOP=1 时 STOP1 由内部 RC 继续给 FCLK/HCLK 供钟，调试器才能保持/重新
 * 连接。复位式 LP 的轻量 boot 不跑 PowerMode_Init，因此只认编译期
 * POWER_LP_KEEP_SWD，运行期不再提供覆盖字段。 */
static void PowerMode_DbgStopEnable(void)
{
#if (POWER_LP_KEEP_SWD != 0u)
    DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
#else
    DBGMCU->CR &= (uint32_t)~DBGMCU_CR_DBG_STOP;
#endif
}

void PowerMode_Loop(void)
{
    if (s_power.degraded != 0u)
    {
        return;
    }

    /* 每个 NORM 主循环检查一次 LSE/LSE→RTC 时钟源。LSE 是 LP 的绝对
     * 时间参考；失效时 LP 由 PowerMode_LpTimebaseReady() 阻止。 */
    PowerMode_LseHealthTask();

    if ((s_power.request_lp != 0u) && (s_power.state == POWER_STATE_NORM))
    {
        s_power.request_lp = 0u;
        (void)LpCycle_RequestStart();   /* RTC 未就绪/BQ 忙时保持 NORM，下一轮重试 */
    }
}

void PowerMode_RequestLp(void)
{
    if (s_power.lp_disable_auto != 0u)
    {
        return;     /* 调试器置 1 后保持 NORM，便于观察 PB5/唤醒后的变量 */
    }
    if ((s_power.degraded == 0u) && (s_power.state == POWER_STATE_NORM))
    {
        s_power.request_lp = 1u;
    }
}

void PowerMode_NoteLpFullWake(uint8_t cause)
{
    s_power.wake_cause = cause;
    s_power.wake_count++;
}

void PowerMode_ForceDisableAutoLp(void)
{
    s_power.lp_disable_auto = 1u;
    s_power.request_lp = 0u;
}

/* 复位式 LP 的轻量 boot 不调用 PowerMode_Init（s_power 也是刚清零的），
 * 因此只按编译期 POWER_LP_KEEP_SWD 决定 STOP1 中是否保留 SWD 时钟。 */
void PowerMode_LpDbgStopApply(void)
{
#if (POWER_LP_KEEP_SWD != 0u)
    DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
#else
    DBGMCU->CR &= (uint32_t)~DBGMCU_CR_DBG_STOP;
#endif
}

power_state_t PowerMode_GetState(void)
{
    return s_power.state;
}

uint8_t PowerMode_IsDegraded(void)
{
    return s_power.degraded;
}

/* RTC WUT 已不再参与 LP；保留一个最小 handler，防止旧固件遗留的 WUT
 * 或 spurious pending 进入默认死循环。 */
void RTC_WKUP_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    EXTI->PR1 = EXTI_PR1_PIF20;
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
}

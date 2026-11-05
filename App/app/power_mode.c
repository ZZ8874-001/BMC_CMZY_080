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
 *   排空通信 → BQ 关 ADC/TEMP、CC_EN=0/清状态 → 黑匣子转换记录 → 布防 RTC WUT
 *   (1 s，自由运行) → 停外设 → 清挂起 → SYSCLK 切 HSI16 → 关 PLL → 关 HSE →
 *   PA1=1/PB0=0 → WFI(STOP1)。
 *   唤醒后：HSI16 下先修 SysTick → PA1=0 → HSE/PLL 80 MHz → PB0=1 → 重建外设。
 *   若 HSE/PLL 恢复失败：保持 5 V、PB3 故障闪烁并进入 DEGRADED。
 *
 * 唤醒原因：RTC（WUT 中断）、PB5 按下（EXTI5 中断）、PA8 BQ ALERT（EXTI 事件，
 * 无中断标志）。三者都不成立时不再误归因为 BQ ALERT，而是记录 unknown 并
 * 完整唤醒；RTC_WKUP 向量/回调次数由 BKP2R/BKP3R 跨复位保留。
 ******************************************************************************
 */
#include "app/power_mode.h"

#include "board.h"
#include "app/main_app.h"
#include "app/black_box.h"
#include "app/faults.h"
#include "app/reset_cause.h"
#include "comm/app_log.h"
#include "bsp/bq_ic.h"
#include "bsp/button.h"
#include "bsp/buzzer.h"
#include "bsp/ws2812_uart.h"
#include "detect/gesture.h"
#include "detect/soc_est.h"

/* LP/STOP1 是否保持 SWD 调试连接（DBGMCU_CR_DBG_STOP）。
 * 0 = STOP1 关 SWD（默认，最低功耗；本构建下运行期写 1 也会被强制清零）
 * 1 = 保持（LP 中可连接/保持调试器；电流升高，部分 IDE 会 halt CPU），
 *     此时才允许运行期写 s_power.lp_keep_swd 选择。
 * 编译：make POWER_LP_KEEP_SWD=0/1；改变后必须 make clean。 */
#ifndef POWER_LP_KEEP_SWD
#define POWER_LP_KEEP_SWD           0u
#endif

#define POWER_CLK_RETRY             3u
/* A/B 验证：上一次已知可唤醒的固件使用 WUTR=1（2 s）；当前故障现场在
 * WUTR=0 时进入 STOP1 后 20 s 无 WUT ISR。先回到 WUTR=1 验证是否为
 * 0 reload 的硬件/HAL 行为问题，确认后再决定是否恢复 1 s。 */
#define POWER_LP_WUT_PERIOD_S       2u
#define POWER_LP_WUT_RELOAD         (POWER_LP_WUT_PERIOD_S - 1u) /* WUTR+1 秒 */
#define POWER_LP_MAX_TICKS          900u     /* 30 min @2 s；到点做 HSI16 储存监测 */
#define POWER_LP_CHARGER_SETTLE_MS  30u
#define POWER_LP_MONITOR_SETTLE_MS  BQ_ADC_SETTLE_MS
#define POWER_LP_SNIFF_LED_MS       50u     /* PB4 每次 WUT 嗅探闪 50 ms；0=关闭 */
#define POWER_DRAIN_TIMEOUT_MS      100u
#define POWER_LP_BUTTON_FLAG_KEEP_MS 2000u  /* 进入 LP 时保留最近按键标志的窗口 */
#define POWER_LP_BKP_MARKER         0xA5A50000u  /* RTC BKP0R 有效标志 */
#define POWER_LP_BKP_ISR_MARKER     0xA5A50000u  /* BKP2R/3R 高 16 位有效标志 */
#define POWER_LP_BKP_CTX_MARKER     0xA5000000u  /* BKP16R 上下文高字节标志 */
#define POWER_LSE_RECOVER_MS        100u   /* LSERDY/RTCSEL 连续正常多久才清 LSE 故障 */

static struct
{
    power_state_t state;
    volatile uint8_t request_lp;     /* 调试器写 1 请求进入 LP（主循环消费） */
    volatile uint8_t wake_cause;
    volatile uint8_t flag_rtc;
    volatile uint8_t flag_button;
    volatile uint32_t flag_button_ms;  /* flag_button 置位时刻（EXTI ISR） */
    uint8_t lse_ok;
    uint8_t degraded;
    uint32_t lse_lost_count;       /* 运行中 LSERDY/RTCSEL 失效次数 */
    uint32_t lse_recover_count;    /* LSE 稳定后自动清故障次数 */
    uint32_t lse_healthy_since_ms; /* 本次 LSERDY 连续正常的起点 */
    uint8_t clock_ok;
    uint32_t lp_enter_count;
    uint32_t wake_count;
    uint32_t wake_rtc_count;
    uint32_t wake_button_count;
    uint32_t wake_alert_count;
    uint32_t wake_charger_count;
    uint32_t wake_monitor_count;
    uint32_t wake_unknown_count;      /* WFI 返回但无 RTC/按键/ALERT 标志 */
    uint32_t clock_fail_count;
    uint32_t lp_prepare_fail_count;
    uint32_t lp_alert_abort_count;   /* BQ ALERT/保护锁存导致取消 LP 的次数 */
    uint32_t wake_clock_fail_count;
    volatile uint8_t lp_stage;   /* ENTER_LP 阶段码，卡死时用于定位 */
    volatile uint8_t hse_off_timeout; /* HSERDY 在限时内未落：继续 LP，供诊断 */
    volatile uint8_t lp_fet_mask;     /* LP 进入时写入的 FET mask；默认 0=全关 */
    volatile uint8_t lp_keep_swd;     /* 运行期调试开关：1=STOP1 保持 DBG_STOP */
    volatile uint8_t lp_dbg_stop_active; /* 最近一次写入后的 DBG_STOP 实际值 */
    volatile uint8_t debugger_attached;  /* CoreDebug->DHCSR.C_DEBUGEN */
    volatile uint16_t diag_self_test_fail; /* 最近一次上电自检失败位图 */
    volatile uint8_t diag_self_test_passed;
    volatile uint8_t diag_blackbox_error;  /* 自检完成时 BlackBox_HasError() */
    volatile uint8_t lp_charger_pending; /* 本次嗅探判定为充电，待完整唤醒 */
    volatile uint16_t lp_monitor_ticks;  /* 30 min 监测阈值；调试器可改小做台架测试 */
    volatile uint16_t lp_sniff_led_ms;   /* PB4 每次 WUT 嗅探闪灯脉宽；0=关闭 */
    volatile uint8_t lp_disable_auto;    /* 1=禁止 5 s 自动 LP（台架/调试用） */
    volatile uint32_t button_irq_count;  /* PB5 EXTI 进入次数（诊断按键唤醒） */
    uint8_t lp_diag_button_logged;       /* 本次 LP 已写一条按键 LP_DIAG 记录 */
    uint8_t lp_diag_entry_logged;   /* 本次 LP 已写进入循环诊断 */
    uint8_t lp_diag_wut_logged;     /* 本次 LP 已写首个 WUT 诊断 */
    uint8_t lp_diag_wfi_logged;     /* 本次 LP 已写首次 WFI 前 RTC 诊断 */
    uint8_t lp_diag_unknown_logged; /* 本次 LP 已写未知唤醒诊断 */
    uint16_t lp_diag_stage_mask;    /* 本次 LP 各关键阶段已写诊断 */
} s_power;

/* LP 1 s 嗅探诊断。Ozone 可直接展开 s_lp_sniff。 */
static struct
{
    volatile uint32_t total_count;      /* 已执行的 1 s 嗅探次数（本次 LP） */
    volatile uint32_t ok_count;         /* I2C 三次读取全部成功次数 */
    volatile uint32_t fail_count;       /* 任一次读取失败次数 */
    volatile uint32_t init_fail_count;  /* HAL_I2C_Init 失败次数 */
    volatile uint32_t last_ms;          /* 最近一次嗅探开始时的 HAL tick */
    volatile uint8_t  last_ok;          /* 最近一次嗅探是否成功 */
    volatile uint8_t  last_error;       /* 0=成功；0xE1=初始化失败；否则 HAL I2C error */
    volatile uint8_t  last_sys_ctrl1;   /* 最近一次读到的 SYS_CTRL1 */
    volatile uint8_t  last_sys_stat;    /* 最近一次读到的 SYS_STAT（已清 CC_READY） */
    volatile uint8_t  last_sys_ctrl2;   /* 最近一次读到的 SYS_CTRL2 */
    volatile uint32_t cc_ready_clear_count;      /* LP 期间清 CC_READY 次数 */
    volatile uint32_t cc_ready_clear_fail_count; /* 清 CC_READY 写失败次数 */
    volatile uint8_t  last_load_present;         /* SYS_CTRL1 bit7，LP FET 全关时有效 */
    volatile uint32_t load_present_count;        /* LOAD_PRESENT=1 的嗅探次数 */
    volatile uint32_t charger_check_count;       /* 在 LP 内启动 COMP2 复核的次数 */
    volatile uint32_t charger_detect_count;      /* COMP2 判定为充电的次数 */
    volatile uint32_t load_reject_count;         /* LOAD_PRESENT=1 但 COMP2 判定非充电 */
    volatile uint32_t charger_check_error_count; /* PA1/COMP2 复核失败次数 */
    volatile uint8_t  last_charger_level;        /* 最近一次 COMP2 电平：1=充电 */
    volatile uint32_t monitor_count;             /* 30 min 储存监测执行次数 */
    volatile uint32_t monitor_ok_count;          /* 监测/恢复成功的次数 */
    volatile uint32_t monitor_fail_count;        /* 监测/恢复失败次数 */
    volatile uint32_t monitor_archive_fail_count;/* 黑匣子写入失败次数 */
    volatile uint8_t  monitor_last_error;        /* 0=成功；0xE4=初始化失败 */
    volatile uint16_t monitor_last_cell_mv[3];   /* 最近一次监测电芯电压 */
    volatile uint16_t monitor_last_pack_mv;
    volatile uint16_t monitor_last_ts1_raw;
    volatile int16_t  monitor_last_cc_raw;
} s_lp_sniff;

static void PowerMode_DbgStopEnable(void);
static uint8_t PowerMode_LpButtonPathFlags(void);
static uint8_t PowerMode_LpWutPathFlags(void);

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

/* LSE + RTC 时钟源是否可用。RTCSEL=LSE 是 RTC 继续走 LSE 的硬条件；
 * 只看 LSERDY 不够（RTCSEL 被改掉时 LSE 仍可能 ready）。 */
static uint8_t PowerMode_LseReady(void)
{
    return ((__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u) &&
            ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_0)) ? 1u : 0u;
}

/* LSE 故障不应该是“一次失败永久锁死”。外部 32.768 kHz 可能被干扰，
 * 干扰撤去后 LSERDY 会恢复；旧代码在 WUT 布防失败时置 sticky LSE，
 * auto-LP 的 APP_LP_BLOCK_LSE 又在 main_app 里读 Fault_GetAll，导致
 * 第一次失败后永远不再尝试 LP（表现为 PB5 回 NORM 后自动 LP 不再闪灯）。
 * 这里用连续 POWER_LSE_RECOVER_MS 的 LSERDY+RTCSEL 正常作为恢复条件：
 *   - 失效时只置 live，保证 auto-LP 停下；
 *   - 稳定后清 live + 历史 sticky，允许下一次 LP 重试；
 *   - 不清除真实失效（LSERDY 不恢复就不会清）。 */
static void PowerMode_LseHealthTask(void)
{
    uint32_t now;

    if (PowerMode_LseReady() == 0u)
    {
        if (s_power.lse_ok != 0u)
        {
            s_power.lse_lost_count++;
        }
        s_power.lse_ok = 0u;
        s_power.lse_healthy_since_ms = 0u;
        Fault_SetLive(FAULT_BIT_LSE);
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
        s_power.lse_recover_count++;
        Fault_ClearLive(FAULT_BIT_LSE);
        Fault_ClearSticky(FAULT_BIT_LSE);
    }
    else
    {
        s_power.lse_healthy_since_ms = 0u;
    }
}

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
            PowerMode_ClkForceHsi16();
            continue;
        }

        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        {
            s_power.clock_fail_count++;
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
            s_power.clock_fail_count++;
            PowerMode_ClkForceHsi16();
            continue;
        }

        s_power.clock_ok = 1u;
        return 1u;
    }

    PowerMode_ClkForceHsi16();   /* 降级模式也必须让 HAL tick 与 HSI16 一致 */
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
    s_power.flag_button_ms = 0u;
    s_power.lse_ok = 0u;
    s_power.degraded = 0u;
    s_power.lse_lost_count = 0u;
    s_power.lse_recover_count = 0u;
    s_power.lse_healthy_since_ms = 0u;
    s_power.clock_ok = 0u;
    s_power.lp_enter_count = 0u;
    s_power.wake_count = 0u;
    s_power.wake_rtc_count = 0u;
    s_power.wake_button_count = 0u;
    s_power.wake_alert_count = 0u;
    s_power.wake_charger_count = 0u;
    s_power.wake_monitor_count = 0u;
    s_power.clock_fail_count = 0u;
    s_power.lp_prepare_fail_count = 0u;
    s_power.lp_alert_abort_count = 0u;
    s_power.wake_clock_fail_count = 0u;
    s_power.lp_stage = 0u;
    s_power.hse_off_timeout = 0u;
    /* LP 安全默认双管关，不跟随 BQ_DEFAULT_FET_MASK（当前宏为 ALL）。 */
    s_power.lp_fet_mask = BQ_FET_MASK_NONE;
    s_power.lp_keep_swd = (POWER_LP_KEEP_SWD != 0u) ? 1u : 0u;
    s_power.lp_dbg_stop_active = 0u;
    s_power.debugger_attached = 0u;
    s_power.diag_self_test_fail = 0u;
    s_power.diag_self_test_passed = 0u;
    s_power.diag_blackbox_error = 0u;
    s_power.lp_charger_pending = 0u;
    s_power.lp_monitor_ticks = POWER_LP_MAX_TICKS;
    s_power.lp_sniff_led_ms = POWER_LP_SNIFF_LED_MS;
    s_power.lp_disable_auto = 0u;
    s_power.button_irq_count = 0u;
    s_power.lp_diag_button_logged = 0u;
    s_power.lp_diag_entry_logged = 0u;
    s_power.lp_diag_wut_logged = 0u;
    s_power.lp_diag_wfi_logged = 0u;
    s_power.lp_diag_unknown_logged = 0u;
    s_power.lp_diag_stage_mask = 0u;

    __HAL_RCC_PWR_CLK_ENABLE();

    /* LSE/RTC 由生成的 MX_RTC_Init（IOC 中的 RTC 条目）初始化，这里只核对状态。 */
    s_power.lse_ok = ((__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u) &&
                      (HAL_RTC_GetState(&hrtc) == HAL_RTC_STATE_READY)) ? 1u : 0u;
    if (s_power.lse_ok == 0u)
    {
        /* 可恢复故障：LSERDY/RTCSEL 稳定后由 PowerMode_LseHealthTask 清除。
         * 旧代码用 sticky，导致一次上电/布防失败后 auto-LP 永久被挡住。 */
        Fault_SetLive(FAULT_BIT_LSE);
    }

    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    EXTI->PR1 = EXTI_PR1_PIF20;   /* 清上电/复位遗留的 WUT 挂起 */

    /* PB5 的 IT_FALLING + 内部上拉 + EXTI9_5 NVIC 全部由 CubeMX 生成的
     * MX_GPIO_Init 配置（.ioc: GPXTI5 / GPIO_MODE_IT_FALLING / GPIO_PULLUP），
     * App 不再对 PB5 做任何 GPIO/EXTI 覆写。 */
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
}

/* DBG_STOP=1 时 STOP1 由内部 RC 继续给 FCLK/HCLK 供钟，调试器才能保持/重新
 * 连接；必须在进入 STOP1 前设置。POWER_LP_KEEP_SWD=0 时编译期强制关闭，
 * 运行期也不能把它打开；=1 时才允许调试器通过 s_power.lp_keep_swd 选择。 */
static void PowerMode_DbgStopEnable(void)
{
    s_power.debugger_attached =
        ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0u) ? 1u : 0u;

#if (POWER_LP_KEEP_SWD == 0)
    /* 量产构建：无调试器时关闭 STOP1 调试时钟以降低功耗。若检测到调试器
     * 仍连接，则保留 DBG_STOP；否则部分调试器在 STOP1 失去同步后会把
     * CPU 保持在 halt/重复连接状态，WUT/EXTI 虽然挂起但 ISR 永远不执行。
     * 台架低功耗测量前必须物理断开 SWD。 */
    s_power.lp_keep_swd = (s_power.debugger_attached != 0u) ? 1u : 0u;
#endif
    if (s_power.lp_keep_swd != 0u)
    {
        DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
    }
    else
    {
        DBGMCU->CR &= (uint32_t)~DBGMCU_CR_DBG_STOP;
    }
    s_power.lp_dbg_stop_active =
        ((DBGMCU->CR & DBGMCU_CR_DBG_STOP) != 0u) ? 1u : 0u;
}

static void PowerMode_StopPeripherals(void)
{
    Button_PrepareForLp();
    Gesture_Init();
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

/* LP 内充电器复核：LOAD_PRESENT 只说明 CHG 引脚被外部拉高，负载也可能
 * 拉高，所以再用 COMP2 判方向。PA1 低使能 PACK_SC 前端；检查完立即抬
 * 高 PA1、停 COMP2，不启动 5 V/HSE。 */
static uint8_t PowerMode_LpCheckCharger(void)
{
    GPIO_PinState level;

    s_lp_sniff.charger_check_count++;

    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_RESET);
    HAL_Delay(POWER_LP_CHARGER_SETTLE_MS);

    if (HAL_COMP_Start(&hcomp2) != HAL_OK)
    {
        s_lp_sniff.charger_check_error_count++;
        s_lp_sniff.last_charger_level = 0u;
        (void)HAL_COMP_Stop(&hcomp2);
        HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);
        return 0u;
    }

    HAL_Delay(POWER_LP_CHARGER_SETTLE_MS);
    level = HAL_COMP_GetOutputLevel(&hcomp2);
    (void)HAL_COMP_Stop(&hcomp2);
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);

    s_lp_sniff.last_charger_level =
        (level == COMP_OUTPUT_LEVEL_HIGH) ? 1u : 0u;
    if (s_lp_sniff.last_charger_level != 0u)
    {
        s_lp_sniff.charger_detect_count++;
        return 1u;
    }

    s_lp_sniff.load_reject_count++;
    return 0u;
}

/* 单次 LP 1 s 嗅探：HSI16 下重新初始化 I2C，读 BQ 控制/状态寄存器。
 * 只用于 LP 诊断/充电判定，不写 Flash、不启动 5 V。 */
static void PowerMode_LpSniffOnce(void)
{
    uint8_t ctrl1 = 0u;
    uint8_t stat = 0u;
    uint8_t ctrl2 = 0u;
    uint8_t clear_failed = 0u;
    int rc;

    s_lp_sniff.last_ms = HAL_GetTick();

    if (BQ_LpSniffInit() != 0)
    {
        s_lp_sniff.init_fail_count++;
        s_lp_sniff.fail_count++;
        s_lp_sniff.last_ok = 0u;
        s_lp_sniff.last_error = 0xE1u;
        s_lp_sniff.total_count++;
        return;
    }

    rc = BQ_LpSniffRead(BQ_REG_SYS_CTRL1, &ctrl1);
    if (rc == 0)
    {
        rc = BQ_LpSniffRead(BQ_REG_SYS_STAT, &stat);
    }
    if (rc == 0)
    {
        rc = BQ_LpSniffRead(BQ_REG_SYS_CTRL2, &ctrl2);
    }

    if ((stat & BQ_SYS_STAT_CC_READY) != 0u)
    {
        if (BQ_LpSniffClearSysStat(BQ_SYS_STAT_CC_READY) == 0)
        {
            stat &= (uint8_t)~BQ_SYS_STAT_CC_READY;
            s_lp_sniff.cc_ready_clear_count++;
        }
        else
        {
            s_lp_sniff.cc_ready_clear_fail_count++;
            clear_failed = 1u;
        }
    }

    s_lp_sniff.last_sys_ctrl1 = ctrl1;
    s_lp_sniff.last_sys_stat = stat;
    s_lp_sniff.last_sys_ctrl2 = ctrl2;
    s_lp_sniff.last_load_present =
        ((ctrl1 & BQ_SYS_CTRL1_LOAD_PRESENT) != 0u) ? 1u : 0u;
    if (s_lp_sniff.last_load_present != 0u)
    {
        s_lp_sniff.load_present_count++;
    }
    s_lp_sniff.total_count++;

    if ((rc == 0) && (clear_failed == 0u))
    {
        s_lp_sniff.ok_count++;
        s_lp_sniff.last_ok = 1u;
        s_lp_sniff.last_error = 0u;
    }
    else
    {
        s_lp_sniff.fail_count++;
        s_lp_sniff.last_ok = 0u;
        s_lp_sniff.last_error = (uint8_t)(HAL_I2C_GetError(&hi2c1) & 0xFFu);
        if (s_lp_sniff.last_error == 0u)
        {
            s_lp_sniff.last_error = (clear_failed != 0u) ? 0xE3u : 0xE2u;
        }
    }

    BQ_LpSniffDeInit();

    /* LOAD_PRESENT 只在 CHG_ON=0 时有效；默认 lp_fet_mask=0，满足条件。
     * 复核为充电才置 pending，纯负载继续睡。 */
    if ((s_lp_sniff.last_load_present != 0u) &&
        ((s_lp_sniff.last_sys_ctrl2 & BQ_SYS_CTRL2_CHG_ON) == 0u))
    {
        if (PowerMode_LpCheckCharger() != 0u)
        {
            s_power.lp_charger_pending = 1u;
        }
    }
}

/* 30 min 储存监测：仍在 HSI16/LP，不启动 5 V、不切 HSE/PLL。
 * 临时打开 BQ ADC/TEMP+CC，采一轮电芯/总压/TS1/CC，写一条周期黑匣子记录，
 * 再恢复 LP 寄存器态。返回 0 失败（调用者完整唤醒）；1 成功继续睡。 */
static uint8_t PowerMode_LpStorageMonitorOnce(void)
{
    bq_snapshot_t snap;
    uint8_t ok = 1u;

    s_lp_sniff.monitor_count++;
    s_lp_sniff.monitor_last_error = 0u;
    s_power.lp_stage = 17u;     /* 17=30 min HSI16 储存监测 */

    if (BQ_LpSniffInit() != 0)
    {
        s_lp_sniff.monitor_fail_count++;
        s_lp_sniff.monitor_last_error = 0xE4u;
        s_power.lp_stage = 15u;
        return 0u;
    }

    if (BQ_LpMonitorStart(s_power.lp_fet_mask) != 0)
    {
        ok = 0u;
        s_lp_sniff.monitor_last_error = 0xE5u;
    }
    else
    {
        HAL_Delay(POWER_LP_MONITOR_SETTLE_MS);   /* ADC 250 ms + 余量 */
        if (BQ_LpMonitorRead(&snap) != 0)
        {
            ok = 0u;
            s_lp_sniff.monitor_last_error = 0xE6u;
        }
    }

    if (ok != 0u)
    {
        s_lp_sniff.monitor_last_cell_mv[0] = snap.cell_mv[0];
        s_lp_sniff.monitor_last_cell_mv[1] = snap.cell_mv[1];
        s_lp_sniff.monitor_last_cell_mv[2] = snap.cell_mv[2];
        s_lp_sniff.monitor_last_pack_mv = snap.pack_mv;
        s_lp_sniff.monitor_last_ts1_raw = snap.ts1_raw;
        s_lp_sniff.monitor_last_cc_raw = snap.cc_raw;
    }

    /* 先把 BQ 恢复为 LP 态，再做 Flash 写；Flash 擦除期间不保持 ADC/CC 开启。 */
    if (BQ_LpMonitorStop(s_power.lp_fet_mask) != 0)
    {
        ok = 0u;
        s_lp_sniff.monitor_last_error = 0xE7u;
    }
    BQ_LpSniffDeInit();

    if (ok != 0u)
    {
        if (BlackBox_WritePeriodicSnapshot(&snap, App_GetUptimeSeconds()) != 0)
        {
            s_lp_sniff.monitor_archive_fail_count++;
        }
        s_lp_sniff.monitor_ok_count++;
    }
    else
    {
        s_lp_sniff.monitor_fail_count++;
    }
    s_power.lp_stage = 15u;     /* 回到“一次嗅探完成” */
    return ok;
}

/* 按键尝试诊断：每次 LP 只写一条黑匣子记录。必须在“检测到按键后、
 * 退出 LP 循环之前”调用，否则上次按键唤醒会在下一次循环前直接退出，
 * 永远到不了循环顶部的诊断代码。 */
static void PowerMode_LpDiagButtonAttempt(uint8_t pif5, uint8_t pb5_low)
{
    uint8_t flags;

    if (s_power.lp_diag_button_logged != 0u)
    {
        return;
    }
    s_power.lp_diag_button_logged = 1u;

    flags = PowerMode_LpButtonPathFlags();
    if (pif5 != 0u)
    {
        flags |= 0x20u;
    }
    if (pb5_low != 0u)
    {
        flags |= 0x10u;
    }

    (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(),
                               s_lp_sniff.total_count,
                               s_power.button_irq_count,
                               s_bb_shadow.diag_lp_loop_count,
                               s_power.lp_stage,
                               flags);
}

/* PB5 唤醒通路现场（stage12/按键/WUT 诊断共用）：
 * bit0=EXTI IMR5, bit1=FTSR5 下降沿, bit2=PB5 内部上拉, bit3=PB5 输入模式,
 * bit4=PB5 当前低, bit5=PIF5 pending, bit6=EXTI9_5 NVIC 使能,
 * bit7=s_power.flag_button。PB5 固定为 GPIOB 的 pin5。 */
static uint8_t PowerMode_LpButtonPathFlags(void)
{
    uint8_t flags = 0u;

    if ((EXTI->IMR1 & EXTI_IMR1_IM5) != 0u)
    {
        flags |= 0x01u;
    }
    if ((EXTI->FTSR1 & EXTI_FTSR1_FT5) != 0u)
    {
        flags |= 0x02u;
    }
    if ((GPIOB->PUPDR & (3u << 10u)) == (1u << 10u))
    {
        flags |= 0x04u;
    }
    if ((SYSCFG->EXTICR[1] & SYSCFG_EXTICR2_EXTI5) ==
        SYSCFG_EXTICR2_EXTI5_PB)
    {
        flags |= 0x08u;     /* EXTI5 确实路由到 PB5 */
    }
    if (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
        GPIO_PIN_RESET)
    {
        flags |= 0x10u;
    }
    if ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u)
    {
        flags |= 0x20u;
    }
    if (((NVIC->ISER[0] >> (uint32_t)EXTI9_5_IRQn) & 1u) != 0u)
    {
        flags |= 0x40u;
    }
    if (s_power.flag_button != 0u)
    {
        flags |= 0x80u;
    }
    return flags;
}

/* RTC WUT 唤醒通路现场（stage30 专用）：
 * bit0=WUTE, bit1=WUTIE, bit2=WUTF, bit3=LSERDY,
 * bit4=EXTI line20 IMR, bit5=EXTI line20 RTSR,
 * bit6=RTC_WKUP NVIC 使能, bit7=EXTI line20 pending。
 * WUTWF/LSE/RTCSEL 等放在 PowerMode_LpRtcAux。 */
static uint8_t PowerMode_LpWutPathFlags(void)
{
    uint8_t flags = 0u;

    if ((RTC->CR & RTC_CR_WUTE) != 0u)
    {
        flags |= 0x01u;
    }
    if ((RTC->CR & RTC_CR_WUTIE) != 0u)
    {
        flags |= 0x02u;
    }
    if ((RTC->ISR & RTC_ISR_WUTF) != 0u)
    {
        flags |= 0x04u;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u)
    {
        flags |= 0x08u;     /* LSE 仍在振荡；WUTWF 放到 aux */
    }
    if ((EXTI->IMR1 & EXTI_IMR1_IM20) != 0u)
    {
        flags |= 0x10u;
    }
    if ((EXTI->RTSR1 & EXTI_RTSR1_RT20) != 0u)
    {
        flags |= 0x20u;
    }
    if (((NVIC->ISER[0] >> (uint32_t)RTC_WKUP_IRQn) & 1u) != 0u)
    {
        flags |= 0x40u;
    }
    if ((EXTI->PR1 & EXTI_PR1_PIF20) != 0u)
    {
        flags |= 0x80u;
    }
    return flags;
}

/* 清掉 RTC WUT 的 stale 挂起：WUTF、EXTI line20、NVIC。只在 WUTF=0
 * 或准备重新布防时调用；WUTF=1 代表一个有效 tick，不能在这里丢。 */
static void PowerMode_LpClearWutPending(void)
{
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    EXTI->PR1 = EXTI_PR1_PIF20;
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
}

static uint8_t PowerMode_LpWutArmed(void)
{
    return ((RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) ==
            (RTC_CR_WUTE | RTC_CR_WUTIE)) ? 1u : 0u;
}

/* BKP18R：最后一次 WUT 布防结果（低 8 位 HAL 状态，bit8=guard 重布防），
 * 跨复位保留。用于判断第二次 LP 是否卡在 WUT 布防失败。 */
static void PowerMode_LpRecordWutArm(HAL_StatusTypeDef status, uint8_t guard)
{
    HAL_PWR_EnableBkUpAccess();
    RTC->BKP18R = POWER_LP_BKP_ISR_MARKER |
                  ((uint32_t)status & 0xFFu) |
                  ((guard != 0u) ? 0x100u : 0u);
}

/* BKP19R：本次 LP 进入前最后一次 BQ/ALERT 现场，跨复位可查。
 * bit31..24=0xA5，bit23..20=BQ 状态机，bit19=verdict_active，bit18=recovery_episode，
 * bit17=adc_recovery_failed，bit16=fresh，bit15..8=SYS_STAT，
 * bit0=PA8/BQ ALERT 电平（1=高）。ADC 异常/恢复后若 PA8 仍高，
 * BQ_PrepareForLpMask 只清 CC_READY/XREADY，不清 OV/UV/SCD/OCD 锁存；
 * 直接睡下会在首个 WUT 的 PA8 检查处完整唤醒，PB4 心跳还没执行。 */
static void PowerMode_LpRecordEntryBq(const bq_snapshot_t *snap, uint8_t pa8_high)
{
    uint32_t value;

    HAL_PWR_EnableBkUpAccess();
    value = POWER_LP_BKP_CTX_MARKER |
            (((uint32_t)BQ_GetState() & 0x0Fu) << 20) |
            ((snap->verdict_active != 0u) ? (1u << 19) : 0u) |
            ((snap->recovery_episode != 0u) ? (1u << 18) : 0u) |
            ((snap->adc_recovery_failed != 0u) ? (1u << 17) : 0u) |
            ((snap->fresh != 0u) ? (1u << 16) : 0u) |
            ((uint32_t)snap->sys_stat << 8) |
            ((pa8_high != 0u) ? 1u : 0u);
    RTC->BKP19R = value;
}

/* BKP13R/14R 计数：PB5 EXTI 进入次数、按钮完整唤醒次数。跨复位保留，
 * 用于判断“按键边沿根本没到 CPU”还是“到了但完整唤醒路径没走”。 */
static void PowerMode_LpBumpBkp(volatile uint32_t *reg)
{
    uint32_t n;

    HAL_PWR_EnableBkUpAccess();
    n = *reg & 0xFFFFu;
    if (n < 0xFFFFu)
    {
        n++;
    }
    *reg = POWER_LP_BKP_ISR_MARKER | n;
}

/* NORM→LP 进入窗口的按键请求：flag、PIF5、PB5 电平任一成立。 */
static uint8_t PowerMode_LpButtonRequested(void)
{
    if (s_power.flag_button != 0u)
    {
        return 1u;
    }
    if ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u)
    {
        return 1u;
    }
    return (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
            GPIO_PIN_RESET) ? 1u : 0u;
}

/* NORM→LP 进入窗口内收到按键：不要继续走完停外设/关 5V/切时钟再靠
 * pre-STOP1 guard 唤醒；直接取消本次 LP，保持 NORM。BQ 已经进入 LP
 * 寄存器态时（300 ms 准备之后）必须先恢复 BQ 配置。按键事件要合成进
 * Button 队列，避免 ISR 已清 PIF5 后丢失。 */
static void PowerMode_LpAbortForButton(uint8_t bq_prepared)
{
    Button_OnWake(1u,
                  (uint8_t)((HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port,
                                              WAKE_EXTI5_Pin) ==
                             GPIO_PIN_RESET) ? 1u : 0u),
                  HAL_GetTick());
    s_power.flag_button = 0u;
    EXTI->PR1 = EXTI_PR1_PIF5;
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

    if (bq_prepared != 0u)
    {
        BQ_ReInitAfterLp();   /* 把 SYS_CTRL1/2/FET/保护恢复到 NORM */
    }

    s_power.request_lp = 0u;
    s_power.state = POWER_STATE_NORM;
    s_power.lp_stage = 0u;
}

static void PowerMode_LpEnsureButtonPath(void)
{
    uint8_t flags = PowerMode_LpButtonPathFlags();

    if ((flags & 0x0Fu) != 0x0Fu)
    {
        GPIO_InitTypeDef gpio = {0};

        __HAL_RCC_SYSCFG_CLK_ENABLE();
        gpio.Pin = WAKE_EXTI5_Pin;
        gpio.Mode = GPIO_MODE_IT_FALLING;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(WAKE_EXTI5_GPIO_Port, &gpio);
    }

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    if (((EXTI->PR1 & EXTI_PR1_PIF5) == 0u) &&
        (s_power.flag_button == 0u) &&
        (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) !=
         GPIO_PIN_RESET))
    {
        HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    }

    HAL_PWR_EnableBkUpAccess();
    RTC->BKP15R = POWER_LP_BKP_ISR_MARKER | (uint32_t)flags;
}

/* 在真正 WFI 前把睡眠/中断现场写入 BKP4R..BKP8R，跨复位保留：
 *   BKP4R = PWR_CR1[15:0] | SCB_SCR[23:16] | PRIMASK[24] | BASEPRI[31:25]
 *   BKP5R = NVIC ISER[0]（bit3=RTC_WKUP）
 *   BKP6R = NVIC ISPR[0]
 *   BKP7R = EXTI line5/8/20 的 IMR/EMR/RTSR/PR 关键位
 *   BKP8R = RTC_CR[15:0] | RTC_ISR[31:16]
 * 这能区分“真的进了 STOP1 且中断全开”与“HardFault/中断被屏蔽/没设 SLEEPDEEP”。 */
static void PowerMode_LpSnapshotSleepRegs(void)
{
    HAL_PWR_EnableBkUpAccess();
    RTC->BKP4R = ((uint32_t)PWR->CR1 & 0xFFFFu) |
                 (((uint32_t)SCB->SCR & 0xFFu) << 16) |
                 (((uint32_t)__get_PRIMASK() & 1u) << 24) |
                 (((uint32_t)__get_BASEPRI() & 0x7Fu) << 25);
    RTC->BKP5R = NVIC->ISER[0];
    RTC->BKP6R = NVIC->ISPR[0];
    RTC->BKP7R = ((EXTI->IMR1 & EXTI_IMR1_IM5) ? 0x01u : 0u) |
                 ((EXTI->EMR1 & EXTI_EMR1_EM5) ? 0x02u : 0u) |
                 ((EXTI->IMR1 & EXTI_IMR1_IM8) ? 0x04u : 0u) |
                 ((EXTI->EMR1 & EXTI_EMR1_EM8) ? 0x08u : 0u) |
                 ((EXTI->IMR1 & EXTI_IMR1_IM20) ? 0x10u : 0u) |
                 ((EXTI->EMR1 & EXTI_EMR1_EM20) ? 0x20u : 0u) |
                 ((EXTI->RTSR1 & EXTI_RTSR1_RT20) ? 0x40u : 0u) |
                 ((EXTI->PR1 & EXTI_PR1_PIF20) ? 0x80u : 0u);
    RTC->BKP8R = ((uint32_t)RTC->CR & 0xFFFFu) |
                 (((uint32_t)RTC->ISR & 0xFFFFu) << 16);
}

/* stage30 的附加 RTC 时钟信息，放入记录的 cell3（低 16 位）：
 * bit0..2=WUCKSEL, bit3=WUTWF, bit4=LSEON, bit5=RTCSEL=LSE,
 * bit6=RTCEN, bit7=LSERDY。 */
static uint32_t PowerMode_LpRtcAux(void)
{
    uint32_t aux = (uint32_t)(RTC->CR & RTC_CR_WUCKSEL);

    if ((RTC->ISR & RTC_ISR_WUTWF) != 0u)
    {
        aux |= 0x08u;
    }
    if ((RCC->BDCR & RCC_BDCR_LSEON) != 0u)
    {
        aux |= 0x10u;
    }
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_0)
    {
        aux |= 0x20u;
    }
    if ((RCC->BDCR & RCC_BDCR_RTCEN) != 0u)
    {
        aux |= 0x40u;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u)
    {
        aux |= 0x80u;
    }
    return aux;
}

/* RTC 备份寄存器最后阶段标记：不写 Flash，STOP1/复位后仍保留。
 * BKP0R[15:0] = stage | flags<<8，BKP0R[31:16] = marker。 */
static void PowerMode_LpMarkStage(uint8_t stage, uint8_t flags)
{
    HAL_PWR_EnableBkUpAccess();
    RTC->BKP0R = POWER_LP_BKP_MARKER |
                 (uint32_t)stage |
                 ((uint32_t)flags << 8);
}

void PowerMode_SetSelfTestResult(uint16_t fail_mask, uint8_t blackbox_error)
{
    s_power.diag_self_test_fail = fail_mask;
    s_power.diag_self_test_passed = (fail_mask == 0u) ? 1u : 0u;
    s_power.diag_blackbox_error = blackbox_error;
}

/* LP 上电后的 RTC/LSE 现场 + 上一次 LP 最后阶段（BKP0R）。 */
void PowerMode_LogBootRtcState(void)
{
    uint8_t flags = 0u;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u)
    {
        flags |= 0x01u;
    }
    if ((RCC->BDCR & RCC_BDCR_LSEON) != 0u)
    {
        flags |= 0x02u;
    }
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_0)
    {
        flags |= 0x04u;
    }
    if ((RCC->BDCR & RCC_BDCR_RTCEN) != 0u)
    {
        flags |= 0x08u;
    }
    if (HAL_RTC_GetState(&hrtc) == HAL_RTC_STATE_READY)
    {
        flags |= 0x10u;
    }
    if ((RTC->CR & RTC_CR_WUTE) != 0u)
    {
        flags |= 0x20u;
    }
    if ((RTC->CR & RTC_CR_WUTIE) != 0u)
    {
        flags |= 0x40u;
    }
    if ((RTC->ISR & RTC_ISR_WUTF) != 0u)
    {
        flags |= 0x80u;
    }

    /* Ozone 导出会显示 cc_raw/ts1_raw，所以把两个备份寄存器的低 16 位
     * 放到高 16 位：cc = BKP0R[15:0]（最后 stage|flags），
     * ts1 = BKP1R[15:0]（WUT ISR 是否真正进入，正常应为 0x000F）。 */
    (void)BlackBox_WriteLpDiag(
        App_GetUptimeSeconds(),
        (RTC->BKP0R & 0xFFFFu) << 16,
        (RTC->BKP1R & 0xFFFFu) << 16,
        (uint32_t)RTC->ISR & 0xFFFFu,
        60u, flags);

    /* stage61：上一次运行的 RTC_WKUP 向量/回调计数（BKP2R/BKP3R），
     * 以及复位原因。写入成功后清零，供本次上电重新累计。
     * 字段：cc_raw=BKP2R[15:0], ts1_raw=BKP3R[15:0],
     *       cell3=ResetCause_GetLatched()。 */
    if (BlackBox_WriteLpDiag(
            App_GetUptimeSeconds(),
            (RTC->BKP2R & 0xFFFFu) << 16,
            (RTC->BKP3R & 0xFFFFu) << 16,
            (uint32_t)ResetCause_GetLatched(),
            61u, 0u) == 0)
    {
        HAL_PWR_EnableBkUpAccess();
        RTC->BKP2R = 0u;
        RTC->BKP3R = 0u;
    }

    /* stage62：PB5 诊断。cc_raw=BKP13R（PB5 EXTI ISR 次数），
     * ts1_raw=BKP14R（按钮完整唤醒次数），cell3=BKP15R（最后一次
     * LP 进入前的 PB5 通路 flags）。这里不清 BKP13R/14R/15R：
     * NRST 后 Ozone 可直接读；它们是本电源周期的累计值。
     * 若整机断电（无 VBAT），备份寄存器会丢，但本条 stage62 Flash
     * 记录仍保留上一次 boot 时的值。 */
    (void)BlackBox_WriteLpDiag(
        App_GetUptimeSeconds(),
        (RTC->BKP13R & 0xFFFFu) << 16,
        (RTC->BKP14R & 0xFFFFu) << 16,
        (uint32_t)(RTC->BKP15R & 0xFFFFu),
        62u, 0u);

    /* stage63：完整 BKP16R（最后一次 PB5 ISR 上下文）。整机断电会清
     * 备份寄存器，这条 Flash 记录保留断电前最后一次 boot 读到的值。
     * 24 B 映射：cell1=BKP16R[15:0]，cc=BKP16R[31:16]。 */
    (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(), RTC->BKP16R, 0u, 0u,
                               63u, 0u);

    /* stage64：BKP17R=累计进入 LP 次数，BKP18R=最后一次 WUT 布防结果
     * （低 8 位 HAL 状态，bit8=guard 重布防）。cc_raw=BKP17R[15:0]，
     * ts1_raw=BKP18R[15:0]。 */
    (void)BlackBox_WriteLpDiag(
        App_GetUptimeSeconds(),
        (RTC->BKP17R & 0xFFFFu) << 16,
        (RTC->BKP18R & 0xFFFFu) << 16,
        0u, 64u, 0u);

    /* stage65：最近一次 LP 进入前的 BQ ALERT/SYS_STAT 现场（BKP19R）。
     * 24 B 映射：cell1=BKP19R[15:0]，cc=BKP19R[31:16]；布局见
     * PowerMode_LpRecordEntryBq。用于区分“ADC 异常后假保护锁存仍高”
     * 与“WUT 没起来”。不清零，保持最近一次进入现场。 */
    (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(), RTC->BKP19R,
                               0u, 0u, 65u, 0u);

    /* stage66：最近一次 LP 准备失败（BKP20R）。上面 stage65 只在真正
     * 走到 BQ 准备成功后才写；ADC 恢复回合中 BQ_PrepareForLpMask 返回
     * -2 时 LP 根本没进入，BKP19 不会更新。24 B 映射同 stage65：
     * cell1=BKP20R[15:0]，cc=BKP20R[31:16]；布局见 EnterLp 的失败分支。 */
    (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(), RTC->BKP20R,
                               0u, 0u, 66u, 0u);
}

/* LP 进入阶段诊断：每次 LP 进入、每个阶段只写一条记录。上一次 LP 成功
 * 唤醒后，下一次进入仍会重新记录；若最新一次 LP 卡死，复位后黑匣子里
 * 最后一条 stage=... 就是卡住位置。 */
static void PowerMode_LpDiagStageFlags(uint8_t stage, uint16_t bit,
                                       uint8_t flags)
{
    if ((s_power.lp_diag_stage_mask & bit) != 0u)
    {
        return;
    }
    s_power.lp_diag_stage_mask |= bit;
    (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(), 0u,
                               s_power.button_irq_count, 0u, stage, flags);
}

static void PowerMode_LpDiagStage(uint8_t stage, uint16_t bit)
{
    PowerMode_LpDiagStageFlags(stage, bit, 0u);
}

void PowerMode_EnterLp(void)
{
    uint32_t drain_start;
    uint8_t cause = 0u;
    uint8_t full_wake;
    HAL_StatusTypeDef wut_status;

    if (s_power.state != POWER_STATE_NORM)
    {
        return;
    }
    PowerMode_LseHealthTask();
    if (s_power.lse_ok == 0u)
    {
        /* 没有 LSE 就没有 RTC WUT；不要为了进入 LP 而冒“睡下后无唤醒源”
         * 的风险。LSE 恢复后 health task 会清 live 故障，auto-LP 自动重试。 */
        s_power.request_lp = 0u;
        return;
    }
    s_power.state = POWER_STATE_ENTER_LP;
    s_power.lp_stage = 1u;      /* 1=drain */

    /* 每次 LP 进入清 BKP1R，避免把上一次 LP 的 0x0E/0x0F 误认为
     * 本次 WUT ISR 已经进入；BKP1R 由本次 RTC_WKUP 向量/回调重新写。 */
    HAL_PWR_EnableBkUpAccess();
    RTC->BKP1R = 0u;
    /* BKP4R..8R 不在这里清：保留上一次 LP 的睡眠现场，直到本次真正
     * 走到 stage32 再覆盖。这样即使下一次 LP 在进入早期失败，上次
     * 的 PRIMASK/SCB/EXTI/RTC 证据也不会丢。 */

    /* 不要无条件清 flag_button：请求之后到 STOP1 之间（约几百 ms）的
     * 按键可能已被 EXTI ISR 消费掉 PIF5，只剩这个标志。只清 2 s 以前
     * 的旧标志（NORM 手势残留）；最近的按键保留到 pre-STOP1 检查。 */
    if ((s_power.flag_button != 0u) &&
        ((HAL_GetTick() - s_power.flag_button_ms) >
         POWER_LP_BUTTON_FLAG_KEEP_MS))
    {
        s_power.flag_button = 0u;
    }

    /* 诊断状态按“每次 LP 进入”重置，而不是按每次上电：上一次 LP 成功
     * 唤醒后，下一次 LP 若卡住/按键失效，仍会重新写入 stage0/4/40/.../12
     * 和本次 WUT 记录，否则最新一次失败会被上电标志屏蔽掉。 */
    s_power.lp_diag_button_logged = 0u;
    s_power.lp_diag_entry_logged = 0u;
    s_power.lp_diag_wut_logged = 0u;
    s_power.lp_diag_wfi_logged = 0u;
    s_power.lp_diag_unknown_logged = 0u;
    s_power.lp_diag_stage_mask = 0u;

    /* stage=0：LP 进入前的 RTC/LSE 现场。flags：
     * bit0=lse_ok, bit1=hrtc READY, bit2=LSERDY, bit3=FAULT_LSE,
     * bit4=RTCSEL=LSE, bit5=hrtc.Lock!=UNLOCKED, bit6..7=hrtc.State。
     * 若后续只有 stage=4/41 而没有 stage=40，说明
     * HAL_RTCEx_SetWakeUpTimer_IT 失败，flags 里可直接看到 LSE/RTC 状态。 */
    {
        uint8_t diag_flags = 0u;

        if (s_power.lse_ok != 0u)
        {
            diag_flags |= 0x01u;
        }
        if (HAL_RTC_GetState(&hrtc) == HAL_RTC_STATE_READY)
        {
            diag_flags |= 0x02u;
        }
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u)
        {
            diag_flags |= 0x04u;
        }
        if ((Fault_GetAll() & FAULT_BIT_LSE) != 0u)
        {
            diag_flags |= 0x08u;
        }
        if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_0)
        {
            diag_flags |= 0x10u;
        }
        if (hrtc.Lock != HAL_UNLOCKED)
        {
            diag_flags |= 0x20u;
        }
        diag_flags |= (uint8_t)(((uint32_t)hrtc.State & 0x03u) << 6);
        PowerMode_LpDiagStageFlags(0u, 0x0080u, diag_flags);
    }
    /* 不要在这里清 flag_button：自动 LP 请求后、真正进入 STOP1 之前的
     * 几百 ms 内用户可能按下按键；EXTI ISR 已清 PIF5 并置 flag_button，
     * 进入函数时再清会把这次唤醒请求丢掉，造成“有时唤不醒”。旧标志
     * 在 PowerMode_RequestLp() 接受请求时清。 */
    /* 是否保持 DBG_STOP 由 PowerMode_DbgStopEnable() 按 C_DEBUGEN 决定：
     * 无调试器 -> 关闭；连接调试器 -> 保留，避免 bench 测试时 STOP1
     * 唤醒被调试器 halt 状态掩盖。 */
    SocEst_PrepareForLp();      /* SOH 学习结果在睡下前落盘 */
    PowerMode_DbgStopEnable();  /* 关外设/切时钟前就保持调试口 */

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
    if (PowerMode_LpButtonRequested() != 0u)
    {
        PowerMode_LpAbortForButton(0u);
        return;
    }

    /* 2) BQ：关 ADC/TEMP（SYS_CTRL1=0x00，LP 期间无电压/电流/温度检测）+
     *    CC_EN=0（FET 按 s_power.lp_fet_mask；默认全关）+ 清 SYS_STAT，
     *    防 CC_READY 唤醒风暴。准备失败时不得进入 LP：否则可能停在
     *    “ADC 已关但 CC 未关”的半配置状态，ALERT 会长期为高。 */
    {
        int prep = BQ_PrepareForLpMask(s_power.lp_fet_mask);

        if (prep != 0)
        {
            /* BKP20R：LP 准备失败现场。ADC 恢复回合/配置/verdict 期间
             * BQ_PrepareForLpMask 返回 -2，本次请求被静默作废（自动 LP
             * 要再等 5 s）；跨复位保留，避免只看到“没有 PB4 心跳”。
             * bit31..24=0xA6，bit19..16=BQ 状态，bit22=recovering，
             * bit21=adc_recovery_failed，bit20=fresh，bit15..8=prep。 */
            HAL_PWR_EnableBkUpAccess();
            RTC->BKP20R = 0xA6000000u |
                          (((uint32_t)BQ_GetState() & 0x0Fu) << 16) |
                          ((BQ_IsRecovering() != 0u) ? (1u << 22) : 0u) |
                          ((BQ_GetAdcRecoveryFailed() != 0u) ? (1u << 21) : 0u) |
                          ((BQ_IsFresh(HAL_GetTick()) != 0u) ? (1u << 20) : 0u) |
                          (((uint32_t)(uint8_t)prep) << 8);
            /* 不在这里写 Flash：prep=-2 时 BQ 正在 ADC 恢复回合，Flash
             * 编程的指令取指停顿会拉长 I2C/DMA 事务。BKP20R 跨 NRST
             * 保留，下一次 boot 的 stage66 再落盘。 */
            s_power.lp_prepare_fail_count++;
            s_power.request_lp = 0u;
            s_power.state = POWER_STATE_NORM;
            if (prep == -2)
            {
                /* BQ 正在 ADC 重启/配置：本次 LP 请求作废，不误报通信故障。 */
                return;
            }
            Fault_SetSticky(FAULT_BIT_BQ_COMM);
            if (BQ_IsIdle() != 0u)
            {
                BQ_ReInitAfterLp();     /* 把 SYS_CTRL1/2 恢复到 NORM 配置 */
            }
            return;
        }
    }
    s_power.lp_stage = 2u;      /* 2=BQ prepare done */

    /* 300 ms BQ 准备窗口内按下：在停外设/关 5 V 之前取消 LP，并恢复
     * BQ 的 ADC/FET 配置；不要让用户看到 5 V/灯先灭再被 pre-STOP1
     * guard 唤回。 */
    if (PowerMode_LpButtonRequested() != 0u)
    {
        PowerMode_LpAbortForButton(1u);
        return;
    }

    /* 2b) BQ 准备完成后、睡下之前检查 ALERT/SYS_STAT。
     * BQ_PrepareForLpMask 只清 CC_READY/XREADY；ADC 异常、受控重启或
     * 准备窗口内新锁存的 OV/UV/SCD/OCD 会让 PA8 保持高。此时如果直接
     * 进入 STOP1，首个 WUT 醒来会在 PA8 检查处完整唤醒，PB4 心跳分支
     * 还没执行，现场表现为“进了 LP 但没有 LED 脉冲”。先取消本次 LP，
     * 让 NORM 的 5 帧假锁存清理/故障处理跑完；BKP19R 保留 PA8/SYS_STAT
     * 现场。这里是“ALERT 实际为高”的物理判据，保护锁存本身的故障
     * 处置仍由 BQ/状态机负责。 */
    {
        bq_snapshot_t lp_bq;
        uint8_t pa8_high;

        BQ_GetSnapshot(&lp_bq);
        pa8_high = (HAL_GPIO_ReadPin(BQ_ALT_EXTI8_GPIO_Port,
                                     BQ_ALT_EXTI8_Pin) == GPIO_PIN_SET) ? 1u : 0u;
        /* SYS_STAT 的 OCD/SCD/OV/UV 锁存保留是安全设计；这里只以实际
         * PA8 电平作为“睡下会立即被 ALERT 唤醒”的判据。保护锁存本身
         * 仍由状态机/故障处理消化，BKP19R 记录 SYS_STAT 供诊断。 */
        PowerMode_LpRecordEntryBq(&lp_bq, pa8_high);
        if (pa8_high != 0u)
        {
            s_power.lp_alert_abort_count++;
            s_power.request_lp = 0u;
            s_power.state = POWER_STATE_NORM;
            s_power.lp_stage = 0u;
            PowerMode_LpDiagStage(65u, 0x0400u);   /* 65=LP aborted: PA8/BQ ALERT high */
            BQ_ReInitAfterLp();                    /* 恢复 ADC/FET 到 NORM 配置 */
            return;
        }
    }

    /* 3) 黑匣子转换记录（此时仍在 NORM，Flash 可写）。 */
    (void)BlackBox_WriteTransition(BB_KIND_LP_ENTER, BB_WAKE_NONE,
                                   App_GetUptimeSeconds());
    s_power.lp_stage = 3u;      /* 3=black-box LP record done */

    /* 4) 布防 RTC 2 s 嗅探唤醒；充电/按键/ALERT/30 min 维护才回 NORM。
     *    布防失败则不进入 LP。先清掉上一次 WUT 的残留状态，再 arm；
     *    失败重试一次，并把 HAL 返回码写入 BKP18R（跨复位可查）。 */
    s_power.flag_rtc = 0u;
    s_power.wake_cause = 0u;
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 1u, 0u);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    if ((RTC->CR & RTC_CR_WUTE) != 0u)
    {
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    }
    PowerMode_LpClearWutPending();
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    EXTI->PR1 = EXTI_PR1_PIF20;
    PowerMode_LpDiagStage(4u, 0x0001u); /* 4=about to arm RTC WUT */
    wut_status = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, POWER_LP_WUT_RELOAD,
                                             RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    if (wut_status != HAL_OK)
    {
        /* 第二次 LP 若上一次 WUT 状态残留，先彻底清一次再重试。 */
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
        PowerMode_LpClearWutPending();
        HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
        EXTI->PR1 = EXTI_PR1_PIF20;
        wut_status = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, POWER_LP_WUT_RELOAD,
                                                 RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }
    PowerMode_LpRecordWutArm(wut_status, 0u);
    if (wut_status != HAL_OK)
    {
        /* flags = HAL 返回码（1=ERROR, 2=BUSY, 3=TIMEOUT），便于区分
         * LSE/WUTWF 超时与 hrtc 状态机 BUSY。 */
        PowerMode_LpDiagStageFlags(41u, 0x0020u, (uint8_t)wut_status);
        s_power.lse_ok = 0u;
        s_power.request_lp = 0u;
        s_power.state = POWER_STATE_NORM;
        Fault_SetLive(FAULT_BIT_LSE);
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
        HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
        return;
    }
    s_power.lp_stage = 4u;      /* 4=RTC WUT armed */
    PowerMode_LpDiagStage(40u, 0x0002u); /* 40=RTC WUT arm returned OK */

    /* 5) 停外设、灭灯。PB5 的 GPIO/EXTI/NVIC 配置由 MX_GPIO_Init 完成，
     *    这些 DeInit 只动 PA/PB6/PB7，不会碰 PB5，无需每次进 LP 重配。 */
    PowerMode_StopPeripherals();
    s_power.lp_stage = 5u;      /* 5=peripherals stopped */
    PowerMode_LpDiagStage(5u, 0x0004u); /* 5=peripherals stopped */

    /* 6) 只清 PA8 事件；**不要清 PIF5**——按键可能正好在停外设/布防窗口
     *    按下并释放，清 PR1 会把真实下降沿丢掉。PIF5 由 EXTI ISR 清除；
     *    若 ISR 尚未执行，保留的 pending 位还会让 STOP1 立即唤醒。 */
    if ((RTC->ISR & RTC_ISR_WUTF) == 0u)
    {
        PowerMode_LpClearWutPending();
    }
    /* WUTF=1 是有效 tick：保留 WUTF/PR20/NVIC pending，让第一次 WFI
     * 立即醒来并走 callback，不能在这里清掉。 */
    EXTI->PR1 = (uint32_t)EXTI_PR1_PIF8;

    /* 7) LP 时钟：HSI16（1 s 嗅探的 I2C 也在这个时钟下）；STOPWUCK=HSI。 */
    __HAL_RCC_HSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == 0u)
    {
    }
    s_power.lp_stage = 6u;      /* 6=HSI16 ready */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

    /* 8) SYSCLK → HSI16；再降电压等级。 */
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_HSI)
    {
    }
    s_power.lp_stage = 7u;      /* 7=SYSCLK HSI16 */
    SystemCoreClock = 16000000u;
    /* 80 MHz -> 16 MHz 后必须立刻重设 SysTick：否则仍用 80000 的重装值，
     * HAL tick 变成 5 ms，HAL_RTC 的 1 s 超时实际变 5 s，LP 阶段的
     * WUT/I2C 超时全部失真。 */
    (void)HAL_InitTick(TICK_INT_PRIORITY);
    (void)HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
    s_power.lp_stage = 8u;      /* 8=Scale2 */

    /* 9) 关 PLL；关 HSE 前先抬 PA1 停掉 OSC1。 */
    __HAL_RCC_PLL_DISABLE();
    while ((RCC->CR & RCC_CR_PLLRDY) != 0u)
    {
    }
    s_power.lp_stage = 9u;      /* 9=PLL off */

    /* 先关 OSC1（PA1 高），再清 HSEON/HSEBYP：HSE bypass 模式下外部时钟
     * 一直存在，先清 HSEON 不一定能让 HSERDY 落，顺序不能反。 */
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);   /* 清 HSEON 与 HSEBYP，唤醒后才能重配 */
    {
        /* HSE 已不是 SYSCLK/PLL 源；HSERDY 只是状态位，不能无限等待。 */
        uint32_t guard = 0u;

        while (((RCC->CR & RCC_CR_HSERDY) != 0u) && (guard < 10000u))
        {
            guard++;
        }
        if ((RCC->CR & RCC_CR_HSERDY) != 0u)
        {
            s_power.hse_off_timeout = 1u;
        }
    }
    s_power.lp_stage = 10u;     /* 10=HSE off/continued despite HSERDY */
    PowerMode_LpDiagStage(10u, 0x0008u); /* 10=HSE off */

    /* 10) 关 5 V。 */
    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_RESET);
    s_power.lp_stage = 11u;     /* 11=PB0 low */
    PowerMode_LpDiagStage(11u, 0x0010u); /* 11=PB0 low */

    s_power.state = POWER_STATE_LP;
    s_power.lp_enter_count++;
    s_bb_shadow.diag_lp_enter_count = s_power.lp_enter_count;
    PowerMode_LpBumpBkp(&RTC->BKP17R);   /* 跨复位累计进入 LP 次数 */
    s_power.lp_stage = 12u;     /* 12=LP loop entry */

    /* 每次 LP 进入写一条“已进入 STOP 循环”的诊断记录；如果卡在进入
     * LP 之前，Flash 里会只有 LP_ENTER 而没有这条 stage=12 记录。 */
    if (s_power.lp_diag_entry_logged == 0u)
    {
        s_power.lp_diag_entry_logged = 1u;
        (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(), 0u,
                                   s_power.button_irq_count, 0u,
                                   12u, PowerMode_LpButtonPathFlags());
        PowerMode_LpMarkStage(12u, PowerMode_LpButtonPathFlags());
    }
    s_lp_sniff.total_count = 0u;
    s_lp_sniff.ok_count = 0u;
    s_lp_sniff.fail_count = 0u;
    s_lp_sniff.init_fail_count = 0u;
    s_lp_sniff.last_ms = 0u;
    s_lp_sniff.last_ok = 0u;
    s_lp_sniff.last_error = 0u;
    s_lp_sniff.last_sys_ctrl1 = 0u;
    s_lp_sniff.last_sys_stat = 0u;
    s_lp_sniff.last_sys_ctrl2 = 0u;
    s_lp_sniff.cc_ready_clear_count = 0u;
    s_lp_sniff.cc_ready_clear_fail_count = 0u;
    s_lp_sniff.last_load_present = 0u;
    s_lp_sniff.load_present_count = 0u;
    s_lp_sniff.charger_check_count = 0u;
    s_lp_sniff.charger_detect_count = 0u;
    s_lp_sniff.load_reject_count = 0u;
    s_lp_sniff.charger_check_error_count = 0u;
    s_lp_sniff.last_charger_level = 0u;
    s_power.lp_charger_pending = 0u;

    /* 初始 WUT 可能在 StopPeripherals/时钟切换期间已经触发；WUT 是自动
     * 重装的，只要 WUTE/WUTIE 仍置位就继续运行。这里只清标志/挂起，不再
     * 关掉再重布防，避免 HAL 在 1 Hz CK_SPRE 下等待 WUTWF 超时。 */
    s_power.flag_rtc = 0u;
    if ((RTC->ISR & RTC_ISR_WUTF) == 0u)
    {
        PowerMode_LpClearWutPending();
    }
    /* WUTF=1：保留这次已发生的 tick，第一次 STOP1 会立即返回并处理。 */
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    /* 11) 1 s 嗅探循环：RTC tick 做 I2C/充电判定；充电、按键、ALERT 或
     *     1800 tick（30 min）维护点才完整唤醒。 */
    full_wake = 0u;
    cause = 0u;
    if ((RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) !=
        (RTC_CR_WUTE | RTC_CR_WUTIE))
    {
        if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, POWER_LP_WUT_RELOAD,
                                        RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
        {
            s_power.lse_ok = 0u;
            Fault_SetLive(FAULT_BIT_LSE);
            full_wake = 1u;
            cause |= POWER_WAKE_RTC;
        }
    }
    while (full_wake == 0u)
    {
        /* RAM 影子诊断：只写 SRAM，不写 Flash。Ozone 展开 s_bb_shadow
         * 可判断 WUT 循环是否在跑、PB5 EXTI 是否进入、引脚/IMR 状态。 */
        s_bb_shadow.diag_lp_loop_count++;
        s_bb_shadow.diag_wut_count = s_lp_sniff.total_count;
        s_bb_shadow.diag_last_stage = s_power.lp_stage;
        s_bb_shadow.diag_flag_button = s_power.flag_button;
        s_bb_shadow.diag_pb5_level =
            (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
             GPIO_PIN_RESET) ? 1u : 0u;
        s_bb_shadow.diag_exti_imr =
            ((EXTI->IMR1 & EXTI_IMR1_IM5) != 0u) ? 1u : 0u;
        s_bb_shadow.diag_exti_pending =
            ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ? 1u : 0u;

        /* 本次 LP 首次进入循环体；必须在下面的按键检查之前写 Flash，
         * 避免在 pre-STOP1 检查和 WFI 之间插入一次 Flash 编程窗口。 */
        PowerMode_LpDiagStageFlags(13u, 0x0100u,
                                   PowerMode_LpButtonPathFlags());
        PowerMode_LpMarkStage(13u, PowerMode_LpButtonPathFlags());

        {
            uint8_t pb5_low =
                (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
                 GPIO_PIN_RESET) ? 1u : 0u;
            uint8_t pif5 = ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ? 1u : 0u;

            if ((pif5 != 0u) || (s_power.flag_button != 0u) ||
                (pb5_low != 0u))
            {
                PowerMode_LpDiagButtonAttempt(pif5, pb5_low);
            }
        }

        /* 进入 STOP1 前做最后一道保险：RTC WUT 的定时器和中断使能必须
         * 同时为 1，否则 STM32 不会从 STOP1 被 WUT 唤醒。若被意外清除，
         * 这里重布防；重布防失败则直接完整唤醒，绝不在无唤醒源下睡。 */
        {
            uint8_t repair_arm = (PowerMode_LpWutArmed() == 0u) ? 1u : 0u;

            if (repair_arm == 0u)
            {
                /* WUTE/WUTIE 在，但 EXTI/NVIC 侧可能被意外清掉：
                 * 直接补位，避免做一次会等待 WUTWF 的完整重布防。 */
                if ((EXTI->IMR1 & EXTI_IMR1_IM20) == 0u)
                {
                    EXTI->IMR1 |= EXTI_IMR1_IM20;
                }
                if ((EXTI->RTSR1 & EXTI_RTSR1_RT20) == 0u)
                {
                    EXTI->RTSR1 |= EXTI_RTSR1_RT20;
                }
                if (((NVIC->ISER[0] >> (uint32_t)RTC_WKUP_IRQn) & 1u) == 0u)
                {
                    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
                }
                if ((RTC->ISR & RTC_ISR_WUTF) == 0u)
                {
                    /* 只清 stale EXTI/NVIC；WUTF=1 是有效 tick，保留。 */
                    EXTI->PR1 = EXTI_PR1_PIF20;
                    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
                }
            }
            else
            {
                HAL_StatusTypeDef status;

                if ((RTC->CR & RTC_CR_WUTE) != 0u)
                {
                    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
                }
                PowerMode_LpClearWutPending();
                HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
                status = HAL_RTCEx_SetWakeUpTimer_IT(
                    &hrtc, POWER_LP_WUT_RELOAD,
                    RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
                if (status != HAL_OK)
                {
                    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
                    PowerMode_LpClearWutPending();
                    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
                    status = HAL_RTCEx_SetWakeUpTimer_IT(
                        &hrtc, POWER_LP_WUT_RELOAD,
                        RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
                }
                PowerMode_LpRecordWutArm(status, 1u);
                if (status != HAL_OK)
                {
                    s_power.lse_ok = 0u;
                    Fault_SetLive(FAULT_BIT_LSE);
                    full_wake = 1u;
                    cause |= POWER_WAKE_RTC;
                    break;
                }
            }
        }

        /* 最后一层 LSE 健康检查：WUTE/WUTIE 只是寄存器位，LSERDY=0 时
         * WUT 不会产生 tick。ADC/时钟切换或外部干扰可能让 LSE 在布防后
         * 失振；此时绝不进入 STOP1，直接完整唤醒回 NORM，让 health task
         * 在 LSE 恢复后清 live 故障并重试 auto-LP。STOP1 内部 LSE 失振
         * 仍需要一个独立于 LSE 的 watchdog（见 LPTIM/LSI 方案）。 */
        if (PowerMode_LseReady() == 0u)
        {
            s_power.lse_ok = 0u;
            s_power.lse_healthy_since_ms = 0u;
            Fault_SetLive(FAULT_BIT_LSE);
            full_wake = 1u;
            cause |= POWER_WAKE_RTC;
            break;
        }

        /* 首次 WFI 前记录 RTC 的最终布防状态（WUTE/WUTIE/WUTF）和 PB5
         * 通路；必须放在下面的按键检查之前，Flash 写入窗口不会丢掉按键。
         * 若这条 stage=30 是最后一条，说明已经进入 STOP1 但没有被 WUT/按键
         * 唤醒；字段映射见 BlackBox_WriteLpDiag 注释。 */
        if (s_power.lp_diag_wfi_logged == 0u)
        {
            s_power.lp_diag_wfi_logged = 1u;
            PowerMode_LpMarkStage(30u, PowerMode_LpWutPathFlags());
            /* 字段：cell1=CR[15:0], cc=WUTR[15:0],
             * cell2=ISR[15:0], ts1=RTC 辅助位（WUCKSEL/LSE/RTCEN），
             * cell3=PRER[15:0]。 */
            (void)BlackBox_WriteLpDiag(
                App_GetUptimeSeconds(),
                ((uint32_t)RTC->CR & 0xFFFFu) |
                    ((uint32_t)RTC->WUTR << 16),
                ((uint32_t)RTC->ISR & 0xFFFFu) |
                    (PowerMode_LpRtcAux() << 16),
                (uint32_t)RTC->PRER & 0xFFFFu,
                30u, PowerMode_LpWutPathFlags());
        }

        /* 编译开关：默认保持 STOP1 中的 SWD（DBG_STOP），便于观察 lp_stage；
         * 量产低功耗构建用 -DPOWER_LP_KEEP_SWD=0 关闭。 */
        PowerMode_DbgStopEnable();
        PowerMode_LpEnsureButtonPath();

        /* 请求自动 LP 到真正 WFI 之间用户可能已经按下；同时看
         * PIF5 pending（ISR 还没跑）、flag_button（按下后已释放）和
         * 当前电平（临界区/时钟切换期间 EXTI 尚未响应）。三者任一成立
         * 都直接完整唤醒，避免按下瞬间落在嗅探窗口被多睡一个 WUT 周期。 */
        if (((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ||
            (s_power.flag_button != 0u) ||
            (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
             GPIO_PIN_RESET))
        {
            PowerMode_LpDiagButtonAttempt(
                ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ? 1u : 0u,
                (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
                 GPIO_PIN_RESET) ? 1u : 0u);
            full_wake = 1u;
            cause |= POWER_WAKE_BUTTON;
            break;
        }

        s_power.lp_stage = 13u;     /* 13=about to STOP1 */
        /* 任何 Flash/HAL 临界区都不应把 PRIMASK 带到 WFI；这里显式开中断，
         * 防止全局中断被意外关闭后 WUT/EXTI 到达却无法唤醒/进入 ISR。 */
        __enable_irq();
        /* 31=即将执行 HAL_SuspendTick/WFI；这是备份寄存器写入，不写 Flash。
         * flags: bit0=debugger_attached, bit1=lp_keep_swd（DBG_STOP 实际决策）。 */
        PowerMode_LpMarkStage(31u,
            (uint8_t)((s_power.debugger_attached != 0u ? 0x01u : 0u) |
                      (s_power.lp_keep_swd != 0u ? 0x02u : 0u)));
        HAL_SuspendTick();
        /* 32=已挂起 SysTick，下一步就是 WFI；32 存在但无 33 表示
         * CPU 真正进入了 STOP1 且没有返回。 */
        PowerMode_LpMarkStage(32u, PowerMode_LpWutPathFlags());
        /* 与 HAL_PWREx_EnterSTOP1Mode(WFI) 等价，但在设置 SLEEPDEEP/LPMS
         * 之后、WFI 之前把睡眠现场写入 BKP4R..8R。这样复位后能区分
         * “已正确进入 STOP1 且中断全开”与“HardFault/PRIMASK/没设 SLEEPDEEP”。 */
        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP1);
        SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
        PowerMode_LpSnapshotSleepRegs();
        __DSB();
        __WFI();
        CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

        /* 33=WFI 已经返回（即使后续 stage14 的 Flash 诊断失败也可区分）；
         * 31/32 而没有 33 表示 CPU 从未退出 STOP1/WFI。 */
        PowerMode_LpMarkStage(33u, 0u);

        /* 每次 STOP1 醒来都在 HSI16；先恢复 SysTick，再做任何 Flash
         * 写入（HAL_FLASH_Program 的超时依赖 HAL_GetTick，SysTick 仍停
         * 着时写 Flash 可能卡死，表现为 LP 醒来后死机）。 */
        PowerMode_LpMarkStage(14u, PowerMode_LpButtonPathFlags());
        s_power.lp_stage = 14u;     /* 14=woken from STOP1 */
        SystemCoreClock = 16000000u;
        (void)HAL_InitTick(TICK_INT_PRIORITY);
        /* 34=WFI 返回且 SysTick 已恢复；33 有而 34 无表示卡在 tick 恢复。 */
        PowerMode_LpMarkStage(34u, PowerMode_LpButtonPathFlags());
        PowerMode_LpDiagStageFlags(14u, 0x0200u,
                                   PowerMode_LpButtonPathFlags());

        /* LP 嗅探心跳：PB4（IND_NORM，3V3 供电、高有效）每次 WUT tick
         * 闪 lp_sniff_led_ms。必须放在下面的按键/PA8 检查之前：
         * 若 BQ ALERT 在 LP 中变高，旧代码先在 PA8 检查处完整唤醒、
         * 根本走不到原在 flag_rtc 分支里的心跳，现场就表现为“进过 LP
         * 但没有 LED 脉冲”。现在只要有 RTC WUT tick 就先闪一下，再决定
         * 是否完整唤醒；无脉冲仍然表示没有 WUT tick（或根本没进 LP）。
         * 生产测电流可把 lp_sniff_led_ms 写 0。 */
        if ((s_power.flag_rtc != 0u) && (s_power.lp_sniff_led_ms != 0u))
        {
            HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin,
                              GPIO_PIN_SET);
            HAL_Delay(s_power.lp_sniff_led_ms);
            HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin,
                              GPIO_PIN_RESET);
        }

        /* EXTI pending/flag 优先；同时读引脚电平：若 PB5 一直被按到
         * WUT tick，即使 ISR 尚未执行也能唤醒。 */
        if (((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ||
            (s_power.flag_button != 0u) ||
            (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port,
                              WAKE_EXTI5_Pin) == GPIO_PIN_RESET))
        {
            PowerMode_LpDiagButtonAttempt(
                ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ? 1u : 0u,
                (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port,
                                  WAKE_EXTI5_Pin) == GPIO_PIN_RESET) ? 1u : 0u);
            full_wake = 1u;
            cause |= POWER_WAKE_BUTTON;
        }
        /* PA8 配置为 EXTI 事件模式（无挂起位），所以还要读引脚电平：
         * ALERT 在进 STOP1 前已经为高时不会产生新的上升沿，仅靠 PIF8
         * 会漏掉一次真实保护事件。 */
        if (((EXTI->PR1 & EXTI_PR1_PIF8) != 0u) ||
            (HAL_GPIO_ReadPin(BQ_ALT_EXTI8_GPIO_Port,
                              BQ_ALT_EXTI8_Pin) == GPIO_PIN_SET))
        {
            full_wake = 1u;
            cause |= POWER_WAKE_BQ_ALERT;
        }

        if (full_wake == 0u)
        {
            if (s_power.flag_rtc != 0u)
            {
                s_power.flag_rtc = 0u;
                __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
                HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

                /* WUT 保持自由运行（WUTE/WUTIE 不关），这里不再每个 tick
                 * 关掉再重布防：1 Hz CK_SPRE 下 WUTWF 同步可能超过 HAL 的
                 * 1 s 超时，且重布防窗口内到达的 tick 会让回调清 WUTIE，
                 * 使下一条 STOP1 没有 RTC 唤醒源。PB4 心跳已在上面
                 * （任何 RTC tick、按键/PA8 检查之前）完成。 */
                PowerMode_LpSniffOnce();
                s_power.lp_stage = 15u;     /* 15=1 s sniff done */

                /* 本次 LP 第一次 WUT 嗅探完成后写一条诊断：有这条记录
                 * 就证明 WUT 确实把 CPU 唤醒并执行了 LP 代码。 */
                if ((s_power.lp_diag_wut_logged == 0u) &&
                    (s_lp_sniff.total_count >= 1u))
                {
                    s_power.lp_diag_wut_logged = 1u;
                    (void)BlackBox_WriteLpDiag(App_GetUptimeSeconds(),
                                               s_lp_sniff.total_count,
                                               s_power.button_irq_count,
                                               s_bb_shadow.diag_lp_loop_count,
                                               s_power.lp_stage,
                                               PowerMode_LpButtonPathFlags());
                }

                if (s_power.lp_charger_pending != 0u)
                {
                    /* 充电器插入：完整唤醒到 NORM；RTC 位保留给黑匣子低 3 位。 */
                    full_wake = 1u;
                    cause |= (uint8_t)(POWER_WAKE_RTC | POWER_WAKE_CHARGER);
                }
                else if (s_lp_sniff.total_count >=
                         (uint32_t)s_power.lp_monitor_ticks)
                {
                    /* 30 min 到点：优先在 LP/HSI16 内完成储存监测和黑匣子
                     * 记录，不启动 5 V、不切 HSE/PLL；只有监测或 BQ 恢复
                     * 失败才完整唤醒（POWER_WAKE_MONITOR）重配。 */
                    if (PowerMode_LpStorageMonitorOnce() != 0u)
                    {
                        s_lp_sniff.total_count = 0u;
                    }
                    else
                    {
                        full_wake = 1u;
                        cause |= (uint8_t)(POWER_WAKE_RTC | POWER_WAKE_MONITOR);
                    }
                }
            }
            else
            {
                /* WFI 返回但没有任何 RTC/按键/PA8 标志：不要误归因为
                 * BQ ALERT。完整唤醒并留下 stage35 现场，便于区分
                 * stale EXTI/WUTF、调试器 halt 或其它 EXTI 事件。 */
                s_power.wake_unknown_count++;
                full_wake = 1u;
                if (s_power.lp_diag_unknown_logged == 0u)
                {
                    s_power.lp_diag_unknown_logged = 1u;
                    (void)BlackBox_WriteLpDiag(
                        App_GetUptimeSeconds(),
                        (uint32_t)EXTI->PR1,
                        (uint32_t)RTC->ISR & 0xFFFFu,
                        s_power.wake_unknown_count,
                        35u, PowerMode_LpWutPathFlags());
                }
            }
        }

        /* 只清 PA8 事件；PIF5 不能在这里无条件清——它可能正好是本轮
         * 醒来后才到达的按键下降沿，清掉会丢失按键唤醒。PB5 的 pending
         * 位由 EXTI ISR 清除；flag_button 保留到下一轮判断。 */
        EXTI->PR1 = (uint32_t)EXTI_PR1_PIF8;
        /* 不要在这里无条件清 flag_button/flag_rtc：处理窗口内新到的
         * 按键或 WUT tick 必须保留到下一轮判断。 */
    }

    s_power.flag_rtc = 0u;
    s_power.flag_button = 0u;
    EXTI->PR1 = (uint32_t)(EXTI_PR1_PIF5 | EXTI_PR1_PIF8);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    if (s_power.lp_keep_swd == 0u)
    {
        DBGMCU->CR &= (uint32_t)~DBGMCU_CR_DBG_STOP;   /* 运行期关闭调试时恢复最低功耗 */
    }
    s_power.lp_dbg_stop_active =
        ((DBGMCU->CR & DBGMCU_CR_DBG_STOP) != 0u) ? 1u : 0u;

    /* ================= 完整唤醒点 ================= */
    s_power.lp_stage = 16u;     /* 16=full wake */
    s_power.state = POWER_STATE_WAKING;
    s_power.wake_cause = cause;
    s_power.wake_count++;
    s_bb_shadow.diag_wake_count = s_power.wake_count;
    s_bb_shadow.diag_last_wake_cause = cause;
    if ((cause & POWER_WAKE_RTC) != 0u)
    {
        s_power.wake_rtc_count++;
    }
    if ((cause & POWER_WAKE_BUTTON) != 0u)
    {
        s_power.wake_button_count++;
        PowerMode_LpBumpBkp(&RTC->BKP14R);
    }
    if ((cause & POWER_WAKE_BQ_ALERT) != 0u)
    {
        s_power.wake_alert_count++;
    }
    if ((cause & POWER_WAKE_CHARGER) != 0u)
    {
        s_power.wake_charger_count++;
    }
    if ((cause & POWER_WAKE_MONITOR) != 0u)
    {
        s_power.wake_monitor_count++;
    }

    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    EXTI->PR1 = EXTI_PR1_PIF20;   /* 关 WUT 后清 stale EXTI pending */
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    s_power.flag_rtc = 0u;
    s_power.flag_button = 0u;

    /* 当前在 HSI16；恢复 HSE/PLL 80 MHz，失败则降级。先把 5 V 和故障灯
     * 打开：即使 HSE/PLL 恢复失败或卡住，也不能让用户看到“醒了但
     * 5 V/灯全灭”。正常路径随后由 LedStatus 接管。 */
    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IND_ROHT_GPIO_Port, IND_ROHT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_RESET);
    SystemCoreClock = 16000000u;
    (void)HAL_InitTick(TICK_INT_PRIORITY);
    if (PowerMode_ClkSwitchToHsePll() == 0u)
    {
        s_power.degraded = 1u;
        s_power.wake_clock_fail_count++;
        s_power.request_lp = 0u;
        s_power.state = POWER_STATE_DEGRADED;
        s_power.lp_stage = 18u;    /* 18=唤醒后 HSE/PLL 恢复失败 */
        PowerMode_LpMarkStage(18u, 0u);
        /* 这里时钟可能处于 HSE/PLL 尝试失败后的不确定状态，不做 Flash
         * 写入；stage18 备份寄存器 + 5 V + PB3 降级闪烁已足够定位。 */
        Fault_SetSticky(FAULT_BIT_CLK);
        return;
    }

    /* 唤醒键仍按下则从唤醒时刻开始计时；已在时钟恢复期间释放则合成短按。 */
    Button_OnWake((uint8_t)((s_power.wake_cause & POWER_WAKE_BUTTON) != 0u),
                  (uint8_t)((HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port,
                                              WAKE_EXTI5_Pin) ==
                             GPIO_PIN_RESET) ? 1u : 0u),
                  HAL_GetTick());

    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(2u);   /* 5 V 软启动余量 */

    PowerMode_RestorePeripherals();

    /* 30 min 维护唤醒不在这里写记录：5 s 后自动重新进 LP，由下一次
     * LP_ENTER 记录携带当时的电压/电流快照。这样每 30 min 只占 1 条，
     * 1020 条覆盖 ≥15 天。充电唤醒很少见，保留 WAKE_MASK + 事件两条。 */
    if ((s_power.wake_cause & POWER_WAKE_MONITOR) == 0u)
    {
        (void)BlackBox_WriteWake(s_power.wake_cause, App_GetUptimeSeconds());
        if ((s_power.wake_cause & POWER_WAKE_CHARGER) != 0u)
        {
            (void)BlackBox_WriteEvent(BB_EVENT_WAKE_CHARGER,
                                      App_GetUptimeSeconds());
        }
    }

    s_power.request_lp = 0u;
    s_power.state = POWER_STATE_NORM;
}

void PowerMode_Loop(void)
{
    if (s_power.degraded != 0u)
    {
        return;
    }

    /* 每个 NORM 主循环检查一次 LSE/LSE→RTC 时钟源；LSE 受扰恢复后
     * 自动清 live/sticky LSE，让 auto-LP 的 APP_LP_BLOCK_LSE 解除。 */
    PowerMode_LseHealthTask();

    if ((s_power.request_lp != 0u) && (s_power.state == POWER_STATE_NORM))
    {
        PowerMode_EnterLp();
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

uint32_t PowerMode_GetWakeCount(void)
{
    return s_power.wake_count;
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
    /* 先写 BKP1R=0x0E 证明 RTC_WKUP 向量确实进入过（即使 HAL 因 WUTF
     * 已清而没调用 callback）；callback 再覆盖为 0x0F。stage60.ts1 低
     * 16 位为 0 时，才能确定是事件没有到达 CPU/NVIC，而不是主循环问题。
     * BKP2R 低 16 位累计向量进入次数，跨复位保留，用于区分“从未进入”
     * 与“只进入过、callback 被跳过”。 */
    HAL_PWR_EnableBkUpAccess();
    {
        uint32_t isr_count = RTC->BKP2R & 0xFFFFu;

        if (isr_count < 0xFFFFu)
        {
            isr_count++;
        }
        RTC->BKP2R = POWER_LP_BKP_ISR_MARKER | isr_count;
    }
    RTC->BKP1R = 0xA5A5000Eu;
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_ptr)
{
    if (hrtc_ptr->Instance == RTC)
    {
        /* 只置标志，绝不能在这里清 WUTIE：WUTIE=0 时 STM32 不会从
         * STOP1 被唤醒。若 WUT tick 正好落在重布防/进 STOP1 的窗口，
         * 清 WUTIE 会让设备永久睡死。 */
        s_power.flag_rtc = 1u;

        /* BKP1R=0xA5A5000F：证明 RTC WUT ISR 确实进入过。若复位后
         * stage60 的 ts1 低 16 位为 0，而 BKP0R 停在 30，说明 WUT 事件
         * 没有到达 CPU/NVIC，而不是主循环唤醒路径的问题。
         * BKP3R 低 16 位累计 callback 次数；BKP2R>0 而 BKP3R=0 表示
         * 向量进了但 HAL 因 WUTF=0 跳过 callback。 */
        HAL_PWR_EnableBkUpAccess();
        {
            uint32_t cb_count = RTC->BKP3R & 0xFFFFu;

            if (cb_count < 0xFFFFu)
            {
                cb_count++;
            }
            RTC->BKP3R = POWER_LP_BKP_ISR_MARKER | cb_count;
        }
        RTC->BKP1R = 0xA5A5000Fu;
    }
}

/* 生成代码的 EXTI9_5_IRQHandler 只派发 PB5（WAKE_EXTI5）。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == WAKE_EXTI5_Pin)
    {
        s_power.flag_button = 1u;
        s_power.flag_button_ms = HAL_GetTick();
        s_power.button_irq_count++;
        s_bb_shadow.diag_button_irq_count++;
        PowerMode_LpBumpBkp(&RTC->BKP13R);
        /* BKP16R：最后一次 PB5 ISR 发生时的状态/阶段，跨复位可读。
         * high16=marker；bits16..23=state；bits8..15=lp_stage；
         * bit0=ISR 时 PB5 是否为低。用来区分“按键时其实还在 NORM”与
         * “已在 LP 但 EXTI/唤醒路径没走通”。 */
        HAL_PWR_EnableBkUpAccess();
        /* BKP16R 布局：bit31..24=0xA5 标志，bit23..16=state，
         * bit15..8=lp_stage，bit0=ISR 时 PB5 电平低。 */
        RTC->BKP16R = POWER_LP_BKP_CTX_MARKER |
                      ((uint32_t)s_power.state << 16) |
                      ((uint32_t)s_power.lp_stage << 8) |
                      ((HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port,
                                         WAKE_EXTI5_Pin) == GPIO_PIN_RESET) ? 1u : 0u);
        Button_NotifyExtiFromIsr();
    }
}

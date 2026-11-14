/**
 ******************************************************************************
 * @file    lp_cycle.c
 * @brief   IWDG 复位式 LP 的轻量 boot/周期路径。
 ******************************************************************************
 */
#include "app/lp_cycle.h"

#include "board.h"
#include "app/black_box.h"
#include "app/charge_policy.h"
#include "app/lp_session.h"
#include "app/main_app.h"
#include "app/power_mode.h"
#include "app/reset_cause.h"
#include "app/rtc_time.h"
#include "app/state_store.h"
#include "app/watchdog.h"
#include "bsp/bq_ic.h"
#include "bsp/button.h"

#include <string.h>

#define LP_BASE_CYCLE_MS        30000u
#define LP_MIN_SLEEP_MS         1000u
#define LP_MARK_TOLERANCE_MS    1000u
#define LP_CHARGER_SETTLE_MS    30u
#define LP_BOOT_OVERHEAD_MAX_MS 5000u
#define LP_LSI_NOMINAL_HZ       32000u
#define LP_HEARTBEAT_MS         10u     /* PB4 每次 light LP boot 短闪 */

static lp_session_t s_lp;
static uint8_t s_lp_loaded;
static uint8_t s_boot_wake_cause; /* 当前 App_Boot 已校验的 LP_WAKE_*；不是旧 BKP 残留 */
#if (LP_SMBA_WAKE_TEST != 0u)
static uint8_t s_smba_test_active;
#endif
static rtc_epoch_t s_boot_rtc;

static uint32_t LpCycle_LsiHz(void)
{
    uint32_t hz = s_lp.lsi_hz_q16 >> 16;

    return (hz != 0u) ? hz : LP_LSI_NOMINAL_HZ;
}

static uint32_t LpCycle_ExpectedMs(void)
{
    uint32_t div;
    uint32_t hz;

    if ((s_lp.last_pr > 6u) || (s_lp.last_rlr == 0u))
    {
        return 0u;
    }
    div = 4u << s_lp.last_pr;
    hz = LpCycle_LsiHz();
    return ((uint32_t)(s_lp.last_rlr + 1u) * div * 1000u) / hz;
}

static void LpCycle_Save(void)
{
    s_lp.seq++;
    LpSession_Save(&s_lp);
}

/* RTC 的 mark_epoch 是严格跨 LP/NORM 的归档 ID；即使 FullWake 由监测失败
 * 触发也不提前推进标记。只在黑匣子/StateStore 均完成后才确认。 */
static uint8_t LpCycle_FinishMark(uint32_t mark_epoch, uint32_t now_epoch)
{
    lp_session_t previous;
    lp_session_t check;
    uint32_t next_mark;

    if ((s_lp.next_mark_epoch != mark_epoch) ||
        ((s_lp.flags & LP_FLAG_MARK_PENDING) == 0u))
    {
        return 0u;
    }

    previous = s_lp;
    s_lp.flags &= (uint8_t)~(LP_FLAG_MARK_PENDING |
                              LP_FLAG_MARK_LP_ARCHIVED);
    do
    {
        if (s_lp.next_mark_epoch > (0xFFFFFFFFu - 1800u))
        {
            s_lp.next_mark_epoch = 0u;
            break;
        }
        s_lp.next_mark_epoch += 1800u;
    } while (s_lp.next_mark_epoch <= now_epoch);
    next_mark = s_lp.next_mark_epoch;
    LpCycle_Save();
    if ((LpSession_Load(&check) == 0u) ||
        (check.next_mark_epoch != next_mark) ||
        ((check.flags & LP_FLAG_MARK_PENDING) != 0u))
    {
        s_lp = previous;
        return 0u;
    }
    s_lp = check;
    return 1u;
}

static void LpCycle_PreservePendingFaultWake(void)
{
    s_lp.state = LP_SESSION_PAUSED;
    s_lp.wake_cause = (LP_WAKE_MONITOR | LP_WAKE_FAULT);
    s_lp.flags |= (LP_FLAG_FULL_WAKE | LP_FLAG_FAULT);
    LpCycle_Save();
    s_boot_wake_cause = s_lp.wake_cause;
}

static void LpCycle_DisableLegacyWut(void)
{
    /* 旧固件可能把 RTC WUT 留在运行状态；它写 BKP6 会破坏 LP 会话，
     * 新设计不再使用 WUT。每次进入/启动 LP 前彻底关掉。 */
    if ((RTC->CR & RTC_CR_WUTE) != 0u)
    {
        (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    }
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    EXTI->PR1 = EXTI_PR1_PIF20;
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
}

static void LpCycle_ForceFullWake(uint8_t lp_cause)
{
    s_lp.state = LP_SESSION_PAUSED;
    s_lp.wake_cause = lp_cause;
    s_lp.flags |= LP_FLAG_FULL_WAKE;
    if ((lp_cause & LP_WAKE_FAULT) != 0u)
    {
        s_lp.flags |= LP_FLAG_FAULT;
    }
    LpCycle_Save();
    Watchdog_Feed();
    __DSB();
    NVIC_SystemReset();
    for (;;)
    {
    }
}

static void LpCycle_StopUnusedPeripherals(void)
{
    /* 轻量路径最终一定会 reset（IWDG 或软件复位），因此这里可以安全地
     * DeInit 生成代码初始化过但 LP 不用的外设；下一次 boot 的 MX_*_Init
     * 会重新建立它们。 */
    (void)HAL_UART_DeInit(&huart1);
    (void)HAL_UART_DeInit(&huart2);
    (void)HAL_CAN_DeInit(&hcan1);
    (void)HAL_DAC_DeInit(&hdac1);
    (void)HAL_TIM_PWM_DeInit(&htim1);
    (void)HAL_TIM_Base_DeInit(&htim2);

    /* LPTIM1 已不参与唤醒；只清 ENABLE，不做 HAL DeInit 的复杂禁用流程。 */
    CLEAR_BIT(LPTIM1->CR, LPTIM_CR_ENABLE);
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
    HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);
    EXTI->IMR2 &= (uint32_t)~EXTI_IMR2_IM32;
    EXTI->EMR2 &= (uint32_t)~EXTI_EMR2_EM32;
}

static uint8_t LpCycle_CheckCharger(void)
{
    GPIO_PinState level;

    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_RESET);
    HAL_Delay(LP_CHARGER_SETTLE_MS);

    if (HAL_COMP_Start(&hcomp2) != HAL_OK)
    {
        (void)HAL_COMP_Stop(&hcomp2);
        HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);
        return 0u;
    }

    HAL_Delay(LP_CHARGER_SETTLE_MS);
    level = HAL_COMP_GetOutputLevel(&hcomp2);
    (void)HAL_COMP_Stop(&hcomp2);
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);

    return (level == COMP_OUTPUT_LEVEL_HIGH) ? 1u : 0u;
}

static uint8_t LpCycle_IsButtonAsserted(void)
{
    if (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
        GPIO_PIN_RESET)
    {
        return 1u;
    }
    return ((Button_IsPressed() != 0u) ||
            ((EXTI->PR1 & EXTI_PR1_PIF5) != 0u) ||
            (Button_TakeExtiFlag() != 0u)) ? 1u : 0u;
}

static uint8_t LpCycle_IsAlertAsserted(void)
{
    return (HAL_GPIO_ReadPin(BQ_ALT_EXTI8_GPIO_Port, BQ_ALT_EXTI8_Pin) ==
            GPIO_PIN_SET) ? 1u : 0u;
}

/* PA8 高只作为触发；LP 唤醒只看 SCD/OCD/XREADY（OV/UV 在 LP 中
 * 无法区分真实 cell1/2 与不可信 VC5，真实 OV/UV 由 30 min monitor 处理）。
 * CC_READY/XREADY/OVRD_ALERT 在这里清掉，并返回 0（不是真实保护唤醒）。
 * 返回 1=真实保护位，0=仅非保护位或已清，-1=I2C 读取失败。 */
static int LpCycle_AlertConfirm(void)
{
    uint8_t stat = 0u;
    uint8_t nonprot = (uint8_t)(BQ_SYS_STAT_CC_READY |
                                BQ_SYS_STAT_XREADY |
                                BQ_SYS_STAT_OVRD_ALERT);

    if (LpCycle_IsAlertAsserted() == 0u)
    {
        return 0;
    }
    if (BQ_LpSniffInit() != 0)
    {
        BQ_LpSniffDeInit();
        return -1;
    }
    if (BQ_LpSniffRead(BQ_REG_SYS_STAT, &stat) != 0)
    {
        BQ_LpSniffDeInit();
        return -1;
    }
    if ((stat & nonprot) != 0u)
    {
        (void)BQ_LpSniffClearSysStat(nonprot);
        HAL_Delay(2u);
        if (BQ_LpSniffRead(BQ_REG_SYS_STAT, &stat) != 0)
        {
            BQ_LpSniffDeInit();
            return -1;
        }
    }
    BQ_LpSniffDeInit();

    return ((stat & BQ_SYS_STAT_ALERT_WAKE_MASK) != 0u) ? 1 : 0;
}

static uint8_t LpCycle_BqSniffOnce(uint8_t *prot_mask)
{
    bq_lp_status_t status;

    if (prot_mask != NULL)
    {
        *prot_mask = 0u;
    }
    if (BQ_LpSniffInit() != 0)
    {
        BQ_LpSniffDeInit();
        return 0u;
    }
    if (BQ_LpBootPrepare(&status) != 0)
    {
        BQ_LpSniffDeInit();
        return 0u;
    }
    BQ_LpSniffDeInit();

    /* BQ_LpBootPrepare 返回的是清完 CC_READY/XREADY/OVRD_ALERT 之后的状态；
     * 只有 SCD/OCD/XREADY 这些不受 VC5 假锁存影响的位才作为 LP 唤醒原因。 */
    if (prot_mask != NULL)
    {
        *prot_mask = (uint8_t)(status.sys_stat &
                               BQ_SYS_STAT_ALERT_WAKE_MASK);
    }

    if ((status.load_present != 0u) && (status.chg_on == 0u))
    {
        if (LpCycle_CheckCharger() != 0u)
        {
            return 2u;      /* charger */
        }
    }
    return 1u;              /* BQ OK, no charger */
}

static void LpCycle_Calibrate(const rtc_epoch_t *now)
{
    uint32_t expected;
    int32_t elapsed;
    int32_t span;
    uint32_t div;
    uint32_t sample;
    uint32_t hz;
    uint32_t ema;

    if ((s_lp.last_pr > 6u) || (s_lp.last_rlr == 0u))
    {
        return;
    }
    expected = LpCycle_ExpectedMs();
    if ((expected == 0u) || (expected < 5000u))
    {
        /* 30 min 点最后一段可能只有 1–2 s，秒级 RTC 读数误差相对过大，
         * 这一段不参与 LSI 校准。 */
        return;
    }

    elapsed = RtcTime_DiffMs(now, &(rtc_epoch_t){
        .sec = s_lp.last_arm_epoch,
        .ms = s_lp.last_arm_ms
    });
    if ((elapsed < (int32_t)((expected * 3u) / 4u)) ||
        (elapsed > (int32_t)((expected * 5u) / 4u)) ||
        (elapsed <= 0))
    {
        if (s_lp.anomaly_count < 0xFFu)
        {
            s_lp.anomaly_count++;
        }
        return;
    }

    /* elapsed 从进 STOP 前到最后一次 boot 读 RTC，包含复位/boot 与本次
     * sniff 的时间；把上次测得的 boot overhead 扣掉，才是 IWDG 实际计时
     * 段，否则 LSI 估计会系统性地偏低（周期偏长）。 */
    span = elapsed - (int32_t)s_lp.boot_overhead_ms;
    if (span <= 0)
    {
        if (s_lp.anomaly_count < 0xFFu)
        {
            s_lp.anomaly_count++;
        }
        return;
    }

    div = 4u << s_lp.last_pr;
    sample = ((uint32_t)(s_lp.last_rlr + 1u) * div * 1000u) /
             (uint32_t)span;
    if ((sample < 26000u) || (sample > 34000u))
    {
        if (s_lp.anomaly_count < 0xFFu)
        {
            s_lp.anomaly_count++;
        }
        return;
    }

    hz = LpCycle_LsiHz();
    ema = ((hz * 7u) + sample) / 8u;
    s_lp.lsi_hz_q16 = ema << 16;
    s_lp.anomaly_count = 0u;
}

static void LpCycle_UpdateBootOverhead(const rtc_epoch_t *now)
{
    int32_t d = RtcTime_DiffMs(now, &s_boot_rtc);
    uint32_t sample;

    if ((d < 0) || (d > (int32_t)LP_BOOT_OVERHEAD_MAX_MS))
    {
        return;
    }
    sample = (uint32_t)d;
    if (s_lp.boot_overhead_ms == 0u)
    {
        s_lp.boot_overhead_ms = (uint16_t)sample;
    }
    else
    {
        s_lp.boot_overhead_ms = (uint16_t)
            ((((uint32_t)s_lp.boot_overhead_ms * 7u) + sample) / 8u);
    }
}

static void LpCycle_DoMark(rtc_epoch_t *now)
{
    bq_snapshot_t snap;
    state_store_view_t state;
    uint32_t mark_epoch = s_lp.next_mark_epoch;
    uint32_t uptime_s;
    int monitor;
    int commit;

    /* 写盘之前保存原始 RTC 标记：监测/Flash/复位失败只能把该标记交给
     * 完整 NORM 重试，不能把失败窗口误当下一个 30 min 点。 */
    s_lp.flags |= LP_FLAG_MARK_PENDING;
    s_lp.flags &= (uint8_t)~LP_FLAG_MARK_LP_ARCHIVED;
    LpCycle_Save();
    Watchdog_Feed();

    if (BQ_LpSniffInit() != 0)
    {
        BQ_LpSniffDeInit();
        LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
    }
    monitor = BQ_LpMonitorRun(&snap);
    BQ_LpSniffDeInit();
    if (monitor != 0)
    {
        LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
    }

    Watchdog_Feed();
    StateStore_Init();
    StateStore_GetView(&state);
    if ((state.valid != 0u) &&
        (state.archive.last_mark_id >= mark_epoch))
    {
        /* 复位发生在上次 StateStore 落盘与 BKP 标记确认之间；旧标记
         * 已经归档过，绝不再写第二条 LP 黑匣子周期记录。 */
        if (LpCycle_FinishMark(mark_epoch, now->sec) == 0u)
        {
            LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
        }
    }
    else
    {
        BlackBox_Init();
        uptime_s = ((s_lp.flags & LP_FLAG_UPTIME_VALID) != 0u) ?
                   s_lp.uptime_base_s : BlackBox_GetUptimeBase();
        if (uptime_s < BlackBox_GetUptimeBase())
        {
            uptime_s = BlackBox_GetUptimeBase();
        }
        Watchdog_Feed();
        {
            bq_snapshot_t log = snap;
            uint16_t bb_flags = 0u;
            uint8_t estimate_flags = 0u;
            uint8_t mark_quality;

            /* 黑匣子记录真实 VC5/BAT 可信值；不可信则写 0 并置 missing。 */
            log.cell_mv[2] = (snap.log_cell3_valid != 0u) ?
                             snap.log_cell3_mv : 0u;
            log.pack_mv = (snap.log_pack_valid != 0u) ?
                          snap.log_pack_mv : 0u;
            if (snap.log_cell3_valid == 0u)
            {
                bb_flags |= BB_PERIODIC_Q_PARTIAL |
                            BB_PERIODIC_Q_CELL3_MISSING;
                estimate_flags |= STATE_ARCHIVE_Q_CELL3_MISSING;
            }
            if (snap.log_pack_valid == 0u)
            {
                bb_flags |= BB_PERIODIC_Q_PARTIAL |
                            BB_PERIODIC_Q_PACK_MISSING;
                estimate_flags |= STATE_ARCHIVE_Q_PACK_MISSING;
            }
            mark_quality = (estimate_flags == 0u) ?
                           STATE_ARCHIVE_MARK_FULL :
                           STATE_ARCHIVE_MARK_DEGRADED;
            if (BlackBox_WritePeriodicQualified(&log, uptime_s,
                                                bb_flags) != 0)
            {
                LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
            }
            Watchdog_Feed();

            /* 两处 Flash 不是原子事务：先记下黑匣子已经成功，若 StateStore
             * 失败，完整 NORM 仅补交 StateStore，不得重复写同一 LP 快照。 */
            if (LpCycle_NotePendingMarkArchived(mark_epoch) == 0u)
            {
                LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
            }
            commit = StateStore_CommitArchiveMark(mark_epoch, mark_epoch,
                                                  mark_quality, estimate_flags);
        }
        Watchdog_Feed();
        if (commit < 0)
        {
            LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
        }
        if (LpCycle_FinishMark(mark_epoch, now->sec) == 0u)
        {
            LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
        }
    }

    if (RtcTime_Read(now) == 0u)
    {
        LpCycle_ForceFullWake(LP_WAKE_FAULT);
    }
}

static void LpCycle_EnterStop(uint32_t target_ms)
{
    rtc_epoch_t now;
    uint32_t hz = LpCycle_LsiHz();

    if (Watchdog_SetTimeoutMs(target_ms, hz, NULL) == 0u)
    {
        LpCycle_ForceFullWake(LP_WAKE_FAULT);
        return;
    }
    if (RtcTime_Read(&now) == 0u)
    {
        LpCycle_ForceFullWake(LP_WAKE_FAULT);
        return;
    }

    /* 最后一道窗口检查：BQ sniff/30 min 监测/充电检查期间按键或 PA8
     * ALERT 可能已经发生，不能带着已发生的唤醒请求进 STOP。 */
    if (LpCycle_IsButtonAsserted() != 0u)
    {
#if (LP_SMBA_WAKE_TEST != 0u)
        if (s_smba_test_active != 0u)
        {
            s_lp.smba_result = 3u;
            s_lp.pb5_mode = 0u;
            LpCycle_Save();
            Button_SmbaDisable();
            s_smba_test_active = 0u;
        }
#endif
        LpCycle_ForceFullWake(LP_WAKE_BUTTON);
    }
    {
        int alert = LpCycle_AlertConfirm();

        if (alert > 0)
        {
            LpCycle_ForceFullWake(LP_WAKE_ALERT);
        }
        if (alert < 0)
        {
            LpCycle_ForceFullWake(LP_WAKE_FAULT);
        }
    }

    s_lp.last_pr = (uint8_t)(IWDG->PR & 0x07u);
    s_lp.last_rlr = (uint16_t)(IWDG->RLR & 0x0FFFu);
    s_lp.last_arm_epoch = now.sec;
    s_lp.last_arm_ms = now.ms;
    s_lp.state = LP_SESSION_STOPPED;
    LpCycle_Save();

    /* STOP1 中 IWDG 必须继续计数；PWR/RTCAPB/GPIO/SYSCFG 的 Stop 时钟
     * 由 SMEN 保持。 */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    RCC->APB1ENR1 |= (RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN);
    RCC->APB1SMENR1 |= (RCC_APB1SMENR1_PWRSMEN |
                        RCC_APB1SMENR1_RTCAPBSMEN);
#if (LP_SMBA_WAKE_TEST != 0u)
    if (s_smba_test_active != 0u)
    {
        /* 给 SMBA 测试一个机会：I2C1 APB 在 STOP 中保持。 */
        RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
        RCC->APB1SMENR1 |= RCC_APB1SMENR1_I2C1SMEN;
    }
#endif
    /* 唤醒后要读 PB5（GPIOB）和 PA8（GPIOA）：两者的 Stop 时钟都必须
     * 保持，否则第一次 GPIO 访问可能卡在门控的 APB 上，只能等 IWDG。 */
    __HAL_RCC_GPIOA_CLK_SLEEP_ENABLE();
    __HAL_RCC_GPIOB_CLK_SLEEP_ENABLE();
    __HAL_RCC_SYSCFG_CLK_SLEEP_ENABLE();

    __enable_irq();
    Watchdog_Feed();
    HAL_SuspendTick();
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    /* RCC_CFGR.STOPWUCK 在系统复位后为 0（MSI）。退出 STOP1 时必须让硬件
     * 选 HSI16，下面的 SystemCoreClock=16 MHz / HAL_InitTick 才成立；
     * 若保持 0，醒来的是 MSI（默认 4 MHz），HAL tick 会慢 4 倍。 */
    SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);
    MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP1);
    SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    __DSB();
    __WFI();

    /* 非 IWDG 复位路径：PB5/ALERT 等 EXTI 先把 CPU 唤醒。恢复 tick 后
     * 交给完整 boot；IWDG 继续运行，必须先喂。 */
    CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    SystemCoreClock = 16000000u;
    (void)HAL_InitTick(TICK_INT_PRIORITY);
    Watchdog_Feed();

#if (LP_SMBA_WAKE_TEST != 0u)
    if (s_smba_test_active != 0u)
    {
        if (Button_SmbaTakeAlert() != 0u)
        {
            s_lp.smba_result = 1u;
            s_lp.pb5_mode = 0u;
            LpCycle_Save();
            Button_SmbaDisable();
            s_smba_test_active = 0u;
            LpCycle_ForceFullWake(LP_WAKE_BUTTON | LP_WAKE_SMBA);
        }
        if (LpCycle_IsButtonAsserted() != 0u)
        {
            s_lp.smba_result = 3u;   /* EXTI5 在 AF 模式下仍能唤醒 */
            s_lp.pb5_mode = 0u;
            LpCycle_Save();
            Button_SmbaDisable();
            s_smba_test_active = 0u;
            LpCycle_ForceFullWake(LP_WAKE_BUTTON);
        }
        s_lp.smba_result = 2u;       /* 未知唤醒：SMBA 未观察到 ALERT */
        s_lp.pb5_mode = 0u;
        LpCycle_Save();
        Button_SmbaDisable();
        s_smba_test_active = 0u;
        LpCycle_ForceFullWake(LP_WAKE_FAULT);
    }
#endif

    if (LpCycle_IsButtonAsserted() != 0u)
    {
        LpCycle_ForceFullWake(LP_WAKE_BUTTON);
    }
    {
        int alert = LpCycle_AlertConfirm();

        if (alert > 0)
        {
            LpCycle_ForceFullWake(LP_WAKE_ALERT);
        }
        /* alert<0：ALERT 读不到，按 fault 完整 boot。 */
    }
    LpCycle_ForceFullWake(LP_WAKE_FAULT);
}

lp_boot_t LpCycle_ClassifyBoot(void)
{
    uint32_t cause = ResetCause_GetLatched();
    uint8_t has_wdg = ((cause & RESET_CAUSE_WATCHDOG) != 0u) ? 1u : 0u;
    uint8_t has_sw = ((cause & RESET_CAUSE_SOFTWARE) != 0u) ? 1u : 0u;
    rtc_epoch_t now;
    uint8_t rtc_ok;

    s_lp_loaded = LpSession_Load(&s_lp);
    s_boot_wake_cause = 0u;
    rtc_ok = RtcTime_Read(&now);
    if (rtc_ok != 0u)
    {
        s_boot_rtc = now;
    }

    if ((s_lp_loaded != 0u) &&
        ((s_lp.flags & LP_FLAG_MARK_PENDING) != 0u))
    {
        /* 待归档标记优先保留；但是 HardFault/NMI/RTC 故障仍为安全故障，
         * 不能伪装成纯电压恢复。完整 NORM 只能在故障解除后处理标记。 */
        if (((cause & (RESET_CAUSE_HARDFAULT | RESET_CAUSE_NMI)) != 0u) ||
            (PowerMode_LpTimebaseReady() == 0u) || (rtc_ok == 0u))
        {
            LpCycle_PreservePendingFaultWake();
        }
        else if ((has_sw != 0u) && (s_lp.state == LP_SESSION_PAUSED) &&
                 ((s_lp.flags & LP_FLAG_FULL_WAKE) != 0u))
        {
            s_boot_wake_cause = s_lp.wake_cause;
        }
        else
        {
            LpCycle_PreservePendingFaultWake();
        }
        return LP_BOOT_WAKE;
    }

    /* HardFault/NMI 的 handler 也是软件复位；必须先于 START/WAKE token
     * 判定，否则异常复位可能被误当成 LP 启动。PIN/BOR 是冷启动，清会话。 */
    if ((cause & (RESET_CAUSE_HARDFAULT | RESET_CAUSE_NMI)) != 0u)
    {
        LpSession_Invalidate();
        return LP_BOOT_FAULT;
    }

    /* LSE mode with LSI fallback (or missing RTC) must not resume a stale LP
     * session.  LSI mode passes this gate because LP_RTC_USE_LSE=0 expects LSI. */
    if (PowerMode_LpTimebaseReady() == 0u)
    {
        LpSession_Invalidate();
        return (has_wdg != 0u) ? LP_BOOT_FAULT : LP_BOOT_NORMAL;
    }

    if (s_lp_loaded == 0u)
    {
        if (has_wdg != 0u)
        {
            LpSession_Invalidate();
            return LP_BOOT_FAULT;
        }
        return LP_BOOT_NORMAL;
    }

    /* An intentional SW reset to LP must win over an extra PIN/BOR flag in
     * the latched cause.  On this board a software reset can also report
     * PIN/BOR; checking it first discarded the valid START/WAKE session and
     * produced a NORM -> reset -> NORM loop every 5 s.  A genuine PIN/BOR
     * reset has no RESET_CAUSE_SOFTWARE and still falls through to the
     * invalidation below. */
    if ((has_sw != 0u) && (s_lp.state == LP_SESSION_START))
    {
        if (rtc_ok == 0u)
        {
            LpSession_Invalidate();
            return LP_BOOT_FAULT;
        }
        return LP_BOOT_START;
    }

    if ((has_sw != 0u) && (s_lp.state == LP_SESSION_PAUSED) &&
        ((s_lp.flags & LP_FLAG_FULL_WAKE) != 0u))
    {
        s_boot_wake_cause = s_lp.wake_cause;
        return LP_BOOT_WAKE;
    }

    if ((has_wdg != 0u) && (s_lp.state == LP_SESSION_STOPPED) &&
        (rtc_ok != 0u))
    {
        uint32_t expected = LpCycle_ExpectedMs();
        int32_t elapsed = RtcTime_DiffMs(&now, &(rtc_epoch_t){
            .sec = s_lp.last_arm_epoch,
            .ms = s_lp.last_arm_ms
        });
        if ((expected != 0u) && (expected < 5000u))
        {
            /* 最后一段短 interval 受 RTC 秒边界影响大，放宽比例判定；但
             * elapsed 仍要有上限，避免“短目标却睡了很久”被当成合法 tick。 */
            if ((elapsed >= 0) && (elapsed <= 10000))
            {
                return LP_BOOT_RESUME;
            }
        }
        if ((expected != 0u) &&
            (elapsed >= (int32_t)((expected * 3u) / 4u)) &&
            (elapsed <= (int32_t)((expected * 5u) / 4u)))
        {
            return LP_BOOT_RESUME;
        }
    }

    /* A real PIN/BOR reset has no valid WATCHDOG+STOPPED tick and no SW
     * token, so it still clears the session here.  This board can report
     * PIN together with the IWDG reset of an LP tick (NRST/reset-supervisor
     * interaction while 5 V is off); treating that as a cold PIN reset
     * invalidated a valid STOPPED session and exited LP after the first
     * IWDG period. */
    if ((cause & (RESET_CAUSE_PIN | RESET_CAUSE_BOR)) != 0u)
    {
        LpSession_Invalidate();
        return LP_BOOT_NORMAL;
    }

    if (has_wdg != 0u)
    {
        LpSession_Invalidate();
        return LP_BOOT_FAULT;
    }

    return LP_BOOT_NORMAL;
}

void LpCycle_Run(lp_boot_t boot)
{
    rtc_epoch_t now;
    uint32_t remaining_ms;
    uint32_t target_ms;
    uint32_t overhead;
    uint8_t sniff;
    uint8_t sniff_prot = 0u;

    if ((s_lp_loaded == 0u) || (PowerMode_LpTimebaseReady() == 0u) ||
        (RtcTime_Read(&now) == 0u))
    {
        LpCycle_ForceFullWake(LP_WAKE_FAULT);
    }
    if ((s_lp.flags & LP_FLAG_MARK_PENDING) != 0u)
    {
        /* PREP 阶段出现待处理标记不能再次进入轻量监测、重复写 Flash。 */
        LpCycle_ForceFullWake(LP_WAKE_MONITOR | LP_WAKE_FAULT);
    }
    if (boot == LP_BOOT_RESUME)
    {
        LpCycle_Calibrate(&now);
        if (s_lp.anomaly_count >= 2u)
        {
            /* 连续两次 RTC/IWDG 间隔异常：不要继续盲睡，完整 boot 让
             * 时钟/LSE/BQ 重新初始化。 */
            LpCycle_ForceFullWake(LP_WAKE_FAULT);
        }
    }
    else
    {
        s_lp.anomaly_count = 0u;
    }

    s_lp.state = LP_SESSION_PREP;
    s_lp.flags &= (uint8_t)~(LP_FLAG_FULL_WAKE | LP_FLAG_FAULT);
    LpCycle_Save();

    /* 轻量 boot 不跑 PowerMode_Init，必须在这里恢复 STOP1 的调试时钟选择。 */
    PowerMode_LpDbgStopApply();

    /* 轻量 boot 明确关闭 5 V/灯，PA1 高（除非充电检查窗口）。 */
    HAL_GPIO_WritePin(SUP_5V_EN_GPIO_Port, SUP_5V_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_ROHT_GPIO_Port, IND_ROHT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HSE_DIS_GPIO_Port, HSE_DIS_Pin, GPIO_PIN_SET);
    (void)HAL_COMP_Stop(&hcomp2);
    LpCycle_StopUnusedPeripherals();
    Button_WakeInit();

    LpCycle_DisableLegacyWut();
    Watchdog_Feed();

    if (LpCycle_IsButtonAsserted() != 0u)
    {
        LpCycle_ForceFullWake(LP_WAKE_BUTTON);
    }

#if (LP_SMBA_WAKE_TEST != 0u)
    s_smba_test_active = 0u;
    if (s_lp.smba_result == 0u)
    {
        if (s_lp.pb5_mode == 1u)
        {
            /* 上一轮 SMBA 测试 STOP 最终由 IWDG 复位：没有 SMBA 唤醒。 */
            s_lp.smba_result = 2u;
            s_lp.pb5_mode = 0u;
            LpCycle_Save();
        }
        else
        {
            s_lp.pb5_mode = 1u;
            s_lp.smba_result = 0u;
            LpCycle_Save();
            s_smba_test_active = 1u;
        }
    }
#endif

    sniff = LpCycle_BqSniffOnce(&sniff_prot);
    if (sniff == 0u)
    {
        sniff = LpCycle_BqSniffOnce(&sniff_prot);
        if (sniff == 0u)
        {
            LpCycle_ForceFullWake(LP_WAKE_FAULT);
        }
    }

    /* BQ prepare 已关 ADC/CC 并清 CC_READY/XREADY/OVRD_ALERT。只有
     * OV/UV/SCD/OCD/XREADY 这些真实保护位才唤醒；PA8 仅因非保护位为高
     * 不再触发完整 boot，避免 OVRD_ALERT 毛刺造成 5 s LP 往返。 */
    if (sniff_prot != 0u)
    {
        LpCycle_ForceFullWake(LP_WAKE_ALERT);
    }
    if (sniff == 2u)
    {
        LpCycle_ForceFullWake(LP_WAKE_CHARGER);
    }

    /* LP 心跳：每 30 s light boot 闪 PB4 10 ms，便于区分“在跑 IWDG 周期”
     * 与“复位后卡住”。生产测电流可把 LP_HEARTBEAT_MS 改 0。 */
    if (LP_HEARTBEAT_MS != 0u)
    {
        HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_SET);
        HAL_Delay(LP_HEARTBEAT_MS);
        HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_RESET);
    }

    /* boot overhead 只统计 boot→sniff/心跳；必须在 30 min 监测之前测量，
     * 否则监测的两帧 ADC + Flash 会被算进 overhead，下一个 interval 被
     * 无谓缩短。 */
    (void)RtcTime_Read(&now);
    LpCycle_UpdateBootOverhead(&now);

    if (s_lp.next_mark_epoch == 0u)
    {
        s_lp.next_mark_epoch = now.sec + 1800u;
    }

    remaining_ms = 0u;
    if (s_lp.next_mark_epoch > now.sec)
    {
        remaining_ms = (s_lp.next_mark_epoch - now.sec) * 1000u;
        if (remaining_ms > now.ms)
        {
            remaining_ms -= now.ms;
        }
        else
        {
            remaining_ms = 0u;
        }
    }

    if ((remaining_ms <= LP_MARK_TOLERANCE_MS) ||
        (s_lp.next_mark_epoch <= now.sec))
    {
        LpCycle_DoMark(&now);
    }

    (void)RtcTime_Read(&now);
    overhead = s_lp.boot_overhead_ms;

    remaining_ms = 0u;
    if (s_lp.next_mark_epoch > now.sec)
    {
        remaining_ms = (s_lp.next_mark_epoch - now.sec) * 1000u;
        if (remaining_ms > now.ms)
        {
            remaining_ms -= now.ms;
        }
        else
        {
            remaining_ms = 0u;
        }
    }

    if ((remaining_ms != 0u) &&
        (remaining_ms <= (LP_BASE_CYCLE_MS + overhead)))
    {
        target_ms = (remaining_ms > overhead) ?
                    (remaining_ms - overhead) : LP_MIN_SLEEP_MS;
    }
    else
    {
        target_ms = LP_BASE_CYCLE_MS;
        if (target_ms > overhead)
        {
            target_ms -= overhead;
        }
    }
    if (target_ms < LP_MIN_SLEEP_MS)
    {
        target_ms = LP_MIN_SLEEP_MS;
    }

#if (LP_SMBA_WAKE_TEST != 0u)
    if (s_smba_test_active != 0u)
    {
        Button_SmbaEnable();     /* BQ sniff 结束后再占用 PB5/I2C1 */
    }
#endif

    LpCycle_EnterStop(target_ms);
}

uint8_t LpCycle_RequestStart(void)
{
    rtc_epoch_t now;
    uint8_t wait;

    if (PowerMode_LpTimebaseReady() == 0u)
    {
        return 0u;      /* LSE 模式回退 LSI / RTC 未就绪：保持 NORM */
    }

    /* 软件复位会中止正在进行的 BQ I2C 事务；等驱动/I2C 空闲一小段时间，
     * 避免把总线留在半传输状态。 */
    for (wait = 0u; wait < 100u; wait++)
    {
        if ((BQ_IsIdle() != 0u) &&
            ((hi2c1.Instance->ISR & I2C_ISR_BUSY) == 0u))
        {
            break;
        }
        HAL_Delay(1u);
        Watchdog_Feed();
    }
    if (wait >= 100u)
    {
        return 0u;
    }

    /* 自动 LP 仍按用户要求正常运行。若当前 BQ OV/UV 因 VC5/BAT 异常
     * 处于退避，尽力恢复 normal trip 后再睡眠；失败只记录，不阻止 LP，
     * 全量唤醒的 BQ CONFIG 会恢复 normal 阈值。 */
    (void)BQ_RestoreVoltageProtectionForLp();

    if (RtcTime_Read(&now) == 0u)
    {
        return 0u;
    }

    s_lp_loaded = LpSession_Load(&s_lp);
    if ((s_lp_loaded != 0u) &&
        ((s_lp.flags & LP_FLAG_MARK_PENDING) != 0u))
    {
        /* 未完成 RTC 标记必须先由完整 NORM 归档；禁止新的 START
         * 令牌覆盖待处理 mark_epoch（正常 TIM2 窗口亦不可代替该 ID）。 */
        return 0u;
    }
    if ((s_lp_loaded == 0u) ||
        (s_lp.state == LP_SESSION_IDLE) ||
        (s_lp.state == LP_SESSION_FAULT))
    {
        memset(&s_lp, 0, sizeof(s_lp));
        s_lp.state = LP_SESSION_START;
        s_lp.next_mark_epoch = now.sec + 1800u;
        s_lp.lsi_hz_q16 = LP_LSI_NOMINAL_HZ << 16;
        s_lp.last_pr = 6u;
        s_lp.last_rlr = 4095u;
        s_lp.boot_overhead_ms = 300u;
    }
    else
    {
        s_lp.state = LP_SESSION_START;
        s_lp.flags &= (uint8_t)~(LP_FLAG_FULL_WAKE | LP_FLAG_FAULT);
        s_lp.wake_cause = LP_WAKE_NONE;
        s_lp.anomaly_count = 0u;
        while (s_lp.next_mark_epoch <= now.sec)
        {
            s_lp.next_mark_epoch += 1800u;
        }
        if (s_lp.next_mark_epoch == 0u)
        {
            s_lp.next_mark_epoch = now.sec + 1800u;
        }
    }

    /* 黑匣子 24 B 的 uptime_s 是累计运行秒；进 STOP 后不再前进。
     * 利用 LP03 中 CRC 覆盖的空余 BKP 位保存当前 NORM 的精确累计值，
     * 不把 RTC epoch 或较老的黑匣子基准误写为 uptime。 */
    s_lp.uptime_base_s = App_GetUptimeSeconds();
    s_lp.flags |= LP_FLAG_UPTIME_VALID;
    LpCycle_DisableLegacyWut();
    LpCycle_Save();
    Watchdog_Feed();
    __DSB();
    NVIC_SystemReset();
    for (;;)
    {
    }
}

uint8_t LpCycle_GetPendingMark(uint32_t *mark_epoch)
{
    lp_session_t s;

    if (mark_epoch != NULL)
    {
        *mark_epoch = 0u;
    }
    if (LpSession_Load(&s) == 0u ||
        (s.flags & LP_FLAG_MARK_PENDING) == 0u ||
        s.next_mark_epoch == 0u)
    {
        return 0u;
    }
    if (mark_epoch != NULL)
    {
        *mark_epoch = s.next_mark_epoch;
    }
    return 1u;
}

uint8_t LpCycle_IsPendingMarkLpArchived(void)
{
    lp_session_t s;

    if (LpSession_Load(&s) == 0u)
    {
        return 0u;
    }
    return ((s.flags & (LP_FLAG_MARK_PENDING | LP_FLAG_MARK_LP_ARCHIVED)) ==
            (LP_FLAG_MARK_PENDING | LP_FLAG_MARK_LP_ARCHIVED)) ? 1u : 0u;
}

uint8_t LpCycle_NotePendingMarkArchived(uint32_t mark_epoch)
{
    lp_session_t s;

    if ((mark_epoch == 0u) || (LpSession_Load(&s) == 0u) ||
        ((s.flags & LP_FLAG_MARK_PENDING) == 0u) ||
        (s.next_mark_epoch != mark_epoch))
    {
        return 0u;
    }
    s_lp = s;
    s_lp_loaded = 1u;
    s_lp.flags |= LP_FLAG_MARK_LP_ARCHIVED;
    LpCycle_Save();
    return (LpSession_Load(&s) != 0u &&
            (s.flags & (LP_FLAG_MARK_PENDING | LP_FLAG_MARK_LP_ARCHIVED)) ==
            (LP_FLAG_MARK_PENDING | LP_FLAG_MARK_LP_ARCHIVED) &&
            s.next_mark_epoch == mark_epoch) ? 1u : 0u;
}

uint8_t LpCycle_AckPendingMark(uint32_t mark_epoch)
{
    lp_session_t s;
    state_store_view_t state;
    uint32_t now_epoch;

    if ((mark_epoch == 0u) || (LpSession_Load(&s) == 0u) ||
        ((s.flags & LP_FLAG_MARK_PENDING) == 0u) ||
        (s.next_mark_epoch != mark_epoch))
    {
        return 0u;
    }
    StateStore_GetView(&state);
    if ((state.valid == 0u) ||
        (state.archive.last_mark_id != mark_epoch))
    {
        return 0u;
    }
    {
        rtc_epoch_t now;
        if (RtcTime_Read(&now) == 0u)
        {
            return 0u;
        }
        now_epoch = now.sec;
    }
    s_lp = s;
    s_lp_loaded = 1u;
    return LpCycle_FinishMark(mark_epoch, now_epoch);
}

uint32_t LpCycle_GetUptimeBase(void)
{
    lp_session_t s;

    if ((s_boot_wake_cause == 0u) || (LpSession_Load(&s) == 0u) ||
        ((s.flags & LP_FLAG_UPTIME_VALID) == 0u))
    {
        return 0u;
    }
    return s.uptime_base_s;
}

uint8_t LpCycle_GetWakeCause(void)
{
    return s_boot_wake_cause;
}

void LpCycle_ApplyWake(void)
{
    lp_session_t s;
    uint8_t cause = 0u;
    uint8_t pressed;

    if (LpSession_Load(&s) == 0u)
    {
        return;
    }
    if ((s.flags & LP_FLAG_FULL_WAKE) == 0u)
    {
        return;
    }

    if ((s.wake_cause & LP_WAKE_BUTTON) != 0u)
    {
        cause |= POWER_WAKE_BUTTON;
    }
    if ((s.wake_cause & LP_WAKE_CHARGER) != 0u)
    {
        cause |= POWER_WAKE_CHARGER;
    }
    if ((s.wake_cause & LP_WAKE_ALERT) != 0u)
    {
        cause |= POWER_WAKE_BQ_ALERT;
    }
    if ((s.wake_cause & LP_WAKE_MONITOR) != 0u)
    {
        cause |= POWER_WAKE_MONITOR;
    }
    if (cause == 0u)
    {
        cause = POWER_WAKE_RTC;
    }

    PowerMode_NoteLpFullWake(cause);
    pressed = (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
               GPIO_PIN_RESET) ? 1u : 0u;
    Button_OnWake(((cause & POWER_WAKE_BUTTON) != 0u) ? 1u : 0u,
                  pressed, HAL_GetTick());
    ChargePolicy_OnLpWake(HAL_GetTick());

    if (((s.flags & LP_FLAG_FAULT) != 0u) ||
        ((s.wake_cause & LP_WAKE_FAULT) != 0u))
    {
        PowerMode_ForceDisableAutoLp();
    }

    if ((cause & POWER_WAKE_MONITOR) == 0u)
    {
        (void)BlackBox_WriteWake(cause, App_GetUptimeSeconds());
    }

    s.flags &= (uint8_t)~LP_FLAG_FULL_WAKE;
    s.state = LP_SESSION_PAUSED;
    s.seq++;
    LpSession_Save(&s);
}

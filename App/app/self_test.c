/**
 ******************************************************************************
 * @file    self_test.c
 * @brief   上电自检状态机（不阻塞主循环）。
 *
 * 序列：IDLE → WAIT_BQ（3 s）→ [WAIT_BQ_GRACE（最长 12 s，仅 BQ 动态项失败）]
 *       → PASS/FAIL → MONITOR。
 *
 * 目的：BQ 首帧短暂异常（ADC 未建立/正在重启）不应永久锁死三声报错；
 * 只有静态项（LSE/COMP/黑匣子）立即失败，BQ 动态项有恢复宽限。
 * 完成后的 MONITOR 每 1 s 复查：持续 5 s 不健康才重新判失败(只报一次)，
 * 连续健康 3 s 自动恢复。ADC 重启/配置期间不参与健康计时。
 ******************************************************************************
 */
#include "app/self_test.h"

#include "board.h"
#include "bsp/bq_ic.h"
#include "app/black_box.h"

#define SELFTEST_BQ_TIMEOUT_MS      3000u
#define SELFTEST_BQ_GRACE_MS        12000u
#define SELFTEST_MONITOR_PERIOD_MS  1000u
#define SELFTEST_MONITOR_FAIL_MS    5000u
#define SELFTEST_MONITOR_RECOVER_MS 3000u

#define SELFTEST_BQ_MASK  (SELFTEST_FAIL_BQ_LINK | SELFTEST_FAIL_BQ_DATA)

typedef enum
{
    SELFTEST_STATE_IDLE = 0,
    SELFTEST_STATE_WAIT_BQ,
    SELFTEST_STATE_WAIT_BQ_GRACE,
    SELFTEST_STATE_MONITOR
} selftest_state_t;

static struct
{
    selftest_state_t state;
    uint32_t wait_start_ms;
    uint32_t last_monitor_ms;
    uint32_t bq_bad_start_ms;
    uint32_t bq_good_start_ms;
    uint8_t requested;
    uint8_t done;
    uint8_t passed;
    uint8_t event;
    uint16_t fail_mask;
    uint32_t run_count;
    uint32_t last_run_ms;
} s_self_test;

static uint16_t SelfTest_RunChecks(uint32_t now_ms)
{
    uint16_t fail = 0u;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0u)
    {
        fail |= SELFTEST_FAIL_LSE;
    }

    if (BQ_GetState() != BQ_ST_RUN)
    {
        fail |= SELFTEST_FAIL_BQ_LINK;
    }
    if (BQ_IsFresh(now_ms) == 0u)
    {
        fail |= SELFTEST_FAIL_BQ_DATA;
    }

    if (HAL_COMP_GetState(&hcomp2) != HAL_COMP_STATE_BUSY)
    {
        fail |= SELFTEST_FAIL_COMP;
    }

    if (BlackBox_HasError() != 0u)
    {
        fail |= SELFTEST_FAIL_BLACKBOX;
    }

    return fail;
}

static void SelfTest_Finish(uint16_t fail_mask, uint32_t now_ms,
                            uint8_t event_if_passed)
{
    s_self_test.fail_mask = fail_mask;
    s_self_test.passed = (fail_mask == 0u) ? 1u : 0u;
    s_self_test.done = 1u;
    s_self_test.requested = 0u;
    s_self_test.state = SELFTEST_STATE_MONITOR;
    s_self_test.run_count++;
    s_self_test.last_run_ms = now_ms;
    s_self_test.last_monitor_ms = now_ms;
    s_self_test.bq_bad_start_ms = 0u;
    s_self_test.bq_good_start_ms = 0u;

    if (s_self_test.event == SELFTEST_EVENT_NONE)
    {
        s_self_test.event = (s_self_test.passed != 0u) ?
                            event_if_passed : SELFTEST_EVENT_FAIL;
    }
}

void SelfTest_Init(void)
{
    s_self_test.state = SELFTEST_STATE_IDLE;
    s_self_test.wait_start_ms = 0u;
    s_self_test.last_monitor_ms = 0u;
    s_self_test.bq_bad_start_ms = 0u;
    s_self_test.bq_good_start_ms = 0u;
    s_self_test.requested = 0u;
    s_self_test.done = 0u;
    s_self_test.passed = 0u;
    s_self_test.event = SELFTEST_EVENT_NONE;
    s_self_test.fail_mask = 0u;
    s_self_test.run_count = 0u;
    s_self_test.last_run_ms = 0u;
}

void SelfTest_Request(void)
{
    s_self_test.requested = 1u;
    s_self_test.done = 0u;
    s_self_test.passed = 0u;
    s_self_test.event = SELFTEST_EVENT_NONE;
    s_self_test.fail_mask = 0u;
    s_self_test.state = SELFTEST_STATE_WAIT_BQ;
    s_self_test.wait_start_ms = HAL_GetTick();
    s_self_test.bq_bad_start_ms = 0u;
    s_self_test.bq_good_start_ms = 0u;
}

static void SelfTest_Monitor(uint32_t now_ms)
{
    uint16_t checks;
    uint16_t bq_fail;
    uint8_t healthy = 0u;

    if ((s_self_test.last_monitor_ms != 0u) &&
        ((now_ms - s_self_test.last_monitor_ms) < SELFTEST_MONITOR_PERIOD_MS))
    {
        return;
    }
    s_self_test.last_monitor_ms = now_ms;
    s_self_test.last_run_ms = now_ms;

    /* ADC 重启/配置/settle 期间既不算健康也不算病态，清掉计时重新开始。 */
    if (BQ_IsRecovering() != 0u)
    {
        s_self_test.bq_bad_start_ms = 0u;
        s_self_test.bq_good_start_ms = 0u;
        return;
    }

    checks = SelfTest_RunChecks(now_ms);
    bq_fail = (uint16_t)(checks & SELFTEST_BQ_MASK);
    /* 恢复尝试已耗尽也按病态计时，确保最终能上报一次 FAIL。 */
    healthy = ((bq_fail == 0u) && (BQ_IsFresh(now_ms) != 0u) &&
               (BQ_GetAdcRecoveryFailed() == 0u)) ? 1u : 0u;

    if (healthy != 0u)
    {
        s_self_test.bq_bad_start_ms = 0u;
        if (s_self_test.bq_good_start_ms == 0u)
        {
            s_self_test.bq_good_start_ms = now_ms;
        }

        if ((s_self_test.passed == 0u) &&
            ((now_ms - s_self_test.bq_good_start_ms) >=
             SELFTEST_MONITOR_RECOVER_MS))
        {
            s_self_test.fail_mask = checks;
            if ((checks & SELFTEST_BQ_MASK) == 0u)
            {
                s_self_test.passed = (checks == 0u) ? 1u : 0u;
                if (s_self_test.passed != 0u)
                {
                    s_self_test.event = SELFTEST_EVENT_RECOVERED;
                }
            }
        }
    }
    else
    {
        s_self_test.bq_good_start_ms = 0u;
        if (s_self_test.passed != 0u)
        {
            if (s_self_test.bq_bad_start_ms == 0u)
            {
                s_self_test.bq_bad_start_ms = now_ms;
            }
            if ((now_ms - s_self_test.bq_bad_start_ms) >=
                SELFTEST_MONITOR_FAIL_MS)
            {
                if ((checks & SELFTEST_BQ_MASK) == 0u)
                {
                    checks |= SELFTEST_FAIL_BQ_DATA;
                }
                s_self_test.fail_mask = checks;
                s_self_test.passed = 0u;
                s_self_test.event = SELFTEST_EVENT_FAIL;
                s_self_test.bq_bad_start_ms = 0u;   /* 只报一次 */
            }
        }
    }
}

void SelfTest_Poll(uint32_t now_ms)
{
    uint16_t checks;
    uint16_t bq_fail;

    if (s_self_test.requested == 0u)
    {
        if ((s_self_test.done != 0u) &&
            (s_self_test.state == SELFTEST_STATE_MONITOR))
        {
            SelfTest_Monitor(now_ms);
        }
        return;
    }

    switch (s_self_test.state)
    {
    case SELFTEST_STATE_WAIT_BQ:
        if ((BQ_IsFresh(now_ms) == 0u) &&
            ((now_ms - s_self_test.wait_start_ms) < SELFTEST_BQ_TIMEOUT_MS))
        {
            break;
        }

        checks = SelfTest_RunChecks(now_ms);
        bq_fail = (uint16_t)(checks & SELFTEST_BQ_MASK);

        if ((bq_fail == 0u) ||
            ((checks & (uint16_t)~SELFTEST_BQ_MASK) != 0u) ||
            (BQ_GetAdcRecoveryFailed() != 0u))
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
        }
        else
        {
            /* 只有 BQ 动态项失败：给 ADC 重启/首帧建立留恢复宽限，不立即三声报错。 */
            s_self_test.state = SELFTEST_STATE_WAIT_BQ_GRACE;
            s_self_test.last_monitor_ms = now_ms;
        }
        break;

    case SELFTEST_STATE_WAIT_BQ_GRACE:
        checks = SelfTest_RunChecks(now_ms);
        bq_fail = (uint16_t)(checks & SELFTEST_BQ_MASK);

        if ((BQ_GetAdcRecoveryFailed() != 0u) ||
            ((checks & (uint16_t)~SELFTEST_BQ_MASK) != 0u) ||
            ((now_ms - s_self_test.wait_start_ms) >= SELFTEST_BQ_GRACE_MS))
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
        }
        else if ((bq_fail == 0u) &&
                 (BQ_IsFresh(now_ms) != 0u) &&
                 (BQ_GetState() == BQ_ST_RUN))
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
        }
        break;

    case SELFTEST_STATE_IDLE:
    case SELFTEST_STATE_MONITOR:
    default:
        break;
    }
}

uint8_t SelfTest_TakeEvent(void)
{
    uint8_t event = s_self_test.event;

    s_self_test.event = SELFTEST_EVENT_NONE;
    return event;
}

uint8_t SelfTest_IsDone(void)
{
    return s_self_test.done;
}

uint8_t SelfTest_IsPassed(void)
{
    return s_self_test.passed;
}

uint16_t SelfTest_GetFailMask(void)
{
    return s_self_test.fail_mask;
}

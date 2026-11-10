/**
 ******************************************************************************
 * @file    self_test.c
 * @brief   Non-blocking power-on self-test state machine.
 *
 * BQ voltage quality is supplied by the main-loop anomaly coordinator. A
 * qualified PARTIAL_VC5_PACK condition may defer only BQ dynamic completion;
 * static checks and real hardware/protection faults retain normal priority.
 ******************************************************************************/
#include "app/self_test.h"

#include "board.h"
#include "bsp/bq_ic.h"
#include "app/black_box.h"
#include "app/faults.h"
#include "app/power_mode.h"

#define SELFTEST_BQ_TIMEOUT_MS          3000u
#define SELFTEST_BQ_GRACE_MS           12000u
#define SELFTEST_BQ_HARD_MS            30000u
#define SELFTEST_MONITOR_PERIOD_MS      1000u
#define SELFTEST_MONITOR_FAIL_MS        5000u
#define SELFTEST_MONITOR_RECOVER_MS     3000u
#define SELFTEST_RECOVERY_HOLD_MAX_MS  30000u

#define SELFTEST_BQ_MASK  (SELFTEST_FAIL_BQ_LINK | SELFTEST_FAIL_BQ_DATA)

/* BQ_STALE is intentionally excluded: it is the expected symptom of a
 * scoped partial sample. All other live/sticky faults remain authoritative. */
/* BQ OV/UV 由 protection supervisor 处理；只要 protection 未锁存，
 * 驱动过滤后的 VC5 假 UV 不应让自检永久失败。 */
#define SELFTEST_REAL_FAULT_MASK (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY | \
                                  FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD | \
                                  FAULT_BIT_TEMP | FAULT_BIT_LSE | \
                                  FAULT_BIT_FLASH_LOG | FAULT_BIT_PROTECTION | \
                                  FAULT_BIT_POWER | FAULT_BIT_CLK | \
                                  FAULT_BIT_FLASH_STATE | FAULT_BIT_BQ_ADC_STUCK | \
                                  FAULT_BIT_TIMEBASE)

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
    uint32_t grace_accum_ms;
    uint32_t grace_last_ms;
    uint32_t recovery_hold_start_ms;
    uint32_t completion_ms;
    uint8_t requested;
    uint8_t done;
    uint8_t passed;
    uint8_t event;
    uint8_t full_frame_qualified;
    uint16_t fail_mask;
} s_self_test;

static uint8_t SelfTest_RealFaultActive(void)
{
    return ((Fault_GetAll() & SELFTEST_REAL_FAULT_MASK) != 0u) ? 1u : 0u;
}

static uint16_t SelfTest_RunChecks(uint32_t now_ms)
{
    uint16_t fail = 0u;

    if (PowerMode_LpTimebaseReady() == 0u)
    {
        fail |= SELFTEST_FAIL_TIMEBASE;
    }

    if (BQ_GetState() != BQ_ST_RUN)
    {
        fail |= SELFTEST_FAIL_BQ_LINK;
    }
    else
    {
        bq_batch_quality_t batch;
        bq_snapshot_t snap;

        BQ_GetBatchQuality(&batch);
        BQ_GetSnapshot(&snap);
        if ((batch.quality == (uint8_t)BQ_BATCH_FULL_REAL_OK) &&
            (batch.complete != 0u) && (batch.published_seq != 0u) &&
            (batch.published_ms == batch.batch_ms) &&
            (batch.published_ms == snap.last_update_ms))
        {
            s_self_test.full_frame_qualified = 1u;
        }
    }
    if ((BQ_IsFresh(now_ms) == 0u) ||
        (s_self_test.full_frame_qualified == 0u))
    {
        fail |= SELFTEST_FAIL_BQ_DATA;
    }
    if ((BQ_GetAdcRecoveryFailed() != 0u) ||
        (SelfTest_RealFaultActive() != 0u))
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
    /* A real full frame is a prerequisite for PASS. A partial value can never
     * clear this gate, even when its current value happens to be fresh. */
    if ((s_self_test.full_frame_qualified == 0u) ||
        (BQ_GetAdcRecoveryFailed() != 0u) ||
        (SelfTest_RealFaultActive() != 0u))
    {
        fail_mask |= SELFTEST_FAIL_BQ_DATA;
    }

    s_self_test.fail_mask = fail_mask;
    s_self_test.passed = (fail_mask == 0u) ? 1u : 0u;
    s_self_test.done = 1u;
    s_self_test.completion_ms = now_ms;
    s_self_test.requested = 0u;
    s_self_test.state = SELFTEST_STATE_MONITOR;
    s_self_test.last_monitor_ms = now_ms;
    s_self_test.bq_bad_start_ms = 0u;
    s_self_test.bq_good_start_ms = 0u;
    s_self_test.recovery_hold_start_ms = 0u;

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
    s_self_test.grace_accum_ms = 0u;
    s_self_test.grace_last_ms = 0u;
    s_self_test.recovery_hold_start_ms = 0u;
    s_self_test.completion_ms = 0u;
    s_self_test.requested = 0u;
    s_self_test.done = 0u;
    s_self_test.passed = 0u;
    s_self_test.event = SELFTEST_EVENT_NONE;
    s_self_test.full_frame_qualified = 0u;
    s_self_test.fail_mask = 0u;
}

void SelfTest_Request(void)
{
    s_self_test.requested = 1u;
    s_self_test.done = 0u;
    s_self_test.passed = 0u;
    s_self_test.event = SELFTEST_EVENT_NONE;
    s_self_test.fail_mask = 0u;
    s_self_test.completion_ms = 0u;
    s_self_test.full_frame_qualified = 0u;
    s_self_test.state = SELFTEST_STATE_WAIT_BQ;
    s_self_test.wait_start_ms = HAL_GetTick();
    s_self_test.bq_bad_start_ms = 0u;
    s_self_test.bq_good_start_ms = 0u;
    s_self_test.grace_accum_ms = 0u;
    s_self_test.grace_last_ms = 0u;
    s_self_test.recovery_hold_start_ms = 0u;
}

static void SelfTest_Monitor(uint32_t now_ms)
{
    uint16_t checks;
    uint16_t bq_fail;
    uint8_t healthy;

    if ((s_self_test.last_monitor_ms != 0u) &&
        ((now_ms - s_self_test.last_monitor_ms) < SELFTEST_MONITOR_PERIOD_MS))
    {
        return;
    }
    s_self_test.last_monitor_ms = now_ms;

    checks = SelfTest_RunChecks(now_ms);

    /* ADC recovery exhaustion remains a real BQ failure. */
    if (BQ_GetAdcRecoveryFailed() != 0u)
    {
        s_self_test.recovery_hold_start_ms = 0u;
        s_self_test.bq_bad_start_ms = 0u;
        s_self_test.bq_good_start_ms = 0u;
        if (s_self_test.passed != 0u)
        {
            s_self_test.fail_mask = (uint16_t)(checks | SELFTEST_FAIL_BQ_DATA);
            s_self_test.passed = 0u;
            s_self_test.event = SELFTEST_EVENT_FAIL;
        }
        return;
    }

    /* ADC restart/config/settle does not count as healthy or unhealthy, but is
     * bounded so a real failure cannot be hidden forever. */
    if (BQ_IsRecovering() != 0u)
    {
        if (s_self_test.recovery_hold_start_ms == 0u)
        {
            s_self_test.recovery_hold_start_ms = now_ms;
        }
        if ((now_ms - s_self_test.recovery_hold_start_ms) <
            SELFTEST_RECOVERY_HOLD_MAX_MS)
        {
            s_self_test.bq_bad_start_ms = 0u;
            s_self_test.bq_good_start_ms = 0u;
            return;
        }
        s_self_test.recovery_hold_start_ms = 0u;
        s_self_test.bq_good_start_ms = 0u;
        if (s_self_test.bq_bad_start_ms == 0u)
        {
            s_self_test.bq_bad_start_ms = now_ms - SELFTEST_MONITOR_FAIL_MS;
        }
    }
    else
    {
        s_self_test.recovery_hold_start_ms = 0u;
    }

    bq_fail = (uint16_t)(checks & SELFTEST_BQ_MASK);
    healthy = ((bq_fail == 0u) && (BQ_IsFresh(now_ms) != 0u) &&
               (s_self_test.full_frame_qualified != 0u)) ? 1u : 0u;

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
            if (checks == 0u)
            {
                s_self_test.passed = 1u;
                s_self_test.event = SELFTEST_EVENT_RECOVERED;
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
                s_self_test.bq_bad_start_ms = 0u;
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

        if ((checks & (uint16_t)~SELFTEST_BQ_MASK) != 0u)
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
        }
        else if ((bq_fail == 0u) &&
                 (BQ_IsRecovering() == 0u) &&
                 (s_self_test.full_frame_qualified != 0u))
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
        }
        else
        {
            s_self_test.state = SELFTEST_STATE_WAIT_BQ_GRACE;
            s_self_test.last_monitor_ms = now_ms;
            s_self_test.grace_last_ms = now_ms;
        }
        break;

    case SELFTEST_STATE_WAIT_BQ_GRACE:
        checks = SelfTest_RunChecks(now_ms);
        bq_fail = (uint16_t)(checks & SELFTEST_BQ_MASK);

        if ((checks & (uint16_t)~SELFTEST_BQ_MASK) != 0u)
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
            break;
        }
        if (BQ_IsRecovering() == 0u)
        {
            uint32_t delta = now_ms - s_self_test.grace_last_ms;
            if (delta > (0xFFFFFFFFu - s_self_test.grace_accum_ms))
            {
                s_self_test.grace_accum_ms = 0xFFFFFFFFu;
            }
            else
            {
                s_self_test.grace_accum_ms += delta;
            }
        }
        s_self_test.grace_last_ms = now_ms;

        if ((BQ_GetAdcRecoveryFailed() != 0u) ||
            (s_self_test.grace_accum_ms >= SELFTEST_BQ_GRACE_MS) ||
            ((now_ms - s_self_test.wait_start_ms) >= SELFTEST_BQ_HARD_MS))
        {
            SelfTest_Finish(checks, now_ms, SELFTEST_EVENT_PASS);
        }
        else if ((bq_fail == 0u) &&
                 (BQ_IsFresh(now_ms) != 0u) &&
                 (BQ_GetState() == BQ_ST_RUN) &&
                 (BQ_IsRecovering() == 0u) &&
                 (s_self_test.full_frame_qualified != 0u))
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

uint32_t SelfTest_GetCompletionMs(void)
{
    return s_self_test.completion_ms;
}

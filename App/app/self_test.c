/**
 ******************************************************************************
 * @file    self_test.c
 * @brief   上电自检状态机（不阻塞主循环）。
 *
 * 序列：IDLE → WAIT_BQ（等首批有效数据，超时 1500 ms）→ DONE。
 * 初始一次记录 5 项结果；失败项只记录位图，由 main_app 决定灯/蜂鸣器动作。
 ******************************************************************************
 */
#include "app/self_test.h"

#include "board.h"
#include "bsp/bq_ic.h"
#include "app/black_box.h"

#define SELFTEST_BQ_TIMEOUT_MS  1500u

typedef enum
{
    SELFTEST_STATE_IDLE = 0,
    SELFTEST_STATE_WAIT_BQ,
    SELFTEST_STATE_DONE
} selftest_state_t;

static struct
{
    selftest_state_t state;
    uint32_t wait_start_ms;
    uint8_t requested;
    uint8_t done;
    uint8_t passed;
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

void SelfTest_Init(void)
{
    s_self_test.state = SELFTEST_STATE_IDLE;
    s_self_test.wait_start_ms = 0u;
    s_self_test.requested = 0u;
    s_self_test.done = 0u;
    s_self_test.passed = 0u;
    s_self_test.fail_mask = 0u;
    s_self_test.run_count = 0u;
    s_self_test.last_run_ms = 0u;
}

void SelfTest_Request(void)
{
    s_self_test.requested = 1u;
    s_self_test.done = 0u;
    s_self_test.passed = 0u;
    s_self_test.fail_mask = 0u;
    s_self_test.state = SELFTEST_STATE_WAIT_BQ;
    s_self_test.wait_start_ms = HAL_GetTick();
}

void SelfTest_Poll(uint32_t now_ms)
{
    if (s_self_test.requested == 0u)
    {
        return;
    }

    switch (s_self_test.state)
    {
    case SELFTEST_STATE_WAIT_BQ:
        if ((BQ_IsFresh(now_ms) != 0u) ||
            ((now_ms - s_self_test.wait_start_ms) >= SELFTEST_BQ_TIMEOUT_MS))
        {
            s_self_test.fail_mask = SelfTest_RunChecks(now_ms);
            s_self_test.passed = (s_self_test.fail_mask == 0u) ? 1u : 0u;
            s_self_test.done = 1u;
            s_self_test.requested = 0u;
            s_self_test.state = SELFTEST_STATE_DONE;
            s_self_test.run_count++;
            s_self_test.last_run_ms = now_ms;
        }
        break;

    case SELFTEST_STATE_IDLE:
    case SELFTEST_STATE_DONE:
    default:
        break;
    }
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

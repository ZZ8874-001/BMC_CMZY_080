/**
 ******************************************************************************
 * @file    state_machine.c
 * @brief   BOOT/NORM/FAULT 状态 + 充/放/静置分类。
 *
 * L0 分类依据：COMP2 消抖方向（SC_BTTN 对 SC_PACK，高=充电）+ BQ 数据新鲜度。
 * CC 符号的实际极性需上板实测后再固化交叉校验（见计划 §7 bring-up 第 4 步）。
 ******************************************************************************
 */
#include "app/state_machine.h"

#include "board.h"

/* 这些故障一旦出现即进入 FAULT 状态（BQ_STALE 不在此列，只影响灯效）。 */
/* BQ OV/UV 不直接进入通用 FAULT；真实 cell1/2 越限由 protection 锁存后
 * 置 FAULT_BIT_PROTECTION，再映射到 FAULT。这样过滤后的 VC5 假 UV 不会
 * 让状态机/红灯/蜂鸣永久误报。 */
#define SM_CRITICAL_FAULT_MASK  (FAULT_BIT_BQ_COMM | FAULT_BIT_BQ_XREADY | \
                                 FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD |     \
                                 FAULT_BIT_BQ_ADC_STUCK |                  \
                                 FAULT_BIT_PROTECTION |                    \
                                 FAULT_BIT_POWER | FAULT_BIT_CLK)

static struct
{
    sm_state_t state;
    sm_flow_t flow;
    uint32_t update_count;
    uint32_t fault_enter_count;
    uint16_t last_fault_all;
} s_sm;

void StateMachine_Init(void)
{
    s_sm.state = SM_STATE_BOOT;
    s_sm.flow = SM_FLOW_STANDBY;
    s_sm.update_count = 0u;
    s_sm.fault_enter_count = 0u;
    s_sm.last_fault_all = 0u;
}

void StateMachine_Update(uint8_t bq_fresh, uint8_t comp_charging, uint16_t fault_all)
{
    s_sm.update_count++;
    s_sm.last_fault_all = fault_all;

    if ((fault_all & SM_CRITICAL_FAULT_MASK) != 0u)
    {
        if (s_sm.state != SM_STATE_FAULT)
        {
            s_sm.fault_enter_count++;
        }
        s_sm.state = SM_STATE_FAULT;
        s_sm.flow = SM_FLOW_FAULT;
        return;
    }

    if (s_sm.state == SM_STATE_FAULT)
    {
        /* 故障消失：没有新鲜完整帧仍回 BOOT，禁止直接标 NORM。 */
        s_sm.state = (bq_fresh != 0u) ? SM_STATE_NORM : SM_STATE_BOOT;
    }
    else if (s_sm.state == SM_STATE_BOOT)
    {
        if (bq_fresh != 0u)
        {
            s_sm.state = SM_STATE_NORM;
        }
    }

    if (bq_fresh == 0u)
    {
        s_sm.flow = SM_FLOW_STANDBY;
    }
    else
    {
        s_sm.flow = (comp_charging != 0u) ? SM_FLOW_CHARGING : SM_FLOW_DISCHARGING;
    }
}

sm_state_t StateMachine_GetState(void)
{
    return s_sm.state;
}

sm_flow_t StateMachine_GetFlow(void)
{
    return s_sm.flow;
}

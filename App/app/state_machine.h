/**
 ******************************************************************************
 * @file    state_machine.h
 * @brief   系统状态与充/放/静置分类（L0：COMP2 方向 + BQ 数据有效性）。
 ******************************************************************************
 */
#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include <stdint.h>

typedef enum
{
    SM_STATE_BOOT = 0,  /* 等待首批有效 BQ 数据 */
    SM_STATE_NORM,
    SM_STATE_FAULT
} sm_state_t;

typedef enum
{
    SM_FLOW_STANDBY = 0,
    SM_FLOW_CHARGING,
    SM_FLOW_DISCHARGING,
    SM_FLOW_FAULT
} sm_flow_t;

void StateMachine_Init(void);
void StateMachine_Update(uint8_t bq_fresh, uint8_t comp_charging, uint16_t fault_all);

sm_state_t StateMachine_GetState(void);
sm_flow_t StateMachine_GetFlow(void);

#endif /* APP_STATE_MACHINE_H */

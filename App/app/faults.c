/**
 ******************************************************************************
 * @file    faults.c
 * @brief   统一故障注册表 + PVD 欠压监测（PVD_PVM 中断 owner）。
 *
 * 命名约定：运行状态放模块私有静态结构体，字段完整命名，便于调试器观察。
 ******************************************************************************
 */
#include "app/faults.h"

#include "board.h"

/* PVD 阈值 2.8 V：3.3 V 轨跌落到该值以下立即置 sticky POWER 故障。 */
#define FAULT_PVD_LEVEL     PWR_PVDLEVEL_5
#define FAULT_PVD_NVIC_PRIO 4u

static struct
{
    volatile uint16_t live;
    volatile uint16_t sticky;
    volatile uint32_t pvd_irq_count;
    uint32_t set_sticky_count;
} s_fault;

/* 中断与主循环都会做位读改写，用 PRIMASK 保护。 */
static uint32_t Fault_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Fault_ExitCritical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void Fault_Init(void)
{
    s_fault.live = 0u;
    s_fault.sticky = 0u;
    s_fault.pvd_irq_count = 0u;
    s_fault.set_sticky_count = 0u;

    Fault_PvdEnable();
}

void Fault_PvdEnable(void)
{
    PWR_PVDTypeDef pvd;

    pvd.PVDLevel = FAULT_PVD_LEVEL;
    pvd.Mode = PWR_PVD_MODE_IT_RISING;
    (void)HAL_PWR_ConfigPVD(&pvd);

    HAL_NVIC_SetPriority(PVD_PVM_IRQn, FAULT_PVD_NVIC_PRIO, 0u);
    HAL_NVIC_ClearPendingIRQ(PVD_PVM_IRQn);
    HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
    HAL_PWR_EnablePVD();
}

void Fault_PvdDisable(void)
{
    HAL_PWR_DisablePVD();
    HAL_NVIC_DisableIRQ(PVD_PVM_IRQn);
    HAL_NVIC_ClearPendingIRQ(PVD_PVM_IRQn);
}

void Fault_SetLive(uint16_t mask)
{
    uint32_t primask = Fault_EnterCritical();
    s_fault.live |= mask;
    Fault_ExitCritical(primask);
}

void Fault_ClearLive(uint16_t mask)
{
    uint32_t primask = Fault_EnterCritical();
    s_fault.live &= (uint16_t)~mask;
    Fault_ExitCritical(primask);
}

void Fault_SetSticky(uint16_t mask)
{
    uint32_t primask = Fault_EnterCritical();
    if ((s_fault.sticky & mask) != mask)
    {
        s_fault.set_sticky_count++;
    }
    s_fault.sticky |= mask;
    Fault_ExitCritical(primask);
}

void Fault_ClearSticky(uint16_t mask)
{
    uint32_t primask = Fault_EnterCritical();
    s_fault.sticky &= (uint16_t)~mask;
    Fault_ExitCritical(primask);
}

uint16_t Fault_GetLive(void)
{
    return s_fault.live;
}

uint16_t Fault_GetSticky(void)
{
    return s_fault.sticky;
}

uint16_t Fault_GetAll(void)
{
    uint32_t primask = Fault_EnterCritical();
    uint16_t all = (uint16_t)(s_fault.live | s_fault.sticky);
    Fault_ExitCritical(primask);
    return all;
}

/* PVD_PVM 中断：HAL 负责清 EXTI，回调里只置故障位。 */
void PVD_PVM_IRQHandler(void)
{
    HAL_PWREx_PVD_PVM_IRQHandler();
}

void HAL_PWR_PVDCallback(void)
{
    s_fault.pvd_irq_count++;
    Fault_SetSticky(FAULT_BIT_POWER);
}

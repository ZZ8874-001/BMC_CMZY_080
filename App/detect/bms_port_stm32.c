/**
 ******************************************************************************
 * @file    bms_port_stm32.c
 * @brief   SOC/SOH toolkit 的 STM32 HAL 毫秒时基。
 ******************************************************************************
 */
#include "bms_port.h"

#include "stm32l4xx_hal.h"

uint32_t BMS_Port_GetTickMs(void)
{
    return HAL_GetTick();
}

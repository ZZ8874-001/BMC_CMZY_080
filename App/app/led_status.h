/**
 ******************************************************************************
 * @file    led_status.h
 * @brief   状态灯：PB4（IND_NORM）、PB3（IND_ROHT）+ 单灯 RGB（USART2）。
 *
 * Tick（250 Hz 中断）推进相位并驱动 PB3/PB4；Flush（主循环）在相位变化时
 * 发送一帧 RGB。颜色：BOOT 蓝闪、NORM 充电绿/放电蓝/静置暗白、FAULT 红闪。
 ******************************************************************************
 */
#ifndef APP_LED_STATUS_H
#define APP_LED_STATUS_H

#include <stdint.h>

typedef enum
{
    LED_MODE_BOOT = 0,
    LED_MODE_NORM,
    LED_MODE_FAULT
} led_mode_t;

typedef enum
{
    LED_FLOW_STANDBY = 0,
    LED_FLOW_CHARGING,
    LED_FLOW_DISCHARGING,
    LED_FLOW_CAL,
    LED_FLOW_FAULT
} led_flow_t;

void LedStatus_Init(void);
void LedStatus_SetMode(led_mode_t mode, led_flow_t flow);

/* 250 Hz 中断上下文。 */
void LedStatus_Tick(void);

/* 主循环上下文：必要时发送 RGB 帧。 */
void LedStatus_Flush(void);

#endif /* APP_LED_STATUS_H */

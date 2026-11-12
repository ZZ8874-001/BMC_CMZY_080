/**
 ******************************************************************************
 * @file    led_status.h
 * @brief   状态灯：PB4（IND_NORM）、PB3（IND_ROHT）+ 单灯 RGB（USART2）。
 *
 * Tick（250 Hz 中断）推进相位并驱动 PB3/PB4；Flush（主循环）在相位变化或
 * SOC 颜色变化时发送一帧 RGB。NORM 用 SOC 红→黄→绿渐变：STANDBY 5 Hz/
 * 约 60%、DISCHARGE 常亮/95%、CHARGE 1 Hz/95%、充满白色 38 常亮；
 * 低电状态红色 64/5 Hz；BOOT 蓝闪、FAULT 红闪、CAL 琥珀。
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
    LED_FLOW_CHARGE_FULL,
    LED_FLOW_DISCHARGING,
    LED_FLOW_CAL,
    LED_FLOW_LOW_BATT,
    LED_FLOW_FAULT
} led_flow_t;

void LedStatus_Init(void);
void LedStatus_SetMode(led_mode_t mode, led_flow_t flow);
/* 主循环上下文：SOC 0..10000 (0.01%)，valid=0 时保持最后一次有效颜色。 */
void LedStatus_SetSoc(int32_t soc01, uint8_t valid);

/* 250 Hz 中断上下文。 */
void LedStatus_Tick(void);

/* 主循环上下文：必要时发送 RGB 帧。 */
void LedStatus_Flush(void);

#endif /* APP_LED_STATUS_H */

/**
 ******************************************************************************
 * @file    button.h
 * @brief   PB5/WAKE 按键采样、20 ms 消抖、按下/释放事件队列。
 *
 * EXTI 只负责唤醒并置标志；本模块在主循环 4 ms 采样，不使用 ISR 做定时。
 ******************************************************************************
 */
#ifndef APP_BUTTON_H
#define APP_BUTTON_H

#include <stdint.h>

typedef enum
{
    BUTTON_EVENT_DOWN = 0,
    BUTTON_EVENT_UP = 1
} button_event_type_t;

typedef struct
{
    button_event_type_t type;
    uint8_t  pressed;       /* DOWN=1, UP=0 */
    uint32_t timestamp_ms;
    uint32_t held_ms;       /* UP 时有效，DOWN 为 0 */
    uint8_t  wake_origin;   /* 1 = 由 LP 唤醒键合成/起始 */
} button_event_t;

void Button_Init(void);

/* 主循环调用；内部按 4 ms 间隔采样 PB5。 */
void Button_Poll(uint32_t now_ms);

/* EXTI 回调（power_mode.c）只调用本函数，禁止在 ISR 里做消抖/入队。 */
void Button_NotifyExtiFromIsr(void);

/* 进入 STOP1 前清空事件/按下状态。 */
void Button_PrepareForLp(void);

/* 唤醒后调用：按钮仍按下则从唤醒时刻开始计时；已在时钟恢复期间释放则
 * 合成一次短按 DOWN/UP，保证唤醒键计入第一次短按。 */
void Button_OnWake(uint8_t wake_button, uint8_t pressed_now, uint32_t now_ms);

uint8_t Button_GetEvent(button_event_t *out);
uint8_t Button_IsPressed(void);
uint32_t Button_GetPressStartMs(void);

#endif /* APP_BUTTON_H */

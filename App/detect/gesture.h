/**
 ******************************************************************************
 * @file    gesture.h
 * @brief   PB5 按键手势识别（纯逻辑，不依赖 HAL）。
 *
 * 输出短按、3 s/5 s 长按、短按后 2 s 内二次长按、5 s 后 0.5 s 释放窗口、
 * 1 s 内三连击。实际动作由状态机决定。
 ******************************************************************************
 */
#ifndef APP_GESTURE_H
#define APP_GESTURE_H

#include <stdint.h>

#include "bsp/button.h"

typedef enum
{
    GESTURE_NONE = 0,
    GESTURE_SHORT,
    GESTURE_HOLD3,
    GESTURE_PAIR_HOLD3,
    GESTURE_HOLD5,
    GESTURE_RELEASE_AFTER_HOLD5,
    GESTURE_TRIPLE
} gesture_event_t;

void Gesture_Init(void);
void Gesture_OnButtonEvent(const button_event_t *ev);
void Gesture_Poll(uint32_t now_ms);
uint8_t Gesture_GetEvent(gesture_event_t *out);
uint8_t Gesture_IsBusy(void);

#endif /* APP_GESTURE_H */

/**
 ******************************************************************************
 * @file    buzzer.h
 * @brief   TIM1 CH1N（PA7 经 Q8）可调频率蜂鸣器。
 *
 * 频率由软件改写 ARR 决定：f = 80 MHz / ((PSC+1) × (ARR+1))，PSC 固定 0。
 * 音型表只描述（ARR, 时长），主循环 Buzzer_Poll 推进，中断里不操作定时器。
 ******************************************************************************
 */
#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#include <stdint.h>

typedef enum
{
    BUZZER_PATTERN_NONE = 0,
    BUZZER_PATTERN_BOOT_OK,     /* 4 kHz × 2 短 */
    BUZZER_PATTERN_FAULT,       /* 2 kHz × 3 长 */
    BUZZER_PATTERN_LP_ENTER,    /* 2.5 kHz × 1 */
    BUZZER_PATTERN_WAKE         /* 5 kHz × 1 短 */
} buzzer_pattern_t;

void Buzzer_Init(void);
void Buzzer_ReInit(void);
void Buzzer_DeInitForLp(void);

/* 连续音（arr 为 TIM1 ARR 值，见 board.h 的 BUZZER_ARR_*）。 */
void Buzzer_StartTone(uint16_t autoreload);
void Buzzer_StartPattern(buzzer_pattern_t pattern);
void Buzzer_Poll(void);
void Buzzer_Stop(void);

uint8_t Buzzer_IsActive(void);

#endif /* APP_BUZZER_H */

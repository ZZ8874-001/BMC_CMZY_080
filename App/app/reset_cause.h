/**
 ******************************************************************************
 * @file    reset_cause.h
 * @brief   复位原因采集与异常复位记录（RCC->CSR + RTC 备份寄存器）。
 ******************************************************************************
 */
#ifndef APP_RESET_CAUSE_H
#define APP_RESET_CAUSE_H

#include <stdint.h>

#define RESET_CAUSE_HARDFAULT   (1u << 0)
#define RESET_CAUSE_WATCHDOG    (1u << 1)
#define RESET_CAUSE_SOFTWARE    (1u << 2)
#define RESET_CAUSE_OTHER       (1u << 3)
#define RESET_CAUSE_PIN         (1u << 4)
#define RESET_CAUSE_BOR         (1u << 5)
#define RESET_CAUSE_NMI         (1u << 6)

void ResetCause_Init(void);
uint8_t ResetCause_GetLatched(void);

/* Cortex-M 异常处理程序调用：写备份寄存器后立即系统复位。 */
void ResetCause_RecordHardFaultAndReset(void) __attribute__((noreturn));
void ResetCause_RecordNmiAndReset(void) __attribute__((noreturn));

#endif /* APP_RESET_CAUSE_H */

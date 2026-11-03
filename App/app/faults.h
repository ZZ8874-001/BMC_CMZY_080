/**
 ******************************************************************************
 * @file    faults.h
 * @brief   统一故障注册表（live | sticky），全库唯一故障来源。
 *
 * 位定义见 App/board.h：bit0 BQ_COMM、bit1 XREADY、bit2 OV、bit3 UV、
 * bit4 SCD、bit5 OCD、bit6 TEMP、bit7 LSE、bit8 FLASH_LOG、bit9 保留、
 * bit10 POWER(PVD)、bit11 CLK、bit12 BQ_STALE。
 *
 * live：条件存在期间置位，条件消失后由故障源清除；
 * sticky：一旦置位保持到显式清除（本工程仅上电初始化清除）。
 ******************************************************************************
 */
#ifndef APP_FAULTS_H
#define APP_FAULTS_H

#include <stdbool.h>
#include <stdint.h>

void Fault_Init(void);

void Fault_SetLive(uint16_t mask);
void Fault_ClearLive(uint16_t mask);
void Fault_SetSticky(uint16_t mask);
void Fault_ClearSticky(uint16_t mask);

uint16_t Fault_GetLive(void);
uint16_t Fault_GetSticky(void);
uint16_t Fault_GetAll(void);

/* 供电源模式使用：LP 中关闭 PVD 省电，唤醒后恢复。 */
void Fault_PvdEnable(void);
void Fault_PvdDisable(void);

#endif /* APP_FAULTS_H */

/**
 ******************************************************************************
 * @file    app_telemetry.h
 * @brief   1 Hz USART1 SOC/SOH D 帧遥测。
 *
 * 本模块只通过现有 App_Log_Printf 路径输出一行：
 *   D,V_mV,I_mA,T_0p1C,SOC_0p01,SOH_0p01,Q_mAh,R0_mohm
 * 与 li-ion-soc-soh-toolkit/脚本/soc_record.py 的 8 字段协议兼容。
 *
 * 分层说明：comm 层为 host-reporting 例外，读取 detect/soc_est 与 bms/soh
 * 的只读快照；detect 不反向依赖本模块。
 ******************************************************************************
 */
#ifndef APP_APP_TELEMETRY_H
#define APP_APP_TELEMETRY_H

#include <stdint.h>

#include "bsp/bq_ic.h"

void App_Telemetry_Init(void);
void App_Telemetry_Task(uint32_t now_ms, const bq_snapshot_t *bq_snap);

#endif /* APP_APP_TELEMETRY_H */

/**
 ******************************************************************************
 * @file    cal_stream.h
 * @brief   标定串口流：CAL 会话开始/结束 + 每个 BQ 快照一行 CSV。
 *
 * 只在主循环调用；不逐样本写 Flash。黑匣子只写 CAL_BEGIN/CAL_END 事件。
 ******************************************************************************
 */
#ifndef APP_CAL_STREAM_H
#define APP_CAL_STREAM_H

#include <stdint.h>

#include "bsp/bq_ic.h"

void CalStream_Init(void);
void CalStream_Task(uint32_t now_ms, uint32_t uptime_s,
                    const bq_snapshot_t *snap);
uint8_t CalStream_IsEnabled(void);

#endif /* APP_CAL_STREAM_H */

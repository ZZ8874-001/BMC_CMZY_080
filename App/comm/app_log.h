/**
 ******************************************************************************
 * @file    app_log.h
 * @brief   USART1 非阻塞文本日志（环形缓冲 + TX DMA）。
 *
 * 与参考工程不同：文本通路也走 DMA（本工程要求通信外设全部 DMA 搬运）。
 * App_Log_Printf 只格式化进环形缓冲；App_Log_FlushSmall 在主循环把最多 n 字节
 * 拷到暂存区后启动 DMA，完成中断释放 busy。
 ******************************************************************************
 */
#ifndef APP_APP_LOG_H
#define APP_APP_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

void App_Log_Init(void);
void App_Log_ReInit(void);
void App_Log_DeInitForLp(void);

void App_Log_Printf(const char *fmt, ...);
void App_Log_VPrintf(const char *fmt, va_list args);

/* 主循环调用：最多刷 n 字节（非阻塞）。 */
void App_Log_FlushSmall(uint8_t n);

/* 主循环调用：DMA 看门狗（完成中断丢失时恢复通路）。 */
void App_Log_Poll(void);

bool App_Log_TxBusy(void);

/* USART1 TC 完成（由 HAL_UART_TxCpltCallback 分发，ISR 上下文）。 */
void App_Log_IrqTxDone(void);

#endif /* APP_APP_LOG_H */

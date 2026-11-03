/**
 ******************************************************************************
 * @file    ws2812_uart.h
 * @brief   WS2812（U20，1 灯）USART2 TX 反相 + 中断发送驱动。
 *
 * 移植自 Armor_Cmzy26_081/App/bsp/ws2812_uart（已验证编码：3.8 MBd、TXINV、
 * 查表 {0xEF,0x8F,0xEC,0x8C}、GRB 序、12 B/LED）。
 * 本工程在初始化时手动覆写 USART2 速率/极性/引脚模式（不依赖 IOC 配置）。
 ******************************************************************************
 */
#ifndef APP_WS2812_UART_H
#define APP_WS2812_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#define WS2812_LED_BYTES    12u
#define WS2812_FRAME_BYTES  (RGB_LED_COUNT * WS2812_LED_BYTES)

void Ws2812_Init(void);
void Ws2812_ReInit(void);
void Ws2812_DeInitForLp(void);

/* 单灯 RGB → 12 字节编码（GRB 序，MSB 先行，每字节 2 个 WS 位）。 */
void Ws2812_EncodeLed(uint8_t dst[WS2812_LED_BYTES], uint8_t r, uint8_t g, uint8_t b);

/* 发送一帧（USART2 中断后台发送）；返回 0=已启动，-1=忙/参数错。 */
int Ws2812_Send(const uint8_t *frame);

bool Ws2812_IsBusy(void);

/* 中断发送完成（HAL_UART_TxCpltCallback 桥接，ISR 上下文）。 */
void Ws2812_IrqTxDone(void);

#endif /* APP_WS2812_UART_H */

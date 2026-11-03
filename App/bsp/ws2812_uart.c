/**
 ******************************************************************************
 * @file    ws2812_uart.c
 * @brief   WS2812 单灯驱动：USART2 3.8095 MBd + TX 反相 + TX 中断。
 *
 * 编码（同参考工程，实测有效）：
 *   - TXINV：空闲线电平为低 → 天然 WS2812 复位码，帧间无需复位字节；
 *   - 每 WS 位 4 个 UART 位：'0'=[0,0,0,1]、'1'=[0,1,1,1]（线电平）；
 *   - 查表 {0xEF,0x8F,0xEC,0x8C}（已含反相与 start/stop 位）；
 *   - 颜色序 GRB，每颜色字节 4 个编码字节，每灯 12 B。
 *
 * DMA1_Ch7 留给 I2C1_RX（bq_ic.c）；WS2812 的 USART2_TX 改用中断发送
 * （12 字节 @3.8 MBd ≈ 32 µs，USART2 中断优先级高于 TIM2/I2C DMA）。
 *
 * USART2 全局中断、HAL_UART_TxCpltCallback 均由本文件唯一拥有。
 ******************************************************************************
 */
#include "bsp/ws2812_uart.h"

#include "comm/app_log.h"

#include <string.h>

/* 2 WS 位组 → 1 字节编码表（TXINV 反相后线电平：'0'=[0,0,0,1]，'1'=[0,1,1,1]） */
static const uint8_t s_ws_enc[4] =
{
    0xEFu, 0x8Fu, 0xECu, 0x8Cu    /* 00 01 10 11 */
};

static struct
{
    volatile uint8_t busy;
    volatile uint32_t tx_count;
    volatile uint32_t tx_done_count;
    volatile uint8_t last_error;
    uint8_t tx_buf[WS2812_FRAME_BYTES];
} s_ws2812;

static void Ws2812_HwInit(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};

    /* CubeMX 在 16 MHz PCLK1 下初始化 USART2（115200）；NORM 已切到 80 MHz。
     * 先释放再重配，确保 HAL_UART_Init 重新走 MspInit（USART2SEL=PCLK1）。 */
    if (huart2.gState != HAL_UART_STATE_RESET)
    {
        (void)HAL_UART_DeInit(&huart2);
    }

    clk.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    clk.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    (void)HAL_RCCEx_PeriphCLKConfig(&clk);

    /* HAL_UART_Init 内部：UE=0 → 写 BRR → UE=1；必须在它之前设目标波特率。 */
    huart2.Init.BaudRate = RGB_UART_BAUD_3809K;
    (void)HAL_UART_Init(&huart2);

    /* 精确 BRR=21 + TX 反相/清半双工/PA2 推挽；UE=1 时 BRR 只读，先禁用。 */
    __HAL_UART_DISABLE(&huart2);
    huart2.Instance->BRR = RGB_UART_BRR_3809K;
    huart2.Instance->CR2 |= USART_CR2_TXINV;
    huart2.Instance->CR3 &= (uint32_t)~USART_CR3_HDSEL;
    GPIOA->OTYPER &= (uint32_t)~GPIO_OTYPER_OT2;   /* PA2 推挽（IOC 为开漏） */
    __HAL_UART_ENABLE(&huart2);

    /* DMA1_Ch7 留给 I2C1_RX；WS2812 用 USART2 中断发送。 */
    HAL_NVIC_SetPriority(USART2_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void Ws2812_Init(void)
{
    s_ws2812.busy = 0u;
    s_ws2812.tx_count = 0u;
    s_ws2812.tx_done_count = 0u;
    s_ws2812.last_error = 0u;
    memset(s_ws2812.tx_buf, 0, sizeof(s_ws2812.tx_buf));

    Ws2812_HwInit();
}

void Ws2812_ReInit(void)
{
    s_ws2812.busy = 0u;
    Ws2812_HwInit();
}

void Ws2812_DeInitForLp(void)
{
    if (s_ws2812.busy != 0u)
    {
        (void)HAL_UART_AbortTransmit(&huart2);
        s_ws2812.busy = 0u;
    }

    HAL_NVIC_DisableIRQ(USART2_IRQn);
    (void)HAL_UART_DeInit(&huart2);
}

void Ws2812_EncodeLed(uint8_t dst[WS2812_LED_BYTES], uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t c[3];
    uint32_t i;
    uint32_t j = 0u;

    c[0] = g;
    c[1] = r;
    c[2] = b;   /* GRB 序 */

    for (i = 0u; i < 3u; i++)
    {
        dst[j++] = s_ws_enc[(c[i] >> 6) & 0x03u];
        dst[j++] = s_ws_enc[(c[i] >> 4) & 0x03u];
        dst[j++] = s_ws_enc[(c[i] >> 2) & 0x03u];
        dst[j++] = s_ws_enc[(c[i] >> 0) & 0x03u];
    }
}

int Ws2812_Send(const uint8_t *frame)
{
    if ((frame == NULL) || (s_ws2812.busy != 0u))
    {
        return -1;
    }

    memcpy(s_ws2812.tx_buf, frame, WS2812_FRAME_BYTES);
    s_ws2812.busy = 1u;
    if (HAL_UART_Transmit_IT(&huart2, s_ws2812.tx_buf, WS2812_FRAME_BYTES) != HAL_OK)
    {
        s_ws2812.last_error = 1u;
        s_ws2812.busy = 0u;
        return -1;
    }
    s_ws2812.tx_count++;
    return 0;
}

bool Ws2812_IsBusy(void)
{
    return (s_ws2812.busy != 0u) ? true : false;
}

void Ws2812_IrqTxDone(void)
{
    s_ws2812.tx_done_count++;
    s_ws2812.busy = 0u;
    /* TXINV：发送完成后线路回空闲低电平 = WS 复位码；帧间隔远大于 50 µs。 */
}

/* USART2 全局中断：IT 发送完成 → TC 中断 → HAL_UART_TxCpltCallback。 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* weak 回调桥接：全库唯一 owner，按实例分发。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Ws2812_IrqTxDone();
    }
    else if (huart->Instance == USART1)
    {
        App_Log_IrqTxDone();
    }
}

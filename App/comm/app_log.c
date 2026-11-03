/**
 ******************************************************************************
 * @file    app_log.c
 * @brief   USART1 非阻塞文本日志：1 KB 环形缓冲 + 64 B 暂存 + TX DMA。
 *
 * DMA 通道：DMA1_Ch4（USART1_TX 在 DMA1 上唯一可选通道，CxS=0010）。
 * 完成路径：DMA TC → DMA1_Ch4 IRQ → HAL_DMA_IRQHandler → USART1 TC 中断
 * （USART1 IRQ）→ HAL_UART_TxCpltCallback → App_Log_IrqTxDone。
 ******************************************************************************
 */
#include "comm/app_log.h"

#include "board.h"

#include <stdio.h>
#include <string.h>

#define LOG_RING_SIZE       1024u
#define LOG_DMA_CHUNK       64u
#define LOG_TMP_SIZE        128u
#define LOG_TX_TIMEOUT_MS   100u
#define LOG_NVIC_PRIO       4u

/* App 自定义 DMA 句柄：USART1_TX 走 DMA1_Ch4（board.h extern）。 */
DMA_HandleTypeDef hdma_log_usart1_tx;

static struct
{
    char ring[LOG_RING_SIZE];
    volatile uint16_t write_index;
    uint16_t read_index;
    volatile uint32_t drop_bytes;

    uint8_t dma_stage[LOG_DMA_CHUNK];
    volatile uint8_t dma_busy;
    volatile uint32_t dma_count;
    volatile uint32_t dma_drop;
    volatile uint32_t dma_start_ms;
    volatile uint32_t dma_stuck_count;
    volatile uint32_t tx_done_irq_count;
} s_log;

static void App_Log_HwInit(void);

void App_Log_Init(void)
{
    s_log.write_index = 0u;
    s_log.read_index = 0u;
    s_log.drop_bytes = 0u;
    s_log.dma_busy = 0u;
    s_log.dma_count = 0u;
    s_log.dma_drop = 0u;
    s_log.dma_start_ms = 0u;
    s_log.dma_stuck_count = 0u;
    s_log.tx_done_irq_count = 0u;

    App_Log_HwInit();
}

/* 80 MHz 下重配 USART1 + DMA1_Ch4；LP 唤醒后也走这里。 */
static void App_Log_HwInit(void)
{
    (void)HAL_UART_Init(&huart1);

    /* 通信速率由固件覆写，不依赖 IOC 生成值。 */
    huart1.Instance->BRR = LOG_UART_BRR_115200;

    hdma_log_usart1_tx.Instance = DMA1_Channel4;
    hdma_log_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_log_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_log_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_log_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_log_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_log_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_log_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_log_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    (void)HAL_DMA_Init(&hdma_log_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_log_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, LOG_NVIC_PRIO, 0u);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, LOG_NVIC_PRIO, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    __HAL_UART_ENABLE(&huart1);
}

void App_Log_ReInit(void)
{
    s_log.dma_busy = 0u;
    App_Log_HwInit();
}

void App_Log_DeInitForLp(void)
{
    if (s_log.dma_busy != 0u)
    {
        (void)HAL_UART_AbortTransmit(&huart1);
        s_log.dma_busy = 0u;
    }

    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    (void)HAL_DMA_DeInit(&hdma_log_usart1_tx);
    (void)HAL_UART_DeInit(&huart1);
}

void App_Log_VPrintf(const char *fmt, va_list args)
{
    char tmp[LOG_TMP_SIZE];
    int length;
    int i;

    length = vsnprintf(tmp, sizeof(tmp), fmt, args);
    if (length <= 0)
    {
        return;
    }
    if (length > (int)sizeof(tmp))
    {
        length = (int)sizeof(tmp);
    }

    for (i = 0; i < length; i++)
    {
        uint16_t next = (uint16_t)((s_log.write_index + 1u) % LOG_RING_SIZE);

        if (next == s_log.read_index)
        {
            s_log.read_index = (uint16_t)((s_log.read_index + 1u) % LOG_RING_SIZE);
            s_log.drop_bytes++;
        }
        s_log.ring[s_log.write_index] = tmp[i];
        s_log.write_index = next;
    }
}

void App_Log_Printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    App_Log_VPrintf(fmt, args);
    va_end(args);
}

void App_Log_FlushSmall(uint8_t n)
{
    uint16_t count = 0u;

    if (s_log.dma_busy != 0u)
    {
        return;
    }

    while ((count < n) && (s_log.read_index != s_log.write_index))
    {
        s_log.dma_stage[count] = s_log.ring[s_log.read_index];
        s_log.read_index = (uint16_t)((s_log.read_index + 1u) % LOG_RING_SIZE);
        count++;
    }
    if (count == 0u)
    {
        return;
    }

    s_log.dma_busy = 1u;
    s_log.dma_start_ms = HAL_GetTick();
    if (HAL_UART_Transmit_DMA(&huart1, s_log.dma_stage, count) != HAL_OK)
    {
        /* 启动失败：把数据退回环形缓冲不可行（读指针已前进），丢弃并计数。 */
        s_log.dma_drop += count;
        s_log.dma_busy = 0u;
        return;
    }
    s_log.dma_count++;
}

void App_Log_Poll(void)
{
    if ((s_log.dma_busy != 0u) &&
        ((HAL_GetTick() - s_log.dma_start_ms) >= LOG_TX_TIMEOUT_MS))
    {
        (void)HAL_UART_AbortTransmit(&huart1);
        s_log.dma_busy = 0u;
        s_log.dma_stuck_count++;
    }
}

bool App_Log_TxBusy(void)
{
    return (s_log.dma_busy != 0u) ? true : false;
}

void App_Log_IrqTxDone(void)
{
    s_log.tx_done_irq_count++;
    s_log.dma_busy = 0u;
}

/* DMA1_Ch4：USART1 TX DMA（App 自管通道，生成文件无此 handler）。 */
void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_log_usart1_tx);
}

/* USART1 全局中断：DMA 搬完后由 TC 中断结束发送并触发 TxCpltCallback。 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

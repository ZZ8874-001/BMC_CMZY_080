/**
 ******************************************************************************
 * @file    board.h
 * @brief   板级公共定义：外设句柄 extern 的唯一入口、板级常量、I2C/Flash 布局。
 *
 * 约定：任何模块需要生成代码中的外设句柄，都只能在本文件 extern，模块内不得
 * 自行声明。引脚语义宏继续使用 Core/Inc/main.h 中的定义（BQ_BOOT_Pin 等）。
 ******************************************************************************
 */
#ifndef APP_BOARD_H
#define APP_BOARD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== CubeMX 生成的外设句柄（Core/Src/main.c） ==================== */
extern COMP_HandleTypeDef hcomp2;
extern I2C_HandleTypeDef  hi2c1;
extern RTC_HandleTypeDef  hrtc;
extern TIM_HandleTypeDef  htim1;   /* 蜂鸣器 PWM（CH1N/PA7） */
extern TIM_HandleTypeDef  htim2;   /* 250 Hz 应用时基 */
extern UART_HandleTypeDef huart1;  /* 调试日志（PB6/PB7） */
extern UART_HandleTypeDef huart2;  /* WS2812 RGB（PA2） */

extern DMA_HandleTypeDef hdma_i2c1_tx;   /* DMA1_Ch6，I2C1_TX（CubeMX 生成） */
extern DMA_HandleTypeDef hdma_i2c1_rx;   /* DMA1_Ch7，I2C1_RX（CubeMX 生成） */

/* ==================== App 自行定义的 DMA 句柄 ==================== */
extern DMA_HandleTypeDef hdma_log_usart1_tx;/* DMA1_Ch4，USART1_TX */

/* ==================== 调度周期 ==================== */
#define APP_TICK_HZ             250U        /* TIM2 应用中断频率 */
#define APP_TICK_MS             4U
#define APP_BQ_READ_PERIOD_MS   500U        /* BQ 电芯数据 250 ms 更新，500 ms 读一次 */
#define APP_BQ_STALE_MS         2000U       /* 超过该时长无新数据 → FAULT_BQ_STALE */
#define APP_BB_PERIOD_MS        1800000UL   /* 黑匣子 30 min 一条 */

/* ==================== BQ76920 / I2C1 ==================== */
#define BQ_CELL_COUNT           3U       /* 本板 3S：VC0-VC3 */
#define BQ_CURRENT_MA_NUM       844      /* 8.44 µV/LSB ×100 */
#define BQ_CURRENT_MA_DEN       150      /* 分流 1.5 mΩ ×100；I(mA)=cc_raw×844/150 */
#define BQ_I2C_ADDR_7BIT        (0x08U)
#define BQ_I2C_ADDR_WRITE       ((BQ_I2C_ADDR_7BIT) << 1)
#define BQ_I2C_ADDR_READ        (((BQ_I2C_ADDR_7BIT) << 1) | 1U)

/* 80 MHz PCLK1：PRESC=1(40 MHz)、SCLH=164(4.125 µs)、SCLL=255(6.4 µs) → 95.0 kHz。
 * BQ 要求 ≤100 kHz 且 tHIGH ≥ 4.0 µs，IOC 生成的 103.6 kHz/3.93 µs 不合格。 */
#define BQ_I2C_TIMINGR_95K      0x1424A4FFUL
#define BQ_I2C_TIMEOUT_MS       100U

/* ==================== 串口速率（80 MHz 下手动覆写 BRR） ==================== */
#define LOG_UART_BRR_115200     694U        /* 80e6/(16*115200)=43.403 → BRR 0x2B6 */
#define RGB_UART_BRR_3809K      21U         /* 80e6/21=3.8095 MBd */
#define RGB_UART_BAUD_3809K     3809523U    /* 80 MHz / 21 → 3.8095 MBd（HAL 初始化用） */

/* ==================== WS2812 ==================== */
#define RGB_LED_COUNT           1U
#define RGB_FRAME_BYTES         ((RGB_LED_COUNT) * 12U)

/* ==================== 蜂鸣器音调（TIM1 PSC=0，80 MHz） ==================== */
#define BUZZER_ARR_2K0          39999U      /* 2.0 kHz */
#define BUZZER_ARR_2K5          31999U      /* 2.5 kHz */
#define BUZZER_ARR_3K0          26665U      /* 3.0 kHz */
#define BUZZER_ARR_4K0          19999U      /* 4.0 kHz */
#define BUZZER_ARR_5K0          15999U      /* 5.0 kHz */

/* ==================== 黑匣子 Flash 布局 ==================== */
/* 页 52–63（2 KB/页 = 24 KB）：每页 85 条 × 24 B = 2040 B，尾部 8 B 不用。
 * 1020 条 × 30 min ≈ 21.2 天，满足 ≥15 天需求。 */
#define BB_FLASH_BASE           0x0801A000UL
#define BB_FLASH_END            0x08020000UL
#define BB_FLASH_PAGE_SIZE      2048UL
#define BB_FLASH_PAGES          12U
#define BB_RECORDS_PER_PAGE     85U
#define BB_RECORD_BYTES         24U
#define BB_RECORD_COUNT         (BB_FLASH_PAGES * BB_RECORDS_PER_PAGE)

/* ==================== 故障位图（见 App/app/faults.h 的语义注释） ==================== */
#define FAULT_BIT_BQ_COMM       (1u << 0)
#define FAULT_BIT_BQ_XREADY     (1u << 1)
#define FAULT_BIT_BQ_OV         (1u << 2)
#define FAULT_BIT_BQ_UV         (1u << 3)
#define FAULT_BIT_BQ_SCD        (1u << 4)
#define FAULT_BIT_BQ_OCD        (1u << 5)
#define FAULT_BIT_TEMP          (1u << 6)
#define FAULT_BIT_LSE           (1u << 7)
#define FAULT_BIT_FLASH_LOG     (1u << 8)
#define FAULT_BIT_CAN_BUSOFF    (1u << 9)   /* L0 保留 */
#define FAULT_BIT_POWER         (1u << 10)
#define FAULT_BIT_CLK           (1u << 11)
#define FAULT_BIT_BQ_STALE      (1u << 12)

#ifdef __cplusplus
}
#endif

#endif /* APP_BOARD_H */

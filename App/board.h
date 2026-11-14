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
extern CAN_HandleTypeDef  hcan1;
extern DAC_HandleTypeDef  hdac1;

extern DMA_HandleTypeDef hdma_i2c1_tx;   /* DMA1_Ch6，I2C1_TX（CubeMX 生成） */
extern DMA_HandleTypeDef hdma_i2c1_rx;   /* DMA1_Ch7，I2C1_RX（CubeMX 生成） */

/* ==================== App 自行定义的 DMA 句柄 ==================== */
extern DMA_HandleTypeDef hdma_log_usart1_tx;/* DMA1_Ch4，USART1_TX */

/* ==================== 发布版本 ==================== */
/* 与 Git tag vX.Y.Z+hwA.BC 对应；启动日志会打印这两个字符串。 */
#define APP_HW_VERSION_STR      "0.80"
#define APP_FW_VERSION_STR      "0.0.11"

/* ==================== 调度周期 ==================== */
#define APP_TICK_HZ             250U        /* TIM2 应用中断频率 */
#define APP_TICK_MS             4U
#define APP_BQ_READ_PERIOD_MS   500U        /* BQ 电芯数据 250 ms 更新，500 ms 读一次 */
#define APP_BQ_STALE_MS         2000U       /* 超过该时长无新数据 → FAULT_BQ_STALE */

/* ==================== LP RTC 时基 / SMBA 备用路径 ==================== */
/* RTC/LP 时基源。0 = LSI（当前默认；硬件 LSE 未起振），1 = LSE（修好后
 * 手动切换）。LSI 仅作相对时间参考：RTC 与 IWDG 同源，30 min 绝对时间
 * 会漂，IWDG 校准也只能保证相对一致。 */
#ifndef LP_RTC_USE_LSE
#define LP_RTC_USE_LSE          0
#endif

#if (LP_RTC_USE_LSE != 0)
#define LP_RTC_EXPECT_RTCSEL    RCC_BDCR_RTCSEL_0   /* LSE */
#define LP_RTC_EXPECT_RDY_FLAG  RCC_FLAG_LSERDY
#else
#define LP_RTC_EXPECT_RTCSEL    RCC_BDCR_RTCSEL_1   /* LSI */
#define LP_RTC_EXPECT_RDY_FLAG  RCC_FLAG_LSIRDY
#endif

/* LSE 模式启动 LSE 时的驱动能力；按实际晶体调整。 */
#define LP_LSE_DRIVE_CAPABILITY RCC_LSEDRIVE_MEDIUMHIGH

/* SMBA 备用唤醒：0 = 生产只用 EXTI5；1 = 第一个 LP 周期做一次 SMBA
 * host-alert 实测（EXTI5 始终回退），记录后自动恢复 EXTI5。仅台架使用。 */
#ifndef LP_SMBA_WAKE_TEST
#define LP_SMBA_WAKE_TEST       0
#endif

#define BQ_ADC_SETTLE_MS        300U        /* ADC_EN/SHIP 后 250 ms 建立 + 50 ms 余量 */
#define APP_BB_PERIOD_MS        1800000UL   /* 黑匣子 30 min 一条 */

/* ==================== BQ76920 / I2C1 ==================== */
#define BQ_CELL_COUNT           3U       /* 本板 3S：CELL3 = VC5-VC4 */
#define BQ_CURRENT_MA_NUM       844      /* 8.44 µV/LSB ×100 */
#define BQ_CURRENT_MA_DEN       150      /* 分流 1.5 mΩ ×100；I(mA)=cc_raw×844/150 */

/* BQ CC 符号（唯一板级事实，soc_est 与 protection 都从这里派生）：
 *   -1 = 充电时 current_ma 为正、放电为负（TI bq769x0 AFE FAQ / bq78350
 *        约定：VSR = V(SRP) - V(SRN)，SRP 在电池负极侧）；
 *   +1 = 放电时 current_ma 为正（上位机/库约定，若实测 CC 极性相反则改这里）。
 * 首次上板必须用已知充电/放电方向确认（SOC 积分方向）；软件 OCP 的方向
 * 另用 COMP2 判定，不依赖本符号。 */
#define BQ_CC_DISCHARGE_SIGN    (-1)
#define BQ_I2C_ADDR_7BIT        (0x08U)
#define BQ_I2C_ADDR_WRITE       ((BQ_I2C_ADDR_7BIT) << 1)
#define BQ_I2C_ADDR_READ        (((BQ_I2C_ADDR_7BIT) << 1) | 1U)

/* 80 MHz PCLK1：PRESC=1(40 MHz)、SCLH=164(4.125 µs)、SCLL=255(6.4 µs) → 95.0 kHz。
 * BQ 要求 ≤100 kHz 且 tHIGH ≥ 4.0 µs，IOC 生成的 103.6 kHz/3.93 µs 不合格。 */
#define BQ_I2C_TIMINGR_95K      0x1424A4FFUL
/* HSI16 16 MHz PCLK1 下约 100 kHz（PRESC=1 → 8 MHz；SCLH=32/SCLL=47）。
 * 仅用于 LP 轻量 boot 的 BQ sniff；M0 需用示波器确认 tHIGH/tLOW。 */
#define BQ_I2C_TIMINGR_HSI16_100K 0x1024202FUL
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

/* ==================== Flash 分区（L1） ==================== */
/* 代码区限制为 96 KB；页 48–51 预留给 BMS NVM / StateStore，黑匣子保持页 52–63。 */
#define APP_FLASH_BASE          0x08000000UL
#define APP_FLASH_END           0x08018000UL
#define BMS_NVM_FLASH_BASE      0x08018000UL
#define BMS_NVM_FLASH_PAGE_SIZE 2048UL
#define BMS_NVM_FLASH_PAGES     2U
#define BMS_NVM_FLASH_END       (BMS_NVM_FLASH_BASE + \
                                 (BMS_NVM_FLASH_PAGE_SIZE * BMS_NVM_FLASH_PAGES))
#define STATE_STORE_FLASH_BASE  0x08019000UL
#define STATE_STORE_FLASH_PAGES 2U
#define STATE_STORE_FLASH_END   (STATE_STORE_FLASH_BASE + \
                                 (BMS_NVM_FLASH_PAGE_SIZE * STATE_STORE_FLASH_PAGES))
#define BMS_NVM_RECORD_BYTES    132U
#define STATE_STORE_RECORD_BYTES 48U

extern uint32_t _image_end;    /* 链接脚本定义：FLASH 镜像真实末尾 */

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

#if (BMS_NVM_FLASH_END != STATE_STORE_FLASH_BASE)
#error "BMS NVM / StateStore Flash layout mismatch"
#endif
#if (STATE_STORE_FLASH_END != BB_FLASH_BASE)
#error "StateStore / black-box Flash layout mismatch"
#endif
#if (APP_FLASH_END != BMS_NVM_FLASH_BASE)
#error "Application Flash must end at BMS NVM base"
#endif

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
#define FAULT_BIT_PROTECTION    (1u << 9)   /* 保护监督层聚合故障 */
#define FAULT_BIT_POWER         (1u << 10)
#define FAULT_BIT_CLK           (1u << 11)
#define FAULT_BIT_BQ_STALE      (1u << 12)
#define FAULT_BIT_FLASH_STATE   (1u << 13)  /* StateStore/NVM 持久化故障 */
#define FAULT_BIT_BQ_ADC_STUCK  (1u << 14)  /* ADC 重启/回读后仍无有效电压帧 */
#define FAULT_BIT_TIMEBASE      (1u << 15)  /* RTC LP 时基不可用（LP 阻止） */

#ifdef __cplusplus
}
#endif

#endif /* APP_BOARD_H */

/**
 ******************************************************************************
 * @file    bq_ic.c
 * @brief   BQ7692003PWR 驱动：I2C1 + DMA + 每字节 CRC-8，非阻塞事务状态机。
 *
 * 协议要点（SLUSBK2I，§8.3.1.4）：
 *   - 写：[0x10][reg][data][CRC(0x10,reg,data)]；
 *   - 读寄存器指针：[0x10][reg]（无 CRC），Stop 后允许直接重新 Start 读
 *     （Figure 8-5），因此不需要 HAL 的（DMA 不支持的）重复起始；
 *   - 读数据：每数据字节后跟 1 字节 CRC；首字节 CRC 覆盖 (0x11,data)，
 *     块读后续字节只覆盖 data；主机对最后一个 CRC 回 NACK（HAL 硬件行为一致）。
 *   - CRC-8：poly 0x07，初值 0，MSB 先行。
 *
 * DMA 通道（只用 DMA1，RM0394 Table 45）：
 *   I2C1_TX → DMA1_Ch6 / Request 3（生成句柄 hdma_i2c1_tx）
 *   I2C1_RX → DMA1_Ch7 / Request 3（生成句柄 hdma_i2c1_rx，生成 IRQ handler）
 *   USART1_TX → DMA1_Ch4 / Request 2（hdma_log_usart1_tx，见 app_log.c）
 *   USART2_TX（WS2812）改用中断发送，不再占用 DMA1_Ch7。
 *
 * ISR 只置标志；所有 HAL 调用都在 BQ_Poll（主循环）里发起。
 ******************************************************************************
 */
#include "bsp/bq_ic.h"

#include "board.h"
#include "app/faults.h"

#include <stddef.h>


/* ---------------- 寄存器地址 ---------------- */
#define BQ_REG_SYS_STAT     0x00u
#define BQ_REG_SYS_CTRL1    0x04u
#define BQ_REG_SYS_CTRL2    0x05u
#define BQ_REG_CC_CFG       0x0Bu
#define BQ_REG_VC1          0x0Cu   /* 2 字节块读自动递增：VC1_HI/LO */
#define BQ_REG_VC2          0x0Eu
#define BQ_REG_VC3          0x10u
#define BQ_REG_BAT          0x2Au
#define BQ_REG_TS1          0x2Cu
#define BQ_REG_CC           0x32u
#define BQ_REG_ADCGAIN1     0x50u
#define BQ_REG_ADCOFFSET    0x51u
#define BQ_REG_ADCGAIN2     0x59u

#define BQ_CC_CFG_VALUE     0x19u   /* 数据手册要求启动时写入 CC_CFG<5:0> = 0x19 */
/* 实测 CC_CFG 读回可能是 0x59 或 0x79（bit5 不稳定/固定位），低 5 位始终是 0x19；
 * 比较时只看 CC_CFG<4:0>，bit5/6 不参与判定。 */
#define BQ_CC_CFG_MASK      0x1Fu
#define BQ_SYS_CTRL1_VALUE  0x18u   /* ADC_EN | TEMP_SEL（NORMAL 下 ADC 必须保持使能） */
#define BQ_SYS_CTRL1_LP     0x00u   /* LP：清 ADC_EN|TEMP_SEL → 电压/电流/温度检测停，
                                       OV/UV 保护随之停；SCD/OCD 比较器不可关、仍然有效 */
#define BQ_SYS_CTRL2_CC_EN  0x40u   /* CC_EN；FET 位叠加在低两位 */
#define BQ_SYS_CTRL2_RUN    0x43u   /* CC_EN | DSG_ON | CHG_ON */
#define BQ_SYS_CTRL2_LP     0x03u   /* DSG_ON | CHG_ON，CC 关闭 */
#define BQ_SYS_STAT_IRQ_CLR 0xA0u   /* 只清 CC_READY 与 DEVICE_XREADY，保护锁存保留 */

#define BQ_SYS_STAT_XREADY      (1u << 5)
#define BQ_SYS_STAT_OV          (1u << 2)
#define BQ_SYS_STAT_UV          (1u << 3)
#define BQ_SYS_STAT_SCD         (1u << 1)
#define BQ_SYS_STAT_OCD         (1u << 0)

#define BQ_TRANSFER_TIMEOUT_MS  100u
#define BQ_BLOCKING_TIMEOUT_MS  50u
#define BQ_RETRY_LIMIT          3u
#define BQ_RETRY_BACKOFF_MS     20u
#define BQ_FAIL_BACKOFF_MS      2000u
#define BQ_CFG_RETRY_LIMIT      3u

/* ---------------- 事务描述 ---------------- */
typedef enum
{
    BQ_OP_READ1 = 0,
    BQ_OP_READ2,
    BQ_OP_WRITE
} bq_op_t;

typedef struct
{
    uint8_t op;
    uint8_t reg;
    uint8_t data;
} bq_job_t;

/* 配置序列：读回增益/偏移供电压换算，最后打开 CC 与两只 MOS。 */
static const bq_job_t s_bq_cfg_script[] =
{
    {BQ_OP_WRITE, BQ_REG_CC_CFG,    BQ_CC_CFG_VALUE},
    {BQ_OP_READ1, BQ_REG_CC_CFG,    0u},
    {BQ_OP_READ1, BQ_REG_ADCGAIN1,  0u},
    {BQ_OP_READ1, BQ_REG_ADCGAIN2,  0u},
    {BQ_OP_READ1, BQ_REG_ADCOFFSET, 0u},
    {BQ_OP_WRITE, BQ_REG_SYS_CTRL1,  BQ_SYS_CTRL1_VALUE},
    {BQ_OP_WRITE, BQ_REG_SYS_STAT,   0xFFu},
    {BQ_OP_WRITE, BQ_REG_SYS_CTRL2,  BQ_SYS_CTRL2_RUN}
};

/* 运行轮询：先读状态（清 CC_READY/XREADY），再读 3 节电压、总压、TS1、CC。 */
static const bq_job_t s_bq_run_script[] =
{
    {BQ_OP_READ1, BQ_REG_SYS_STAT,  0u},
    {BQ_OP_WRITE, BQ_REG_SYS_STAT,  BQ_SYS_STAT_IRQ_CLR},
    {BQ_OP_READ2, BQ_REG_VC1,       0u},
    {BQ_OP_READ2, BQ_REG_VC2,       0u},
    {BQ_OP_READ2, BQ_REG_VC3,       0u},
    {BQ_OP_READ2, BQ_REG_BAT,       0u},
    {BQ_OP_READ2, BQ_REG_TS1,       0u},
    {BQ_OP_READ2, BQ_REG_CC,        0u}
};

/* ---------------- 模块状态（调试器可直接展开全名字段） ---------------- */
static struct
{
    bq_state_t state;
    uint32_t state_enter_ms;
    uint32_t now_ms;

    const bq_job_t *script;
    uint8_t script_len;
    uint8_t job_index;
    bq_op_t current_op;
    uint8_t current_reg;
    uint8_t data_expected;

    volatile uint8_t irq_tx_done;
    volatile uint8_t irq_rx_done;
    volatile uint8_t irq_error;

    uint8_t xfer_phase;         /* 0=空闲 1=TX 在飞 2=RX 在飞 */
    uint32_t xfer_start_ms;
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];

    /* 运行时 MOS 开关请求。bit0=DSG，bit1=CHG；默认 0x03（全开）。
     * run script 完成后若 request != applied，用一次性 DMA 写 SYS_CTRL2。 */
    volatile uint8_t request_fet_mask;
    uint8_t fet_mask_applied;
    uint8_t fet_sync_active;
    bq_job_t fet_job;
    int32_t current_ma;

    uint8_t retry;
    uint8_t cfg_attempts;

    bq_state_t backoff_return_state;
    uint8_t backoff_kind;       /* 0=重试当前任务 1=重启配置序列 */
    uint32_t backoff_ms;

    volatile uint8_t req_read;
    uint8_t fresh;
    uint32_t last_update_ms;

    uint8_t sys_stat;
    uint8_t cc_cfg_readback;
    uint8_t adc_gain1_raw;
    uint8_t adc_gain2_raw;
    uint8_t adc_gain;
    int8_t adc_offset;

    uint16_t raw_cell[3];
    uint16_t raw_pack;
    uint16_t raw_ts1;
    int16_t cc_raw;

    uint16_t cell_mv[3];
    uint16_t pack_mv;
} s_bq;

/* ---------------- CRC-8（poly 0x07，初值 0，MSB 先行） ---------------- */
static uint8_t Bq_Crc8Step(uint8_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= byte;
    for (i = 0u; i < 8u; i++)
    {
        if ((crc & 0x80u) != 0u)
        {
            crc = (uint8_t)((crc << 1) ^ 0x07u);
        }
        else
        {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t Bq_Crc8_Write(uint8_t reg, uint8_t data)
{
    uint8_t crc = 0u;

    crc = Bq_Crc8Step(crc, BQ_I2C_ADDR_WRITE);
    crc = Bq_Crc8Step(crc, reg);
    crc = Bq_Crc8Step(crc, data);
    return crc;
}

static uint8_t Bq_Crc8_ReadFirst(uint8_t data)
{
    uint8_t crc = 0u;

    crc = Bq_Crc8Step(crc, BQ_I2C_ADDR_READ);
    crc = Bq_Crc8Step(crc, data);
    return crc;
}

static uint8_t Bq_Crc8_ReadNext(uint8_t data)
{
    return Bq_Crc8Step(0u, data);
}

/* ---------------- 硬件层 ---------------- */
static void Bq_RecoverBus(void);

static void Bq_HwInit(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 重新初始化（冷启动幂等；LP 唤醒后恢复）。
     * 95 kHz 必须在 HAL_I2C_Init 之前写进 Init.Timing：HAL_I2C_Init 内部会
     * 先 __HAL_I2C_DISABLE、写 TIMINGR、再 __HAL_I2C_ENABLE；PE=1 时 TIMINGR
     * 只读，Init 之后再写 Instance->TIMINGR 会被硬件忽略（实测仍跑生成值 400 kHz）。 */
    hi2c1.Init.Timing = BQ_I2C_TIMINGR_95K;
    (void)HAL_I2C_Init(&hi2c1);

    /* I2C1_TX：DMA1_Ch6 / Request 3。 */
    hdma_i2c1_tx.Instance = DMA1_Channel6;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_3;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    (void)HAL_DMA_Init(&hdma_i2c1_tx);
    __HAL_LINKDMA(&hi2c1, hdmatx, hdma_i2c1_tx);

    /* I2C1_RX：DMA1_Ch7 / Request 3（生成句柄，生成 IRQ handler 已存在）。 */
    hdma_i2c1_rx.Instance = DMA1_Channel7;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_3;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    (void)HAL_DMA_Init(&hdma_i2c1_rx);
    __HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3u, 0u);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3u, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3u, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 3u, 0u);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    __HAL_I2C_ENABLE(&hi2c1);
}

static void Bq_RecoverBus(void)
{
    s_bq.script = NULL;
    s_bq.xfer_phase = 0u;
    s_bq.irq_tx_done = 0u;
    s_bq.irq_rx_done = 0u;
    s_bq.irq_error = 0u;

    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    (void)HAL_DMA_DeInit(&hdma_i2c1_tx);
    (void)HAL_DMA_DeInit(&hdma_i2c1_rx);
    (void)HAL_I2C_DeInit(&hi2c1);

    Bq_HwInit();
}

/* 有界阻塞写（只用于 LP 进入前，模式切换允许阻塞）。
 * 注意：HAL 的 DevAddress 参数会先发地址，pData 只能是寄存器/数据/CRC，
 * 不能把 0x10 地址再塞进缓冲区（否则地址会重复发送，BQ 把 CRC 当数据 NACK）。 */
static int Bq_BlockingWrite(uint8_t reg, uint8_t data)
{
    uint8_t frame[3];

    frame[0] = reg;
    frame[1] = data;
    frame[2] = Bq_Crc8_Write(reg, data);

    return (HAL_I2C_Master_Transmit(&hi2c1, BQ_I2C_ADDR_WRITE, frame, 3u,
                                    BQ_BLOCKING_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int Bq_BlockingRead1(uint8_t reg, uint8_t *out)
{
    uint8_t ptr[1];
    uint8_t rx[2];

    ptr[0] = reg;   /* 地址由 HAL 的 DevAddress 发送，这里只发寄存器 */

    if (HAL_I2C_Master_Transmit(&hi2c1, BQ_I2C_ADDR_WRITE, ptr, 1u,
                                BQ_BLOCKING_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, BQ_I2C_ADDR_WRITE, rx, 2u,
                               BQ_BLOCKING_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }
    if (rx[1] != Bq_Crc8_ReadFirst(rx[0]))
    {
        return -1;
    }
    *out = rx[0];
    return 0;
}

/* ---------------- 数据换算 ---------------- */
static uint16_t Bq_RawToMv(uint16_t raw)
{
    int32_t microvolts;
    int32_t millivolts;

    /* GAIN = 365 µV/LSB + ADCGAIN<4:0>；V(mV) = GAIN × ADC / 1000 + OFFSET(mV)。 */
    microvolts = (int32_t)(365 + (int32_t)s_bq.adc_gain) * (int32_t)raw;
    millivolts = (microvolts / 1000) + (int32_t)s_bq.adc_offset;
    if (millivolts < 0)
    {
        millivolts = 0;
    }
    return (uint16_t)millivolts;
}

/* BAT（总压）寄存器 = 各节 ADC 之和 / 4，LSB 为单节的 4 倍；
 * 数据手册：V(BAT) = 4 × GAIN × ADC + (节数 × OFFSET)。 */
static uint16_t Bq_RawPackToMv(uint16_t raw)
{
    int32_t microvolts;
    int32_t millivolts;

    microvolts = 4 * (int32_t)(365 + (int32_t)s_bq.adc_gain) * (int32_t)raw;
    millivolts = (microvolts / 1000) +
                 ((int32_t)BQ_CELL_COUNT * (int32_t)s_bq.adc_offset);
    if (millivolts < 0)
    {
        millivolts = 0;
    }
    if (millivolts > 65535)
    {
        millivolts = 65535;
    }
    return (uint16_t)millivolts;
}

static void Bq_UpdateFaults(uint8_t stat)
{
    uint16_t live = 0u;

    if ((stat & BQ_SYS_STAT_XREADY) != 0u)
    {
        live |= FAULT_BIT_BQ_XREADY;
    }
    if ((stat & BQ_SYS_STAT_OV) != 0u)
    {
        live |= FAULT_BIT_BQ_OV;
    }
    if ((stat & BQ_SYS_STAT_UV) != 0u)
    {
        live |= FAULT_BIT_BQ_UV;
    }
    if ((stat & BQ_SYS_STAT_SCD) != 0u)
    {
        live |= FAULT_BIT_BQ_SCD;
    }
    if ((stat & BQ_SYS_STAT_OCD) != 0u)
    {
        live |= FAULT_BIT_BQ_OCD;
    }

    Fault_ClearLive(FAULT_BIT_BQ_XREADY | FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV |
                    FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD);
    Fault_SetLive(live);
}

static void Bq_HandleReadData(uint8_t reg, const uint8_t *data, uint8_t count)
{
    uint16_t value;

    /* READ2（2 字节寄存器）的 4 字节布局：[data_hi, CRC1, data_lo, CRC2]，
     * 低字节在 data[2]，data[1] 是第一个数据字节的 CRC。 */
    switch (reg)
    {
    case BQ_REG_SYS_STAT:
        s_bq.sys_stat = data[0];
        Bq_UpdateFaults(data[0]);
        break;

    case BQ_REG_CC_CFG:
        s_bq.cc_cfg_readback = data[0];
        break;

    case BQ_REG_ADCGAIN1:
        s_bq.adc_gain1_raw = data[0];
        s_bq.adc_gain = (uint8_t)((((s_bq.adc_gain1_raw >> 2) & 0x03u) << 3) |
                                  ((s_bq.adc_gain2_raw >> 5) & 0x07u));
        break;

    case BQ_REG_ADCGAIN2:
        s_bq.adc_gain2_raw = data[0];
        s_bq.adc_gain = (uint8_t)((((s_bq.adc_gain1_raw >> 2) & 0x03u) << 3) |
                                  ((s_bq.adc_gain2_raw >> 5) & 0x07u));
        break;

    case BQ_REG_ADCOFFSET:
        s_bq.adc_offset = (int8_t)data[0];
        break;

    case BQ_REG_VC1:
        if (count == 2u)
        {
            s_bq.raw_cell[0] = (uint16_t)(((uint16_t)data[0] << 8) | data[2]);
        }
        break;

    case BQ_REG_VC2:
        if (count == 2u)
        {
            s_bq.raw_cell[1] = (uint16_t)(((uint16_t)data[0] << 8) | data[2]);
        }
        break;

    case BQ_REG_VC3:
        if (count == 2u)
        {
            s_bq.raw_cell[2] = (uint16_t)(((uint16_t)data[0] << 8) | data[2]);
        }
        break;

    case BQ_REG_BAT:
        if (count == 2u)
        {
            s_bq.raw_pack = (uint16_t)(((uint16_t)data[0] << 8) | data[2]);
        }
        break;

    case BQ_REG_TS1:
        if (count == 2u)
        {
            s_bq.raw_ts1 = (uint16_t)(((uint16_t)data[0] << 8) | data[2]);
        }
        break;

    case BQ_REG_CC:
        if (count == 2u)
        {
            value = (uint16_t)(((uint16_t)data[0] << 8) | data[2]);
            s_bq.cc_raw = (int16_t)value;
        }
        break;

    default:
        break;
    }
}

static void Bq_SnapshotUpdate(void)
{
    s_bq.cell_mv[0] = Bq_RawToMv(s_bq.raw_cell[0]);
    s_bq.cell_mv[1] = Bq_RawToMv(s_bq.raw_cell[1]);
    s_bq.cell_mv[2] = Bq_RawToMv(s_bq.raw_cell[2]);
    s_bq.pack_mv = Bq_RawPackToMv(s_bq.raw_pack);
    /* CC：1 LSB=8.44 µV，分流 1.5 mΩ → I(mA)=cc_raw×844/150（带符号）。 */
    s_bq.current_ma = ((int32_t)s_bq.cc_raw * BQ_CURRENT_MA_NUM) / BQ_CURRENT_MA_DEN;
    s_bq.last_update_ms = s_bq.now_ms;
    s_bq.fresh = 1u;
}

/* ---------------- 事务状态机 ---------------- */
static void Bq_StartNextJob(void);
static void Bq_StartConfig(void);
static void Bq_StartScript(const bq_job_t *script, uint8_t len, bq_state_t state);

static void Bq_ScheduleRetry(uint8_t retry_current_job)
{
    s_bq.xfer_phase = 0u;
    s_bq.backoff_return_state = (s_bq.state == BQ_ST_BACKOFF) ?
                                s_bq.backoff_return_state : s_bq.state;
    s_bq.backoff_kind = (retry_current_job != 0u) ? 0u : 1u;
    s_bq.backoff_ms = (retry_current_job != 0u) ? BQ_RETRY_BACKOFF_MS : BQ_FAIL_BACKOFF_MS;
    s_bq.state = BQ_ST_BACKOFF;
    s_bq.state_enter_ms = HAL_GetTick();
    if (retry_current_job == 0u)
    {
        s_bq.script = NULL;
    }
}

static void Bq_TransferFailed(void)
{
    s_bq.retry++;
    if (s_bq.retry >= BQ_RETRY_LIMIT)
    {
        s_bq.retry = 0u;
        Fault_SetSticky(FAULT_BIT_BQ_COMM);
        Bq_RecoverBus();
        Bq_ScheduleRetry(0u);
    }
    else
    {
        Bq_ScheduleRetry(1u);
    }
}

static void Bq_JobComplete(void)
{
    s_bq.retry = 0u;
    Fault_ClearSticky(FAULT_BIT_BQ_COMM);
    s_bq.job_index++;
    Bq_StartNextJob();
}

static void Bq_ScriptComplete(void)
{
    s_bq.script = NULL;

    /* 一次性 FET 同步写完成：以写入时的 mask 为准（期间若又改请求，下轮再同步）。 */
    if (s_bq.fet_sync_active != 0u)
    {
        s_bq.fet_sync_active = 0u;
        s_bq.fet_mask_applied = (uint8_t)(s_bq.fet_job.data & 0x03u);
        return;
    }

    if (s_bq.state == BQ_ST_CONFIG)
    {
        if ((s_bq.cc_cfg_readback & BQ_CC_CFG_MASK) == BQ_CC_CFG_VALUE)
        {
            s_bq.cfg_attempts = 0u;
            s_bq.fet_mask_applied = 0x03u;  /* 配置脚本最后写 SYS_CTRL2=0x43 */
            s_bq.fet_sync_active = 0u;
            s_bq.state = BQ_ST_RUN;
            s_bq.req_read = 1u;     /* 配置完成后立刻取一轮数据 */
        }
        else
        {
            s_bq.cfg_attempts++;
            if (s_bq.cfg_attempts >= BQ_CFG_RETRY_LIMIT)
            {
                Fault_SetSticky(FAULT_BIT_BQ_COMM);
                s_bq.cfg_attempts = 0u;
                Bq_ScheduleRetry(0u);
            }
            else
            {
                Bq_ScheduleRetry(0u);
            }
        }
    }
    else if (s_bq.state == BQ_ST_RUN)
    {
        Bq_SnapshotUpdate();
        s_bq.req_read = 0u;

        /* 本刷新周期结束，按 MOS 开关请求同步（保留 CC_EN）。 */
        if ((s_bq.request_fet_mask & 0x03u) != s_bq.fet_mask_applied)
        {
            s_bq.fet_job.op = (uint8_t)BQ_OP_WRITE;
            s_bq.fet_job.reg = BQ_REG_SYS_CTRL2;
            s_bq.fet_job.data = (uint8_t)(BQ_SYS_CTRL2_CC_EN |
                                          (s_bq.request_fet_mask & 0x03u));
            s_bq.fet_sync_active = 1u;
            Bq_StartScript(&s_bq.fet_job, 1u, BQ_ST_RUN);
        }
    }
    else
    {
        /* BACKOFF 等状态下的脚本收尾不应发生。 */
    }
}

/* 等上一笔事务的 STOP 真正结束并清掉 STOPF：如果 STOPF 落在下一笔 RX 里，
 * HAL 会把它当成 RX 结束，清掉 RXDMAEN，DMA 的 CNDTR 不再前进（实测 timeout）。 */
static void Bq_WaitBusIdle(void)
{
    uint32_t t0 = HAL_GetTick();

    while (((hi2c1.Instance->ISR & I2C_ISR_BUSY) != 0u) &&
           ((HAL_GetTick() - t0) < 2u))
    {
    }
    if ((hi2c1.Instance->ISR & I2C_ISR_BUSY) == 0u)
    {
        __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
    }
}

static void Bq_StartNextJob(void)
{
    const bq_job_t *job;

    if (s_bq.script == NULL)
    {
        return;
    }

    Bq_WaitBusIdle();
    if (s_bq.job_index >= s_bq.script_len)
    {
        Bq_ScriptComplete();
        return;
    }

    job = &s_bq.script[s_bq.job_index];
    s_bq.current_op = (bq_op_t)job->op;
    s_bq.current_reg = job->reg;
    s_bq.data_expected = (job->op == (uint8_t)BQ_OP_READ2) ? 2u : 1u;

    /* HAL 的 DevAddress 会先发 0x10，tx_buf 从寄存器开始，不能含地址。 */
    s_bq.tx_buf[0] = job->reg;

    if (job->op == (uint8_t)BQ_OP_WRITE)
    {
        s_bq.tx_buf[1] = job->data;
        s_bq.tx_buf[2] = Bq_Crc8_Write(job->reg, job->data);
        s_bq.xfer_phase = 1u;
        s_bq.xfer_start_ms = HAL_GetTick();
        if (HAL_I2C_Master_Transmit_DMA(&hi2c1, BQ_I2C_ADDR_WRITE,
                                        s_bq.tx_buf, 3u) != HAL_OK)
        {
            Bq_TransferFailed();
        }
    }
    else
    {
        /* 只发 [寄存器]，无 CRC；随后 Stop→Start 读数据。 */
        s_bq.xfer_phase = 1u;
        s_bq.xfer_start_ms = HAL_GetTick();
        if (HAL_I2C_Master_Transmit_DMA(&hi2c1, BQ_I2C_ADDR_WRITE,
                                        s_bq.tx_buf, 1u) != HAL_OK)
        {
            Bq_TransferFailed();
        }
    }
}

static void Bq_StartScript(const bq_job_t *script, uint8_t len, bq_state_t state)
{
    s_bq.script = script;
    s_bq.script_len = len;
    s_bq.job_index = 0u;
    s_bq.state = state;
    s_bq.state_enter_ms = HAL_GetTick();
    Bq_StartNextJob();
}

static void Bq_StartConfig(void)
{
    s_bq.cc_cfg_readback = 0u;
    Bq_StartScript(s_bq_cfg_script,
                   (uint8_t)(sizeof(s_bq_cfg_script) / sizeof(s_bq_cfg_script[0])),
                   BQ_ST_CONFIG);
}

static void Bq_StartRunScript(void)
{
    s_bq.script = s_bq_run_script;
    s_bq.script_len = (uint8_t)(sizeof(s_bq_run_script) / sizeof(s_bq_run_script[0]));
    s_bq.job_index = 0u;
    s_bq.state_enter_ms = HAL_GetTick();
    Bq_StartNextJob();
}

/* ---------------- 中断完成处理（只置标志） ---------------- */
static void Bq_HandleTxDone(void)
{
    if (s_bq.xfer_phase != 1u)
    {
        return;
    }

    /* TX 阶段结束（写数据或读指针）。必须清零：否则最后一笔写完成配置后
     * xfer_phase 仍为 1，100 ms 看门狗会误判超时，配置永远停在 BACKOFF。 */
    s_bq.xfer_phase = 0u;

    if (s_bq.current_op == BQ_OP_WRITE)
    {
        Bq_JobComplete();
        return;
    }

    /* 读事务：先等指针写的 STOP 真正结束并清 STOPF，再启动数据接收。 */
    Bq_WaitBusIdle();
    s_bq.xfer_phase = 2u;
    s_bq.xfer_start_ms = HAL_GetTick();
    if (HAL_I2C_Master_Receive_DMA(&hi2c1, BQ_I2C_ADDR_WRITE, s_bq.rx_buf,
                                   (uint16_t)(2u * s_bq.data_expected)) != HAL_OK)
    {
        Bq_TransferFailed();
    }
}

static void Bq_HandleRxDone(void)
{
    uint8_t ok = 1u;
    uint8_t count = s_bq.data_expected;

    if (s_bq.xfer_phase != 2u)
    {
        return;
    }
    s_bq.xfer_phase = 0u;

    if (count == 1u)
    {
        if (s_bq.rx_buf[1] != Bq_Crc8_ReadFirst(s_bq.rx_buf[0]))
        {
            ok = 0u;
        }
    }
    else
    {
        if (s_bq.rx_buf[1] != Bq_Crc8_ReadFirst(s_bq.rx_buf[0]))
        {
            ok = 0u;
        }
        if (s_bq.rx_buf[3] != Bq_Crc8_ReadNext(s_bq.rx_buf[2]))
        {
            ok = 0u;
        }
    }

    if (ok != 0u)
    {
        Bq_HandleReadData(s_bq.current_reg, s_bq.rx_buf, count);
        Bq_JobComplete();
    }
    else
    {
        Bq_TransferFailed();
    }
}

/* ---------------- 公共 API ---------------- */
void BQ_Init(void)
{
    s_bq.state = BQ_ST_RESET;
    s_bq.state_enter_ms = 0u;
    s_bq.now_ms = 0u;
    s_bq.script = NULL;
    s_bq.script_len = 0u;
    s_bq.job_index = 0u;
    s_bq.current_op = BQ_OP_READ1;
    s_bq.current_reg = 0u;
    s_bq.data_expected = 1u;
    s_bq.irq_tx_done = 0u;
    s_bq.irq_rx_done = 0u;
    s_bq.irq_error = 0u;
    s_bq.xfer_phase = 0u;
    s_bq.xfer_start_ms = 0u;
    s_bq.request_fet_mask = 0x03u;
    s_bq.fet_mask_applied = 0u;
    s_bq.fet_sync_active = 0u;
    s_bq.fet_job.op = 0u;
    s_bq.fet_job.reg = 0u;
    s_bq.fet_job.data = 0u;
    s_bq.current_ma = 0;
    s_bq.retry = 0u;
    s_bq.cfg_attempts = 0u;
    s_bq.backoff_return_state = BQ_ST_CONFIG;
    s_bq.backoff_kind = 1u;
    s_bq.backoff_ms = 0u;
    s_bq.req_read = 0u;
    s_bq.fresh = 0u;
    s_bq.last_update_ms = 0u;
    s_bq.sys_stat = 0u;
    s_bq.cc_cfg_readback = 0u;
    s_bq.adc_gain1_raw = 0u;
    s_bq.adc_gain2_raw = 0u;
    s_bq.adc_gain = 0u;
    s_bq.adc_offset = 0;
    s_bq.raw_cell[0] = 0u;
    s_bq.raw_cell[1] = 0u;
    s_bq.raw_cell[2] = 0u;
    s_bq.raw_pack = 0u;
    s_bq.raw_ts1 = 0u;
    s_bq.cc_raw = 0;
    s_bq.cell_mv[0] = 0u;
    s_bq.cell_mv[1] = 0u;
    s_bq.cell_mv[2] = 0u;
    s_bq.pack_mv = 0u;

    Bq_HwInit();

    /* PB1 高脉冲 ≥2 ms 保证从 SHIP 启动；boot 后必须回低（否则短路 NTC）。 */
    HAL_GPIO_WritePin(BQ_BOOT_GPIO_Port, BQ_BOOT_Pin, GPIO_PIN_SET);
    s_bq.state = BQ_ST_BOOT_PULSE;
    s_bq.state_enter_ms = HAL_GetTick();
}

void BQ_ReInitAfterLp(void)
{
    s_bq.xfer_phase = 0u;
    s_bq.irq_tx_done = 0u;
    s_bq.irq_rx_done = 0u;
    s_bq.irq_error = 0u;
    s_bq.retry = 0u;
    s_bq.req_read = 0u;

    Bq_HwInit();
    Bq_StartConfig();
}

void BQ_DeInitForLp(void)
{
    s_bq.script = NULL;
    s_bq.xfer_phase = 0u;

    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);

    (void)HAL_DMA_DeInit(&hdma_i2c1_tx);
    (void)HAL_DMA_DeInit(&hdma_i2c1_rx);
    (void)HAL_I2C_DeInit(&hi2c1);
}

int BQ_PrepareForLp(void)
{
    uint8_t stat = 0u;

    if (BQ_IsIdle() == 0u)
    {
        return -1;
    }

    /* 关 ADC 与 TS1/NTC 偏置（SYS_CTRL1=0x00）：LP 期间无电压/电流/温度检测，
     * BQ 由 60 µA 档降到 40 µA 档；OV/UV 保护随 ADC 停，SCD/OCD 仍有效。
     * 唤醒后由配置序列恢复 SYS_CTRL1=0x18（ADC_EN|TEMP_SEL）。 */
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_LP) != 0)
    {
        return -1;
    }

    /* CC_EN=0，保持 DSG_ON/CHG_ON；再清 SYS_STAT（含 CC_READY）防 ALERT 唤醒。 */
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2, BQ_SYS_CTRL2_LP) != 0)
    {
        return -1;
    }
    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_STAT, 0xFFu) != 0)
    {
        return -1;
    }

    s_bq.sys_stat = 0u;
    s_bq.req_read = 0u;
    s_bq.fet_mask_applied = 0x03u;   /* LP 写 SYS_CTRL2=0x03，两颗 MOS 保持导通 */
    return 0;
}

void BQ_RequestRead(void)
{
    s_bq.req_read = 1u;
}

void BQ_Poll(void)
{
    s_bq.now_ms = HAL_GetTick();

    /* 1) 事务看门狗：DMA/中断丢失时强制恢复总线。 */
    if ((s_bq.xfer_phase != 0u) &&
        ((s_bq.now_ms - s_bq.xfer_start_ms) >= BQ_TRANSFER_TIMEOUT_MS))
    {
        /* 总线可能已挂死：重建 I2C + DMA 并重跑配置。 */
        s_bq.retry = 0u;
        Fault_SetSticky(FAULT_BIT_BQ_COMM);
        Bq_RecoverBus();
        Bq_ScheduleRetry(0u);
    }

    /* 2) 处理中断标志（HAL 回调只置位）。 */
    if (s_bq.irq_error != 0u)
    {
        s_bq.irq_error = 0u;
        s_bq.irq_tx_done = 0u;
        s_bq.irq_rx_done = 0u;
        s_bq.xfer_phase = 0u;
        Bq_TransferFailed();
    }
    else if (s_bq.irq_tx_done != 0u)
    {
        s_bq.irq_tx_done = 0u;
        Bq_HandleTxDone();
    }
    else if (s_bq.irq_rx_done != 0u)
    {
        s_bq.irq_rx_done = 0u;
        Bq_HandleRxDone();
    }
    else
    {
        /* 无事件 */
    }

    /* 3) 状态机推进（Boot 时序、退避、运行轮询触发）。 */
    switch (s_bq.state)
    {
    case BQ_ST_BOOT_PULSE:
        if ((s_bq.now_ms - s_bq.state_enter_ms) >= 2u)
        {
            HAL_GPIO_WritePin(BQ_BOOT_GPIO_Port, BQ_BOOT_Pin, GPIO_PIN_RESET);
            s_bq.state = BQ_ST_BOOT_WAIT;
            s_bq.state_enter_ms = s_bq.now_ms;
        }
        break;

    case BQ_ST_BOOT_WAIT:
        if ((s_bq.now_ms - s_bq.state_enter_ms) >= 20u)
        {
            Bq_StartConfig();
        }
        break;

    case BQ_ST_CONFIG:
        if ((s_bq.script == NULL) && (s_bq.xfer_phase == 0u) &&
            (s_bq.state == BQ_ST_CONFIG))
        {
            /* 脚本已收尾但未切到 RUN：配置未完成（不应发生），重试。 */
            Bq_StartConfig();
        }
        break;

    case BQ_ST_RUN:
        if ((s_bq.script == NULL) && (s_bq.req_read != 0u) && (s_bq.xfer_phase == 0u))
        {
            Bq_StartRunScript();
        }
        break;

    case BQ_ST_BACKOFF:
        if ((s_bq.now_ms - s_bq.state_enter_ms) >= s_bq.backoff_ms)
        {
            if (s_bq.backoff_kind == 0u)
            {
                s_bq.state = s_bq.backoff_return_state;
                Bq_StartNextJob();
            }
            else
            {
                Bq_StartConfig();
            }
        }
        break;

    default:
        break;
    }
}

uint8_t BQ_IsFresh(uint32_t now_ms)
{
    if (s_bq.fresh == 0u)
    {
        return 0u;
    }
    return ((now_ms - s_bq.last_update_ms) <= APP_BQ_STALE_MS) ? 1u : 0u;
}

void BQ_GetSnapshot(bq_snapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->cell_mv[0] = s_bq.cell_mv[0];
    out->cell_mv[1] = s_bq.cell_mv[1];
    out->cell_mv[2] = s_bq.cell_mv[2];
    out->pack_mv = s_bq.pack_mv;
    out->cc_raw = s_bq.cc_raw;
    out->current_ma = s_bq.current_ma;
    out->ts1_raw = s_bq.raw_ts1;
    out->sys_stat = s_bq.sys_stat;
    out->adc_gain = s_bq.adc_gain;
    out->adc_offset = s_bq.adc_offset;
    out->fresh = s_bq.fresh;
    out->last_update_ms = s_bq.last_update_ms;
}

bq_state_t BQ_GetState(void)
{
    return s_bq.state;
}

uint8_t BQ_IsIdle(void)
{
    return ((s_bq.script == NULL) && (s_bq.xfer_phase == 0u)) ? 1u : 0u;
}

/* ---------------- IRQ / HAL 回调（全库唯一 owner） ---------------- */
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_bq.irq_tx_done = 1u;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_bq.irq_rx_done = 1u;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_bq.irq_error = 1u;
    }
}

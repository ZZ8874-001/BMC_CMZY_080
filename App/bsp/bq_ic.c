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
#include "bsp/button.h"

#include <stddef.h>
#include <string.h>


/* ---------------- 寄存器地址/位定义见 bq_ic.h ---------------- */

#define BQ_CC_CFG_VALUE     0x19u   /* 数据手册要求启动时写入 CC_CFG<5:0> = 0x19 */
/* 实测 CC_CFG 读回可能是 0x59 或 0x79（bit5 不稳定/固定位），低 5 位始终是 0x19；
 * 比较时只看 CC_CFG<4:0>，bit5/6 不参与判定。 */
#define BQ_CC_CFG_MASK      0x1Fu
#define BQ_SYS_CTRL1_VALUE  0x18u   /* ADC_EN | TEMP_SEL（NORMAL 下 ADC 必须保持使能） */
#define BQ_SYS_CTRL1_LP     0x00u   /* LP：清 ADC_EN|TEMP_SEL → 电压/电流/温度检测停，
                                       OV/UV 保护随之停；SCD/OCD 比较器不可关、仍然有效 */
/* 只清 CC_READY、DEVICE_XREADY 和 OVRD_ALERT；OV/UV/SCD/OCD 保护锁存保留。
 * OVRD_ALERT 不是保护故障，但会把 CHG/DSG 驱动强制为 0，必须定期清掉，
 * 否则一次 ALERT 引脚毛刺就会跨 MCU 复位永久锁存。 */
#define BQ_SYS_STAT_IRQ_CLR  (BQ_SYS_STAT_CC_READY | BQ_SYS_STAT_XREADY | \
                              BQ_SYS_STAT_OVRD_ALERT)
/* VCx/TS1 是 14 位 ADC：高字节 bit7:6 为保留位，必须屏蔽后再换算。 */
#define BQ_CELL_ADC_MASK    0x3FFFu

/* 保护阈值（3S 三元锂 NMC 1500 mAh / 1.5 mΩ 分流，数据手册 Table 8-9..8-13）：
 *   PROTECT1: RSNS=1, SCD 67 mV(44.7 A)/70 µs（SCD_D=00）
 *   PROTECT2: OCD 56 mV(37.3 A)/320 ms（仅放电方向）
 *   PROTECT3: OV delay 1 s, UV delay 4 s
 *   OV/UV trip 按 ADC gain/offset 在配置完成时计算。
 * NMC 典型保护值取 OV 4.25 V/cell、UV 2.80 V/cell（充电上限 4.20 V，
 * 留 50 mV 防 CV 末端误触发；UV 为硬件最后保护，软件 5%/0% 更早）。
 * BQ769x0 没有充电过流硬件保护；OCD/SCD 只作用于放电。 */
#define BQ_PROTECT1_VALUE   0x89u
#define BQ_PROTECT2_VALUE   0x57u
#define BQ_PROTECT3_VALUE   0x40u
#define BQ_OV_TRIP_MV       4250
#define BQ_UV_TRIP_MV       2800

#define BQ_TRANSFER_TIMEOUT_MS  100u
#define BQ_BLOCKING_TIMEOUT_MS  50u
#define BQ_RETRY_LIMIT          3u
#define BQ_RETRY_BACKOFF_MS     20u
#define BQ_FAIL_BACKOFF_MS      2000u
#define BQ_CFG_RETRY_LIMIT      3u

/* VC1/VC2 控制合理性与日志可信门限。
 * PACK_* 只用于判断真实 BAT 是否可写入黑匣子/StateStore，不再参与控制。 */
#define BQ_CELL_VALID_MIN_MV    1000
#define BQ_CELL_VALID_MAX_MV    5000
#define BQ_PACK_VALID_MIN_MV    3000
#define BQ_PACK_VALID_MAX_MV    15000
#define BQ_PACK_SUM_TOL_MV      300
/* OV/UV 归因/清锁存余量。VC5 仍可能触发 BQ 的全局 OV/UV；只要实测
 * VC1/VC2 已经明显在阈值安全侧，就把该锁存归因于不可信 VC5。
 * 20 mV 只覆盖 ADC/换算误差，避免旧的 50/100 mV 余量把临近阈值的
 * 真实 cell1/2 误判成“安全”或反之。 */
#define BQ_OV_CLEAR_GUARD_MV    20
#define BQ_UV_CLEAR_GUARD_MV    20

/* 受控 ADC 重启/恢复。 */
#define BQ_ADC_CONVERSION_MS             250u  /* BQ ADC 帧周期，采样最小间隔 */
#define BQ_ADC_RESAMPLE_MS               500u  /* OOR 后重采样间隔（250 + 余量） */
#define BQ_ADC_RESTART_OFF_HOLD_MS       800u
#define BQ_ADC_RESTART_MAX_ATTEMPTS      2u
#define BQ_ADC_VERDICT_MS                1500u /* 每轮重启后等待有效发布的窗口 */
#define BQ_ADC_GOOD_FRAMES_CLEAR         4u    /* 连续有效发布帧数 → 清故障/额度 */
#define BQ_FRAME_CONFIRM_RAW_LSB         4u    /* 首次发布：每寄存器 ±4 raw LSB */
#define BQ_STRICT_MAX_MISMATCHES         3u    /* 连续不严格一致后回退粗确认 */
#define BQ_FRAME_CONFIRM_CELL_DELTA_MV   250u  /* 普通拒绝后的粗确认 */
#define BQ_OOR_STREAK_RESTART            8u
#define BQ_CC_FRESH_MS                   1000u
#define BQ_BATCH_FAULT_MASK              (BQ_SYS_STAT_PROT_MASK | BQ_SYS_STAT_XREADY | \
                                          BQ_SYS_STAT_OVRD_ALERT)
#define BQ_CC_BATCH_BASE                 (BQ_FIELD_CC | BQ_FIELD_SYS_STAT | \
                                          BQ_FIELD_CTRL1 | BQ_FIELD_CTRL2 | \
                                          BQ_FIELD_PROTECTION)

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

/* 配置序列：读回增益/偏移供电压换算；最后只开 CC，FET 默认全关，
 * 由状态机/调试变量显式请求。 */
static const bq_job_t s_bq_cfg_script[] =
{
    {BQ_OP_WRITE, BQ_REG_CC_CFG,    BQ_CC_CFG_VALUE},
    {BQ_OP_READ1, BQ_REG_CC_CFG,    0u},
    {BQ_OP_READ1, BQ_REG_ADCGAIN1,  0u},
    {BQ_OP_READ1, BQ_REG_ADCGAIN2,  0u},
    {BQ_OP_READ1, BQ_REG_ADCOFFSET, 0u},
    {BQ_OP_WRITE, BQ_REG_SYS_STAT,   BQ_SYS_STAT_IRQ_CLR}
};

/* 运行轮询：先读状态（清 CC_READY/XREADY），再读 3 节电压、总压、TS1、CC。 */
static const bq_job_t s_bq_run_script[] =
{
    {BQ_OP_READ1, BQ_REG_SYS_STAT,  0u},
    {BQ_OP_WRITE, BQ_REG_SYS_STAT,  BQ_SYS_STAT_IRQ_CLR},
    {BQ_OP_READ2, BQ_REG_VC1,       0u},
    {BQ_OP_READ2, BQ_REG_VC2,       0u},
    {BQ_OP_READ2, BQ_REG_VC5,       0u},  /* optional raw VC5 诊断，不参与控制 */
    {BQ_OP_READ2, BQ_REG_VC3,       0u},  /* optional raw 诊断 */
    {BQ_OP_READ2, BQ_REG_VC4,       0u},  /* optional raw 诊断 */
    {BQ_OP_READ2, BQ_REG_BAT,       0u},  /* optional raw BAT 诊断，不参与控制 */
    {BQ_OP_READ2, BQ_REG_TS1,       0u},
    {BQ_OP_READ2, BQ_REG_CC,        0u},
    {BQ_OP_READ1, BQ_REG_SYS_CTRL1, 0u},
    {BQ_OP_READ1, BQ_REG_SYS_CTRL2, 0u},
    {BQ_OP_READ1, BQ_REG_PROTECT1,  0u},
    {BQ_OP_READ1, BQ_REG_PROTECT2,  0u},
    {BQ_OP_READ1, BQ_REG_PROTECT3,  0u},
    {BQ_OP_READ1, BQ_REG_SYS_STAT,  0u} /* 末尾再读，防批次期间保护位改变 */
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

    /* 运行时 MOS 开关请求。bit0=CHG，bit1=DSG；默认全关（0x00）。
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

    uint8_t sys_stat;           /* 过滤后的有效 SYS_STAT */
    uint8_t raw_sys_stat;       /* 原始 SYS_STAT，仅诊断 */
    uint8_t cc_cfg_readback;
    uint8_t adc_gain1_raw;
    uint8_t adc_gain2_raw;
    uint8_t adc_gain;
    int8_t adc_offset;
    uint8_t protect1;
    uint8_t protect2;
    uint8_t protect3;
    uint8_t ov_trip;
    uint8_t uv_trip;
    uint8_t protection_ok;
    uint8_t boot_prot_latch_mask;
    uint8_t boot_prot_capture_armed;

    uint16_t raw_cell[3];
    uint16_t raw_pack;
    uint16_t raw_ts1;
    uint16_t raw_vc3;
    uint16_t raw_vc4;
    int16_t cc_raw;
    int16_t published_cc_raw;
    uint16_t published_ts1_raw;

    uint16_t cell_mv[3];        /* [2] 真实 VC5 可信值，否则两节均值 */
    uint16_t pack_mv;           /* 真实 BAT 可信值，否则三节估算和 */
    uint16_t log_cell3_mv;      /* 真实 VC5 可信值，否则 0 */
    uint16_t log_pack_mv;       /* 真实 BAT 可信值，否则 0 */
    uint8_t log_cell3_valid;
    uint8_t log_pack_valid;
    uint8_t ref_cell2_only;     /* 已发布：VC5/BAT 不可信，保护/SOC 只看 VC2 */
    uint8_t run_ref_cell2_only; /* 当前 RUN 批次的过滤决策 */
    uint8_t ref_decision_final; /* 当前批次 VC5/BAT 证据已确认 */
    uint8_t cell12_meas_valid;  /* 最近一次 VC1/VC2 换算值在 1..5 V */
    uint8_t cell12_measured_once;
    uint16_t cell12_meas_mv[2]; /* 最近一次 VC1/VC2 实测 mV（OV/UV 过滤用） */
    uint8_t voltage_settled;    /* BQ_ST_SETTLE 到期后置 1 */
    uint8_t frame_valid;        /* 最近一次完整帧通过校验 */
    uint8_t reject_reason;      /* bq_frame_reject_t */
    uint16_t max_cell_mv;
    uint16_t min_cell_mv;
    uint16_t ov_trip_mv;        /* 由实际写回的 OV_TRIP 反算 */
    uint16_t uv_trip_mv;
    uint32_t frame_ok_count;
    uint32_t frame_reject_count;
    uint32_t reject_unsettled_count;
    uint32_t reject_cell_count;
    uint16_t reject_streak;
    uint16_t reject_since_publish;
    uint16_t last_reject_cell_mv[3];
    uint16_t last_reject_pack_mv;
    uint16_t last_reject_vc3_raw;
    uint16_t last_reject_vc4_raw;
    uint32_t last_reject_ms;
    uint8_t last_reject_reason;
    uint8_t last_reject_sys_stat;

    /* 首帧必须连续两帧一致才发布。 */
    uint8_t frame_confirm_count;
    uint8_t strict_window;      /* 1=启动/LP/ADC 重启后的首次发布用 ±4 raw LSB */
    uint8_t strict_mismatch_count;
    uint16_t pending_cell_mv[3];
    uint16_t pending_raw_cell[3];
    uint32_t frame_confirm_reset_count;

    /* OOR 快速重采样。 */
    uint16_t oor_streak;
    uint16_t resample_count;
    uint8_t resample_pending;
    uint32_t resample_due_ms;
    uint32_t last_sample_ms;

    /* 受控 ADC 重启/恢复。 */
    uint8_t pre_ctrl1;
    uint8_t pre_ctrl2;
    uint8_t pre_sys_stat;
    uint8_t pre_cellbal1;
    uint8_t post_cellbal1;      /* 每次开 ADC/进 LP 后回读，必须为 0 */
    uint8_t pre_snapshot_valid;
    uint8_t post_ctrl1;
    uint8_t post_ctrl2;
    uint8_t post_sys_stat;
    uint8_t settle_sys_stat;
    uint8_t adc_off_ctrl1;
    uint8_t adc_off_ctrl2;
    uint8_t xready_at_settle;
    uint16_t xready_count;
    uint8_t xready_streak;
    uint8_t adc_restart_pending;
    uint8_t adc_restart_reason;
    uint8_t adc_restart_attempts;
    uint8_t adc_recovery_failed;
    uint8_t recovery_episode;
    uint8_t verdict_arm_pending;
    uint8_t verdict_active;
    uint32_t verdict_deadline_ms;
    uint16_t good_publish_streak;
    uint8_t adc_event_flags;
    uint32_t adc_restart_count;
    uint32_t adc_restart_last_ms;

    /* RUN 全量脚本逐寄存器记录。批次完成后复制到 batch；DMA 出错/CRC
     * 重试将本批标记为失败，不能把前一次原始值冒充同批有效证据。 */
    uint16_t run_crc_mask;
    uint16_t run_format_mask;
    uint8_t run_failed;
    uint8_t run_status_reads;
    uint8_t run_protect_reads;
    uint8_t run_stat_first;
    uint8_t run_stat_last;
    uint8_t run_ctrl1;
    uint8_t run_ctrl2;
    uint8_t run_protect[3];
    uint32_t run_start_ms;
    bq_batch_quality_t batch;
    bq_cc_sample_t cc_sample;
    uint32_t cc_last_start_ms;

    volatile uint8_t pending_sysstat_clear; /* 只清 OV/UV 的延迟位 */
    uint8_t sysstat_sync_active;
    bq_job_t sysstat_job;
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
static void Bq_RequestAdcRestart(uint8_t reason);
static void Bq_LatchAdcFault(void);
static void Bq_ConfigFailed(void);
static int Bq_PreConfigSnapshot(void);
static int Bq_ForceAdcOff(void);
static int Bq_EnableAdcVerified(void);
static int Bq_VerifySettleStatus(void);

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

/* 连续读 1..8 个寄存器，返回数据字节（丢弃每字节 CRC）。 */
static int Bq_BlockingReadBlock(uint8_t reg, uint8_t *out, uint8_t count)
{
    uint8_t ptr[1];
    uint8_t rx[16];
    uint8_t i;

    if ((out == NULL) || (count == 0u) || (count > 8u))
    {
        return -1;
    }

    ptr[0] = reg;
    if (HAL_I2C_Master_Transmit(&hi2c1, BQ_I2C_ADDR_WRITE, ptr, 1u,
                                BQ_BLOCKING_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, BQ_I2C_ADDR_WRITE, rx,
                               (uint16_t)(2u * count),
                               BQ_BLOCKING_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    if (rx[1] != Bq_Crc8_ReadFirst(rx[0]))
    {
        return -1;
    }
    for (i = 1u; i < count; i++)
    {
        if (rx[(2u * i) + 1u] != Bq_Crc8_ReadNext(rx[2u * i]))
        {
            return -1;
        }
    }
    for (i = 0u; i < count; i++)
    {
        out[i] = rx[2u * i];
    }
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

/* 控制电压逐字段选择：真实 VC5/BAT 可信时优先实测，否则用 VC1/VC2 估算。
 * pack 估算必须使用最终选中的 cell3，保证 pack == c1 + c2 + c3；
 * 记录值不在这里改写，仍由 log_cell3_mv/log_pack_mv 保持 real-or-0。 */
static void Bq_SelectControlVoltages(uint16_t c1, uint16_t c2,
                                     uint16_t real_c3, uint8_t c3_valid,
                                     uint16_t real_pack, uint8_t pack_valid,
                                     uint16_t *cell3_mv, uint16_t *pack_mv)
{
    uint16_t est_c3 = (uint16_t)(((uint32_t)c1 + (uint32_t)c2 + 1u) / 2u);
    uint16_t c3 = (c3_valid != 0u) ? real_c3 : est_c3;
    uint16_t pack;

    if ((pack_valid != 0u) && (c3_valid != 0u))
    {
        pack = real_pack;
    }
    else
    {
        pack = (uint16_t)((uint32_t)c1 + (uint32_t)c2 + (uint32_t)c3);
    }
    *cell3_mv = c3;
    *pack_mv = pack;
}

/* 计算真实 VC5/BAT 的记录值，并返回“保护/SOC 是否应只使用 VC2”。
 * 这里只使用本批 raw/CRC 证据，绝不读取 sys_stat/quality，避免过滤环。 */
static uint8_t Bq_EvaluateRealC3Pack(uint16_t raw_c3, uint16_t raw_pack,
                                     uint8_t c3_field_ok, uint8_t pack_field_ok,
                                     uint16_t c1, uint16_t c2,
                                     uint16_t *log_cell3_mv, uint16_t *log_pack_mv,
                                     uint8_t *log_cell3_valid, uint8_t *log_pack_valid)
{
    uint32_t real_c3 = Bq_RawToMv(raw_c3);
    uint32_t real_pack = Bq_RawPackToMv(raw_pack);
    int32_t residual;

    *log_cell3_mv = 0u;
    *log_pack_mv = 0u;
    *log_cell3_valid = 0u;
    *log_pack_valid = 0u;

    if ((c3_field_ok != 0u) &&
        (real_c3 >= BQ_CELL_VALID_MIN_MV) &&
        (real_c3 <= BQ_CELL_VALID_MAX_MV))
    {
        *log_cell3_valid = 1u;
        *log_cell3_mv = (uint16_t)real_c3;
    }
    if ((*log_cell3_valid != 0u) && (pack_field_ok != 0u) &&
        (real_pack >= BQ_PACK_VALID_MIN_MV) &&
        (real_pack <= BQ_PACK_VALID_MAX_MV))
    {
        residual = (int32_t)real_pack -
                   ((int32_t)c1 + (int32_t)c2 + (int32_t)real_c3);
        if ((residual <= BQ_PACK_SUM_TOL_MV) &&
            (residual >= -BQ_PACK_SUM_TOL_MV))
        {
            *log_pack_valid = 1u;
            *log_pack_mv = (uint16_t)real_pack;
        }
    }

    return (uint8_t)((*log_cell3_valid == 0u) || (*log_pack_valid == 0u));
}

/* SCD/OCD 是硬件比较器，和 ADC 建立无关，任何 SYS_STAT 读取都要立即更新。 */
static void Bq_UpdateFastFaults(uint8_t stat)
{
    uint16_t live = 0u;

    if ((stat & BQ_SYS_STAT_SCD) != 0u)
    {
        live |= FAULT_BIT_BQ_SCD;
    }
    if ((stat & BQ_SYS_STAT_OCD) != 0u)
    {
        live |= FAULT_BIT_BQ_OCD;
    }

    Fault_ClearLive(FAULT_BIT_BQ_SCD | FAULT_BIT_BQ_OCD);
    Fault_SetLive(live);
}

/* 有效 SYS_STAT 的 OV/UV/XREADY 映射。OV/UV 已经由 Bq_FilterHwOvUv()
 * 按实测 VC1/VC2 过滤；XREADY 与 cell3/pack 无关，始终保留。 */
static void Bq_UpdateVoltageFaults(uint8_t stat)
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

    Fault_ClearLive(FAULT_BIT_BQ_XREADY | FAULT_BIT_BQ_OV | FAULT_BIT_BQ_UV);
    Fault_SetLive(live);
}

/* BQ 的 OV/UV 是全局锁存，无法区分哪一节；本板 VC5/BAT 还可能异常。
 * 规则：
 *   - 首个完整 VC1/VC2 读取前：屏蔽 OV/UV（硬件 FET 保护仍生效）。
 *   - VC5/BAT 不可信（run_ref_cell2_only）：只校验 VC2；VC2 在 1..5 V 时
 *     按 VC2 判断 OV/UV，VC2 无效则保留原始锁存。
 *   - 正常模式：VC1/VC2 都在 1..5 V 时，OV 侧 max(c1,c2) <= trip-20 mV、
 *     UV 侧 min(c1,c2) >= trip+20 mV 才屏蔽对应位。
 * 锁存归属只用 VC1/VC2 作为可验证证据；真实 VC5 不参与该过滤，BQ 硬件
 * 比较器仍会在真实过压/欠压时动作并重新锁存。 */
static uint8_t Bq_FilterHwOvUv(uint8_t raw)
{
    uint8_t eff = raw;
    uint16_t c1;
    uint16_t c2;
    uint8_t have_cells = 0u;

    if ((raw & (BQ_SYS_STAT_OV | BQ_SYS_STAT_UV)) == 0u)
    {
        return raw;
    }

    /* 优先用当前批 VC1/VC2 实测；当前批无效/不可信时退回最近一次已发布
     * 的 cell1/2（该值已经过发布校验）。两者都没有才进入启动宽限。 */
    if ((s_bq.cell12_measured_once != 0u) && (s_bq.cell12_meas_valid != 0u))
    {
        c1 = s_bq.cell12_meas_mv[0];
        c2 = s_bq.cell12_meas_mv[1];
        have_cells = 1u;
    }
    else if (s_bq.frame_valid != 0u)
    {
        c1 = s_bq.cell_mv[0];
        c2 = s_bq.cell_mv[1];
        have_cells = 1u;
    }

    if (have_cells == 0u)
    {
        /* 首个有效 cell1/2 批次前：不能把 VC5 假锁存当故障上报。 */
        return (uint8_t)(raw & (uint8_t)~(BQ_SYS_STAT_OV | BQ_SYS_STAT_UV));
    }

    if (s_bq.run_ref_cell2_only != 0u)
    {
        /* cell3/pack 异常时只信任 VC2；VC2 无效必须保留原始锁存。 */
        if ((c2 < BQ_CELL_VALID_MIN_MV) || (c2 > BQ_CELL_VALID_MAX_MV))
        {
            return raw;
        }
        if (c2 <=
            ((s_bq.ov_trip_mv > BQ_OV_CLEAR_GUARD_MV) ?
             (uint16_t)(s_bq.ov_trip_mv - BQ_OV_CLEAR_GUARD_MV) : 0u))
        {
            eff &= (uint8_t)~BQ_SYS_STAT_OV;
        }
        if ((uint32_t)c2 >=
            ((uint32_t)s_bq.uv_trip_mv + BQ_UV_CLEAR_GUARD_MV))
        {
            eff &= (uint8_t)~BQ_SYS_STAT_UV;
        }
        return eff;
    }

    if ((c1 < BQ_CELL_VALID_MIN_MV) || (c1 > BQ_CELL_VALID_MAX_MV) ||
        (c2 < BQ_CELL_VALID_MIN_MV) || (c2 > BQ_CELL_VALID_MAX_MV))
    {
        /* 已发布值理论上不会越界；这里只在当前批/回退值都不可用时走到，
         * 无法证明安全，保留原始 OV/UV。 */
        return raw;
    }

    if (((c1 >= c2) ? c1 : c2) <=
        ((s_bq.ov_trip_mv > BQ_OV_CLEAR_GUARD_MV) ?
         (uint16_t)(s_bq.ov_trip_mv - BQ_OV_CLEAR_GUARD_MV) : 0u))
    {
        eff &= (uint8_t)~BQ_SYS_STAT_OV;
    }
    if ((uint32_t)((c1 <= c2) ? c1 : c2) >=
        ((uint32_t)s_bq.uv_trip_mv + BQ_UV_CLEAR_GUARD_MV))
    {
        eff &= (uint8_t)~BQ_SYS_STAT_UV;
    }
    return eff;
}

/* 用 raw_sys_stat 重新计算有效 sys_stat、故障位和待清锁存。
 * 只有本批 VC5/BAT 证据已最终确认、且当前参考节有效时，才安排清
 * SYS_STAT 的假 OV/UV；读过程中的暂态决策不得清真实锁存。 */
static void Bq_RefreshEffectiveSysStat(void)
{
    uint8_t raw = s_bq.raw_sys_stat;
    uint8_t eff = Bq_FilterHwOvUv(raw);

    s_bq.sys_stat = eff;
    if ((s_bq.ref_decision_final != 0u) &&
        (s_bq.cell12_measured_once != 0u) && (s_bq.cell12_meas_valid != 0u))
    {
        if (((raw & BQ_SYS_STAT_OV) != 0u) && ((eff & BQ_SYS_STAT_OV) == 0u))
        {
            s_bq.pending_sysstat_clear |= BQ_SYS_STAT_OV;
        }
        if (((raw & BQ_SYS_STAT_UV) != 0u) && ((eff & BQ_SYS_STAT_UV) == 0u))
        {
            s_bq.pending_sysstat_clear |= BQ_SYS_STAT_UV;
        }
    }
    Bq_UpdateFastFaults(eff);
    Bq_UpdateVoltageFaults(eff);
}

/* 仅 RUN 完整批次内、逐笔 CRC 通过后置位；不能把别的批次的 shadow
 * 或失败事务的部分值充作本批证据。状态读两次、保护读三次才算齐全。 */
static uint16_t Bq_RunFieldBit(uint8_t reg)
{
    switch (reg)
    {
    case BQ_REG_VC1: return BQ_FIELD_VC1;
    case BQ_REG_VC2: return BQ_FIELD_VC2;
    case BQ_REG_VC5: return BQ_FIELD_VC5;
    case BQ_REG_BAT: return BQ_FIELD_BAT;
    case BQ_REG_VC3: return BQ_FIELD_VC3;
    case BQ_REG_VC4: return BQ_FIELD_VC4;
    case BQ_REG_TS1: return BQ_FIELD_TS1;
    case BQ_REG_CC: return BQ_FIELD_CC;
    case BQ_REG_SYS_STAT: return BQ_FIELD_SYS_STAT;
    case BQ_REG_SYS_CTRL1: return BQ_FIELD_CTRL1;
    case BQ_REG_SYS_CTRL2: return BQ_FIELD_CTRL2;
    case BQ_REG_PROTECT1:
    case BQ_REG_PROTECT2:
    case BQ_REG_PROTECT3: return BQ_FIELD_PROTECTION;
    default: return 0u;
    }
}

static void Bq_RunReadVerified(uint8_t reg, const uint8_t *data, uint8_t count)
{
    uint16_t field = Bq_RunFieldBit(reg);

    if ((s_bq.state != BQ_ST_RUN) || (s_bq.script != s_bq_run_script))
    {
        return;
    }
    if (((field & (BQ_FIELD_VC1 | BQ_FIELD_VC2 |
                   BQ_FIELD_TS1 | BQ_FIELD_CC)) != 0u) && (count != 2u))
    {
        s_bq.run_failed = 1u;
        return;
    }
    if ((count == 2u) && (reg != BQ_REG_CC) &&
        ((data[0] & 0xC0u) != 0u))
    {
        /* 14-bit VC/TS1 寄存器保留高位不能当作可用码值。 */
        s_bq.run_format_mask |= field;
    }
    if (reg == BQ_REG_SYS_STAT)
    {
        if (s_bq.run_status_reads == 0u)
        {
            s_bq.run_stat_first = data[0];
        }
        else
        {
            s_bq.run_stat_last = data[0];
        }
        s_bq.run_status_reads++;
    }
    else if (reg == BQ_REG_SYS_CTRL1)
    {
        s_bq.run_ctrl1 = data[0];
    }
    else if (reg == BQ_REG_SYS_CTRL2)
    {
        s_bq.run_ctrl2 = data[0];
    }
    else if ((reg >= BQ_REG_PROTECT1) && (reg <= BQ_REG_PROTECT3))
    {
        s_bq.run_protect[reg - BQ_REG_PROTECT1] = data[0];
        s_bq.run_protect_reads++;
    }
    s_bq.run_crc_mask |= field;
}

static void Bq_HandleReadData(uint8_t reg, const uint8_t *data, uint8_t count)
{
    uint16_t value;

    /* READ2（2 字节寄存器）的 4 字节布局：[data_hi, CRC1, data_lo, CRC2]，
     * 低字节在 data[2]，data[1] 是第一个数据字节的 CRC。 */
    switch (reg)
    {
    case BQ_REG_SYS_STAT:
        /* 首次状态负责既有快速保护处理；末尾状态只补齐同批证据，
         * 但新发生的硬件保护/ XREADY 必须立即上报。 */
        if ((s_bq.script == s_bq_run_script) &&
            (s_bq.run_status_reads != 0u))
        {
            s_bq.raw_sys_stat |= data[0];
            Bq_RefreshEffectiveSysStat();
            if ((data[0] & BQ_SYS_STAT_XREADY) != 0u)
            {
                Fault_SetLive(FAULT_BIT_BQ_XREADY);
            }
            break;
        }
        s_bq.raw_sys_stat = data[0];
        Bq_RefreshEffectiveSysStat();
        if ((data[0] & BQ_SYS_STAT_XREADY) != 0u)
        {
            s_bq.xready_count++;
            if (s_bq.xready_streak < 0xFFu)
            {
                s_bq.xready_streak++;
            }
            if (s_bq.xready_streak >= 2u)
            {
                Bq_RequestAdcRestart((uint8_t)BQ_RESTART_REASON_XREADY);
            }
        }
        else
        {
            s_bq.xready_streak = 0u;
        }
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
            s_bq.raw_cell[0] = (uint16_t)((((uint16_t)data[0] << 8) |
                                           data[2]) & BQ_CELL_ADC_MASK);
        }
        break;

    case BQ_REG_VC2:
        if (count == 2u)
        {
            s_bq.raw_cell[1] = (uint16_t)((((uint16_t)data[0] << 8) |
                                           data[2]) & BQ_CELL_ADC_MASK);
            s_bq.cell12_meas_mv[0] = Bq_RawToMv(s_bq.raw_cell[0]);
            s_bq.cell12_meas_mv[1] = Bq_RawToMv(s_bq.raw_cell[1]);
            /* OV/UV 归因只看换算后的 cell1/2 是否在合理范围；14-bit
             * 保留高位只影响帧发布质量，不应让 UV 过滤退回原始状态。 */
            s_bq.cell12_meas_valid =
                ((s_bq.cell12_meas_mv[0] >= BQ_CELL_VALID_MIN_MV) &&
                 (s_bq.cell12_meas_mv[0] <= BQ_CELL_VALID_MAX_MV) &&
                 (s_bq.cell12_meas_mv[1] >= BQ_CELL_VALID_MIN_MV) &&
                 (s_bq.cell12_meas_mv[1] <= BQ_CELL_VALID_MAX_MV)) ? 1u : 0u;
            s_bq.cell12_measured_once = 1u;
            /* VC1/VC2 更新后重新决定 OV/UV 是否来自 VC5 假锁存。 */
            Bq_RefreshEffectiveSysStat();
        }
        break;

    case BQ_REG_VC5:
        if (count == 2u)
        {
            /* 原始 VC5：只作诊断，绝不参与控制/保护/发布。 */
            s_bq.raw_cell[2] = (uint16_t)((((uint16_t)data[0] << 8) |
                                           data[2]) & BQ_CELL_ADC_MASK);
        }
        break;

    case BQ_REG_VC3:
        if (count == 2u)
        {
            s_bq.raw_vc3 = (uint16_t)((((uint16_t)data[0] << 8) |
                                       data[2]) & BQ_CELL_ADC_MASK);
        }
        break;

    case BQ_REG_VC4:
        if (count == 2u)
        {
            s_bq.raw_vc4 = (uint16_t)((((uint16_t)data[0] << 8) |
                                       data[2]) & BQ_CELL_ADC_MASK);
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
            s_bq.raw_ts1 = (uint16_t)((((uint16_t)data[0] << 8) |
                                       data[2]) & BQ_CELL_ADC_MASK);
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

/* 由实际写入的 8 位 OV/UV middle-code 反算 mV，供假锁存安全判断使用。 */
static uint16_t Bq_TripFullToMv(uint16_t full)
{
    int32_t millivolts = ((int32_t)(365 + (int32_t)s_bq.adc_gain) *
                          (int32_t)full) / 1000 + (int32_t)s_bq.adc_offset;

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

static void Bq_UpdateTripThresholds(void)
{
    uint16_t ov_full = (uint16_t)(0x2000u |
                                  (((uint16_t)s_bq.ov_trip & 0xFFu) << 4) |
                                  0x8u);
    uint16_t uv_full = (uint16_t)(0x1000u |
                                  (((uint16_t)s_bq.uv_trip & 0xFFu) << 4));

    s_bq.ov_trip_mv = Bq_TripFullToMv(ov_full);
    s_bq.uv_trip_mv = Bq_TripFullToMv(uv_full);
}

/* 控制帧只校验 VC1/VC2 实测值；cell3/pack 可能是真实值或估算值，
 * 不参与校验。VC5/BAT/VC3/VC4 的原始读数异常不阻塞发布。 */
static uint8_t Bq_ValidateFrameValues(const uint16_t cells[3], uint8_t *reason)
{
    if ((cells[0] < BQ_CELL_VALID_MIN_MV) || (cells[0] > BQ_CELL_VALID_MAX_MV) ||
        (cells[1] < BQ_CELL_VALID_MIN_MV) || (cells[1] > BQ_CELL_VALID_MAX_MV))
    {
        *reason = (uint8_t)BQ_FRAME_REJECT_CELL_RANGE;
        return 0u;
    }

    *reason = (uint8_t)BQ_FRAME_OK;
    return 1u;
}

static uint8_t Bq_ValidateFrame(const uint16_t cells[3], uint8_t *reason)
{
    if (s_bq.voltage_settled == 0u)
    {
        *reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
        return 0u;
    }
    if (s_bq.protection_ok == 0u)
    {
        *reason = (uint8_t)BQ_FRAME_REJECT_NOT_CONFIGURED;
        return 0u;
    }
    return Bq_ValidateFrameValues(cells, reason);
}

static void Bq_LatchAdcFault(void)
{
    if (s_bq.adc_recovery_failed == 0u)
    {
        s_bq.adc_event_flags |= BQ_ADC_EVENT_FAULT;
    }
    s_bq.adc_restart_pending = 0u;
    s_bq.verdict_arm_pending = 0u;
    s_bq.verdict_active = 0u;
    s_bq.resample_pending = 0u;
    s_bq.adc_recovery_failed = 1u;
    Fault_SetLive(FAULT_BIT_BQ_ADC_STUCK);
}

static void Bq_RequestAdcRestart(uint8_t reason)
{
    if ((s_bq.adc_recovery_failed != 0u) ||
        (s_bq.adc_restart_pending != 0u))
    {
        return;
    }

    if (s_bq.adc_restart_attempts >= BQ_ADC_RESTART_MAX_ATTEMPTS)
    {
        Bq_LatchAdcFault();
        return;
    }

    s_bq.adc_restart_pending = 1u;
    s_bq.adc_restart_reason = reason;
    s_bq.verdict_active = 0u;   /* 当前轮提前结束，交给下一次重启 */
}

static uint8_t Bq_IsOorReason(uint8_t reason)
{
    return (reason == (uint8_t)BQ_FRAME_REJECT_CELL_RANGE) ? 1u : 0u;
}

static void Bq_RecordReject(const uint16_t cells[3], uint16_t pack,
                            uint8_t reason)
{
    s_bq.frame_valid = 0u;
    s_bq.reject_reason = reason;
    s_bq.frame_reject_count++;
    s_bq.last_reject_cell_mv[0] = cells[0];
    s_bq.last_reject_cell_mv[1] = cells[1];
    s_bq.last_reject_cell_mv[2] = cells[2];
    s_bq.last_reject_pack_mv = pack;
    s_bq.last_reject_vc3_raw = s_bq.raw_vc3;
    s_bq.last_reject_vc4_raw = s_bq.raw_vc4;
    s_bq.last_reject_sys_stat = s_bq.sys_stat;
    s_bq.last_reject_reason = reason;
    s_bq.last_reject_ms = s_bq.now_ms;
    s_bq.frame_confirm_count = 0u;
    s_bq.fresh = 0u;
    s_bq.good_publish_streak = 0u;

    if (s_bq.reject_streak < 0xFFFFu)
    {
        s_bq.reject_streak++;
    }
    if (s_bq.reject_since_publish < 0xFFFFu)
    {
        s_bq.reject_since_publish++;
    }

    if (reason == (uint8_t)BQ_FRAME_REJECT_UNSETTLED)
    {
        s_bq.reject_unsettled_count++;
    }
    else if (reason == (uint8_t)BQ_FRAME_REJECT_CELL_RANGE)
    {
        s_bq.reject_cell_count++;
    }
    /* 只有 VC1/VC2 数值越界的帧计入 OOR；UNSETTLED/NOT_CONFIGURED 不算。
     * 连续 8 帧 OOR 才触发一次受控 ADC 重启。 */
    if (Bq_IsOorReason(reason) != 0u)
    {
        if (s_bq.oor_streak < 0xFFFFu)
        {
            s_bq.oor_streak++;
        }

        if (s_bq.adc_recovery_failed == 0u)
        {
            s_bq.resample_pending = 1u;
            s_bq.resample_due_ms = s_bq.now_ms + BQ_ADC_RESAMPLE_MS;
            if (s_bq.oor_streak >= BQ_OOR_STREAK_RESTART)
            {
                Bq_RequestAdcRestart((uint8_t)BQ_RESTART_REASON_REJECT_STREAK);
            }
        }
    }
}

static void Bq_PublishFrame(const uint16_t cells[3], uint16_t pack)
{
    s_bq.cell_mv[0] = cells[0];
    s_bq.cell_mv[1] = cells[1];
    s_bq.cell_mv[2] = cells[2];
    s_bq.pack_mv = pack;
    s_bq.log_cell3_mv = s_bq.batch.log_cell3_mv;
    s_bq.log_pack_mv = s_bq.batch.log_pack_mv;
    s_bq.log_cell3_valid = s_bq.batch.log_cell3_valid;
    s_bq.log_pack_valid = s_bq.batch.log_pack_valid;
    s_bq.ref_cell2_only = s_bq.batch.ref_cell2_only;
    s_bq.run_ref_cell2_only = s_bq.batch.ref_cell2_only;
    s_bq.ref_decision_final = 1u;
    s_bq.max_cell_mv = (cells[0] >= cells[1]) ? cells[0] : cells[1];
    if (cells[2] > s_bq.max_cell_mv)
    {
        s_bq.max_cell_mv = cells[2];
    }
    s_bq.min_cell_mv = (cells[0] <= cells[1]) ? cells[0] : cells[1];
    if (cells[2] < s_bq.min_cell_mv)
    {
        s_bq.min_cell_mv = cells[2];
    }

    /* CC：1 LSB=8.44 µV，分流 1.5 mΩ → I(mA)=cc_raw×844/150（带符号）。 */
    s_bq.current_ma = ((int32_t)s_bq.cc_raw * BQ_CURRENT_MA_NUM) / BQ_CURRENT_MA_DEN;
    s_bq.published_cc_raw = s_bq.cc_raw;
    s_bq.published_ts1_raw = s_bq.raw_ts1;
    s_bq.batch.published_seq++;
    s_bq.batch.published_ms = s_bq.now_ms;
    if (s_bq.batch.quality == (uint8_t)BQ_BATCH_FULL_CANDIDATE)
    {
        s_bq.batch.quality = (uint8_t)BQ_BATCH_FULL_REAL_OK;
    }
    s_bq.last_update_ms = s_bq.now_ms;
    s_bq.frame_valid = 1u;
    s_bq.reject_reason = (uint8_t)BQ_FRAME_OK;
    s_bq.frame_confirm_count = 2u;
    s_bq.strict_window = 0u;
    s_bq.strict_mismatch_count = 0u;
    s_bq.verdict_arm_pending = 0u;
    s_bq.verdict_active = 0u;       /* 本轮 ADC 重启已有结果 */
    s_bq.adc_restart_pending = 0u;
    s_bq.resample_pending = 0u;
    s_bq.oor_streak = 0u;
    s_bq.reject_streak = 0u;
    s_bq.reject_since_publish = 0u;
    s_bq.fresh = 1u;

    if (s_bq.good_publish_streak < 0xFFFFu)
    {
        s_bq.good_publish_streak++;
    }

    /* 连续 4 帧有效才清 ADC 故障/额度，避免单帧偶发恢复造成反复。 */
    if (s_bq.good_publish_streak >= BQ_ADC_GOOD_FRAMES_CLEAR)
    {
        uint8_t had_fault = s_bq.adc_recovery_failed;

        s_bq.good_publish_streak = 0u;
        s_bq.adc_restart_attempts = 0u;
        s_bq.recovery_episode = 0u;
        s_bq.verdict_arm_pending = 0u;
        s_bq.verdict_active = 0u;
        s_bq.adc_recovery_failed = 0u;
        s_bq.adc_restart_pending = 0u;
        Fault_ClearLive(FAULT_BIT_BQ_ADC_STUCK);
        if (had_fault != 0u)
        {
            s_bq.adc_event_flags |= BQ_ADC_EVENT_RECOVERED;
        }
    }

    /* 用最新 VC1/VC2 重新决定 OV/UV 是否来自 VC5 假锁存。 */
    Bq_RefreshEffectiveSysStat();
}

/* RUN 所有寄存器分别 CRC 校验、脚本无重试之后才生成同批证据。
 * 分组顺序不等于 BQ 内部 ADC 原子帧；两个采样批次相隔至少一次转换。
 * CC_READY 与独立 CRC/new batch 同时满足才认定新的 BQ CC 转换。 */
static uint8_t Bq_FinishRunBatch(void)
{
    bq_batch_quality_t *b = &s_bq.batch;
    uint16_t ok;
    uint8_t status_ok;
    uint8_t config_ok;
    uint32_t real_c3;
    uint32_t real_pack;

    b->batch_seq++;
    b->batch_ms = s_bq.now_ms;
    b->sample_start_ms = s_bq.run_start_ms;
    b->crc_ok_mask = s_bq.run_crc_mask;
    b->field_ok_mask = 0u;
    b->complete = 0u;
    b->quality = (uint8_t)BQ_BATCH_INVALID;
    b->sys_stat = s_bq.sys_stat;
    b->raw_sys_stat = s_bq.raw_sys_stat;
    b->ctrl1 = s_bq.run_ctrl1;
    b->ctrl2 = s_bq.run_ctrl2;
    b->protect1 = s_bq.run_protect[0];
    b->protect2 = s_bq.run_protect[1];
    b->protect3 = s_bq.run_protect[2];
    b->raw_cell[0] = s_bq.raw_cell[0];
    b->raw_cell[1] = s_bq.raw_cell[1];
    b->raw_cell[2] = s_bq.raw_cell[2];
    b->raw_pack = s_bq.raw_pack;
    b->raw_vc3 = s_bq.raw_vc3;
    b->raw_vc4 = s_bq.raw_vc4;
    b->ts1_raw = s_bq.raw_ts1;
    b->cc_raw = s_bq.cc_raw;
    b->current_ma = ((int32_t)s_bq.cc_raw * BQ_CURRENT_MA_NUM) / BQ_CURRENT_MA_DEN;
    b->cell_mv[0] = Bq_RawToMv(s_bq.raw_cell[0]);
    b->cell_mv[1] = Bq_RawToMv(s_bq.raw_cell[1]);
    b->cell_mv[2] = (uint16_t)(((uint32_t)b->cell_mv[0] +
                                (uint32_t)b->cell_mv[1] + 1u) / 2u);
    b->pack_mv = (uint16_t)((uint32_t)b->cell_mv[0] +
                            (uint32_t)b->cell_mv[1] + b->cell_mv[2]);
    b->log_cell3_mv = 0u;
    b->log_pack_mv = 0u;
    b->log_cell3_valid = 0u;
    b->log_pack_valid = 0u;
    b->ref_cell2_only = 0u;

    if ((s_bq.run_failed != 0u) ||
        ((s_bq.run_crc_mask & BQ_FIELD_REQUIRED) != BQ_FIELD_REQUIRED) ||
        (s_bq.run_status_reads != 2u) ||
        (s_bq.run_protect_reads != 3u))
    {
        s_bq.cc_sample.valid = 0u;
        return 0u;
    }

    b->complete = 1u;  /* 完整传输，不意味着每个数据字段都可信。 */
    ok = (uint16_t)(s_bq.run_crc_mask & (uint16_t)~s_bq.run_format_mask);
    if (((ok & BQ_FIELD_VC1) != 0u) &&
        (b->cell_mv[0] >= BQ_CELL_VALID_MIN_MV) &&
        (b->cell_mv[0] <= BQ_CELL_VALID_MAX_MV))
    {
        b->field_ok_mask |= BQ_FIELD_VC1;
    }
    if (((ok & BQ_FIELD_VC2) != 0u) &&
        (b->cell_mv[1] >= BQ_CELL_VALID_MIN_MV) &&
        (b->cell_mv[1] <= BQ_CELL_VALID_MAX_MV))
    {
        b->field_ok_mask |= BQ_FIELD_VC2;
    }
    /* optional 诊断字段：读取完整即记录；数值不参与发布/保护。 */
    if ((ok & BQ_FIELD_VC5) != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_VC5;
    }
    if ((ok & BQ_FIELD_BAT) != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_BAT;
    }
    if ((ok & BQ_FIELD_VC3) != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_VC3;
    }
    if ((ok & BQ_FIELD_VC4) != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_VC4;
    }
    if ((ok & BQ_FIELD_TS1) != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_TS1;
    }

    /* 先用本批 raw VC5/BAT 决定参考节，再过滤本批有效 OV/UV。
     * 这样虚假低 VC1 不会先把状态判成 INVALID，也不会形成
     * filter -> sys_stat -> quality -> filter 的循环。 */
    b->ref_cell2_only = Bq_EvaluateRealC3Pack(
        b->raw_cell[2], b->raw_pack,
        (uint8_t)(((b->field_ok_mask & BQ_FIELD_VC5) != 0u) ? 1u : 0u),
        (uint8_t)(((b->field_ok_mask & BQ_FIELD_BAT) != 0u) ? 1u : 0u),
        b->cell_mv[0], b->cell_mv[1],
        &b->log_cell3_mv, &b->log_pack_mv,
        &b->log_cell3_valid, &b->log_pack_valid);
    if ((b->field_ok_mask & (BQ_FIELD_VC1 | BQ_FIELD_VC2)) !=
        (BQ_FIELD_VC1 | BQ_FIELD_VC2))
    {
        b->ref_cell2_only = 0u;
    }
    s_bq.run_ref_cell2_only = b->ref_cell2_only;
    s_bq.ref_decision_final = 1u;
    Bq_RefreshEffectiveSysStat();
    b->sys_stat = s_bq.sys_stat;

    /* STATUS: 首尾两次 CRC，任一读取有保护/XREADY/ALERT 均不能部分放行。
     * CC_READY 是 CC 转换标志，不是保护；脚本中写 1 清零。 */
    status_ok = (((b->sys_stat & BQ_BATCH_FAULT_MASK) == 0u) &&
                 ((ok & BQ_FIELD_SYS_STAT) != 0u)) ? 1u : 0u;
    if (status_ok != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_SYS_STAT;
    }
    config_ok = ((s_bq.voltage_settled != 0u) &&
                 (s_bq.protection_ok != 0u) &&
                 (s_bq.run_protect[0] == s_bq.protect1) &&
                 (s_bq.run_protect[1] == s_bq.protect2) &&
                 (s_bq.run_protect[2] == s_bq.protect3)) ? 1u : 0u;
    if ((config_ok != 0u) && ((ok & BQ_FIELD_PROTECTION) != 0u))
    {
        b->field_ok_mask |= BQ_FIELD_PROTECTION;
    }
    if (((ok & BQ_FIELD_CTRL1) != 0u) &&
        ((b->ctrl1 & BQ_SYS_CTRL1_ADC_EN) != 0u))
    {
        b->field_ok_mask |= BQ_FIELD_CTRL1;
    }
    if (((ok & BQ_FIELD_CTRL2) != 0u) &&
        ((b->ctrl2 & BQ_SYS_CTRL2_CC_EN) != 0u) &&
        ((b->ctrl2 & BQ_FET_MASK_ALL) == s_bq.fet_mask_applied))
    {
        b->field_ok_mask |= BQ_FIELD_CTRL2;
    }
    if ((ok & BQ_FIELD_CC) != 0u)
    {
        b->field_ok_mask |= BQ_FIELD_CC;
    }
    /* 控制值逐字段优先真实值；日志值保持上面的 real-or-0。 */
    real_c3 = Bq_RawToMv(b->raw_cell[2]);
    real_pack = Bq_RawPackToMv(b->raw_pack);
    Bq_SelectControlVoltages(b->cell_mv[0], b->cell_mv[1],
                             (uint16_t)real_c3, b->log_cell3_valid,
                             (uint16_t)real_pack, b->log_pack_valid,
                             &b->cell_mv[2], &b->pack_mv);

    /* 没有 CC_READY 不证明读到新的 CC 转换。若 CC_READY 在读取过程中
     * 才出现于末尾状态，则留待下一轮读取，不能把本轮旧 CC 认作新值。 */
    if (((s_bq.run_stat_first & BQ_SYS_STAT_CC_READY) != 0u) &&
        ((b->field_ok_mask & BQ_CC_BATCH_BASE) == BQ_CC_BATCH_BASE) &&
        ((s_bq.cc_sample.sample_seq == 0u) ||
         ((s_bq.run_start_ms - s_bq.cc_last_start_ms) >= BQ_ADC_CONVERSION_MS)))
    {
        s_bq.cc_last_start_ms = s_bq.run_start_ms;
        s_bq.cc_sample.sample_seq++;
        s_bq.cc_sample.sample_ms = s_bq.now_ms;
        s_bq.cc_sample.batch_seq = b->batch_seq;
        s_bq.cc_sample.cc_raw = b->cc_raw;
        s_bq.cc_sample.current_ma = b->current_ma;
        s_bq.cc_sample.valid = 1u;
    }
    else if (((b->field_ok_mask & BQ_CC_BATCH_BASE) != BQ_CC_BATCH_BASE) ||
             ((s_bq.run_stat_first & BQ_SYS_STAT_CC_READY) == 0u))
    {
        s_bq.cc_sample.valid = 0u;
    }

    if ((b->field_ok_mask & BQ_FIELD_REQUIRED) == BQ_FIELD_REQUIRED)
    {
        b->quality = (uint8_t)BQ_BATCH_FULL_CANDIDATE;
    }
    return 1u;
}

/* 只有完整、已稳定且通过电压/包压合理性检查的帧才发布。
 * 首个有效帧只作为候选，第二帧一致后才发布。 */
static void Bq_FrameCommit(void)
{
    uint16_t cells[3];
    uint16_t pack;
    uint8_t reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
    uint8_t delta_ok = 1u;
    uint8_t i;

    if (Bq_FinishRunBatch() == 0u)
    {
        Bq_RecordReject(s_bq.batch.cell_mv, s_bq.batch.pack_mv,
                        (uint8_t)BQ_FRAME_REJECT_INCOMPLETE);
        return;
    }

    /* 控制值优先真实 VC5/BAT，异常时回退 VC1/VC2 估算；发布校验仍只看 VC1/VC2。 */
    cells[0] = s_bq.batch.cell_mv[0];
    cells[1] = s_bq.batch.cell_mv[1];
    cells[2] = s_bq.batch.cell_mv[2];
    pack = s_bq.batch.pack_mv;

    if (Bq_ValidateFrame(cells, &reason) == 0u)
    {
        Bq_RecordReject(cells, pack, reason);
        return;
    }

    s_bq.frame_ok_count++;
    s_bq.reject_streak = 0u;
    s_bq.oor_streak = 0u;
    s_bq.resample_pending = 0u;

    if (s_bq.frame_confirm_count == 0u)
    {
        for (i = 0u; i < 3u; i++)
        {
            s_bq.pending_cell_mv[i] = cells[i];
            s_bq.pending_raw_cell[i] = s_bq.raw_cell[i];
        }
        s_bq.frame_confirm_count = 1u;
        s_bq.frame_valid = 0u;
        s_bq.fresh = 0u;
        if ((s_bq.strict_window != 0u) || (s_bq.recovery_episode != 0u))
        {
            /* 首次/恢复确认不等 500 ms 调度：300 ms 后再取一帧。 */
            s_bq.resample_pending = 1u;
            s_bq.resample_due_ms = s_bq.now_ms + BQ_ADC_RESAMPLE_MS;
        }
        return;
    }

    if (s_bq.frame_confirm_count == 1u)
    {
        if (s_bq.strict_window != 0u)
        {
            /* 启动/LP/ADC 重启后的首次发布：只比较 VC1/VC2 原始码 ±4 LSB。
             * cell3/pack 不作为该双帧确认条件，也不比较 raw VC5/BAT。 */
            for (i = 0u; i < 2u; i++)
            {
                uint16_t diff = (s_bq.raw_cell[i] >= s_bq.pending_raw_cell[i]) ?
                                (uint16_t)(s_bq.raw_cell[i] - s_bq.pending_raw_cell[i]) :
                                (uint16_t)(s_bq.pending_raw_cell[i] - s_bq.raw_cell[i]);
                if (diff > BQ_FRAME_CONFIRM_RAW_LSB)
                {
                    delta_ok = 0u;
                }
            }
        }
        else
        {
            /* 普通运行期偶发拒绝后的粗确认：只比较 VC1/VC2 mV。 */
            for (i = 0u; i < 2u; i++)
            {
                uint16_t diff = (cells[i] >= s_bq.pending_cell_mv[i]) ?
                                (uint16_t)(cells[i] - s_bq.pending_cell_mv[i]) :
                                (uint16_t)(s_bq.pending_cell_mv[i] - cells[i]);
                if (diff > BQ_FRAME_CONFIRM_CELL_DELTA_MV)
                {
                    delta_ok = 0u;
                }
            }
        }

        if (delta_ok == 0u)
        {
            for (i = 0u; i < 3u; i++)
            {
                s_bq.pending_cell_mv[i] = cells[i];
                s_bq.pending_raw_cell[i] = s_bq.raw_cell[i];
            }
            s_bq.frame_confirm_count = 1u;
            s_bq.frame_confirm_reset_count++;
            s_bq.frame_valid = 0u;
            s_bq.fresh = 0u;
            if (s_bq.strict_window != 0u)
            {
                if (s_bq.strict_mismatch_count < 0xFFu)
                {
                    s_bq.strict_mismatch_count++;
                }
                if (s_bq.strict_mismatch_count >= BQ_STRICT_MAX_MISMATCHES)
                {
                    /* 可能是真实电压回弹/负载变化而非 ADC 异常：
                     * 回退到 250/300 mV 粗确认，避免误判 ADC 故障。 */
                    s_bq.strict_window = 0u;
                }
            }
            if ((s_bq.strict_window != 0u) || (s_bq.recovery_episode != 0u))
            {
                /* 启动/恢复窗口内不等待 500 ms 调度：300 ms 后再取一帧。 */
                s_bq.resample_pending = 1u;
                s_bq.resample_due_ms = s_bq.now_ms + BQ_ADC_RESAMPLE_MS;
            }
            return;
        }
        s_bq.frame_confirm_count = 2u;
    }

    Bq_PublishFrame(cells, pack);
}

/* 按实际 ADCGAIN/ADCOFFSET 计算 OV/UV 的 8 位 middle-code。
 * V(mV) = (365 + gain) × ADC / 1000 + offset；OV 全 14 位以 10 开头、
 * 低 4 位 1000，UV 以 01 开头、低 4 位 0000，寄存器只存 middle 8 位。 */
static int Bq_ApplyProtectionConfig(void)
{
    int32_t gain_uv;
    int32_t full;
    uint8_t ov_trip;
    uint8_t uv_trip;
    uint8_t rb;

    gain_uv = 365 + (int32_t)s_bq.adc_gain;
    if (gain_uv <= 0)
    {
        return -1;
    }

    full = (((int32_t)BQ_OV_TRIP_MV - (int32_t)s_bq.adc_offset) * 1000) / gain_uv;
    if (full < 0x2000)
    {
        full = 0x2000;
    }
    if (full > 0x2FFF)
    {
        full = 0x2FFF;
    }
    ov_trip = (uint8_t)((full >> 4) & 0xFF);

    full = (((int32_t)BQ_UV_TRIP_MV - (int32_t)s_bq.adc_offset) * 1000) / gain_uv;
    if (full < 0x1000)
    {
        full = 0x1000;
    }
    if (full > 0x1FFF)
    {
        full = 0x1FFF;
    }
    uv_trip = (uint8_t)((full >> 4) & 0xFF);

    if (Bq_BlockingWrite(BQ_REG_PROTECT1, BQ_PROTECT1_VALUE) != 0 ||
        Bq_BlockingWrite(BQ_REG_PROTECT2, BQ_PROTECT2_VALUE) != 0 ||
        Bq_BlockingWrite(BQ_REG_PROTECT3, BQ_PROTECT3_VALUE) != 0 ||
        Bq_BlockingWrite(BQ_REG_OV_TRIP, ov_trip) != 0 ||
        Bq_BlockingWrite(BQ_REG_UV_TRIP, uv_trip) != 0)
    {
        return -1;
    }

    if ((Bq_BlockingRead1(BQ_REG_PROTECT1, &rb) != 0) || (rb != BQ_PROTECT1_VALUE) ||
        (Bq_BlockingRead1(BQ_REG_PROTECT2, &rb) != 0) || (rb != BQ_PROTECT2_VALUE) ||
        (Bq_BlockingRead1(BQ_REG_PROTECT3, &rb) != 0) || (rb != BQ_PROTECT3_VALUE) ||
        (Bq_BlockingRead1(BQ_REG_OV_TRIP, &rb) != 0) || (rb != ov_trip) ||
        (Bq_BlockingRead1(BQ_REG_UV_TRIP, &rb) != 0) || (rb != uv_trip))
    {
        return -1;
    }

    s_bq.protect1 = BQ_PROTECT1_VALUE;
    s_bq.protect2 = BQ_PROTECT2_VALUE;
    s_bq.protect3 = BQ_PROTECT3_VALUE;
    s_bq.ov_trip = ov_trip;
    s_bq.uv_trip = uv_trip;
    s_bq.protection_ok = 1u;
    Bq_UpdateTripThresholds();
    return 0;
}

/* 受控 ADC 重启：配置前快照、ADC/CC/FET 关闭、最后再使能。 */
static int Bq_PreConfigSnapshot(void)
{
    uint8_t rb1 = 0u;
    uint8_t rb2 = 0u;
    uint8_t stat = 0u;
    uint8_t bal = 0u;

    s_bq.pre_snapshot_valid = 0u;
    if ((Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &rb1) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &rb2) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0) ||
        (Bq_BlockingRead1(BQ_REG_CELLBAL1, &bal) != 0))
    {
        return -1;
    }

    s_bq.pre_ctrl1 = rb1;
    s_bq.pre_ctrl2 = rb2;
    s_bq.pre_sys_stat = stat;
    s_bq.pre_cellbal1 = bal;
    s_bq.pre_snapshot_valid = 1u;

    /* 只记录本次 boot/LP 唤醒后第一次 CONFIG 前的锁存；运行期 ADC 重启
     * 再次出现的保护锁存不算 boot 锁存，交给保护监督层按真实故障处理。 */
    if (s_bq.boot_prot_capture_armed != 0u)
    {
        /* OV/UV 可能来自不可信 VC5，boot 锁存只保留 SCD/OCD。 */
        s_bq.boot_prot_latch_mask =
            (uint8_t)(stat & (BQ_SYS_STAT_SCD | BQ_SYS_STAT_OCD));
        s_bq.boot_prot_capture_armed = 0u;
    }
    return 0;
}

static int Bq_BalanceOffVerified(void)
{
    uint8_t rb = 0u;

    if ((Bq_BlockingWrite(BQ_REG_CELLBAL1, 0x00u) != 0) ||
        (Bq_BlockingRead1(BQ_REG_CELLBAL1, &rb) != 0))
    {
        return -1;
    }
    s_bq.post_cellbal1 = rb;
    return (rb == 0u) ? 0 : -1;
}

static int Bq_ForceAdcOff(void)
{
    uint8_t rb1 = 0u;
    uint8_t rb2 = 0u;
    uint8_t bal = 0u;

    if ((Bq_BlockingWrite(BQ_REG_SYS_CTRL2, 0x00u) != 0) ||
        (Bq_BalanceOffVerified() != 0) ||
        (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_LP) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &rb1) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &rb2) != 0) ||
        (Bq_BlockingRead1(BQ_REG_CELLBAL1, &bal) != 0))
    {
        return -1;
    }

    s_bq.adc_off_ctrl1 = rb1;
    s_bq.adc_off_ctrl2 = rb2;
    s_bq.post_cellbal1 = bal;
    if (((rb1 & (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) != 0u) ||
        ((rb2 & (BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL)) != 0u) ||
        (bal != 0u))
    {
        return -1;
    }
    return 0;
}

static int Bq_EnableAdcVerified(void)
{
    uint8_t rb1 = 0u;
    uint8_t rb2 = 0u;
    uint8_t stat = 0u;

    /* 均衡永久关闭：开 ADC/CC 前必须确认 CELLBAL1==0。 */
    if (Bq_BalanceOffVerified() != 0)
    {
        return -1;
    }
    if ((Bq_BlockingWrite(BQ_REG_SYS_CTRL2, BQ_SYS_CTRL2_CC_EN) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &rb2) != 0))
    {
        return -1;
    }
    s_bq.post_ctrl2 = rb2;
    if ((rb2 & (BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL)) !=
        BQ_SYS_CTRL2_CC_EN)
    {
        return -1;
    }

    if ((Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_VALUE) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &rb1) != 0))
    {
        return -1;
    }
    s_bq.post_ctrl1 = rb1;
    if ((rb1 & (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) !=
        (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL))
    {
        return -1;
    }

    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }
    s_bq.post_sys_stat = stat;
    s_bq.xready_at_settle =
        ((stat & BQ_SYS_STAT_XREADY) != 0u) ? 1u : 0u;
    return 0;
}

static int Bq_VerifySettleStatus(void)
{
    uint8_t stat = 0u;
    uint8_t rb1 = 0u;
    uint8_t bal = 0u;

    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }
    s_bq.settle_sys_stat = stat;

    if ((stat & BQ_SYS_STAT_XREADY) != 0u)
    {
        s_bq.xready_count++;
        if (Bq_BlockingWrite(BQ_REG_SYS_STAT, BQ_SYS_STAT_XREADY) != 0)
        {
            return -1;
        }
        HAL_Delay(10u);
        if ((Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0) ||
            ((stat & BQ_SYS_STAT_XREADY) != 0u))
        {
            Fault_SetLive(FAULT_BIT_BQ_XREADY);
            return -1;
        }
        s_bq.settle_sys_stat = stat;
    }

    if (Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &rb1) != 0)
    {
        return -1;
    }
    if ((rb1 & (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) !=
        (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL))
    {
        return -1;
    }
    if ((Bq_BlockingRead1(BQ_REG_CELLBAL1, &bal) != 0) || (bal != 0u))
    {
        s_bq.post_cellbal1 = bal;
        return -1;
    }
    s_bq.post_cellbal1 = bal;
    return 0;
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
    /* Any error invalidates the in-flight batch immediately.  A normal retry
     * may later complete a new batch, but it must never expose the old batch
     * as current evidence. */
    if (s_bq.script == s_bq_run_script)
    {
        s_bq.run_failed = 1u;
        s_bq.batch.complete = 0u;
        s_bq.batch.quality = (uint8_t)BQ_BATCH_INVALID;
        s_bq.cc_sample.valid = 0u;
    }
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

static void Bq_ConfigFailed(void)
{
    s_bq.protection_ok = 0u;
    s_bq.voltage_settled = 0u;
    s_bq.frame_valid = 0u;
    s_bq.fresh = 0u;
    s_bq.req_read = 0u;
    s_bq.frame_confirm_count = 0u;
    s_bq.reject_reason = (uint8_t)BQ_FRAME_REJECT_NOT_CONFIGURED;

    s_bq.cfg_attempts++;
    if (s_bq.cfg_attempts >= BQ_CFG_RETRY_LIMIT)
    {
        Fault_SetSticky(FAULT_BIT_BQ_COMM);
        s_bq.cfg_attempts = 0u;
    }
    Bq_ScheduleRetry(0u);
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

    /* 一次性 SYS_STAT 清锁存写完成：只从影子中消掉对应 OV/UV 位。 */
    if (s_bq.sysstat_sync_active != 0u)
    {
        s_bq.sysstat_sync_active = 0u;
        s_bq.sys_stat &= (uint8_t)~((uint8_t)s_bq.sysstat_job.data &
                                    (BQ_SYS_STAT_OV | BQ_SYS_STAT_UV));
        return;
    }

    /* 一次性 FET 同步写完成：以写入时的 mask 为准（期间若又改请求，下轮再同步）。 */
    if (s_bq.fet_sync_active != 0u)
    {
        s_bq.fet_sync_active = 0u;
        s_bq.fet_mask_applied = (uint8_t)(s_bq.fet_job.data & 0x03u);
        return;
    }

    if (s_bq.state == BQ_ST_CONFIG)
    {
        if (((s_bq.cc_cfg_readback & BQ_CC_CFG_MASK) != BQ_CC_CFG_VALUE) ||
            (Bq_ApplyProtectionConfig() != 0) ||
            (Bq_EnableAdcVerified() != 0))
        {
            Bq_ConfigFailed();
        }
        else
        {
            s_bq.cfg_attempts = 0u;
            s_bq.fet_mask_applied = BQ_FET_MASK_NONE;  /* 配置脚本最后只开 CC（0x40） */
            s_bq.fet_sync_active = 0u;
            s_bq.voltage_settled = 0u;
            s_bq.frame_valid = 0u;
            s_bq.frame_confirm_count = 0u;
            s_bq.strict_window = 1u;   /* 重启后首次发布严格 ±4 raw LSB */
            s_bq.strict_mismatch_count = 0u;
            s_bq.oor_streak = 0u;
            s_bq.resample_pending = 0u;
            s_bq.verdict_active = 0u;
            s_bq.good_publish_streak = 0u;
            s_bq.reject_streak = 0u;
            s_bq.reject_since_publish = 0u;
            s_bq.reject_reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
            s_bq.fresh = 0u;
            s_bq.req_read = 0u;
            s_bq.state = BQ_ST_SETTLE;
            s_bq.state_enter_ms = s_bq.now_ms;
        }
    }
    else if (s_bq.state == BQ_ST_RUN)
    {
        Bq_FrameCommit();
        s_bq.req_read = 0u;

        /* 本刷新周期结束，按 MOS 开关请求同步（保留 CC_EN）。
         * SYS_STAT 假锁存清理排在 FET 同步之前，由 BQ_Poll 发起。 */
        if (s_bq.pending_sysstat_clear != 0u)
        {
            uint8_t clear_mask = s_bq.pending_sysstat_clear;

            s_bq.pending_sysstat_clear = 0u;
            s_bq.sysstat_job.op = (uint8_t)BQ_OP_WRITE;
            s_bq.sysstat_job.reg = BQ_REG_SYS_STAT;
            s_bq.sysstat_job.data =
                (uint8_t)(clear_mask | BQ_SYS_STAT_IRQ_CLR);
            s_bq.sysstat_sync_active = 1u;
            Bq_StartScript(&s_bq.sysstat_job, 1u, BQ_ST_RUN);
            return;
        }

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
    if (script == s_bq_run_script)
    {
        s_bq.run_crc_mask = 0u;
        s_bq.run_format_mask = 0u;
        s_bq.run_failed = 0u;
        s_bq.run_status_reads = 0u;
        s_bq.run_protect_reads = 0u;
        s_bq.run_stat_first = 0u;
        s_bq.run_stat_last = 0u;
        s_bq.run_ctrl1 = 0u;
        s_bq.run_ctrl2 = 0u;
        s_bq.run_protect[0] = 0u;
        s_bq.run_protect[1] = 0u;
        s_bq.run_protect[2] = 0u;
        s_bq.run_ref_cell2_only = s_bq.ref_cell2_only;
        s_bq.ref_decision_final = 0u;
        s_bq.run_start_ms = HAL_GetTick();
    }
    Bq_StartNextJob();
}

static void Bq_StartConfig(void)
{
    s_bq.script = NULL;
    s_bq.cc_cfg_readback = 0u;
    s_bq.protection_ok = 0u;
    s_bq.voltage_settled = 0u;
    s_bq.frame_valid = 0u;
    s_bq.frame_confirm_count = 0u;
    s_bq.strict_window = 0u;
    s_bq.strict_mismatch_count = 0u;
    s_bq.reject_reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
    s_bq.fresh = 0u;
    s_bq.req_read = 0u;
    s_bq.oor_streak = 0u;
    s_bq.resample_pending = 0u;
    s_bq.verdict_active = 0u;
    /* 运行期 ADC 恢复回合中的每次 CONFIG（含总线恢复重试）都重新 arm verdict。 */
    s_bq.verdict_arm_pending =
        ((s_bq.recovery_episode != 0u) &&
         (s_bq.adc_recovery_failed == 0u)) ? 1u : 0u;
    s_bq.good_publish_streak = 0u;
    s_bq.pending_sysstat_clear = 0u;
    s_bq.sysstat_sync_active = 0u;
    s_bq.ref_decision_final = 0u;
    s_bq.run_ref_cell2_only = s_bq.ref_cell2_only;
    s_bq.xready_at_settle = 0u;
    s_bq.xready_streak = 0u;

    if ((Bq_PreConfigSnapshot() != 0) || (Bq_ForceAdcOff() != 0))
    {
        Bq_ConfigFailed();
        return;
    }

    /* ADC/CC/FET 已确认关闭；等一个 BQ ADC 帧后再写配置和重新使能。 */
    s_bq.state = BQ_ST_ADC_RESTART_OFF;
    s_bq.state_enter_ms = HAL_GetTick();
}

static void Bq_StartConfigScript(void)
{
    Bq_StartScript(s_bq_cfg_script,
                   (uint8_t)(sizeof(s_bq_cfg_script) / sizeof(s_bq_cfg_script[0])),
                   BQ_ST_CONFIG);
}

static void Bq_StartRunScript(void)
{
    s_bq.last_sample_ms = HAL_GetTick();
    Bq_StartScript(s_bq_run_script,
                   (uint8_t)(sizeof(s_bq_run_script) / sizeof(s_bq_run_script[0])),
                   BQ_ST_RUN);
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
        Bq_RunReadVerified(s_bq.current_reg, s_bq.rx_buf, count);
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
    s_bq.request_fet_mask = BQ_DEFAULT_FET_MASK;
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
    s_bq.protect1 = 0u;
    s_bq.protect2 = 0u;
    s_bq.protect3 = 0u;
    s_bq.ov_trip = 0u;
    s_bq.uv_trip = 0u;
    s_bq.protection_ok = 0u;
    s_bq.boot_prot_latch_mask = 0u;
    s_bq.boot_prot_capture_armed = 1u;
    s_bq.raw_cell[0] = 0u;
    s_bq.raw_cell[1] = 0u;
    s_bq.raw_cell[2] = 0u;
    s_bq.raw_pack = 0u;
    s_bq.raw_ts1 = 0u;
    s_bq.raw_vc3 = 0u;
    s_bq.raw_vc4 = 0u;
    s_bq.cc_raw = 0;
    s_bq.cell_mv[0] = 0u;
    s_bq.cell_mv[1] = 0u;
    s_bq.cell_mv[2] = 0u;
    s_bq.pack_mv = 0u;
    s_bq.log_cell3_mv = 0u;
    s_bq.log_pack_mv = 0u;
    s_bq.log_cell3_valid = 0u;
    s_bq.log_pack_valid = 0u;
    s_bq.ref_cell2_only = 0u;
    s_bq.run_ref_cell2_only = 0u;
    s_bq.ref_decision_final = 0u;
    s_bq.cell12_meas_valid = 0u;
    s_bq.cell12_measured_once = 0u;
    s_bq.cell12_meas_mv[0] = 0u;
    s_bq.cell12_meas_mv[1] = 0u;
    s_bq.voltage_settled = 0u;
    s_bq.frame_valid = 0u;
    s_bq.reject_reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
    s_bq.max_cell_mv = 0u;
    s_bq.min_cell_mv = 0u;
    s_bq.ov_trip_mv = 0u;
    s_bq.uv_trip_mv = 0u;
    s_bq.frame_ok_count = 0u;
    s_bq.frame_reject_count = 0u;
    s_bq.reject_unsettled_count = 0u;
    s_bq.reject_cell_count = 0u;
    s_bq.reject_streak = 0u;
    s_bq.reject_since_publish = 0u;
    s_bq.last_reject_cell_mv[0] = 0u;
    s_bq.last_reject_cell_mv[1] = 0u;
    s_bq.last_reject_cell_mv[2] = 0u;
    s_bq.last_reject_pack_mv = 0u;
    s_bq.last_reject_vc3_raw = 0u;
    s_bq.last_reject_vc4_raw = 0u;
    s_bq.last_reject_ms = 0u;
    s_bq.last_reject_reason = 0u;
    s_bq.last_reject_sys_stat = 0u;
    s_bq.frame_confirm_count = 0u;
    s_bq.strict_window = 0u;
    s_bq.strict_mismatch_count = 0u;
    s_bq.pending_cell_mv[0] = 0u;
    s_bq.pending_cell_mv[1] = 0u;
    s_bq.pending_cell_mv[2] = 0u;
    s_bq.pending_raw_cell[0] = 0u;
    s_bq.pending_raw_cell[1] = 0u;
    s_bq.pending_raw_cell[2] = 0u;
    s_bq.frame_confirm_reset_count = 0u;
    s_bq.oor_streak = 0u;
    s_bq.resample_count = 0u;
    s_bq.resample_pending = 0u;
    s_bq.resample_due_ms = 0u;
    s_bq.last_sample_ms = 0u;
    s_bq.pre_ctrl1 = 0u;
    s_bq.pre_ctrl2 = 0u;
    s_bq.pre_sys_stat = 0u;
    s_bq.pre_cellbal1 = 0u;
    s_bq.post_cellbal1 = 0u;
    s_bq.pre_snapshot_valid = 0u;
    s_bq.post_ctrl1 = 0u;
    s_bq.post_ctrl2 = 0u;
    s_bq.post_sys_stat = 0u;
    s_bq.settle_sys_stat = 0u;
    s_bq.adc_off_ctrl1 = 0u;
    s_bq.adc_off_ctrl2 = 0u;
    s_bq.xready_at_settle = 0u;
    s_bq.xready_count = 0u;
    s_bq.xready_streak = 0u;
    s_bq.adc_restart_pending = 0u;
    s_bq.adc_restart_reason = 0u;
    s_bq.adc_restart_attempts = 0u;
    s_bq.adc_recovery_failed = 0u;
    s_bq.recovery_episode = 0u;
    s_bq.verdict_arm_pending = 0u;
    s_bq.verdict_active = 0u;
    s_bq.verdict_deadline_ms = 0u;
    s_bq.good_publish_streak = 0u;
    s_bq.adc_event_flags = 0u;
    s_bq.adc_restart_count = 0u;
    s_bq.adc_restart_last_ms = 0u;
    s_bq.run_crc_mask = 0u;
    s_bq.run_format_mask = 0u;
    s_bq.run_failed = 0u;
    s_bq.run_status_reads = 0u;
    s_bq.run_protect_reads = 0u;
    s_bq.run_stat_first = 0u;
    s_bq.run_stat_last = 0u;
    s_bq.run_ctrl1 = 0u;
    s_bq.run_ctrl2 = 0u;
    s_bq.run_protect[0] = 0u;
    s_bq.run_protect[1] = 0u;
    s_bq.run_protect[2] = 0u;
    s_bq.run_start_ms = 0u;
    s_bq.batch = (bq_batch_quality_t){0};
    s_bq.cc_sample = (bq_cc_sample_t){0};
    s_bq.cc_last_start_ms = 0u;
    s_bq.pending_sysstat_clear = 0u;
    s_bq.sysstat_sync_active = 0u;
    s_bq.sysstat_job.op = 0u;
    s_bq.sysstat_job.reg = 0u;
    s_bq.sysstat_job.data = 0u;

    Bq_HwInit();

    /* BQ 不一定随 MCU 复位：若它已 ACK（NORMAL/已上电），跳过 PB1 TS1 脉冲，
     * 直接进入配置；配置序列会强制 ADC off→on 并回读验证。SHIP/无应答时
     * 才用 PB1 高脉冲 ≥2 ms 唤醒，boot 后必须回低（否则短路 NTC）。 */
    {
        uint8_t probe = 0u;

        if (Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &probe) == 0)
        {
            /* BQ 已运行：确保 BOOT 回低（Q4 会短接 NTC）。 */
            HAL_GPIO_WritePin(BQ_BOOT_GPIO_Port, BQ_BOOT_Pin, GPIO_PIN_RESET);
            s_bq.state = BQ_ST_BOOT_WAIT;
            s_bq.state_enter_ms = HAL_GetTick();
        }
        else
        {
            HAL_GPIO_WritePin(BQ_BOOT_GPIO_Port, BQ_BOOT_Pin, GPIO_PIN_SET);
            s_bq.state = BQ_ST_BOOT_PULSE;
            s_bq.state_enter_ms = HAL_GetTick();
        }
    }
}

void BQ_ReInitAfterLp(void)
{
    s_bq.xfer_phase = 0u;
    s_bq.irq_tx_done = 0u;
    s_bq.irq_rx_done = 0u;
    s_bq.irq_error = 0u;
    s_bq.retry = 0u;
    s_bq.req_read = 0u;
    /* LP 期间 HAL tick 暂停；旧 last_update_ms 可能仍被误判为 fresh，
     * 强制等唤醒后的第一帧新数据再允许策略动作。 */
    s_bq.fresh = 0u;

    /* LP 期间 ADC 完全断电，唤醒后的受控重启算新恢复回合。 */
    s_bq.adc_restart_pending = 0u;
    s_bq.adc_restart_attempts = 0u;
    s_bq.adc_recovery_failed = 0u;
    s_bq.recovery_episode = 0u;
    s_bq.verdict_arm_pending = 0u;
    s_bq.verdict_active = 0u;
    s_bq.resample_pending = 0u;
    s_bq.oor_streak = 0u;
    s_bq.good_publish_streak = 0u;
    s_bq.strict_window = 0u;
    s_bq.strict_mismatch_count = 0u;
    s_bq.adc_event_flags = 0u;

    /* LP ends the previous BQ conversion epoch. */
    s_bq.batch = (bq_batch_quality_t){0};
    s_bq.cc_sample = (bq_cc_sample_t){0};
    s_bq.cc_last_start_ms = 0u;
    s_bq.log_cell3_mv = 0u;
    s_bq.log_pack_mv = 0u;
    s_bq.log_cell3_valid = 0u;
    s_bq.log_pack_valid = 0u;
    s_bq.ref_cell2_only = 0u;
    s_bq.run_ref_cell2_only = 0u;
    s_bq.ref_decision_final = 0u;
    s_bq.cell12_meas_valid = 0u;
    s_bq.cell12_measured_once = 0u;

    /* 新的 NORM 回合：重新捕获 boot/LP 唤醒前的保护锁存。 */
    s_bq.boot_prot_latch_mask = 0u;
    s_bq.boot_prot_capture_armed = 1u;

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

int BQ_PrepareForLpMask(uint8_t fet_mask)
{
    uint8_t stat = 0u;
    uint8_t rb1 = 0u;
    uint8_t rb2 = 0u;

    /* ADC 重启/配置/settle/verdict/故障锁存期间不允许进入 LP：否则会中断
     * 恢复序列或隐藏已上报的 ADC 故障。 */
    if ((BQ_IsIdle() == 0u) || (s_bq.state != BQ_ST_RUN) ||
        (s_bq.adc_restart_pending != 0u) ||
        (s_bq.verdict_active != 0u) ||
        (s_bq.recovery_episode != 0u) ||
        (s_bq.adc_recovery_failed != 0u))
    {
        return -2;
    }

    /* 关 ADC 与 TS1/NTC 偏置（SYS_CTRL1=0x00）：LP 期间无电压/电流/温度检测，
     * BQ 由 60 µA 档降到 40 µA 档；OV/UV 保护随 ADC 停，SCD/OCD 仍有效。
     * 唤醒后由配置序列恢复 SYS_CTRL1=0x18（ADC_EN|TEMP_SEL）。 */
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_LP) != 0)
    {
        return -1;
    }

    /* CC_EN=0，FET 按传入 mask；等待最后一笔 CC 转换结束后只清
     * CC_READY/XREADY，保护锁存（OV/UV/SCD/OCD）保留。 */
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2,
                         (uint8_t)(fet_mask & BQ_FET_MASK_ALL)) != 0)
    {
        return -1;
    }
    if (Bq_BalanceOffVerified() != 0)
    {
        return -1;
    }
    HAL_Delay(300u);   /* 一个 CC 周期 250 ms + 余量，防最后一笔转换重新置位 */
    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_STAT, BQ_SYS_STAT_IRQ_CLR) != 0)
    {
        return -1;
    }

    /* 回读确认 ADC/TEMP 与 CC 已真正关闭（FET 位可能被硬件保护覆盖，不在此检查）。 */
    if ((Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &rb1) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &rb2) != 0))
    {
        return -1;
    }
    if (((rb1 & (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) != 0u) ||
        ((rb2 & BQ_SYS_CTRL2_CC_EN) != 0u))
    {
        return -1;
    }

    s_bq.sys_stat = (uint8_t)(stat & ~BQ_SYS_STAT_IRQ_CLR);
    s_bq.req_read = 0u;
    s_bq.voltage_settled = 0u;
    s_bq.frame_valid = 0u;
    /* LP mask 成为新基线，避免唤醒后把进入 LP 前的 NORM 请求重新应用。 */
    s_bq.request_fet_mask = (uint8_t)(fet_mask & BQ_FET_MASK_ALL);
    s_bq.fet_mask_applied = (uint8_t)(fet_mask & BQ_FET_MASK_ALL);
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

    case BQ_ST_ADC_RESTART_OFF:
        if ((s_bq.script == NULL) && (s_bq.xfer_phase == 0u) &&
            ((s_bq.now_ms - s_bq.state_enter_ms) >= BQ_ADC_RESTART_OFF_HOLD_MS))
        {
            Bq_StartConfigScript();
        }
        break;

    case BQ_ST_CONFIG:
        if ((s_bq.script == NULL) && (s_bq.xfer_phase == 0u) &&
            (s_bq.state == BQ_ST_CONFIG))
        {
            /* 脚本已收尾但未切到 RUN：配置未完成（不应发生），重试。 */
            Bq_ConfigFailed();
        }
        break;

    case BQ_ST_SETTLE:
        if ((s_bq.script == NULL) && (s_bq.xfer_phase == 0u) &&
            ((s_bq.now_ms - s_bq.state_enter_ms) >= BQ_ADC_SETTLE_MS))
        {
            if (Bq_VerifySettleStatus() == 0)
            {
                s_bq.voltage_settled = 1u;
                s_bq.state = BQ_ST_RUN;
                s_bq.state_enter_ms = s_bq.now_ms;
                s_bq.req_read = 1u;
                if (s_bq.verdict_arm_pending != 0u)
                {
                    /* 本轮重启开始判定期；必须有成功发布才算“有果”。 */
                    s_bq.verdict_arm_pending = 0u;
                    s_bq.verdict_active = 1u;
                    s_bq.verdict_deadline_ms = s_bq.now_ms + BQ_ADC_VERDICT_MS;
                }
            }
            else
            {
                Bq_ConfigFailed();
            }
        }
        break;

    case BQ_ST_RUN:
        if ((s_bq.script == NULL) && (s_bq.xfer_phase == 0u))
        {
            if (s_bq.adc_restart_pending != 0u)
            {
                s_bq.adc_restart_pending = 0u;
                if (s_bq.adc_restart_attempts >= BQ_ADC_RESTART_MAX_ATTEMPTS)
                {
                    Bq_LatchAdcFault();
                    break;
                }

                s_bq.adc_restart_attempts++;
                s_bq.adc_restart_count++;
                s_bq.adc_restart_last_ms = s_bq.now_ms;
                s_bq.recovery_episode = 1u;
                s_bq.verdict_active = 0u;
                s_bq.voltage_settled = 0u;
                s_bq.frame_valid = 0u;
                s_bq.fresh = 0u;
                s_bq.frame_confirm_count = 0u;
                s_bq.reject_since_publish = 0u;
                s_bq.resample_pending = 0u;
                s_bq.adc_event_flags |= BQ_ADC_EVENT_RESTART;
                Bq_StartConfig();
                break;
            }

            if ((s_bq.verdict_active != 0u) &&
                ((int32_t)(s_bq.now_ms - s_bq.verdict_deadline_ms) >= 0))
            {
                s_bq.verdict_active = 0u;
                if (s_bq.adc_restart_attempts >= BQ_ADC_RESTART_MAX_ATTEMPTS)
                {
                    Bq_LatchAdcFault();
                }
                else
                {
                    Bq_RequestAdcRestart((uint8_t)BQ_RESTART_REASON_REJECT_STREAK);
                }
                break;
            }

            if ((s_bq.resample_pending != 0u) &&
                ((int32_t)(s_bq.now_ms - s_bq.resample_due_ms) >= 0) &&
                ((s_bq.now_ms - s_bq.last_sample_ms) >= BQ_ADC_CONVERSION_MS))
            {
                s_bq.resample_pending = 0u;
                if (s_bq.resample_count < 0xFFFFu)
                {
                    s_bq.resample_count++;
                }
                Bq_StartRunScript();
                break;
            }

            if ((s_bq.req_read != 0u) &&
                ((s_bq.now_ms - s_bq.last_sample_ms) >= BQ_ADC_CONVERSION_MS))
            {
                Bq_StartRunScript();
            }
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

    *out = (bq_snapshot_t){0};
    out->cell_mv[0] = s_bq.cell_mv[0];
    out->cell_mv[1] = s_bq.cell_mv[1];
    out->cell_mv[2] = s_bq.cell_mv[2];
    out->pack_mv = s_bq.pack_mv;
    out->log_cell3_mv = s_bq.log_cell3_mv;
    out->log_pack_mv = s_bq.log_pack_mv;
    out->log_cell3_valid = s_bq.log_cell3_valid;
    out->log_pack_valid = s_bq.log_pack_valid;
    out->ref_cell2_only = s_bq.ref_cell2_only;
    out->ov_trip_mv = s_bq.ov_trip_mv;
    out->uv_trip_mv = s_bq.uv_trip_mv;
    out->cell12_meas_mv[0] = s_bq.cell12_meas_mv[0];
    out->cell12_meas_mv[1] = s_bq.cell12_meas_mv[1];
    out->cell12_meas_valid = s_bq.cell12_meas_valid;
    out->cell12_measured_once = s_bq.cell12_measured_once;
    out->cc_raw = s_bq.cc_raw;
    out->current_ma = s_bq.current_ma;
    out->ts1_raw = s_bq.raw_ts1;
    out->sys_stat = s_bq.sys_stat;
    out->raw_sys_stat = s_bq.raw_sys_stat;
    out->protect1 = s_bq.protect1;
    out->protect2 = s_bq.protect2;
    out->protect3 = s_bq.protect3;
    out->protection_ok = s_bq.protection_ok;
    out->boot_prot_latch_mask = s_bq.boot_prot_latch_mask;
    out->adc_gain = s_bq.adc_gain;
    out->adc_offset = s_bq.adc_offset;
    out->fresh = s_bq.fresh;
    out->last_update_ms = s_bq.last_update_ms;
    out->valid = s_bq.frame_valid;
    out->reject_reason = s_bq.reject_reason;

    out->raw_cell[0] = s_bq.raw_cell[0];
    out->raw_cell[1] = s_bq.raw_cell[1];
    out->raw_cell[2] = s_bq.raw_cell[2];
    out->raw_pack = s_bq.raw_pack;
    out->raw_vc3 = s_bq.raw_vc3;
    out->raw_vc4 = s_bq.raw_vc4;
    out->frame_ok_count = s_bq.frame_ok_count;
    out->frame_reject_count = s_bq.frame_reject_count;
    out->reject_streak = s_bq.reject_streak;
    out->reject_since_publish = s_bq.reject_since_publish;
    out->xready_count = s_bq.xready_count;
    out->frame_confirm_count = s_bq.frame_confirm_count;
    out->frame_confirm_reset_count = s_bq.frame_confirm_reset_count;
    out->oor_streak = s_bq.oor_streak;
    out->resample_count = s_bq.resample_count;
    out->strict_window = s_bq.strict_window;
    out->recovery_episode = s_bq.recovery_episode;
    out->verdict_active = s_bq.verdict_active;
    out->verdict_deadline_ms = s_bq.verdict_deadline_ms;
    out->good_publish_streak = s_bq.good_publish_streak;
    out->pending_raw_cell[0] = s_bq.pending_raw_cell[0];
    out->pending_raw_cell[1] = s_bq.pending_raw_cell[1];
    out->pending_raw_cell[2] = s_bq.pending_raw_cell[2];
    out->adc_restart_count = s_bq.adc_restart_count;
    out->adc_restart_attempts = s_bq.adc_restart_attempts;
    out->adc_restart_reason = s_bq.adc_restart_reason;
    out->adc_recovery_failed = s_bq.adc_recovery_failed;
    out->adc_restart_last_ms = s_bq.adc_restart_last_ms;
    out->pre_ctrl1 = s_bq.pre_ctrl1;
    out->pre_ctrl2 = s_bq.pre_ctrl2;
    out->pre_sys_stat = s_bq.pre_sys_stat;
    out->pre_cellbal1 = s_bq.pre_cellbal1;
    out->post_cellbal1 = s_bq.post_cellbal1;
    out->pre_snapshot_valid = s_bq.pre_snapshot_valid;
    out->post_ctrl1 = s_bq.post_ctrl1;
    out->post_ctrl2 = s_bq.post_ctrl2;
    out->post_sys_stat = s_bq.post_sys_stat;
    out->settle_sys_stat = s_bq.settle_sys_stat;
    out->adc_off_ctrl1 = s_bq.adc_off_ctrl1;
    out->adc_off_ctrl2 = s_bq.adc_off_ctrl2;
    out->xready_at_settle = s_bq.xready_at_settle;
    out->last_reject_reason = s_bq.last_reject_reason;
    out->last_reject_cell_mv[0] = s_bq.last_reject_cell_mv[0];
    out->last_reject_cell_mv[1] = s_bq.last_reject_cell_mv[1];
    out->last_reject_cell_mv[2] = s_bq.last_reject_cell_mv[2];
    out->last_reject_pack_mv = s_bq.last_reject_pack_mv;
    out->last_reject_vc3_raw = s_bq.last_reject_vc3_raw;
    out->last_reject_vc4_raw = s_bq.last_reject_vc4_raw;
    out->last_reject_ms = s_bq.last_reject_ms;
    out->last_reject_sys_stat = s_bq.last_reject_sys_stat;
}

uint16_t BQ_GetRefCellMv(const bq_snapshot_t *snap)
{
    uint16_t ref;

    if ((snap == NULL) || (snap->valid == 0u))
    {
        return 0u;
    }
    if (snap->ref_cell2_only != 0u)
    {
        ref = snap->cell_mv[1];
        return ((ref >= BQ_CELL_VALID_MIN_MV) &&
                (ref <= BQ_CELL_VALID_MAX_MV)) ? ref : 0u;
    }

    ref = snap->cell_mv[0];
    if (snap->cell_mv[1] < ref)
    {
        ref = snap->cell_mv[1];
    }
    if (snap->cell_mv[2] < ref)
    {
        ref = snap->cell_mv[2];
    }
    return ((ref >= BQ_CELL_VALID_MIN_MV) &&
            (ref <= BQ_CELL_VALID_MAX_MV)) ? ref : 0u;
}

void BQ_GetBatchQuality(bq_batch_quality_t *out)
{
    uint32_t now_ms;

    if (out == NULL)
    {
        return;
    }
    *out = s_bq.batch;
    now_ms = HAL_GetTick();
    /* A completed batch remains stable while the next normal DMA transaction
     * is in flight, but not beyond the ordinary BQ stale interval. */
    if ((out->complete != 0u) &&
        ((now_ms - out->batch_ms) > APP_BQ_STALE_MS))
    {
        out->complete = 0u;
        out->quality = (uint8_t)BQ_BATCH_INVALID;
    }
}

void BQ_GetCcSample(bq_cc_sample_t *out)
{
    if (out == NULL)
    {
        return;
    }
    *out = s_bq.cc_sample;
}

uint8_t BQ_IsCcFresh(uint32_t now_ms)
{
    if ((s_bq.cc_sample.valid == 0u) ||
        (s_bq.state != BQ_ST_RUN) ||
        (s_bq.voltage_settled == 0u) ||
        (s_bq.protection_ok == 0u) ||
        ((s_bq.sys_stat & BQ_SYS_STAT_PROT_MASK) != 0u) ||
        ((s_bq.sys_stat & (BQ_SYS_STAT_XREADY | BQ_SYS_STAT_OVRD_ALERT)) != 0u))
    {
        return 0u;
    }
    return ((now_ms - s_bq.cc_sample.sample_ms) <= BQ_CC_FRESH_MS) ? 1u : 0u;
}

uint8_t BQ_IsRecovering(void)
{
    if (s_bq.adc_recovery_failed != 0u)
    {
        return 0u;
    }
    return ((s_bq.adc_restart_pending != 0u) ||
            (s_bq.recovery_episode != 0u) ||
            (s_bq.verdict_active != 0u) ||
            (s_bq.state == BQ_ST_CONFIG) ||
            (s_bq.state == BQ_ST_SETTLE) ||
            (s_bq.state == BQ_ST_ADC_RESTART_OFF)) ? 1u : 0u;
}

uint8_t BQ_GetAdcRecoveryFailed(void)
{
    return s_bq.adc_recovery_failed;
}

uint8_t BQ_TakeAdcEvent(void)
{
    uint8_t flags = s_bq.adc_event_flags;

    s_bq.adc_event_flags = 0u;
    return flags;
}

void BQ_RequestFetMask(uint8_t mask)
{
    s_bq.request_fet_mask = (uint8_t)(mask & BQ_FET_MASK_ALL);
}

uint8_t BQ_GetFetMaskRequested(void)
{
    return s_bq.request_fet_mask;
}

uint8_t BQ_GetFetMaskApplied(void)
{
    return s_bq.fet_mask_applied;
}

int BQ_ApplyFetMaskNow(uint8_t mask)
{
    if ((BQ_IsIdle() == 0u) || (s_bq.state != BQ_ST_RUN))
    {
        return -1;
    }

    mask &= BQ_FET_MASK_ALL;
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2,
                         (uint8_t)(BQ_SYS_CTRL2_CC_EN | mask)) != 0)
    {
        return -1;
    }
    s_bq.request_fet_mask = mask;
    s_bq.fet_mask_applied = mask;
    return 0;
}

uint8_t BQ_IsSameStateWriteReady(void)
{
    if ((s_bq.state != BQ_ST_RUN) || (BQ_IsIdle() == 0u) ||
        (s_bq.voltage_settled == 0u) || (s_bq.protection_ok == 0u))
    {
        return 0u;
    }
    if ((s_bq.sys_stat & BQ_BATCH_FAULT_MASK) != 0u)
    {
        return 0u;
    }
    if ((s_bq.request_fet_mask != s_bq.fet_mask_applied) ||
        (s_bq.fet_sync_active != 0u) ||
        (s_bq.sysstat_sync_active != 0u) ||
        (s_bq.pending_sysstat_clear != 0u) ||
        (s_bq.adc_restart_pending != 0u) ||
        (s_bq.verdict_arm_pending != 0u) ||
        (s_bq.verdict_active != 0u) ||
        (s_bq.recovery_episode != 0u) ||
        (s_bq.adc_recovery_failed != 0u) ||
        (s_bq.resample_pending != 0u) ||
        (s_bq.req_read != 0u) ||
        (s_bq.retry != 0u))
    {
        return 0u;
    }
    return 1u;
}

int BQ_ApplyFetMaskIfUnchanged(uint8_t expected_mask)
{
    uint8_t ctrl2 = 0u;
    uint8_t rb = 0u;
    uint8_t mask_field;

    expected_mask &= BQ_FET_MASK_ALL;
    if (BQ_IsSameStateWriteReady() == 0u)
    {
        return -1;
    }
    if ((s_bq.fet_mask_applied != expected_mask) ||
        (s_bq.request_fet_mask != expected_mask))
    {
        return -1;
    }

    Bq_WaitBusIdle();
    if (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &ctrl2) != 0)
    {
        return -2;
    }
    if (((ctrl2 & (BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL)) !=
         (uint8_t)(BQ_SYS_CTRL2_CC_EN | expected_mask)) ||
        ((ctrl2 & BQ_SYS_CTRL2_CC_ONESHOT) != 0u))
    {
        /* 实际寄存器已被保护/外部改写时，绝不用本次写重新使能 FET。 */
        return -1;
    }

    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2, ctrl2) != 0)
    {
        return -2;
    }
    if (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &rb) != 0)
    {
        return -2;
    }
    mask_field = (uint8_t)(BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL);
    if ((rb & mask_field) != (ctrl2 & mask_field))
    {
        return -2;
    }
    return 0;
}

int BQ_ReadRegSafe(uint8_t reg, uint8_t *out)
{
    return BQ_ReadBlockSafe(reg, out, 1u);
}

int BQ_ReadBlockSafe(uint8_t reg, uint8_t *out, uint8_t count)
{
    if ((BQ_IsIdle() == 0u) || (s_bq.state != BQ_ST_RUN))
    {
        return -1;
    }
    return Bq_BlockingReadBlock(reg, out, count);
}

int BQ_LpSniffInit(void)
{
    if (hi2c1.State != HAL_I2C_STATE_RESET)
    {
        (void)HAL_I2C_DeInit(&hi2c1);
    }

    hi2c1.Init.Timing = BQ_I2C_TIMINGR_HSI16_100K;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        return -1;
    }

    /* 与生成配置保持一致；HAL_I2C_Init 不负责这两个滤波设置。 */
    (void)HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 4u);
    return 0;
}

void BQ_LpSniffDeInit(void)
{
    (void)HAL_I2C_DeInit(&hi2c1);
}

int BQ_LpSniffRead(uint8_t reg, uint8_t *out)
{
    if (out == NULL)
    {
        return -1;
    }
    return Bq_BlockingRead1(reg, out);
}

int BQ_LpSniffClearSysStat(uint8_t sys_stat_mask)
{
    if (sys_stat_mask == 0u)
    {
        return 0;
    }
    return (Bq_BlockingWrite(BQ_REG_SYS_STAT, sys_stat_mask) == 0) ? 0 : -1;
}

/* 30 min 储存监测：临时打开 ADC/TEMP 与 CC。FET mask 保持不变（默认 0），
 * 因此 LOAD_PRESENT 仍有效、PACK 输出仍断开。调用者负责等待 ADC 稳定。 */
int BQ_LpMonitorStart(uint8_t fet_mask)
{
    /* LP monitor 开 ADC/CC 前也必须确认均衡关闭。 */
    if (Bq_BalanceOffVerified() != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_VALUE) != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2,
                         (uint8_t)(BQ_SYS_CTRL2_CC_EN |
                                   (fet_mask & BQ_FET_MASK_ALL))) != 0)
    {
        return -1;
    }
    return 0;
}

int BQ_LpMonitorRead(bq_snapshot_t *out)
{
    uint8_t vc[6];
    uint8_t vc3[2];
    uint8_t vc4[2];
    uint8_t bat[2];
    uint8_t ts[2];
    uint8_t cc[2];
    uint8_t ctrl1 = 0u;
    uint8_t ctrl2 = 0u;
    uint8_t protect[3];
    uint8_t stat = 0u;
    uint8_t effective_stat;
    uint16_t raw[3];
    uint16_t raw_pack;
    uint16_t raw_vc3;
    uint16_t raw_vc4;
    uint32_t real_c3;
    uint32_t real_pack;
    uint8_t reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
    uint8_t cells12_ok;
    uint8_t state_ok;
    uint8_t protection_ok;
    int16_t cc_raw;

    if (out == NULL)
    {
        return -1;
    }
    *out = (bq_snapshot_t){0};
    out->valid = 0u;
    out->fresh = 0u;
    out->reject_reason = (uint8_t)BQ_FRAME_REJECT_UNSETTLED;
    /* 3S 正确接法：CELL3 在 VC5-VC4，不能从 VC1 连续读到。
     * VC5/BAT 可信时进入控制值，否则用 VC1/VC2 估算；VC3/VC4 只作 raw 诊断。 */
    if ((Bq_BlockingReadBlock(BQ_REG_VC1, &vc[0], 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_VC2, &vc[2], 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_VC5, &vc[4], 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_VC3, vc3, 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_VC4, vc4, 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_BAT, bat, 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_TS1, ts, 2u) != 0) ||
        (Bq_BlockingReadBlock(BQ_REG_CC, cc, 2u) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &ctrl1) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &ctrl2) != 0) ||
        (Bq_BlockingRead1(BQ_REG_PROTECT1, &protect[0]) != 0) ||
        (Bq_BlockingRead1(BQ_REG_PROTECT2, &protect[1]) != 0) ||
        (Bq_BlockingRead1(BQ_REG_PROTECT3, &protect[2]) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0))
    {
        return -1;
    }

    raw[0] = (uint16_t)((((uint16_t)vc[0] << 8) | vc[1]) & BQ_CELL_ADC_MASK);
    raw[1] = (uint16_t)((((uint16_t)vc[2] << 8) | vc[3]) & BQ_CELL_ADC_MASK);
    raw[2] = (uint16_t)((((uint16_t)vc[4] << 8) | vc[5]) & BQ_CELL_ADC_MASK);
    raw_vc3 = (uint16_t)((((uint16_t)vc3[0] << 8) | vc3[1]) & BQ_CELL_ADC_MASK);
    raw_vc4 = (uint16_t)((((uint16_t)vc4[0] << 8) | vc4[1]) & BQ_CELL_ADC_MASK);
    raw_pack = (uint16_t)(((uint16_t)bat[0] << 8) | bat[1]);
    cc_raw = (int16_t)(((uint16_t)cc[0] << 8) | cc[1]);

    out->raw_cell[0] = raw[0];
    out->raw_cell[1] = raw[1];
    out->raw_cell[2] = raw[2];
    out->raw_pack = raw_pack;
    out->raw_vc3 = raw_vc3;
    out->raw_vc4 = raw_vc4;
    out->cell_mv[0] = Bq_RawToMv(raw[0]);
    out->cell_mv[1] = Bq_RawToMv(raw[1]);
    out->cell_mv[2] = (uint16_t)(((uint32_t)out->cell_mv[0] +
                                  (uint32_t)out->cell_mv[1] + 1u) / 2u);
    out->pack_mv = (uint16_t)((uint32_t)out->cell_mv[0] +
                              (uint32_t)out->cell_mv[1] + out->cell_mv[2]);
    out->cc_raw = cc_raw;
    out->current_ma = ((int32_t)cc_raw * BQ_CURRENT_MA_NUM) / BQ_CURRENT_MA_DEN;
    out->ts1_raw = (uint16_t)((((uint16_t)ts[0] << 8) | ts[1]) & BQ_CELL_ADC_MASK);
    out->raw_sys_stat = stat;
    out->sys_stat = stat;
    out->protect1 = protect[0];
    out->protect2 = protect[1];
    out->protect3 = protect[2];
    out->protection_ok = 0u;
    out->adc_gain = s_bq.adc_gain;
    out->adc_offset = s_bq.adc_offset;
    out->last_update_ms = HAL_GetTick();

    cells12_ok = ((out->cell_mv[0] >= BQ_CELL_VALID_MIN_MV) &&
                  (out->cell_mv[0] <= BQ_CELL_VALID_MAX_MV) &&
                  (out->cell_mv[1] >= BQ_CELL_VALID_MIN_MV) &&
                  (out->cell_mv[1] <= BQ_CELL_VALID_MAX_MV)) ? 1u : 0u;

    /* 先按同批 VC5/BAT 决定参考节，再过滤 OV/UV。LP 读取已经保证各寄存器
     * 阻塞读成功；这里不使用 CRC mask，也不依赖任何 sys_stat/quality。 */
    out->ref_cell2_only = Bq_EvaluateRealC3Pack(
        raw[2], raw_pack, 1u, 1u,
        out->cell_mv[0], out->cell_mv[1],
        &out->log_cell3_mv, &out->log_pack_mv,
        &out->log_cell3_valid, &out->log_pack_valid);
    if (cells12_ok == 0u)
    {
        out->ref_cell2_only = 0u;
    }

    /* LP 中 ADC 关闭，SYS_STAT 的 OV/UV 可能来自 VC5 假锁存。
     * 正常时 cell1/2 实测安全才忽略；异常时只信任 VC2。 */
    effective_stat = stat;
    if (cells12_ok != 0u)
    {
        uint16_t c1 = out->cell_mv[0];
        uint16_t c2 = out->cell_mv[1];

        if (out->ref_cell2_only != 0u)
        {
            if ((c2 >= BQ_CELL_VALID_MIN_MV) &&
                (c2 <= BQ_CELL_VALID_MAX_MV))
            {
                if (c2 <=
                    ((BQ_OV_TRIP_MV > BQ_OV_CLEAR_GUARD_MV) ?
                     (uint16_t)(BQ_OV_TRIP_MV - BQ_OV_CLEAR_GUARD_MV) : 0u))
                {
                    effective_stat &= (uint8_t)~BQ_SYS_STAT_OV;
                }
                if ((uint32_t)c2 >=
                    ((uint32_t)BQ_UV_TRIP_MV + BQ_UV_CLEAR_GUARD_MV))
                {
                    effective_stat &= (uint8_t)~BQ_SYS_STAT_UV;
                }
            }
        }
        else
        {
            if (((c1 >= c2) ? c1 : c2) <=
                ((BQ_OV_TRIP_MV > BQ_OV_CLEAR_GUARD_MV) ?
                 (uint16_t)(BQ_OV_TRIP_MV - BQ_OV_CLEAR_GUARD_MV) : 0u))
            {
                effective_stat &= (uint8_t)~BQ_SYS_STAT_OV;
            }
            if ((uint32_t)((c1 <= c2) ? c1 : c2) >=
                ((uint32_t)BQ_UV_TRIP_MV + BQ_UV_CLEAR_GUARD_MV))
            {
                effective_stat &= (uint8_t)~BQ_SYS_STAT_UV;
            }
        }
    }
    out->sys_stat = effective_stat;

    state_ok = (((effective_stat & BQ_BATCH_FAULT_MASK) == 0u) &&
                ((ctrl1 & (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) ==
                 (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) &&
                ((ctrl2 & (BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL)) ==
                 BQ_SYS_CTRL2_CC_EN)) ? 1u : 0u;
    protection_ok = ((protect[0] == BQ_PROTECT1_VALUE) &&
                     (protect[1] == BQ_PROTECT2_VALUE) &&
                     (protect[2] == BQ_PROTECT3_VALUE)) ? 1u : 0u;
    out->protection_ok = protection_ok;

    /* 控制值逐字段优先真实值；日志值保持 real-or-0。 */
    real_c3 = Bq_RawToMv(raw[2]);
    real_pack = Bq_RawPackToMv(raw_pack);
    Bq_SelectControlVoltages(out->cell_mv[0], out->cell_mv[1],
                             (uint16_t)real_c3, out->log_cell3_valid,
                             (uint16_t)real_pack, out->log_pack_valid,
                             &out->cell_mv[2], &out->pack_mv);

    if ((cells12_ok == 0u) || (state_ok == 0u) || (protection_ok == 0u))
    {
        out->reject_reason = (uint8_t)BQ_FRAME_REJECT_INCOMPLETE;
        return -1;
    }
    if (Bq_ValidateFrameValues(out->cell_mv, &reason) == 0u)
    {
        out->reject_reason = reason;
        return -1;
    }

    out->valid = 1u;
    out->reject_reason = (uint8_t)BQ_FRAME_OK;
    out->fresh = 1u;
    return 0;
}

int BQ_LpMonitorStop(uint8_t fet_mask)
{
    return BQ_PrepareForLpMask(fet_mask);
}

/* ---------------- 复位安全 LP 路径（IWDG 轻量 boot） ---------------- */

static int Bq_LpReadCalibration(void)
{
    uint8_t g1 = 0u;
    uint8_t g2 = 0u;
    uint8_t off = 0u;

    if ((Bq_BlockingRead1(BQ_REG_ADCGAIN1, &g1) != 0) ||
        (Bq_BlockingRead1(BQ_REG_ADCGAIN2, &g2) != 0) ||
        (Bq_BlockingRead1(BQ_REG_ADCOFFSET, &off) != 0))
    {
        return -1;
    }

    s_bq.adc_gain1_raw = g1;
    s_bq.adc_gain2_raw = g2;
    s_bq.adc_gain = (uint8_t)((((g1 >> 2) & 0x03u) << 3) |
                              ((g2 >> 5) & 0x07u));
    s_bq.adc_offset = (int8_t)off;
    return 0;
}

int BQ_LpBootPrepare(bq_lp_status_t *out)
{
    uint8_t ctrl1 = 0u;
    uint8_t ctrl2 = 0u;
    uint8_t stat = 0u;

    if (out == NULL)
    {
        return -1;
    }
    /* LP 轻量 boot 一开始就强制均衡关闭；调试器/旧固件留下的掩码不能存活。 */
    if (Bq_BalanceOffVerified() != 0)
    {
        return -1;
    }
    out->load_present = 0u;
    out->chg_on = 0u;
    out->adc_on = 0u;
    out->cc_on = 0u;
    out->fet_mask = 0u;

    if (Bq_LpReadCalibration() != 0)
    {
        return -1;
    }

    if ((Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &ctrl1) != 0) ||
        (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &ctrl2) != 0))
    {
        return -1;
    }

    /* ADC/TEMP/CC 必须在 LP 中关闭；若上一次状态残留，等一个转换周期再
     * 回读，避免最后一笔 ADC/CC 还在更新。不能比较整个 ctrl1：bit7
     * LOAD_PRESENT 是只读状态，有负载时恒为 1，会把每个 sniff tick 都拖
     * 进 300 ms 等待。 */
    if (((ctrl1 & (BQ_SYS_CTRL1_ADC_EN | BQ_SYS_CTRL1_TEMP_SEL)) != 0u) ||
        ((ctrl2 & (BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL)) != 0u))
    {
        if ((Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_LP) != 0) ||
            (Bq_BlockingWrite(BQ_REG_SYS_CTRL2, 0u) != 0))
        {
            return -1;
        }
        HAL_Delay(300u);
        if ((Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &ctrl1) != 0) ||
            (Bq_BlockingRead1(BQ_REG_SYS_CTRL2, &ctrl2) != 0))
        {
            return -1;
        }
    }

    /* 必须在 300 ms 延时之后重新读 SYS_STAT：最后一笔 CC 可能在等待期间
     * 完成并重新置 CC_READY，用延时前的旧值判断会漏清。OVRD_ALERT 不是
     * 保护故障，但它会把 CHG/DSG 驱动强制为 0，也必须在这里清掉；
     * OV/UV/SCD/OCD 保护锁存保留。 */
    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }
    if ((stat & (BQ_SYS_STAT_CC_READY | BQ_SYS_STAT_XREADY |
                 BQ_SYS_STAT_OVRD_ALERT)) != 0u)
    {
        if (Bq_BlockingWrite(BQ_REG_SYS_STAT,
                             (uint8_t)(BQ_SYS_STAT_CC_READY |
                                       BQ_SYS_STAT_XREADY |
                                       BQ_SYS_STAT_OVRD_ALERT)) != 0)
        {
            return -1;
        }
        HAL_Delay(2u);
        if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
        {
            return -1;
        }
    }

    out->sys_ctrl1 = ctrl1;
    out->sys_ctrl2 = ctrl2;
    out->sys_stat = stat;
    out->load_present = ((ctrl1 & BQ_SYS_CTRL1_LOAD_PRESENT) != 0u) ? 1u : 0u;
    out->chg_on = ((ctrl2 & BQ_SYS_CTRL2_CHG_ON) != 0u) ? 1u : 0u;
    out->adc_on = ((ctrl1 & BQ_SYS_CTRL1_ADC_EN) != 0u) ? 1u : 0u;
    out->cc_on = ((ctrl2 & BQ_SYS_CTRL2_CC_EN) != 0u) ? 1u : 0u;
    out->fet_mask = (uint8_t)(ctrl2 & BQ_FET_MASK_ALL);

    return (((ctrl1 & BQ_SYS_CTRL1_ADC_EN) == 0u) &&
            ((ctrl2 & (BQ_SYS_CTRL2_CC_EN | BQ_FET_MASK_ALL)) == 0u)) ?
           0 : -1;
}

static uint8_t Bq_LpFrameClose(const bq_snapshot_t *a, const bq_snapshot_t *b)
{
    uint8_t i;

    if ((a->valid == 0u) || (b->valid == 0u))
    {
        return 0u;
    }
    /* 正常比较 VC1/VC2；两边都已进入 VC2-only 模式时 VC1 被有意忽略，
     * 只比较 VC2。cell3/pack 不作为该 LP 双帧条件，raw VC5/BAT 仅诊断。 */
    i = ((a->ref_cell2_only != 0u) && (b->ref_cell2_only != 0u)) ? 1u : 0u;
    for (; i < 2u; i++)
    {
        int32_t d = (int32_t)a->raw_cell[i] - (int32_t)b->raw_cell[i];
        if ((d > 4) || (d < -4))
        {
            return 0u;
        }
    }
    return 1u;
}

static int Bq_LpForceOff(void)
{
    uint8_t stat = 0u;

    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, BQ_SYS_CTRL1_LP) != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2, 0u) != 0)
    {
        return -1;
    }
    if (Bq_BalanceOffVerified() != 0)
    {
        return -1;
    }
    HAL_Delay(300u);
    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }
    if ((stat & (BQ_SYS_STAT_CC_READY | BQ_SYS_STAT_XREADY |
                 BQ_SYS_STAT_OVRD_ALERT)) != 0u)
    {
        (void)Bq_BlockingWrite(BQ_REG_SYS_STAT,
                               (uint8_t)BQ_SYS_STAT_IRQ_CLR);
    }
    return 0;
}

int BQ_LpMonitorRun(bq_snapshot_t *out)
{
    bq_snapshot_t first;
    bq_snapshot_t second;
    uint8_t attempt;

    if (out == NULL)
    {
        return -1;
    }
    memset(out, 0, sizeof(*out));

    if (Bq_LpReadCalibration() != 0)
    {
        return -1;
    }

    for (attempt = 0u; attempt < 2u; attempt++)
    {
        if (Bq_LpForceOff() != 0)
        {
            return -1;
        }
        if (BQ_LpMonitorStart(0u) != 0)
        {
            (void)Bq_LpForceOff();
            return -1;
        }
        HAL_Delay(BQ_ADC_SETTLE_MS);
        if (BQ_LpMonitorRead(&first) != 0)
        {
            (void)Bq_LpForceOff();
            return -1;
        }
        HAL_Delay(300u);
        if (BQ_LpMonitorRead(&second) != 0)
        {
            (void)Bq_LpForceOff();
            return -1;
        }

        if (Bq_LpFrameClose(&first, &second) != 0u)
        {
            *out = second;
            if (Bq_LpForceOff() != 0)
            {
                return -1;
            }
            return 0;
        }
    }

    (void)Bq_LpForceOff();
    return -1;
}

static int Bq_BlockingAllowed(void)
{
    return ((BQ_IsIdle() != 0u) && (s_bq.state == BQ_ST_RUN)) ? 0 : -1;
}

int BQ_ShipMode(void)
{
    if (Bq_BlockingAllowed() != 0)
    {
        return -1;
    }

    /* 进 SHIP 前先确认均衡关闭。 */
    if (Bq_BalanceOffVerified() != 0)
    {
        return -1;
    }

    /* 先关 CC/FET，再连续写 SHUT_A/SHUT_B；两次 SYS_CTRL1 之间不得插其他写。 */
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL2, 0x00u) != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, 0x01u) != 0)
    {
        return -1;
    }
    if (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, 0x02u) != 0)
    {
        return -1;
    }
    return 0;
}

int BQ_ClearProtection(uint8_t sys_stat_mask)
{
    if (Bq_BlockingAllowed() != 0)
    {
        return -1;
    }
    if (sys_stat_mask == 0u)
    {
        return 0;
    }
    return (Bq_BlockingWrite(BQ_REG_SYS_STAT, sys_stat_mask) == 0) ? 0 : -1;
}

uint8_t BQ_GetProtectionLatchMask(void)
{
    return (uint8_t)(s_bq.sys_stat & BQ_SYS_STAT_PROT_MASK);
}

uint8_t BQ_GetBootProtectionLatchMask(void)
{
    return s_bq.boot_prot_latch_mask;
}

int BQ_ClearProtectionVerified(uint8_t mask, uint8_t *sys_stat_after)
{
    uint8_t stat = 0u;

    if ((mask == 0u) ||
        ((mask & (uint8_t)~(BQ_SYS_STAT_SCD | BQ_SYS_STAT_OCD)) != 0u) ||
        (Bq_BlockingAllowed() != 0))
    {
        return -1;
    }

    if (Bq_BlockingWrite(BQ_REG_SYS_STAT, mask) != 0)
    {
        return -1;
    }

    /* 写 1 清锁存后回读；SCD/OCD 是硬件比较器，若仍置位只能按真实保护处理。 */
    HAL_Delay(2u);
    if (Bq_BlockingRead1(BQ_REG_SYS_STAT, &stat) != 0)
    {
        return -1;
    }

    s_bq.raw_sys_stat = stat;
    Bq_RefreshEffectiveSysStat();
    if ((stat & mask) == 0u)
    {
        s_bq.boot_prot_latch_mask &= (uint8_t)~mask;
    }
    if (sys_stat_after != NULL)
    {
        *sys_stat_after = stat;
    }
    return ((stat & mask) == 0u) ? 0 : -1;
}

int BQ_WriteProtectionRaw(const uint8_t protect[3], uint8_t ov_trip,
                          uint8_t uv_trip)
{
    if ((protect == NULL) || (Bq_BlockingAllowed() != 0))
    {
        return -1;
    }
    if ((Bq_BlockingWrite(BQ_REG_PROTECT1, protect[0]) != 0) ||
        (Bq_BlockingWrite(BQ_REG_PROTECT2, protect[1]) != 0) ||
        (Bq_BlockingWrite(BQ_REG_PROTECT3, protect[2]) != 0) ||
        (Bq_BlockingWrite(BQ_REG_OV_TRIP, ov_trip) != 0) ||
        (Bq_BlockingWrite(BQ_REG_UV_TRIP, uv_trip) != 0))
    {
        return -1;
    }
    s_bq.ov_trip = ov_trip;
    s_bq.uv_trip = uv_trip;
    Bq_UpdateTripThresholds();
    return 0;
}

int BQ_SetTempSource(uint8_t die_temp)
{
    uint8_t ctrl1 = 0u;

    if (Bq_BlockingAllowed() != 0)
    {
        return -1;
    }
    if (Bq_BlockingRead1(BQ_REG_SYS_CTRL1, &ctrl1) != 0)
    {
        return -1;
    }

    if (die_temp != 0u)
    {
        ctrl1 &= (uint8_t)~BQ_SYS_CTRL1_TEMP_SEL;   /* 0 = 内部 die */
    }
    else
    {
        ctrl1 |= BQ_SYS_CTRL1_TEMP_SEL;             /* 1 = 外部 NTC */
    }
    return (Bq_BlockingWrite(BQ_REG_SYS_CTRL1, ctrl1) == 0) ? 0 : -1;
}

int BQ_ReadTs1RawSafe(uint16_t *raw)
{
    uint8_t data[2];

    if ((raw == NULL) || (Bq_BlockingAllowed() != 0))
    {
        return -1;
    }
    if (Bq_BlockingReadBlock(BQ_REG_TS1, data, 2u) != 0)
    {
        return -1;
    }
    *raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return 0;
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
    /* SMBA 实测时 I2C1_ER 由 SMBHEN/ALERTEN/ERRIE 打开；ALERT 不是普通
     * I2C 错误，先置按钮标志并清 ALERTCF，再交给 HAL 处理其余错误。 */
    if ((I2C1->ISR & I2C_ISR_ALERT) != 0u)
    {
        I2C1->ICR = I2C_ICR_ALERTCF;
        Button_NotifySmbaFromIsr();
    }
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

/**
 ******************************************************************************
 * @file    bq_ic.h
 * @brief   BQ7692003PWR 电池管理前端（I2C1 + 每字节 CRC + DMA）。
 *
 * 事务全部非阻塞：ISR 只置完成/错误标志，状态机在 BQ_Poll（主循环）推进。
 * 复位式 LP 的 BQ_LpBootPrepare/BQ_LpMonitorRun 使用有界阻塞 I2C（轻量 boot
 * 允许）；BQ_PrepareForLpMask 是旧的 NORM/LP 切换 API，保留兼容。
 ******************************************************************************
 */
#ifndef APP_BQ_IC_H
#define APP_BQ_IC_H

#include <stdint.h>

/* ---------------- 寄存器地址 ---------------- */
#define BQ_REG_SYS_STAT     0x00u
#define BQ_REG_CELLBAL1     0x01u
#define BQ_REG_CELLBAL2     0x02u
#define BQ_REG_CELLBAL3     0x03u
#define BQ_REG_SYS_CTRL1    0x04u
#define BQ_REG_SYS_CTRL2    0x05u
#define BQ_REG_PROTECT1     0x06u
#define BQ_REG_PROTECT2     0x07u
#define BQ_REG_PROTECT3     0x08u
#define BQ_REG_OV_TRIP      0x09u
#define BQ_REG_UV_TRIP      0x0Au
#define BQ_REG_CC_CFG       0x0Bu
#define BQ_REG_VC1          0x0Cu   /* 2 字节块读自动递增：VC1_HI/LO */
#define BQ_REG_VC2          0x0Eu
#define BQ_REG_VC3          0x10u
#define BQ_REG_VC4          0x12u
#define BQ_REG_VC5          0x14u

/* 3S 固定接法（数据手册 Table 9-2）：
 *   CELL1 = VC1-VC0, CELL2 = VC2-VC1, CELL3 = VC5-VC4；
 *   VC4-VC3、VC3-VC2 在板上短接。飞线已完成，不再保留 4S/VC3 兼容分支。 */
#define BQ_REG_CELL3         BQ_REG_VC5
#define BQ_REG_BAT          0x2Au
#define BQ_REG_TS1          0x2Cu
#define BQ_REG_CC           0x32u
#define BQ_REG_ADCGAIN1     0x50u
#define BQ_REG_ADCOFFSET    0x51u
#define BQ_REG_ADCGAIN2     0x59u

/* SYS_CTRL2：bit0=CHG_ON，bit1=DSG_ON（BQ769x0 Table 8-8）。 */
#define BQ_SYS_CTRL2_CHG_ON     (1u << 0)
#define BQ_SYS_CTRL2_DSG_ON     (1u << 1)
#define BQ_SYS_CTRL2_CC_ONESHOT (1u << 5)
#define BQ_SYS_CTRL2_CC_EN      (1u << 6)
#define BQ_SYS_CTRL2_DELAY_DIS  (1u << 7)
#define BQ_SYS_CTRL1_TEMP_SEL   (1u << 3)
#define BQ_SYS_CTRL1_ADC_EN     (1u << 4)

#define BQ_FET_MASK_NONE        0x00u
#define BQ_FET_MASK_CHG         BQ_SYS_CTRL2_CHG_ON
#define BQ_FET_MASK_DSG         BQ_SYS_CTRL2_DSG_ON
#define BQ_FET_MASK_ALL         (BQ_FET_MASK_CHG | BQ_FET_MASK_DSG)

/* FET 上电初值（当前用户设为全开）；ChargePolicy_Init 会请求 0，
 * 运行期实际由 ChargePolicy_Task / LP mask 接管。 */
#define BQ_DEFAULT_FET_MASK     BQ_FET_MASK_ALL

#define BQ_SYS_CTRL1_LOAD_PRESENT (1u << 7)

typedef enum
{
    BQ_ST_RESET = 0,    /* 未初始化 */
    BQ_ST_BOOT_PULSE,   /* PB1 高脉冲中（TS1 boot） */
    BQ_ST_BOOT_WAIT,    /* 等待 tBOOTREADY */
    BQ_ST_CONFIG,       /* CC_CFG/增益/控制字初始化序列 */
    BQ_ST_RUN,          /* 正常轮询 */
    BQ_ST_BACKOFF,      /* 通信退避，随后重入 CONFIG */
    BQ_ST_SETTLE,       /* ADC_EN 后等待首帧建立（BQ_ADC_SETTLE_MS） */
    BQ_ST_ADC_RESTART_OFF /* 已强制 ADC/CC/FET 关闭，等待一个 ADC 帧 */
} bq_state_t;

typedef enum
{
    BQ_FRAME_OK = 0,
    BQ_FRAME_REJECT_UNSETTLED = 1,
    BQ_FRAME_REJECT_CELL_RANGE = 2,
    BQ_FRAME_REJECT_PACK_RANGE = 3,
    BQ_FRAME_REJECT_PACK_SUM = 4,
    BQ_FRAME_REJECT_NOT_CONFIGURED = 5,
    BQ_FRAME_REJECT_VC34_SANITY = 6,
    BQ_FRAME_REJECT_INCOMPLETE = 7 /* CRC/状态/保护配置不完整；不按 OOR 重启 */
} bq_frame_reject_t;

typedef enum
{
    BQ_RESTART_REASON_NONE = 0,
    BQ_RESTART_REASON_INIT = 1,
    BQ_RESTART_REASON_LP_WAKE = 2,
    BQ_RESTART_REASON_REJECT_STREAK = 3,
    BQ_RESTART_REASON_XREADY = 4,
    BQ_RESTART_REASON_COMM = 5,
    BQ_RESTART_REASON_PARTIAL = 6 /* 仅关输出后、每次间隔 >=10 s 的 ADC 重试 */
} bq_restart_reason_t;

/* 每个 RUN 脚本完成时的同批读取证据；寄存器是顺序读取，不是 BQ 的原子 ADC 帧。
 * crc_ok_mask 表示本批每个寄存器均经独立 CRC 校验；field_ok_mask 进一步要求
 * 原始格式/量程、保护配置及控制位合理。任何 DMA/CRC 重试污染整批质量。
 * PARTIAL 要求连续两批间隔 >=250 ms、VC1/VC2 范围正常、VC3/4/状态/保护
 * 正常，且 VC5 或 BAT 有明确的单字段范围/格式错误。仅 BAT-sum 不一致无法
 * 判定坏的是 BAT 还是 VC5：归为 INVALID，不猜测故障通道。 */
typedef enum
{
    BQ_BATCH_INVALID = 0,
    BQ_BATCH_FULL_CANDIDATE = 1, /* 合理但尚未完成原有双帧发布 */
    BQ_BATCH_FULL_REAL_OK = 2,   /* 仅本批产生真实成功发布 */
    BQ_BATCH_PARTIAL_CANDIDATE = 3, /* 首批符合窄条件，证据尚未二次确认 */
    BQ_BATCH_PARTIAL_VC5_PACK = 4
} bq_batch_class_t;

#define BQ_FIELD_VC1        (1u << 0)
#define BQ_FIELD_VC2        (1u << 1)
#define BQ_FIELD_VC5        (1u << 2)
#define BQ_FIELD_BAT        (1u << 3)
#define BQ_FIELD_VC3        (1u << 4)
#define BQ_FIELD_VC4        (1u << 5)
#define BQ_FIELD_TS1        (1u << 6)
#define BQ_FIELD_CC         (1u << 7)
#define BQ_FIELD_SYS_STAT   (1u << 8)
#define BQ_FIELD_CTRL1      (1u << 9)
#define BQ_FIELD_CTRL2      (1u << 10)
#define BQ_FIELD_PROTECTION (1u << 11) /* PROTECT1/2/3 必须都读回一致 */
#define BQ_FIELD_ALL        ((1u << 12) - 1u)
#define BQ_EST_CELL3        (1u << 0)
#define BQ_EST_PACK         (1u << 1)

typedef struct
{
    uint32_t batch_seq;          /* 完成 RUN 脚本才增加；0 为尚无完成批次 */
    uint32_t batch_ms;           /* 该批读取结束的 HAL ms（非 RTC 时间） */
    uint32_t sample_start_ms;    /* 该批发起时刻；连续批次 >=250 ms */
    uint32_t published_seq;      /* 真正发布成功的累积序号，候选帧不增加 */
    uint32_t published_ms;       /* 最后一次真实发布；不随部分批次变化 */
    uint16_t crc_ok_mask;        /* 本批实际经 CRC 的各寄存器 */
    uint16_t field_ok_mask;      /* 本批 CRC + 合理性/配置校验 */
    uint16_t raw_cell[3];       /* VC1/VC2/VC5，本批实际读值 */
    uint16_t cell_mv[3];        /* 本批实测换算值（无效时仅供诊断） */
    uint16_t raw_pack;
    uint16_t pack_mv;
    uint16_t raw_vc3;
    uint16_t raw_vc4;
    uint16_t ts1_raw;         /* 同批 TS1（CRC+14-bit 格式），不验证 NTC 物理接线 */
    uint16_t estimated_cell3_mv; /* 仅 estimated_mask 指明时供显示/记录 */
    uint16_t estimated_pack_mv;  /* 非独立电压观测、绝不可用作保护 */
    int16_t cc_raw;            /* 本批 BQ CC；不等于发布帧的 cc_raw */
    int32_t current_ma;        /* 同批 CC 换算，非独立物理传感器 */
    uint8_t sys_stat;         /* 本批两次状态读取的故障位并集 */
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t protect1;
    uint8_t protect2;
    uint8_t protect3;
    uint8_t estimated_mask;   /* BQ_EST_*；没有估算时必为 0 */
    uint8_t partial_confirm_count; /* 0/1/2，只有达到 2 才归类 PARTIAL */
    uint8_t complete;         /* 最近一次完整 RUN 批次的完整性；正常下一次 DMA 期间保持，失败即清零 */
    uint8_t quality;          /* bq_batch_class_t；绝不从 reject_reason 猜 */
} bq_batch_quality_t;

typedef struct
{
    int16_t cc_raw;
    int32_t current_ma;
    uint32_t sample_seq;       /* 每次确认新 CC 转换后才增加 */
    uint32_t sample_ms;        /* 本次 CRC 完整读取结束的 HAL ms */
    uint32_t batch_seq;        /* 来源完整批次序号；电压可无效 */
    uint8_t valid;             /* CC_READY 且 >=250 ms、新 CC CRC、状态/配置可信 */
} bq_cc_sample_t;

typedef struct
{
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    int16_t  cc_raw;
    int32_t  current_ma;    /* CC 换算电流，带符号（250 ms 平均，未板级校准） */
    uint16_t ts1_raw;
    uint8_t  sys_stat;
    uint8_t  protect1;
    uint8_t  protect2;
    uint8_t  protect3;
    uint8_t  protection_ok;
    uint8_t  boot_prot_latch_mask; /* 本次 boot/LP 唤醒首次配置前已锁存 */
    uint8_t  adc_gain;      /* 365..396 µV/LSB 的偏移量 */
    int8_t   adc_offset;    /* mV */
    uint8_t  fresh;         /* 最近一帧是有效帧且 last_update_ms 在老化窗口内 */
    uint32_t last_update_ms;
    uint8_t  valid;         /* 最近一次完整帧通过电压/包压校验 */
    uint8_t  reject_reason; /* bq_frame_reject_t；0=OK */
    int16_t  pack_residual_mv; /* pack_mv - (cell0+cell1+cell2) */

    /* 调试/恢复诊断；发布值仍只用上面的 cell_mv/pack_mv。 */
    uint16_t raw_cell[3];
    uint16_t raw_pack;
    uint16_t raw_vc3;
    uint16_t raw_vc4;
    uint32_t frame_ok_count;
    uint32_t frame_reject_count;
    uint16_t reject_streak;
    uint16_t reject_since_publish;
    uint16_t xready_count;
    uint8_t  frame_confirm_count;
    uint32_t frame_confirm_reset_count;
    uint16_t oor_streak;        /* 连续超出合理性范围的帧数 */
    uint16_t resample_count;    /* OOR 触发的快速重采样次数 */
    uint8_t  strict_window;     /* 1=下次首次发布用 ±4 raw LSB 严格确认 */
    uint8_t  recovery_episode;  /* 1=ADC 恢复回合进行中 */
    uint8_t  verdict_active;    /* 1=本轮 ADC 重启等待“有果” */
    uint32_t verdict_deadline_ms;
    uint16_t good_publish_streak; /* 连续成功发布帧数，5 帧清除故障 */
    uint16_t pending_raw_cell[3];
    uint16_t pending_raw_pack;
    uint32_t adc_restart_count;
    uint8_t  adc_restart_attempts;
    uint8_t  adc_restart_reason;
    uint8_t  adc_recovery_failed;
    uint32_t adc_restart_last_ms;
    uint8_t  pre_ctrl1;
    uint8_t  pre_ctrl2;
    uint8_t  pre_sys_stat;
    uint8_t  pre_cellbal1;
    uint8_t  pre_snapshot_valid;
    uint8_t  post_ctrl1;
    uint8_t  post_ctrl2;
    uint8_t  post_sys_stat;
    uint8_t  settle_sys_stat;
    uint8_t  adc_off_ctrl1;
    uint8_t  adc_off_ctrl2;
    uint8_t  xready_at_settle;
    uint8_t  last_reject_reason;
    uint8_t  last_reject_sys_stat;
    uint32_t last_reject_ms;
    uint16_t last_reject_cell_mv[3];
    uint16_t last_reject_pack_mv;
    uint16_t last_reject_vc3_raw;
    uint16_t last_reject_vc4_raw;
} bq_snapshot_t;

void BQ_Init(void);
void BQ_ReInitAfterLp(void);
void BQ_DeInitForLp(void);

/* M1 基础 API：FET mask 位定义 bit0=CHG、bit1=DSG。
 * Request 只更新影子；Apply 必须在主循环、BQ_IsIdle() 且 RUN 状态调用。 */
void BQ_RequestFetMask(uint8_t mask);
uint8_t BQ_GetFetMaskRequested(void);
uint8_t BQ_GetFetMaskApplied(void);
int BQ_ApplyFetMaskNow(uint8_t mask);

/* 旧的非复位 LP 进入 API（保留兼容）：指定 FET mask（默认 0x00）。
 * 返回 0=成功，-1=通信/回读失败，-2=正在 ADC 重启/配置，暂不允许进入 LP。 */
int BQ_PrepareForLpMask(uint8_t fet_mask);

/* 主循环有界阻塞读（1..8 个连续寄存器，每字节 CRC）。
 * 仅用于初始化/模式切换/诊断，不要与 DMA 事务并发。 */
int BQ_ReadRegSafe(uint8_t reg, uint8_t *out);
int BQ_ReadBlockSafe(uint8_t reg, uint8_t *out, uint8_t count);

/* LP 轻量 boot 嗅探专用：HSI16 下重新初始化 I2C（无 DMA），阻塞读单个寄存器。
 * 进入前必须已经 BQ_DeInitForLp()；嗅探结束调用 BQ_LpSniffDeInit()。 */
int BQ_LpSniffInit(void);
void BQ_LpSniffDeInit(void);
int BQ_LpSniffRead(uint8_t reg, uint8_t *out);

/* LP 储存监测（30 min）底层 API：在 BQ_LpSniffInit() 之后调用。当前 30 min
 * 路径使用复位安全的 BQ_LpMonitorRun()；Start 临时打开 ADC/TEMP 与 CC，不改
 * FET，Stop 恢复 LP 状态并清 CC_READY/XREADY。 */
int BQ_LpMonitorStart(uint8_t fet_mask);
int BQ_LpMonitorRead(bq_snapshot_t *out);
int BQ_LpMonitorStop(uint8_t fet_mask);

/* LP 嗅探期间清除 SYS_STAT 的指定位（只建议清 CC_READY/XREADY，保护位保留）。 */
int BQ_LpSniffClearSysStat(uint8_t sys_stat_mask);

/* 复位安全 LP 路径：不依赖 s_bq.state / BQ_IsIdle()，供 IWDG 复位后的轻量
 * boot 使用。调用前先 BQ_LpSniffInit()，结束后 BQ_LpSniffDeInit()。 */
typedef struct
{
    uint8_t sys_ctrl1;
    uint8_t sys_ctrl2;
    uint8_t sys_stat;
    uint8_t load_present;
    uint8_t chg_on;
    uint8_t adc_on;
    uint8_t cc_on;
    uint8_t fet_mask;
} bq_lp_status_t;

int BQ_LpBootPrepare(bq_lp_status_t *out);
#define BQ_LP_MONITOR_VOLTAGE_ANOMALY (-2)
/* 复位安全 LP：0=两批独立实测确认，-2=CRC 完整且局限 VC5/BAT
 * 的电压数据异常，-1=通信/CRC/VC1/VC2/VC3/VC4/状态/保护故障或未知；
 * -2 只用于归档/唤醒分级，不用估算电压恢复 FET/保护。 */
int BQ_LpMonitorRun(bq_snapshot_t *out);

#define BQ_SYS_STAT_OCD         (1u << 0)
#define BQ_SYS_STAT_SCD         (1u << 1)
#define BQ_SYS_STAT_OV          (1u << 2)
#define BQ_SYS_STAT_UV          (1u << 3)
#define BQ_SYS_STAT_OVRD_ALERT  (1u << 4)   /* 非保护：ALERT 被外部强制拉高 */
#define BQ_SYS_STAT_XREADY      (1u << 5)
#define BQ_SYS_STAT_CC_READY    (1u << 7)
#define BQ_SYS_STAT_PROT_MASK   (BQ_SYS_STAT_OCD | BQ_SYS_STAT_SCD | \
                                 BQ_SYS_STAT_OV | BQ_SYS_STAT_UV)
/* 只有这些位是真正的保护/芯片故障唤醒；CC_READY/OVRD_ALERT 不是。 */
#define BQ_SYS_STAT_ALERT_WAKE_MASK (BQ_SYS_STAT_PROT_MASK | \
                                     BQ_SYS_STAT_XREADY)

/* 保护锁存查询/验证式清除：清除只允许 SCD/OCD，且要求 BQ idle/RUN。
 * OV/UV 的假锁存清理仍由 bq_ic 内部的稳定帧逻辑负责。 */
uint8_t BQ_GetProtectionLatchMask(void);
uint8_t BQ_GetBootProtectionLatchMask(void);
int BQ_ClearProtectionVerified(uint8_t mask, uint8_t *sys_stat_after);

/* M1b 预留 API：均衡/SHIP/保护/温度源。全部要求主循环、BQ_IsIdle()、RUN。 */
int BQ_SetBalanceMask(uint8_t mask);
int BQ_ShipMode(void);
int BQ_ClearProtection(uint8_t sys_stat_mask);
int BQ_WriteProtectionRaw(const uint8_t protect[3], uint8_t ov_trip,
                          uint8_t uv_trip);
int BQ_SetTempSource(uint8_t die_temp);   /* 1=内部 die，0=外部 NTC */
int BQ_ReadTs1RawSafe(uint16_t *raw);

void BQ_Poll(void);

/* 主循环请求一轮电芯数据读取（500 ms 节奏由 main_app 给出）。 */
void BQ_RequestRead(void);

/* 数据新鲜度（now_ms 与最后一次完整快照的时间差 ≤ APP_BQ_STALE_MS）。 */
uint8_t BQ_IsFresh(uint32_t now_ms);

void BQ_GetSnapshot(bq_snapshot_t *out);

/* 独立于旧快照的同批证据。batch==0/quality==INVALID 表示尚无可用批次，
 * 对同一批只处理一次 batch_seq；候选不是已发布帧。
 * CC 是 BQ 的单一电流 ADC，CC_READY 是新转换的证据，不是冗余测量。
 * CC valid 可保持到下批；消费时一定检查 age 与 sample_seq 是否前进。
 * 如没有观察到 CC_READY（或脚本中状态/CTRL/CRC 失败），CC 立即失效。
 * 不得用估算值改写 bq_snapshot_t.valid/fresh 或保护判据。 */
void BQ_GetBatchQuality(bq_batch_quality_t *out);
void BQ_GetCcSample(bq_cc_sample_t *out);
uint8_t BQ_IsCcFresh(uint32_t now_ms);

/* 仅对严格 PARTIAL 且 FET 请求/影子均为 0，按 >=10 s 间隔触发受控重启。
 * 1=排队成功，0=不合条件/正在等待；真正通信/XREADY 故障继续走原路径。
 * 两次额度耗尽时允许按冷却重试，但不清故障诊断；四个真实发布帧才复位。 */
uint8_t BQ_RequestPartialRetry(void);

/* ADC 受控重启/恢复状态，主循环和自检只读使用。 */
uint8_t BQ_IsRecovering(void);
uint8_t BQ_GetAdcRecoveryFailed(void);

/* ADC 恢复过程的一次性事件位；由主循环取走并写黑匣子，bsp 不直接写 Flash。 */
#define BQ_ADC_EVENT_RESTART    (1u << 0)
#define BQ_ADC_EVENT_FAULT      (1u << 1)
#define BQ_ADC_EVENT_RECOVERED  (1u << 2)
uint8_t BQ_TakeAdcEvent(void);

bq_state_t BQ_GetState(void);

/* 无在飞事务且无脚本在跑：LP 进入前用它确认总线空闲。 */
uint8_t BQ_IsIdle(void);

#endif /* APP_BQ_IC_H */

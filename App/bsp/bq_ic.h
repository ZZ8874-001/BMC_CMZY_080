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
#define BQ_REG_CELLBAL1     0x01u   /* 必须始终为 0；本工程从不启用均衡 */
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
 *   VC4-VC3、VC3-VC2 在板上短接。飞线已完成，不再保留 4S/VC3 兼容分支。
 * 注意：本板 VC5/CELL3 与 BAT/PACK 可能间歇异常。控制值在真实值通过
 * 合理性检查时优先采用实测，否则回退为 VC1/VC2 估算（cell3 = 两节均值，
 * pack = 三节和）。VC3/VC4 始终只作 raw 诊断。 */
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

#define BQ_DEFAULT_FET_MASK     BQ_FET_MASK_NONE

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
    BQ_FRAME_REJECT_CELL_RANGE = 2,   /* 仅 VC1/VC2 数值越界 */
    BQ_FRAME_REJECT_NOT_CONFIGURED = 5,
    BQ_FRAME_REJECT_INCOMPLETE = 7    /* CRC/状态/保护配置不完整；不按 OOR 重启 */
} bq_frame_reject_t;

typedef enum
{
    BQ_RESTART_REASON_NONE = 0,
    BQ_RESTART_REASON_INIT = 1,
    BQ_RESTART_REASON_LP_WAKE = 2,
    BQ_RESTART_REASON_REJECT_STREAK = 3,
    BQ_RESTART_REASON_XREADY = 4,
    BQ_RESTART_REASON_COMM = 5
} bq_restart_reason_t;

/* 每个 RUN 脚本完成时的同批读取证据；寄存器是顺序读取，不是 BQ 的原子 ADC 帧。
 * crc_ok_mask 表示本批每个寄存器均经独立 CRC 校验；field_ok_mask 进一步要求
 * 必需字段的格式/量程、保护配置及控制位合理。VC5/BAT/VC3/VC4 是 optional
 * 诊断字段：读数异常只影响 raw 诊断，绝不影响 VC1/VC2 发布。 */
typedef enum
{
    BQ_BATCH_INVALID = 0,
    BQ_BATCH_FULL_CANDIDATE = 1, /* 合理但尚未完成双帧发布 */
    BQ_BATCH_FULL_REAL_OK = 2    /* 仅本批产生真实成功发布 */
} bq_batch_class_t;

#define BQ_FIELD_VC1        (1u << 0)
#define BQ_FIELD_VC2        (1u << 1)
#define BQ_FIELD_VC5        (1u << 2)  /* optional raw VC5 诊断 */
#define BQ_FIELD_BAT        (1u << 3)  /* optional raw BAT 诊断 */
#define BQ_FIELD_VC3        (1u << 4)  /* optional raw VC3 诊断 */
#define BQ_FIELD_VC4        (1u << 5)  /* optional raw VC4 诊断 */
#define BQ_FIELD_TS1        (1u << 6)
#define BQ_FIELD_CC         (1u << 7)
#define BQ_FIELD_SYS_STAT   (1u << 8)
#define BQ_FIELD_CTRL1      (1u << 9)
#define BQ_FIELD_CTRL2      (1u << 10)
#define BQ_FIELD_PROTECTION (1u << 11) /* PROTECT1/2/3 必须都读回一致 */
#define BQ_FIELD_ALL        ((1u << 12) - 1u)
/* 帧发布只依赖这些字段；VC5/BAT/VC3/VC4 不在此列。 */
#define BQ_FIELD_REQUIRED   (BQ_FIELD_VC1 | BQ_FIELD_VC2 | BQ_FIELD_TS1 | \
                             BQ_FIELD_CC | BQ_FIELD_SYS_STAT | BQ_FIELD_CTRL1 | \
                             BQ_FIELD_CTRL2 | BQ_FIELD_PROTECTION)

typedef struct
{
    uint32_t batch_seq;          /* 完成 RUN 脚本才增加；0 为尚无完成批次 */
    uint32_t batch_ms;           /* 该批读取结束的 HAL ms（非 RTC 时间） */
    uint32_t sample_start_ms;    /* 该批发起时刻；连续批次 >=250 ms */
    uint32_t published_seq;      /* 真正发布成功的累积序号，候选帧不增加 */
    uint32_t published_ms;       /* 最后一次真实发布；不随候选批次变化 */
    uint16_t crc_ok_mask;        /* 本批实际经 CRC 的各寄存器 */
    uint16_t field_ok_mask;      /* 本批 CRC + 必需字段合理性/配置校验 */
    uint16_t raw_cell[3];        /* VC1/VC2/VC5 原始码；VC5 仅诊断 */
    uint16_t cell_mv[3];         /* [0]=VC1 实测, [1]=VC2 实测,
                                  * [2]=真实 VC5 可信值，否则两节均值 */
    uint16_t raw_pack;           /* BAT 原始码，仅诊断 */
    uint16_t pack_mv;            /* 真实 BAT 可信值，否则三节估算和（控制值） */
    uint16_t raw_vc3;
    uint16_t raw_vc4;
    uint16_t ts1_raw;            /* 同批 TS1（CRC+14-bit 格式） */
    uint16_t log_cell3_mv;       /* 真实 VC5：可信时记录实测，否则 0 */
    uint16_t log_pack_mv;        /* 真实 BAT：可信时记录实测，否则 0 */
    int16_t cc_raw;              /* 本批 BQ CC；不等于发布帧的 cc_raw */
    int32_t current_ma;          /* 同批 CC 换算，非独立物理传感器 */
    uint8_t sys_stat;            /* 本批过滤后的有效故障位并集 */
    uint8_t raw_sys_stat;        /* 本批原始故障位并集，仅诊断 */
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t protect1;
    uint8_t protect2;
    uint8_t protect3;
    uint8_t log_cell3_valid;     /* 真实 VC5 通过 1..5 V 合理性检查 */
    uint8_t log_pack_valid;      /* 真实 BAT 通过范围 + 三节残差检查 */
    uint8_t ref_cell2_only;      /* 1=VC5/BAT 不可信，保护/SOC 只看 VC2 */
    uint8_t complete;            /* 必需字段完整；optional 诊断失败不影响 */
    uint8_t quality;             /* bq_batch_class_t */
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
    uint16_t cell_mv[3];    /* [0]=VC1, [1]=VC2, [2]=真实 VC5 或两节均值 */
    uint16_t pack_mv;       /* 真实 BAT 或三节估算和 */
    uint16_t log_cell3_mv;  /* 真实 VC5：可信时记录实测，否则 0 */
    uint16_t log_pack_mv;   /* 真实 BAT：可信时记录实测，否则 0 */
    int16_t  cc_raw;
    int32_t  current_ma;    /* CC 换算电流，带符号（250 ms 平均，未板级校准） */
    uint16_t ts1_raw;
    uint8_t  sys_stat;      /* 过滤后的有效 SYS_STAT（假 OV/UV 已屏蔽） */
    uint8_t  raw_sys_stat;  /* 原始 SYS_STAT，仅诊断 */
    uint8_t  protect1;
    uint8_t  protect2;
    uint8_t  protect3;
    uint8_t  protection_ok;
    uint8_t  boot_prot_latch_mask; /* 本次 boot/LP 唤醒首次配置前已锁存 */
    uint8_t  adc_gain;      /* 365..396 µV/LSB 的偏移量 */
    int8_t   adc_offset;    /* mV */
    uint8_t  fresh;         /* 最近一帧是有效帧且 last_update_ms 在老化窗口内 */
    uint32_t last_update_ms;
    uint8_t  valid;         /* 最近一次完整帧通过 VC1/VC2 与配置校验 */
    uint8_t  reject_reason; /* bq_frame_reject_t；0=OK */
    uint8_t  log_cell3_valid; /* 真实 VC5 可信，可用于黑匣子/StateStore */
    uint8_t  log_pack_valid;  /* 真实 BAT 可信，可用于黑匣子/StateStore */
    uint8_t  ref_cell2_only;  /* 1=VC5/BAT 不可信，保护/SOC 用 VC2 */
    uint8_t  voltage_backoff; /* 1=BQ OV/UV 已暂时退避，软件 VC2 OV/UV 接管 */
    uint8_t  voltage_backoff_pending; /* 等待主循环暂停/恢复 BQ OV/UV */
    uint8_t  voltage_backoff_restore_pending; /* 3 个正常批次后等待恢复 */
    uint8_t  voltage_backoff_failed;  /* 退避或恢复写/回读失败，FET 保持关闭 */
    uint8_t  voltage_backoff_normal_streak; /* 连续正常发布批次数 */
    uint8_t  voltage_ov_trip_normal;  /* 退避前的正常 OV_TRIP 码 */
    uint8_t  voltage_uv_trip_normal;  /* 退避前的正常 UV_TRIP 码 */
    uint8_t  voltage_hold_mask;       /* 退避/重锁存期间禁止重开的 FET 位 */
    uint8_t  voltage_relatch_mask;    /* 退避期间重新出现的 raw OV/UV */
    uint8_t  voltage_backoff_clear_done; /* 本次退避已完成一次性 OV/UV 清除 */
    uint8_t  voltage_backoff_clear_failed; /* 清除失败，仅诊断；active 期间仍忽略 OV/UV */
    uint32_t voltage_backoff_probe_until_ms; /* 旧观察窗字段，保持 0 */
    uint8_t  voltage_lp_restore_failed;
    uint16_t ov_trip_mv;      /* 实际写入 OV_TRIP 反算值 */
    uint16_t uv_trip_mv;      /* 实际写入 UV_TRIP 反算值 */
    uint16_t cell12_meas_mv[2]; /* 最近一次 VC1/VC2 实测，用于 OV/UV 归因 */
    uint8_t  cell12_meas_valid; /* 上述值在 1..5 V */
    uint8_t  cell12_measured_once;

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
    uint16_t good_publish_streak; /* 连续成功发布帧数，4 帧清除故障 */
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
    uint8_t  post_cellbal1;     /* 每次开 ADC/进 LP 后回读，必须为 0 */
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

/* 同态重写资格：BQ RUN/总线空闲，无 FET/SYS_STAT 同步，无 ADC 重启/恢复/
 * 重采样/verdict，request==applied，保护配置已确认。只读，无 I2C。 */
uint8_t BQ_IsSameStateWriteReady(void);

/* 仅当实际 SYS_CTRL2 与 expected_mask 一致时，把同一 FET 状态重写一次。
 * 0=成功；-1=不允许/实际状态不符/影子不一致；-2=I2C 写或回读失败。
 * 不改变 request_fet_mask / fet_mask_applied，不改变策略请求。 */
int BQ_ApplyFetMaskIfUnchanged(uint8_t expected_mask);

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
/* 复位安全 LP：0=两批独立实测确认（cell1/2 实测；cell3/pack 可信时实测，
 * 否则估算），-1=通信/CRC/VC1/VC2/状态/保护故障或未知。VC3/VC4 只作
 * raw 诊断，不产生“电压异常”专用返回码。 */
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
/* LP 唤醒只用不受 VC5/BAT 假锁存影响的位：SCD/OCD/XREADY。
 * OV/UV 在 LP 中 ADC 关闭，无法区分真实 cell1/2 与 VC5 假触发；
 * 真实 OV/UV 由 30 min monitor 测到 cell1/2 后按 fault wake 处理。 */
#define BQ_SYS_STAT_ALERT_WAKE_MASK (BQ_SYS_STAT_SCD | BQ_SYS_STAT_OCD | \
                                     BQ_SYS_STAT_XREADY)

/* 保护锁存查询/验证式清除：清除只允许 SCD/OCD，且要求 BQ idle/RUN。
 * OV/UV 的假锁存清理仍由 bq_ic 内部的稳定帧逻辑负责。 */
uint8_t BQ_GetProtectionLatchMask(void);
uint8_t BQ_GetBootProtectionLatchMask(void);
int BQ_ClearProtectionVerified(uint8_t mask, uint8_t *sys_stat_after);

/* 保护/温度源 API。全部要求主循环、BQ_IsIdle()、RUN。
 * 均衡永久关闭：不再提供 BQ_SetBalanceMask()。 */
int BQ_ShipMode(void);
int BQ_ClearProtection(uint8_t sys_stat_mask);
int BQ_WriteProtectionRaw(const uint8_t protect[3], uint8_t ov_trip,
                          uint8_t uv_trip);
int BQ_SetTempSource(uint8_t die_temp);   /* 1=内部 die，0=外部 NTC */
int BQ_ReadTs1RawSafe(uint16_t *raw);

/* 保护/SOC 的唯一参考节：正常=三节最低值；ref_cell2_only=1 时只取 VC2。
 * 引用无效时返回 0（调用方必须 fail-closed）。 */
uint16_t BQ_GetRefCellMv(const bq_snapshot_t *snap);

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

/* ADC 受控重启/恢复状态，主循环和自检只读使用。 */
uint8_t BQ_IsRecovering(void);

/* VC5/BAT 不可信时，BQ OV/UV 硬件保护暂时退避，由 Protection 的 VC2
 * 软件 OV/UV 接管；SCD/OCD、OVRD_ALERT、XREADY 仍保持有效。 */
uint8_t BQ_IsVoltageBackoffActive(void);
uint8_t BQ_IsVoltageBackoffPending(void);
uint8_t BQ_IsVoltageBackoffFailed(void);
uint8_t BQ_GetVoltageFetHoldMask(void);
int BQ_ServiceVoltageBackoff(void);          /* 主循环、BQ RUN/idle 调用 */
int BQ_RestoreVoltageProtectionForLp(void);  /* NORM 进 LP 前尽力恢复 */
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

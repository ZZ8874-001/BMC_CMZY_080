/**
 ******************************************************************************
 * @file    soc_est.h
 * @brief   li-ion-soc-soh-toolkit 适配层：BQ 快照 -> SOC/SOH -> NVM。
 *
 * 约定：
 *   - 输入使用 BQ 参考节（正常三节最低值；VC5/BAT 异常时仅 VC2）
 *     和 BQ CC 电流；
 *   - 只在 BQ 快照时间戳变化时喂一帧（约 500 ms）；
 *   - TS1 温度按 MF52（10 kΩ，B=3950）+ 10 kΩ 上拉到 5 V 换算；
 *     实际上拉不同时修改 SOC_EST_TS1_PULL_MV/OHM 和 NTC 表；
 *   - SOC_EST_CAL_VALID=1 允许 SOC/SOH 参与 3.1 V 低电判断和显示；
 *     OCV/tau 仍有占位成分，2.8 V 电压硬保护仍是独立兜底。
 ******************************************************************************
 */
#ifndef APP_SOC_EST_H
#define APP_SOC_EST_H

#include <stdint.h>

#include "board.h"
#include "bsp/bq_ic.h"

/* 当前 NMC 1500 mAh 的 SOC/SOH 已允许参与低电判断；2.8 V 电压硬保护
 * 仍是独立兜底，不能仅凭 SOC 关闭/开放放电。 */
#define SOC_EST_CAL_VALID            1u

/* TS1 NTC：MF52（10 kΩ @25°C，B=3950），10 kΩ 上拉到 5 V。
 * 若实际改成 3.3 V/VC1 或不同上拉，只改这两个值和 NTC 表。 */
#define SOC_EST_TS1_PULL_MV          5000u
#define SOC_EST_TS1_PULL_OHM         10000u

/* 库约定放电为正；符号统一由 board.h 的 BQ_CC_DISCHARGE_SIGN 决定，
 * 不在 SOC 层单独维护第二套。 */
#define SOC_EST_CURRENT_SIGN         (BQ_CC_DISCHARGE_SIGN)

typedef enum
{
    SOC_EST_QUALITY_NONE = 0,       /* 当前无可信新样本，SOC 仅保持旧值 */
    SOC_EST_QUALITY_REAL = 1        /* 已发布的三节 + BAT 合成/实测帧 */
} soc_est_quality_t;

typedef struct
{
    uint8_t  frame_valid;      /* 仅完整真实 BQ 帧可用于通常的算法/保护 */
    uint8_t  temp_valid;       /* TS1 温度换算有效（无新 TS1 则为 0） */
    uint8_t  cal_valid;        /* 标定标记（当前 =SOC_EST_CAL_VALID，OCV/tau 仍有占位） */
    soc_est_quality_t quality; /* 本次 SOC 更新所用电压的来源 */
    uint8_t  soc_estimated;    /* 历史估算标志；正常真实帧静置重同步后清零 */
    int32_t  soc01;            /* 0..10000 */
    int32_t  soh01;            /* 0..10000 */
    int16_t  temp_dc;          /* 0.1 °C */
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    uint16_t ref_cell_mv;      /* 保护/SOC 参考节：正常最低节，异常 VC2 */
    uint8_t  ref_cell2_only;   /* 1=VC5/BAT 不可信，SOC/SOH 只用 VC2 */
    uint16_t ts1_raw;
    uint16_t ts1_mv;
    int32_t  current_ma;
    uint32_t update_count;
    uint32_t last_update_ms;   /* 最后一次真实完整发布帧，不随 partial 更新 */
    uint32_t sample_ms;        /* 最后一次成功输入 SOC 的新实测帧/独立 CC 时间戳 */
    uint32_t source_batch_seq; /* partial 的同批序号；真实帧仅匹配新发布 batch */
} soc_est_snapshot_t;

void SocEst_Init(void);

/* 主循环在 BQ_Poll 后调用；只消费 fresh/valid 的完整真实发布帧。 */
void SocEst_OnSnapshot(const bq_snapshot_t *snap, uint32_t now_ms);

/* 无法证明当前批次可信时清质量与温度有效位，保留显示 SOC 值而不学习。 */
void SocEst_InvalidateInput(void);

/* NVM 去抖落盘；主循环每圈调用。 */
void SocEst_Task(void);

/* LP 进入前调用：有未落盘 SOH 学习结果则立即保存。 */
void SocEst_PrepareForLp(void);

/* 换电/新电池后由 M5 调用：恢复标称容量并清学习计数。 */
void SocEst_ResetForNewPack(void);

uint8_t SocEst_IsValid(void);       /* 标定 + 本帧 + 温度都有效才为 1 */
uint8_t SocEst_IsTempValid(void);
int32_t SocEst_GetSoc01(void);
uint8_t SocEst_GetSocPercent(void);
int32_t SocEst_GetSoh01(void);
int16_t SocEst_GetTempDc(void);
void SocEst_GetSnapshot(soc_est_snapshot_t *out);

#endif /* APP_SOC_EST_H */

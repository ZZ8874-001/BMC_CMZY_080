/**
 ******************************************************************************
 * @file    soc_est.h
 * @brief   li-ion-soc-soh-toolkit 适配层：BQ 快照 -> SOC/SOH -> StateStore/NVM。
 *
 * 约定：
 *   - 输入只使用三节中的最低节电压（库是 1S 模型）和 BQ CC 电流；
 *   - 只在 BQ 快照时间戳变化时喂一帧（约 500 ms）；
 *   - TS1 温度按 MF52（10 kΩ，B=3950）+ 10 kΩ 上拉到 5 V 换算；
 *     实际上拉不同时修改 SOC_EST_TS1_PULL_MV/OHM 和 NTC 表；
 *   - 三元锂（NMC）标定完成前 SOC_EST_CAL_VALID=0，SocEst_IsValid() 返回 0，
 *     SOC 只能显示/记录，不能单独作为 5%/0% 硬保护依据。
 ******************************************************************************
 */
#ifndef APP_SOC_EST_H
#define APP_SOC_EST_H

#include <stdint.h>

#include "board.h"
#include "bsp/bq_ic.h"
#include "app/state_store.h"

/* 标定完成前保持 0；NMC 表目前是典型值占位。 */
#define SOC_EST_CAL_VALID            0u

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
    SOC_EST_QUALITY_REAL = 1,       /* 已发布的三节 + BAT 全实测帧 */
    SOC_EST_QUALITY_ESTIMATED = 2   /* 两节实测 + 独立新 CC，SOC 非安全输入 */
} soc_est_quality_t;

typedef struct
{
    uint8_t  frame_valid;      /* 仅完整真实 BQ 帧可用于通常的算法/保护 */
    uint8_t  temp_valid;       /* TS1 温度换算有效（无新 TS1 则为 0） */
    uint8_t  cal_valid;        /* NMC 标定有效（当前固定 0） */
    soc_est_quality_t quality; /* 本次 SOC 更新所用电压的来源 */
    uint8_t  soc_estimated;    /* 曾以部分数据更新；等待全真实静置 OCV 重同步 */
    int32_t  soc01;            /* 0..10000 */
    int32_t  soh01;            /* 0..10000 */
    int16_t  temp_dc;          /* 0.1 °C */
    uint16_t cell_mv[3];
    uint16_t pack_mv;
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

/* 仅在 BQ 严格归类本批 PARTIAL 后调用；验证同批 VC1/VC2、配置/状态
 * 合格和独立新 CC 转换（CRC、时间、序号），不读旧 snapshot.current_ma。
 * 缺 CC 时改调用 SocEst_InvalidateInput()，不喂算法。 */
void SocEst_OnPartialSample(const bq_batch_quality_t *batch,
                            const bq_cc_sample_t *cc);

/* 无法证明当前批次可信时清质量与温度有效位，保留显示 SOC 值而不学习。 */
void SocEst_InvalidateInput(void);

/* NVM 去抖落盘；主循环每圈调用。 */
void SocEst_Task(void);

/* LP 进入前调用：有未落盘 SOH 学习结果则立即保存。 */
void SocEst_PrepareForLp(void);

/* 由同批次冻结的 SOC/温度与调用方提供的实测或清零电压构造归档 view。
 * 不把两节估算包压写成实测 pack_mv；仅当参数、来源和电压清零策略
 * 一致时返回 0，否则 -1。调用方再用 StateStore_CommitArchiveSnapshot()
 * 原子提交标记及 view；不得用更晚的最新 SOC 替换已认证候选。 */
int SocEst_FillStateStoreView(const soc_est_snapshot_t *captured,
                              const uint16_t measured_cell_mv[3],
                              uint16_t measured_pack_mv, uint16_t archive_flags,
                              state_store_view_t *out);

/* 旧常规 30 min 存档路径；新归档应优先使用冻结快照的 FillStateStoreView。 */
int SocEst_CommitStateStore(void);

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

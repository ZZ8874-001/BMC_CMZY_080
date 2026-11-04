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

#include "bsp/bq_ic.h"

/* 标定完成前保持 0；NMC 表目前是典型值占位。 */
#define SOC_EST_CAL_VALID            0u

/* TS1 NTC：MF52（10 kΩ @25°C，B=3950），10 kΩ 上拉到 5 V。
 * 若实际改成 3.3 V/VC1 或不同上拉，只改这两个值和 NTC 表。 */
#define SOC_EST_TS1_PULL_MV          5000u
#define SOC_EST_TS1_PULL_OHM         10000u

/* 库约定放电为正。BQ CC 的符号若实测相反（充电时 SOC 反而下降），
 * 把本宏改成 -1 即可，不改驱动/显示。 */
#define SOC_EST_CURRENT_SIGN         (1)

typedef struct
{
    uint8_t  frame_valid;      /* 本帧 BQ 数据可用于算法 */
    uint8_t  temp_valid;       /* TS1 温度换算有效 */
    uint8_t  cal_valid;        /* NMC 标定有效（当前固定 0） */
    int32_t  soc01;            /* 0..10000 */
    int32_t  soh01;            /* 0..10000 */
    int16_t  temp_dc;          /* 0.1 °C */
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    uint16_t ts1_raw;
    uint16_t ts1_mv;
    int32_t  current_ma;
    uint32_t update_count;
    uint32_t last_update_ms;
} soc_est_snapshot_t;

void SocEst_Init(void);

/* 主循环在 BQ_Poll 后调用；内部会丢弃重复时间戳/无效帧。 */
void SocEst_OnSnapshot(const bq_snapshot_t *snap, uint32_t now_ms);

/* NVM 去抖落盘；主循环每圈调用。 */
void SocEst_Task(void);

/* LP 进入前调用：有未落盘 SOH 学习结果则立即保存。 */
void SocEst_PrepareForLp(void);

/* 30 min 存档时调用：把当前 SOC/SOH/电压写入 StateStore（保留其它 flags）。 */
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

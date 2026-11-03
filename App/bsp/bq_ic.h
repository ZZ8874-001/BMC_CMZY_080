/**
 ******************************************************************************
 * @file    bq_ic.h
 * @brief   BQ7692003PWR 电池管理前端（I2C1 + 每字节 CRC + DMA）。
 *
 * 事务全部非阻塞：ISR 只置完成/错误标志，状态机在 BQ_Poll（主循环）推进。
 * 仅 LP 进入前的 BQ_PrepareForLp 使用有界阻塞调用（模式切换允许）。
 ******************************************************************************
 */
#ifndef APP_BQ_IC_H
#define APP_BQ_IC_H

#include <stdint.h>

typedef enum
{
    BQ_ST_RESET = 0,    /* 未初始化 */
    BQ_ST_BOOT_PULSE,   /* PB1 高脉冲中（TS1 boot） */
    BQ_ST_BOOT_WAIT,    /* 等待 tBOOTREADY */
    BQ_ST_CONFIG,       /* CC_CFG/增益/控制字初始化序列 */
    BQ_ST_RUN,          /* 正常轮询 */
    BQ_ST_BACKOFF       /* 通信退避，随后重入 CONFIG */
} bq_state_t;

typedef struct
{
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    int16_t  cc_raw;
    int32_t  current_ma;    /* CC 换算电流，带符号（250 ms 平均，未板级校准） */
    uint16_t ts1_raw;
    uint8_t  sys_stat;
    uint8_t  adc_gain;      /* 365..396 µV/LSB 的偏移量 */
    int8_t   adc_offset;    /* mV */
    uint8_t  fresh;
    uint32_t last_update_ms;
} bq_snapshot_t;

void BQ_Init(void);
void BQ_ReInitAfterLp(void);
void BQ_DeInitForLp(void);

/* LP 进入前：CC_EN=0（FET 保持导通）+ 清 SYS_STAT，防 CC_READY 唤醒。
 * 阻塞、有界（每步 50 ms 超时）；返回 0 成功。 */
int BQ_PrepareForLp(void);

void BQ_Poll(void);

/* 主循环请求一轮电芯数据读取（500 ms 节奏由 main_app 给出）。 */
void BQ_RequestRead(void);

/* 数据新鲜度（now_ms 与最后一次完整快照的时间差 ≤ APP_BQ_STALE_MS）。 */
uint8_t BQ_IsFresh(uint32_t now_ms);

void BQ_GetSnapshot(bq_snapshot_t *out);

bq_state_t BQ_GetState(void);

/* 无在飞事务且无脚本在跑：LP 进入前用它确认总线空闲。 */
uint8_t BQ_IsIdle(void);

#endif /* APP_BQ_IC_H */

/**
 ******************************************************************************
 * @file    state_store.h
 * @brief   L1 跨掉电状态日志（页 50–51，48 B 记录，两页环形）。
 *
 * 仅保存“下次上电需要知道”的状态：换电/锁存/死存储标志、最后 SOC/SOH、
 * 最后稳定电芯/包压。初始化只读扫描；写入必须由主循环显式调用 Commit。
 ******************************************************************************
 */
#ifndef APP_STATE_STORE_H
#define APP_STATE_STORE_H

#include <stdint.h>

#define STATE_FLAG_EXPECTED_SWAP    (1u << 0)
#define STATE_FLAG_SWAP_ERROR       (1u << 1)
#define STATE_FLAG_CUTOFF_LATCH     (1u << 2)
#define STATE_FLAG_DEAD_STORAGE     (1u << 3)
#define STATE_FLAG_CHARGE_FULL      (1u << 4)
#define STATE_FLAG_SOC_VALID        (1u << 5)
#define STATE_FLAG_PACK_VALID       (1u << 6)

typedef struct
{
    uint16_t flags;
    uint32_t seq;
    uint32_t battery_epoch;
    uint16_t cell_mv[3];
    uint16_t pack_mv;
    uint16_t soc01;
    uint16_t soh01;
    int16_t  temp_dc;
    uint8_t  valid;
} state_store_view_t;

void StateStore_Init(void);

uint8_t StateStore_IsValid(void);
void StateStore_GetView(state_store_view_t *out);
uint32_t StateStore_GetSeq(void);
uint16_t StateStore_GetActiveSlot(void);

/* 主循环调用；返回 0 成功，-1 失败（同时置 FAULT_BIT_FLASH_STATE）。 */
int StateStore_Commit(const state_store_view_t *view);

/* 清除指定 flags 并提交当前记录；失败时内存中的 flags 也不改变。 */
int StateStore_ClearLatch(uint16_t flags);

#endif /* APP_STATE_STORE_H */

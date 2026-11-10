/**
 ******************************************************************************
 * @file    protection.h
 * @brief   BQ/电池保护监督层（M2：硬件锁存 + 软件 OCP + NTC 温度）。
 *
 * 只做策略评估、锁存与恢复排序；FET 请求仍由 charge_policy 唯一拥有。
 * M1：BQ 硬件 OV/UV/SCD/OCD 观测、boot/LP 后 SCD/OCD 安全清除。
 * M2a：软件充电 5 A / 放电 30 A OCP，锁存 + 安全人工清除。
 * M2b：外部 NTC 充放电温度门控（PROT_TEMP_ENABLE 标定后启用）。
 ******************************************************************************
 */
#ifndef APP_PROTECTION_H
#define APP_PROTECTION_H

#include <stdint.h>

#include "bsp/bq_ic.h"

typedef enum
{
    PROTECTION_INIT = 0,
    PROTECTION_NORMAL,
    PROTECTION_RECOVERY,
    PROTECTION_LATCHED
} protection_state_t;

/* 详细原因，供 s_protection/日志使用；不扩展 16-bit 故障 API。 */
#define PROT_RSN_BQ_OV              (1u << 0)
#define PROT_RSN_BQ_UV              (1u << 1)
#define PROT_RSN_BQ_SCD             (1u << 2)
#define PROT_RSN_BQ_OCD             (1u << 3)
#define PROT_RSN_BQ_XREADY          (1u << 4)
#define PROT_RSN_BQ_NOT_READY       (1u << 5)
#define PROT_RSN_BOOT_LATCH         (1u << 6)
#define PROT_RSN_CLEAR_FAILED       (1u << 7)
#define PROT_RSN_SW_CHG_OCP         (1u << 8)
#define PROT_RSN_SW_DSG_OCP         (1u << 9)
#define PROT_RSN_TEMP_CHG           (1u << 10)
#define PROT_RSN_TEMP_DSG           (1u << 11)
#define PROT_RSN_TEMP_INVALID       (1u << 12)
#define PROT_RSN_CURRENT_SIGN       (1u << 13)
#define PROT_RSN_SOC_CUTOFF         (1u << 14)  /* M3 预留 */

/* 一次性 UI/黑匣子事件。 */
#define PROT_EVENT_TRIP             (1u << 0)
#define PROT_EVENT_CLEARED          (1u << 1)
#define PROT_EVENT_CLEAR_FAILED     (1u << 2)
#define PROT_EVENT_LATCHED          (1u << 3)
#define PROT_EVENT_SW_CHG_OCP       (1u << 4)
#define PROT_EVENT_SW_DSG_OCP       (1u << 5)
#define PROT_EVENT_TEMP_INHIBIT     (1u << 6)
#define PROT_EVENT_TEMP_RECOVER     (1u << 7)

/* ---------------- M2a 软件 OCP ----------------
 * 方向由 COMP2（消抖后）判定，幅值用 CC 积分值；这样 CC 极性如果与 TI
 * 约定相反，也不会把 5 A 放电误判成充电过流。CC 符号只影响 SOC/诊断。 */
#define PROT_OCP_CHG_MA             5000    /* 充电持续过流门槛 */
#define PROT_OCP_DSG_MA             30000   /* 放电持续过流门槛 */
#define PROT_OCP_CONFIRM_FRAMES     2u      /* 2 个“新发布帧”≈0.5–1 s */
#define PROT_OCP_SIGN_HINT_MA       2000    /* 与 COMP2 交叉检查的最小电流 */
#define PROT_OCP_IMPLAUSIBLE_MA     60000   /* 高于 SCD 44.7 A，视为 CC 异常帧 */

/* ---------------- M2b NTC 温度 ---------------- */
/* TS1 上拉两点校验通过后置 1；=0 时只计算/记录，不改允许掩码。 */
#define PROT_TEMP_ENABLE            0u
#define PROT_TEMP_DWELL_MS          2000u
#define PROT_TEMP_CHG_MIN_DC        0       /* 0.0 °C，低于则禁充 */
#define PROT_TEMP_CHG_MAX_DC        450     /* 45.0 °C，高于则禁充 */
#define PROT_TEMP_CHG_REC_MIN_DC    20      /* +2.0 °C 回差 */
#define PROT_TEMP_CHG_REC_MAX_DC    430     /* 43.0 °C 回差 */
#define PROT_TEMP_DSG_MIN_DC        (-200)  /* -20.0 °C，低于则禁放 */
#define PROT_TEMP_DSG_MAX_DC        600     /* 60.0 °C，高于则禁放 */
#define PROT_TEMP_DSG_REC_MIN_DC    (-180)  /* -18.0 °C 回差 */
#define PROT_TEMP_DSG_REC_MAX_DC    580     /* 58.0 °C 回差 */

/* 清锁存被拒绝的原因位，供 Ozone/日志。 */
#define PROT_CLEAR_BLOCK_READY      (1u << 0)   /* 数据未就绪/BQ busy */
#define PROT_CLEAR_BLOCK_FET        (1u << 1)   /* FET 未确认全关 */
#define PROT_CLEAR_BLOCK_CURRENT    (1u << 2)   /* |I| > 500 mA */
#define PROT_CLEAR_BLOCK_CHARGER    (1u << 3)   /* 充电器仍在位 */

typedef struct
{
    protection_state_t state;
    uint16_t reasons;
    uint8_t  allow_mask;
    uint8_t  boot_latched_mask;
    uint8_t  clear_requested;
    uint8_t  clear_failed;
    uint8_t  clear_block_reason;
    uint8_t  last_clear_result;   /* 0=none, 1=success, 2=failed */
    uint8_t  ocp_chg_latched;
    uint8_t  ocp_dsg_latched;
    uint8_t  ocp_chg_streak;
    uint8_t  ocp_dsg_streak;
    uint16_t ocp_chg_limit_ma;
    uint16_t ocp_dsg_limit_ma;
    int32_t  ocp_last_current_ma;
    uint32_t ocp_trip_count;
    uint32_t ocp_last_snap_ms;
    uint32_t implausible_current_count;
    uint8_t  sign_mismatch_streak;
    uint32_t sign_mismatch_count;
    uint8_t  temp_chg_inhibit;
    uint8_t  temp_dsg_inhibit;
    uint8_t  temp_invalid;
    uint8_t  temp_valid_last;
    int16_t  temp_dc_last;
    uint8_t  comp_charging_last;
} protection_snapshot_t;

void Protection_Init(void);

/* 主循环评估；不做阻塞 I2C。必须在 StateMachine_Update 之前调用。
 * comp_charging: COMP2 消抖方向（1=充电）；temp_*: SocEst 的 TS1 结果。 */
void Protection_Task(uint32_t now_ms, const bq_snapshot_t *snap,
                     uint8_t comp_charging,
                     uint8_t temp_valid, int16_t temp_dc);

/* 主循环服务；只在 BQ RUN/idle 时执行安全清锁存。 */
void Protection_Service(uint32_t now_ms, const bq_snapshot_t *snap);

/* FET 允许掩码：bit0=CHG，bit1=DSG。 */
uint8_t Protection_GetAllowedFetMask(void);

uint8_t Protection_IsLpBlocked(void);
protection_state_t Protection_GetState(void);
uint16_t Protection_GetReasons(void);
uint8_t Protection_GetBootLatchMask(void);
void Protection_GetSnapshot(protection_snapshot_t *out);

/* 调试器/后续手势入口：请求一次人工清锁存；只在安全条件下执行。 */
void Protection_RequestLatchClear(void);

uint8_t Protection_TakeEvent(void);

#endif /* APP_PROTECTION_H */

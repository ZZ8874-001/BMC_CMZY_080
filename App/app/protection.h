/**
 ******************************************************************************
 * @file    protection.h
 * @brief   BQ/电池保护监督层（M2：硬件锁存 + 软件 OCP + NTC 温度）。
 *
 * 只做策略评估、锁存与恢复排序；FET 请求仍由 charge_policy 唯一拥有。
 * M1：BQ 硬件 OV/UV/SCD/OCD 观测、boot/LP 后 SCD/OCD 安全清除。
 * M2a：软件充电 5.5 A / 放电 33 A OCP，锁存 + 安全人工清除。
 * M2b：外部 NTC 包温度门控（4–54 °C，充放电都收紧）。
 * 低电：参考节<3.1 V 或 SOC<10% 进入 CHG-only 恢复状态；2.8 V 以下硬禁 DSG。
 ******************************************************************************
 */
#ifndef APP_PROTECTION_H
#define APP_PROTECTION_H

#include <stdint.h>

#include "app/charge_policy.h"
#include "detect/soc_est.h"
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
#define PROT_RSN_SOC_CUTOFF         (1u << 14)  /* 早期 SOC<10% + 3.1 V 锁存 */
#define PROT_RSN_CELL_UV            (1u << 15)  /* MCU 单节 <2.8 V 硬保护 */

/* 一次性 UI/黑匣子事件。 */
#define PROT_EVENT_TRIP             (1u << 0)
#define PROT_EVENT_CLEARED          (1u << 1)
#define PROT_EVENT_CLEAR_FAILED     (1u << 2)
#define PROT_EVENT_LATCHED          (1u << 3)
#define PROT_EVENT_SW_CHG_OCP       (1u << 4)
#define PROT_EVENT_SW_DSG_OCP       (1u << 5)
#define PROT_EVENT_TEMP_INHIBIT     (1u << 6)
#define PROT_EVENT_TEMP_RECOVER     (1u << 7)
#define PROT_EVENT_EARLY_SOC_CUTOFF         (1u << 8)
#define PROT_EVENT_EARLY_SOC_CUTOFF_CLEARED (1u << 9)
#define PROT_EVENT_CELL_UV                  (1u << 10)
#define PROT_EVENT_CELL_UV_CLEARED          (1u << 11)
#define PROT_EVENT_CHARGE_FULL              (1u << 12)
#define PROT_EVENT_CHARGE_FULL_CLEARED      (1u << 13)
#define PROT_EVENT_CELL_OV                  (1u << 14)
#define PROT_EVENT_CELL_OV_CLEARED          (1u << 15)

/* ---------------- M2a 软件 OCP ----------------
 * 方向由 COMP2（消抖后）判定，幅值用 CC 积分值；这样 CC 极性如果与 TI
 * 约定相反，也不会把充电/放电过流误判。CC 符号只影响 SOC/诊断。
 * 阈值/时间按原计划：充电 5.5 A/1 s，放电 33 A/500 ms；至少 2 个新发布帧。 */
#define PROT_OCP_CHG_MA             5500    /* 充电持续过流门槛 */
#define PROT_OCP_DSG_MA             33000   /* 放电持续过流门槛 */
#define PROT_OCP_CHG_MS             1000u   /* 充电越界持续时间 */
#define PROT_OCP_DSG_MS             500u    /* 放电越界持续时间 */
#define PROT_OCP_CONFIRM_FRAMES     2u      /* 至少 2 个“新发布帧” */
#define PROT_OCP_SIGN_HINT_MA       2000    /* 与 COMP2 交叉检查的最小电流 */
#define PROT_OCP_IMPLAUSIBLE_MA     60000   /* 高于 SCD 44.7 A，视为 CC 异常帧 */

/* ---------------- M2b 包温度门控 ----------------
 * TS1 有效后启用；包温度 4.0–54.0 °C 为允许窗口，2 °C 回差/2 s 驻留。
 * 低/高温同时收紧 CHG 与 DSG；传感器无效 2 s 后也同时禁止。 */
#define PROT_TEMP_ENABLE            1u
#define PROT_TEMP_DWELL_MS          2000u
#define PROT_TEMP_CHG_MIN_DC        40      /* 4.0 °C，低于则禁充 */
#define PROT_TEMP_CHG_MAX_DC        540     /* 54.0 °C，高于则禁充 */
#define PROT_TEMP_CHG_REC_MIN_DC    60      /* +2.0 °C 回差 */
#define PROT_TEMP_CHG_REC_MAX_DC    520     /* 52.0 °C 回差 */
#define PROT_TEMP_DSG_MIN_DC        40      /* 4.0 °C，低于则禁放 */
#define PROT_TEMP_DSG_MAX_DC        540     /* 54.0 °C，高于则禁放 */
#define PROT_TEMP_DSG_REC_MIN_DC    60      /* +2.0 °C 回差 */
#define PROT_TEMP_DSG_REC_MAX_DC    520     /* 52.0 °C 回差 */

/* ---------------- 低压分级 ----------------
 * 2.8 V 单节硬保护：不依赖 SOC，立即只关 DSG；CHG 保持允许。
 * 低电状态：参考节<3.1 V 或 SOC<10% 进入，CHG-only；3.1 V 且 SOC>=10%
 * 恢复。低于 2.8 V 绝不开放 DSG，即使充电电流 >200 mA。 */
#define PROT_CELL_UV_MV             2800u
#define PROT_CELL_UV_RECOVER_MV     3000u
#define PROT_CELL_UV_RECOVER_FRAMES 2u

#define PROT_EARLY_SOC_01           1000u
#define PROT_EARLY_CELL_MV          3100u
#define PROT_EARLY_CONFIRM_FRAMES   2u
#define PROT_EARLY_RECOVER_SOC_01   1000u
#define PROT_EARLY_RECOVER_FRAMES   2u

/* VC5/BAT 不可信、BQ OV/UV 退避期间的 MCU VC2 软件过压。 */
#define PROT_CELL_OV_MV             4250u
#define PROT_CELL_OV_RECOVER_MV     4150u
#define PROT_CELL_OV_CONFIRM_FRAMES 2u
#define PROT_CELL_OV_RECOVER_FRAMES 2u

/* ---------------- 充满提示（NMC） ----------------
 * 只置 charge_full/白灯，不关 CHG；BQ OV 4.25 V 仍是最终硬件保护。 */
#define PROT_FULL_SOC01             9800u
#define PROT_FULL_CELL_MV           4150u
#define PROT_FULL_TAPER_MA          75
#define PROT_FULL_TAPER_MS          30000u
#define PROT_FULL_EXIT_MA           150
#define PROT_FULL_CLEAR_FRAMES      2u

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
    uint32_t ocp_chg_since_ms;
    uint32_t ocp_dsg_since_ms;
    uint32_t implausible_current_count;
    uint8_t  sign_mismatch_streak;
    uint32_t sign_mismatch_count;
    uint8_t  temp_chg_inhibit;
    uint8_t  temp_dsg_inhibit;
    uint8_t  temp_invalid;
    uint8_t  temp_valid_last;
    int16_t  temp_dc_last;
    uint8_t  comp_charging_last;

    /* 低压分级 */
    uint16_t soc01;
    uint16_t min_cell_mv;      /* 参考节 mV（正常最低节，异常为 VC2） */
    uint8_t  soc_valid;
    uint8_t  ref_cell2_only;   /* 保护当前使用 VC2-only 降级 */
    uint8_t  cell_uv_active;
    uint8_t  cell_uv_streak;
    uint8_t  cell_ov_active;
    uint8_t  cell_ov_streak;
    uint8_t  cell_ov_recover_streak;
    uint8_t  voltage_backoff;      /* BQ OV/UV 退避 active/pending */
    uint8_t  bq_fet_hold_mask;    /* BQ 退避期间禁止重开的 FET 位 */
    uint16_t cell_ov_ref_mv;
    uint32_t cell_ov_count;
    uint8_t  early_soc_latched;   /* 低电状态（兼容旧命名） */
    uint8_t  early_soc_streak;
    uint8_t  early_soc_recover_streak;
    uint8_t  low_batt_dsg_allowed; /* 低电充电 DSG boost 当前允许 */
    uint8_t  charge_full;
    uint32_t charge_full_since_ms;
    uint32_t cell_uv_count;
    uint32_t early_soc_count;
    uint32_t charge_full_count;
} protection_snapshot_t;

void Protection_Init(void);

/* 主循环评估；不做阻塞 I2C。必须在 StateMachine_Update 之前调用。
 * comp_charging: COMP2 消抖方向（1=充电）；charger_present: 上一轮策略判断；
 * policy_state: 上一轮 ChargePolicy 状态；
 * low_batt_dsg_on: 上一轮低电充电策略的 200/120 mA DSG 窗口状态；
 * soc: 同一轮 SocEst 快照。 */
void Protection_Task(uint32_t now_ms, const bq_snapshot_t *snap,
                     const soc_est_snapshot_t *soc,
                     uint8_t comp_charging, uint8_t charger_present,
                     charge_policy_state_t policy_state,
                     uint8_t low_batt_dsg_on);

/* 主循环服务；只在 BQ RUN/idle 时执行安全清锁存。 */
void Protection_Service(uint32_t now_ms, const bq_snapshot_t *snap);

/* FET 允许掩码：bit0=CHG，bit1=DSG。 */
uint8_t Protection_GetAllowedFetMask(void);

uint8_t Protection_IsLpBlocked(void);
protection_state_t Protection_GetState(void);
uint16_t Protection_GetReasons(void);
uint8_t Protection_GetBootLatchMask(void);
void Protection_GetSnapshot(protection_snapshot_t *out);

uint8_t Protection_IsCellUvActive(void);
uint8_t Protection_IsCellOvActive(void);
uint8_t Protection_IsLowBattery(void);
uint8_t Protection_IsSocCutoffLatched(void);   /* 兼容旧命名 */
uint8_t Protection_GetChargeFull(void);
void Protection_RestoreSocCutoff(uint8_t latched);

/* 调试器/后续手势入口：请求一次人工清锁存；只在安全条件下执行。 */
void Protection_RequestLatchClear(void);

uint16_t Protection_TakeEvent(void);

#endif /* APP_PROTECTION_H */

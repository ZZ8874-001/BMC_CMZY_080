/**
 ******************************************************************************
 * @file    app_telemetry.c
 * @brief   1 Hz USART1 SOC/SOH D 帧遥测。
 *
 * 输出格式（仅此一行，约 35-49 B）：
 *   D,V_mV,I_mA,T_0p1C,SOC_0p01,SOH_0p01,Q_mAh,R0_mohm\r\n
 *
 * I 采用“放电为正、充电为负”，与 soc_record.py 一致；BQ 原始 current_ma
 * 是充电为正，因此乘 BQ_CC_DISCHARGE_SIGN。
 *
 * 数据闸门：BQ frame valid/fresh、SocEst frame_valid、温度有效、V/I/T 在
 * soc_record.py 的合理范围内；任一失败跳过该秒，不发送全 0 假数据。
 * SOC_EST_CAL_VALID=1 时 SOC/SOH 参与低电判断/显示；2.8 V 电压硬保护仍是
 * 独立兜底，D 帧不是保护通道。
 ******************************************************************************
 */
#include "comm/app_telemetry.h"

#include "board.h"
#include "comm/app_log.h"
#include "detect/bms/soh.h"
#include "detect/soc_est.h"

#include <stddef.h>

#define APP_TELEMETRY_PERIOD_MS     1000u

typedef enum
{
    APP_TLM_SKIP_NONE = 0,
    APP_TLM_SKIP_BQ,
    APP_TLM_SKIP_SOC,
    APP_TLM_SKIP_TEMP,
    APP_TLM_SKIP_RANGE
} app_tlm_skip_t;

static struct
{
    uint32_t last_ms;
    uint32_t emit_count;
    uint32_t skip_bq_count;
    uint32_t skip_soc_count;
    uint32_t skip_temp_count;
    uint32_t skip_range_count;
    uint8_t  last_skip_reason;

    uint16_t last_v_mv;
    int32_t  last_i_ma;
    int16_t  last_temp_dc;
    int32_t  last_soc01;
    int32_t  last_soh01;
    uint32_t last_q_mah;
    uint16_t last_r0_mohm;
} s_telemetry;

void App_Telemetry_Init(void)
{
    s_telemetry.last_ms = 0u;
    s_telemetry.emit_count = 0u;
    s_telemetry.skip_bq_count = 0u;
    s_telemetry.skip_soc_count = 0u;
    s_telemetry.skip_temp_count = 0u;
    s_telemetry.skip_range_count = 0u;
    s_telemetry.last_skip_reason = (uint8_t)APP_TLM_SKIP_NONE;
    s_telemetry.last_v_mv = 0u;
    s_telemetry.last_i_ma = 0;
    s_telemetry.last_temp_dc = 0;
    s_telemetry.last_soc01 = 0;
    s_telemetry.last_soh01 = 0;
    s_telemetry.last_q_mah = 0u;
    s_telemetry.last_r0_mohm = 0u;
}

void App_Telemetry_Task(uint32_t now_ms, const bq_snapshot_t *bq_snap)
{
    soc_est_snapshot_t soc;
    uint16_t v_mv;
    int32_t i_ma;
    int16_t temp_dc;
    uint32_t q_mah;
    uint16_t r0_mohm;

    if ((s_telemetry.last_ms != 0u) &&
        ((now_ms - s_telemetry.last_ms) < APP_TELEMETRY_PERIOD_MS))
    {
        return;
    }
    s_telemetry.last_ms = now_ms;

    if ((bq_snap == NULL) || (bq_snap->valid == 0u) ||
        (bq_snap->fresh == 0u) || (BQ_IsFresh(now_ms) == 0u))
    {
        s_telemetry.skip_bq_count++;
        s_telemetry.last_skip_reason = (uint8_t)APP_TLM_SKIP_BQ;
        return;
    }

    SocEst_GetSnapshot(&soc);
    if (soc.frame_valid == 0u)
    {
        s_telemetry.skip_soc_count++;
        s_telemetry.last_skip_reason = (uint8_t)APP_TLM_SKIP_SOC;
        return;
    }
    if (soc.temp_valid == 0u)
    {
        s_telemetry.skip_temp_count++;
        s_telemetry.last_skip_reason = (uint8_t)APP_TLM_SKIP_TEMP;
        return;
    }

    v_mv = BQ_GetRefCellMv(bq_snap);
    i_ma = bq_snap->current_ma * BQ_CC_DISCHARGE_SIGN;
    temp_dc = soc.temp_dc;

    if ((v_mv < 1500u) || (v_mv > 5000u) ||
        (i_ma < -30000) || (i_ma > 30000) ||
        (temp_dc < -400) || (temp_dc > 1250))
    {
        s_telemetry.skip_range_count++;
        s_telemetry.last_skip_reason = (uint8_t)APP_TLM_SKIP_RANGE;
        return;
    }

    q_mah = (SOH_IsCapacityValid() != 0u) ?
            SOH_GetLearnedCapacityMAh() : 0u;
    r0_mohm = (SOH_IsR0Valid() != 0u) ?
              SOH_GetR0AvgMOhm() : 0u;

    App_Log_Printf("D,%u,%ld,%d,%ld,%ld,%lu,%u\r\n",
                   (unsigned)v_mv,
                   (long)i_ma,
                   (int)temp_dc,
                   (long)soc.soc01,
                   (long)soc.soh01,
                   (unsigned long)q_mah,
                   (unsigned)r0_mohm);

    s_telemetry.last_skip_reason = (uint8_t)APP_TLM_SKIP_NONE;
    s_telemetry.last_v_mv = v_mv;
    s_telemetry.last_i_ma = i_ma;
    s_telemetry.last_temp_dc = temp_dc;
    s_telemetry.last_soc01 = soc.soc01;
    s_telemetry.last_soh01 = soc.soh01;
    s_telemetry.last_q_mah = q_mah;
    s_telemetry.last_r0_mohm = r0_mohm;
    s_telemetry.emit_count++;
}

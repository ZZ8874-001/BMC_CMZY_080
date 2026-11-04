/**
 ******************************************************************************
 * @file    cal_stream.c
 * @brief   CAL 标定 CSV 流（USART1 非阻塞日志）。
 *
 * 每次 BQ 快照变化输出一行，约 2 Hz；行宽必须 <127 B，避免 App_Log_VPrintf
 * 的 128 B 截断。会话开始/结束各写一条黑匣子事件，不逐样本写 Flash。
 ******************************************************************************
 */
#include "app/cal_stream.h"

#include "app/black_box.h"
#include "app/charge_policy.h"
#include "comm/app_log.h"
#include "detect/soc_est.h"

#include <stddef.h>

static struct
{
    volatile uint8_t enabled;      /* 默认 1；调试器可写 0 只保留会话标记 */
    uint8_t in_session;
    uint32_t last_update_ms;
    uint32_t line_count;
    uint32_t begin_count;
    uint32_t end_count;
    uint32_t blackbox_fail_count;

    /* Flash 事件只在主循环、BQ 总线空闲时写；CAL 进入/退出先排队。 */
    uint16_t bb_event_code[2];
    uint32_t bb_event_uptime[2];
    uint8_t bb_event_count;
} s_cal_stream;

static void CalStream_QueueEvent(uint16_t code, uint32_t uptime_s)
{
    if (s_cal_stream.bb_event_count >= 2u)
    {
        s_cal_stream.blackbox_fail_count++;
        return;
    }
    s_cal_stream.bb_event_code[s_cal_stream.bb_event_count] = code;
    s_cal_stream.bb_event_uptime[s_cal_stream.bb_event_count] = uptime_s;
    s_cal_stream.bb_event_count++;
}

static void CalStream_ServiceEvents(void)
{
    if ((s_cal_stream.bb_event_count == 0u) || (BQ_IsIdle() == 0u))
    {
        return;
    }

    if (BlackBox_WriteEvent(s_cal_stream.bb_event_code[0],
                            s_cal_stream.bb_event_uptime[0]) != 0)
    {
        s_cal_stream.blackbox_fail_count++;
    }

    if (s_cal_stream.bb_event_count > 1u)
    {
        s_cal_stream.bb_event_code[0] = s_cal_stream.bb_event_code[1];
        s_cal_stream.bb_event_uptime[0] = s_cal_stream.bb_event_uptime[1];
    }
    s_cal_stream.bb_event_count--;
}

void CalStream_Init(void)
{
    s_cal_stream.enabled = 1u;
    s_cal_stream.in_session = 0u;
    s_cal_stream.last_update_ms = 0u;
    s_cal_stream.line_count = 0u;
    s_cal_stream.begin_count = 0u;
    s_cal_stream.end_count = 0u;
    s_cal_stream.blackbox_fail_count = 0u;
    s_cal_stream.bb_event_code[0] = 0u;
    s_cal_stream.bb_event_code[1] = 0u;
    s_cal_stream.bb_event_uptime[0] = 0u;
    s_cal_stream.bb_event_uptime[1] = 0u;
    s_cal_stream.bb_event_count = 0u;
}

uint8_t CalStream_IsEnabled(void)
{
    return s_cal_stream.enabled;
}

void CalStream_Task(uint32_t now_ms, uint32_t uptime_s,
                    const bq_snapshot_t *snap)
{
    charge_policy_state_t policy_state = ChargePolicy_GetState();
    uint8_t valid = 0u;

    CalStream_ServiceEvents();

    if ((policy_state == CHARGE_POLICY_CAL) &&
        (s_cal_stream.in_session == 0u))
    {
        s_cal_stream.in_session = 1u;
        s_cal_stream.begin_count++;
        s_cal_stream.last_update_ms = 0u;

        App_Log_Printf("[CAL] BEGIN t=%lu mask=%u\r\n",
                       (unsigned long)now_ms,
                       (unsigned)ChargePolicy_GetRequestedMask());
        App_Log_Printf("[CAL] HDR t_ms,c1mv,c2mv,c3mv,pmv,cc,ima,"
                       "ts1r,ts1mv,temp_dc,soc01,soh01,pst,req,app,valid\r\n");
        CalStream_QueueEvent(BB_EVENT_CAL_BEGIN, uptime_s);
    }
    else if ((policy_state != CHARGE_POLICY_CAL) &&
             (s_cal_stream.in_session != 0u))
    {
        s_cal_stream.in_session = 0u;
        s_cal_stream.end_count++;
        App_Log_Printf("[CAL] END t=%lu reason=%u\r\n",
                       (unsigned long)now_ms,
                       (unsigned)ChargePolicy_GetCalEndReason());
        CalStream_QueueEvent(BB_EVENT_CAL_END, uptime_s);
        return;
    }

    if ((s_cal_stream.in_session == 0u) ||
        (s_cal_stream.enabled == 0u) ||
        (snap == NULL) || (snap->fresh == 0u) || (snap->valid == 0u) ||
        (snap->last_update_ms == s_cal_stream.last_update_ms))
    {
        return;
    }
    s_cal_stream.last_update_ms = snap->last_update_ms;

    {
        soc_est_snapshot_t soc;

        SocEst_GetSnapshot(&soc);
        valid = (uint8_t)(((snap->fresh != 0u) ? 0x01u : 0u) |
                          ((soc.frame_valid != 0u) ? 0x02u : 0u) |
                          ((soc.temp_valid != 0u) ? 0x04u : 0u) |
                          ((soc.cal_valid != 0u) ? 0x08u : 0u));

        App_Log_Printf("[CAL] %lu,%u,%u,%u,%u,%d,%ld,%u,%u,%d,"
                       "%ld,%ld,%u,%u,%u,%02X\r\n",
                       (unsigned long)now_ms,
                       (unsigned)snap->cell_mv[0],
                       (unsigned)snap->cell_mv[1],
                       (unsigned)snap->cell_mv[2],
                       (unsigned)snap->pack_mv,
                       (int)snap->cc_raw,
                       (long)snap->current_ma,
                       (unsigned)snap->ts1_raw,
                       (unsigned)soc.ts1_mv,
                       (int)soc.temp_dc,
                       (long)soc.soc01,
                       (long)soc.soh01,
                       (unsigned)policy_state,
                       (unsigned)ChargePolicy_GetRequestedMask(),
                       (unsigned)BQ_GetFetMaskApplied(),
                       (unsigned)valid);
        s_cal_stream.line_count++;
    }
}

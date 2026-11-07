/**
 ******************************************************************************
 * @file    lp_session.c
 * @brief   IWDG 复位式 LP 会话的 BKP 持久化。
 ******************************************************************************
 */
#include "app/lp_session.h"

#include "board.h"

#include <string.h>

static volatile uint32_t *LpSession_Bkp(uint8_t index)
{
    return &RTC->BKP0R + index;
}

static uint16_t LpSession_Crc16(const uint32_t *words, uint8_t count)
{
    uint16_t crc = 0xFFFFu;
    uint8_t i;
    uint8_t b;
    uint8_t k;
    uint32_t v;

    for (i = 0u; i < count; i++)
    {
        v = words[i];
        for (b = 0u; b < 4u; b++)
        {
            crc ^= (uint16_t)((uint8_t)(v >> (8u * b)) << 8);
            for (k = 0u; k < 8u; k++)
            {
                crc = ((crc & 0x8000u) != 0u) ?
                      (uint16_t)((crc << 1) ^ 0x1021u) :
                      (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void LpSession_Pack(const lp_session_t *s, uint32_t *w)
{
    w[0] = ((uint32_t)s->state & 0xFFu) |
           (((uint32_t)s->anomaly_count & 0xFFu) << 8) |
           (((uint32_t)s->flags & 0xFFu) << 16);            /* BKP3 */
    w[1] = s->next_mark_epoch;                               /* BKP4 */
    w[2] = s->lsi_hz_q16;                                    /* BKP5 */
    w[3] = ((uint32_t)s->last_pr & 0x07u) |
           (((uint32_t)s->last_rlr & 0x0FFFu) << 3);        /* BKP6 */
    w[4] = s->last_arm_epoch;                                /* BKP7 */
    w[5] = ((uint32_t)s->last_arm_ms & 0x03FFu) |
           (((uint32_t)s->boot_overhead_ms & 0xFFFFu) << 10);/* BKP8 */
    w[6] = ((uint32_t)s->wake_cause & 0xFFu) |
           (((uint32_t)s->pb5_mode & 0x03u) << 8) |
           (((uint32_t)s->smba_result & 0x03u) << 10);      /* BKP9 */
    w[7] = (uint32_t)s->seq & 0xFFFFu;                       /* BKP10 */
}

static void LpSession_Unpack(const uint32_t *w, lp_session_t *s)
{
    memset(s, 0, sizeof(*s));

    s->state = (uint8_t)(w[0] & 0xFFu);
    s->anomaly_count = (uint8_t)((w[0] >> 8) & 0xFFu);
    s->flags = (uint8_t)((w[0] >> 16) & 0xFFu);
    s->next_mark_epoch = w[1];
    s->lsi_hz_q16 = w[2];
    s->last_pr = (uint8_t)(w[3] & 0x07u);
    s->last_rlr = (uint16_t)((w[3] >> 3) & 0x0FFFu);
    s->last_arm_epoch = w[4];
    s->last_arm_ms = (uint16_t)(w[5] & 0x03FFu);
    s->boot_overhead_ms = (uint16_t)((w[5] >> 10) & 0xFFFFu);
    s->wake_cause = (uint8_t)(w[6] & 0xFFu);
    s->pb5_mode = (uint8_t)((w[6] >> 8) & 0x03u);
    s->smba_result = (uint8_t)((w[6] >> 10) & 0x03u);
    s->seq = (uint16_t)(w[7] & 0xFFFFu);
}

static uint8_t LpSession_Valid(const lp_session_t *s)
{
    if (s->state > (uint8_t)LP_SESSION_FAULT)
    {
        return 0u;
    }
    if (s->last_pr > 6u)
    {
        return 0u;
    }
    if ((s->last_rlr > 4095u) && (s->last_rlr != 0u))
    {
        return 0u;
    }
    if (s->lsi_hz_q16 != 0u)
    {
        uint32_t hz = s->lsi_hz_q16 >> 16;
        if ((hz < 26000u) || (hz > 34000u))
        {
            return 0u;
        }
    }
    return 1u;
}

uint8_t LpSession_Load(lp_session_t *s)
{
    uint32_t w[8];
    uint16_t crc;
    uint16_t saved;
    uint8_t i;

    if (s == NULL)
    {
        return 0u;
    }
    memset(s, 0, sizeof(*s));

    HAL_PWR_EnableBkUpAccess();

    if ((*LpSession_Bkp(LP_SESSION_BKP_MAGIC) != LP_SESSION_MAGIC))
    {
        return 0u;
    }

    for (i = 0u; i < 8u; i++)
    {
        w[i] = *LpSession_Bkp((uint8_t)(LP_SESSION_BKP_FIRST + i));
    }
    saved = (uint16_t)(*LpSession_Bkp(LP_SESSION_BKP_CRC) & 0xFFFFu);
    crc = LpSession_Crc16(w, 8u);
    if (saved != crc)
    {
        LpSession_Invalidate();
        return 0u;
    }

    LpSession_Unpack(w, s);
    if (LpSession_Valid(s) == 0u)
    {
        LpSession_Invalidate();
        memset(s, 0, sizeof(*s));
        return 0u;
    }
    return 1u;
}

void LpSession_Save(const lp_session_t *s)
{
    uint32_t w[8];
    uint16_t crc;
    uint8_t i;

    if (s == NULL)
    {
        return;
    }

    HAL_PWR_EnableBkUpAccess();
    LpSession_Pack(s, w);
    crc = LpSession_Crc16(w, 8u);

    /* 先清 magic，避免中断在数据/CRC 写到一半时把半截会话当成有效。 */
    *LpSession_Bkp(LP_SESSION_BKP_MAGIC) = 0u;
    for (i = 0u; i < 8u; i++)
    {
        *LpSession_Bkp((uint8_t)(LP_SESSION_BKP_FIRST + i)) = w[i];
    }
    *LpSession_Bkp(LP_SESSION_BKP_CRC) = (uint32_t)crc;
    *LpSession_Bkp(LP_SESSION_BKP_MAGIC) = LP_SESSION_MAGIC;

    /* The caller immediately does NVIC_SystemReset().  Without a barrier the
     * last posted APB writes (CRC/magic are written last) can be lost across
     * the reset: data reaches BKP3..BKP10 but BKP1/BKP2 stay 0, the next boot
     * sees no session and does a normal boot, producing an endless
     * NORM->reset->NORM loop at the 5 s auto-LP cadence. */
    __DSB();
}

void LpSession_Invalidate(void)
{
    HAL_PWR_EnableBkUpAccess();
    *LpSession_Bkp(LP_SESSION_BKP_MAGIC) = 0u;
    *LpSession_Bkp(LP_SESSION_BKP_CRC) = 0u;
    __DSB();
}

/**
 ******************************************************************************
 * @file    rtc_time.c
 * @brief   RTC 绝对时间读取（LSE；LP_RTC_USE_LSE=0 时用 LSI 调试）。
 ******************************************************************************
 */
#include "app/rtc_time.h"

#include "board.h"

static uint8_t s_rtc_ready;

static uint8_t RtcTime_WaitSynchro(void)
{
    return (HAL_RTC_WaitForSynchro(&hrtc) == HAL_OK) ? 1u : 0u;
}

static uint8_t RtcTime_IsLeap(uint32_t year)
{
    return (((year % 4u) == 0u) && (((year % 100u) != 0u) ||
                                    ((year % 400u) == 0u))) ? 1u : 0u;
}

static uint32_t RtcTime_DaysSince2000(uint32_t year, uint8_t month,
                                      uint8_t date)
{
    static const uint8_t mdays[12] =
    {
        31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u
    };
    uint32_t days = 0u;
    uint32_t y;
    uint8_t m;

    if ((year < 2000u) || (year > 2099u) ||
        (month < 1u) || (month > 12u) ||
        (date < 1u) || (date > 31u))
    {
        return 0xFFFFFFFFu;
    }

    for (y = 2000u; y < year; y++)
    {
        days += (RtcTime_IsLeap(y) != 0u) ? 366u : 365u;
    }
    for (m = 1u; m < month; m++)
    {
        days += (uint32_t)mdays[m - 1u];
        if ((m == 2u) && (RtcTime_IsLeap(year) != 0u))
        {
            days++;
        }
    }
    days += (uint32_t)(date - 1u);

    return days;
}

static uint8_t RtcTime_ReadRaw(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
    if (RtcTime_WaitSynchro() == 0u)
    {
        return 0u;
    }
    if (HAL_RTC_GetTime(&hrtc, time, RTC_FORMAT_BIN) != HAL_OK)
    {
        return 0u;
    }
    if (HAL_RTC_GetDate(&hrtc, date, RTC_FORMAT_BIN) != HAL_OK)
    {
        return 0u;
    }
    return 1u;
}

uint32_t RtcTime_MspClockSource(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

#if (LP_RTC_USE_LSE != 0u)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u)
    {
        return RCC_RTCCLKSOURCE_LSE;
    }

    __HAL_RCC_LSEDRIVE_CONFIG(LP_LSE_DRIVE_CAPABILITY);
    {
        RCC_OscInitTypeDef osc = {0};

        osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
        osc.LSEState = RCC_LSE_ON;
        (void)HAL_RCC_OscConfig(&osc);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0u)
    {
        return RCC_RTCCLKSOURCE_LSE;
    }

    /* LSE dead: clear LSEON.  HAL's RTC source-change path saves/restores
     * BDCR around BDRST; if LSEON is set it waits RCC_LSE_TIMEOUT_VALUE
     * (~5 s) for LSERDY.  Clearing LSEON lets the LSI fallback boot promptly;
     * LP will remain disabled because LP_RTC_USE_LSE still expects LSE. */
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
    return RCC_RTCCLKSOURCE_LSI;
#else
    /* LSI mode: never leave a stale LSEON around a backup-domain reset. */
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
    return RCC_RTCCLKSOURCE_LSI;
#endif
}

uint8_t RtcTime_Init(void)
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint8_t clock_ok = 0u;

    s_rtc_ready = 0u;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    /* Strict source check: LSE mode must not silently run LP on the LSI
     * fallback; LSI mode must not accept a stale LSE clock source. */
    if (((RCC->BDCR & RCC_BDCR_RTCSEL) == LP_RTC_EXPECT_RTCSEL) &&
        (__HAL_RCC_GET_FLAG(LP_RTC_EXPECT_RDY_FLAG) != 0u))
    {
        clock_ok = 1u;
    }
    if ((clock_ok == 0u) ||
        (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY))
    {
        return 0u;
    }
    if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0u)
    {
        __HAL_RCC_RTC_ENABLE();
    }

    if (RtcTime_ReadRaw(&time, &date) == 0u)
    {
        return 0u;
    }

    /* Year==0 is the reset calendar (2000-01-01).  Set an arbitrary fixed
     * anchor once so the RTC is marked initialized and keeps running across
     * system/IWDG resets.  This is not wall-clock time. */
    if (date.Year == 0u)
    {
        RTC_TimeTypeDef set_time = {0};
        RTC_DateTypeDef set_date = {0};

        set_time.Hours = 0u;
        set_time.Minutes = 0u;
        set_time.Seconds = 0u;
        set_time.TimeFormat = RTC_HOURFORMAT_24;
        set_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        set_time.StoreOperation = RTC_STOREOPERATION_RESET;

        set_date.WeekDay = RTC_WEEKDAY_MONDAY;
        set_date.Month = RTC_MONTH_JANUARY;
        set_date.Date = 1u;
        set_date.Year = 1u;      /* 2001, nonzero => calendar initialized */

        if (HAL_RTC_SetTime(&hrtc, &set_time, RTC_FORMAT_BIN) != HAL_OK)
        {
            return 0u;
        }
        if (HAL_RTC_SetDate(&hrtc, &set_date, RTC_FORMAT_BIN) != HAL_OK)
        {
            return 0u;
        }
        if (RtcTime_WaitSynchro() == 0u)
        {
            return 0u;
        }
    }

    s_rtc_ready = 1u;
    return 1u;
}

uint8_t RtcTime_Read(rtc_epoch_t *out)
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint32_t days;
    uint32_t sub_sec;

    if ((out == NULL) || (s_rtc_ready == 0u))
    {
        return 0u;
    }

    if (RtcTime_ReadRaw(&time, &date) == 0u)
    {
        return 0u;
    }
    if ((time.Hours > 23u) || (time.Minutes > 59u) ||
        (time.Seconds > 59u))
    {
        return 0u;
    }

    days = RtcTime_DaysSince2000(2000u + (uint32_t)date.Year,
                                 date.Month, date.Date);
    if (days == 0xFFFFFFFFu)
    {
        return 0u;
    }

    out->sec = days * 86400u +
               (uint32_t)time.Hours * 3600u +
               (uint32_t)time.Minutes * 60u +
               (uint32_t)time.Seconds;

    /* SSR is a down-counter; (255 - SSR)/256 is elapsed fraction in the
     * current second (sync prescaler = 255). */
    sub_sec = (uint32_t)time.SubSeconds;
    if (sub_sec > 255u)
    {
        sub_sec = 255u;
    }
    out->ms = (uint16_t)(((255u - sub_sec) * 1000u) / 256u);

    return 1u;
}

int32_t RtcTime_DiffMs(const rtc_epoch_t *newer, const rtc_epoch_t *older)
{
    int32_t sec_diff;

    if ((newer == NULL) || (older == NULL))
    {
        return 0;
    }
    sec_diff = (int32_t)(newer->sec - older->sec);
    return (sec_diff * 1000) + (int32_t)newer->ms - (int32_t)older->ms;
}

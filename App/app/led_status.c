/**
 ******************************************************************************
 * @file    led_status.c
 * @brief   PB3/PB4 status LEDs plus the WS2812 single RGB LED.
 *
 * The voltage-quality overlay is intentionally independent from the RGB mode:
 * partial data flashes HOT/PB3 at 1 Hz, and the anomaly alarm at 5 Hz. A real
 * LED_MODE_FAULT always owns PB3 and the RGB fault indication.
 ******************************************************************************/
#include "app/led_status.h"

#include "board.h"
#include "bsp/ws2812_uart.h"

#define LED_BLINK_1HZ_TICKS 250u
#define LED_BLINK_5HZ_TICKS 50u

static struct
{
    volatile led_mode_t mode;
    volatile led_flow_t flow;
    volatile uint32_t tick_count;
    volatile uint8_t phase_1hz;
    volatile uint8_t phase_5hz;
    volatile uint8_t rgb_dirty;
    uint32_t rgb_send_count;
    uint8_t last_phase_1hz;
    uint8_t last_phase_5hz;
    uint8_t last_mode;
    uint8_t last_flow;
    uint8_t rgb_frame[WS2812_FRAME_BYTES];

    int32_t  soc01;
    uint8_t  soc_valid;
    uint8_t  soc_has_valid;
    uint8_t  soc_bin;          /* 1% 量化，减少 SOC 更新造成的发帧 */
    uint8_t  last_sent_valid;
    uint8_t  last_sent_rgb[3];
} s_led;

static void LedStatus_DrivePins(void)
{
    uint8_t norm_on = 0u;
    uint8_t roht_on = 0u;

    switch (s_led.mode)
    {
    case LED_MODE_BOOT:
        norm_on = s_led.phase_1hz;
        break;

    case LED_MODE_NORM:
        norm_on = 1u;
        break;

    case LED_MODE_FAULT:
        roht_on = s_led.phase_5hz;
        break;

    default:
        break;
    }

    HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin,
                      (norm_on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_ROHT_GPIO_Port, IND_ROHT_Pin,
                      (roht_on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LedStatus_Init(void)
{
    s_led.mode = LED_MODE_BOOT;
    s_led.flow = LED_FLOW_STANDBY;
    s_led.tick_count = 0u;
    s_led.phase_1hz = 0u;
    s_led.phase_5hz = 0u;
    s_led.rgb_dirty = 1u;
    s_led.rgb_send_count = 0u;
    s_led.last_phase_1hz = 0xFFu;
    s_led.last_phase_5hz = 0xFFu;
    s_led.last_mode = 0xFFu;
    s_led.last_flow = 0xFFu;

    HAL_GPIO_WritePin(IND_NORM_GPIO_Port, IND_NORM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_ROHT_GPIO_Port, IND_ROHT_Pin, GPIO_PIN_RESET);

    s_led.soc01 = 0;
    s_led.soc_valid = 0u;
    s_led.soc_has_valid = 0u;
    s_led.soc_bin = 0xFFu;
    s_led.last_sent_valid = 0u;
    s_led.last_sent_rgb[0] = 0u;
    s_led.last_sent_rgb[1] = 0u;
    s_led.last_sent_rgb[2] = 0u;
}

/* 0%=红, 50%=黄, 100%=绿；max 为当前模式亮度上限。 */
static void LedStatus_Gradient(uint8_t max, uint8_t *r, uint8_t *g)
{
    int32_t soc = s_led.soc01;

    if (soc < 0)
    {
        soc = 0;
    }
    else if (soc > 10000)
    {
        soc = 10000;
    }

    if (soc <= 5000)
    {
        *r = max;
        *g = (uint8_t)(((uint16_t)max * (uint16_t)soc) / 5000u);
    }
    else
    {
        *r = (uint8_t)(((uint16_t)max * (uint16_t)(10000 - soc)) / 5000u);
        *g = max;
    }
}

void LedStatus_SetSoc(int32_t soc01, uint8_t valid)
{
    uint8_t bin;

    if (valid == 0u)
    {
        return;             /* 保持最后有效颜色 */
    }
    if (soc01 < 0)
    {
        soc01 = 0;
    }
    else if (soc01 > 10000)
    {
        soc01 = 10000;
    }
    bin = (uint8_t)((soc01 + 50) / 100);   /* 1% 量化 */

    s_led.soc_valid = 1u;
    if ((s_led.soc_has_valid != 0u) && (bin == s_led.soc_bin))
    {
        return;
    }
    s_led.soc01 = soc01;
    s_led.soc_has_valid = 1u;
    s_led.soc_bin = bin;
    s_led.rgb_dirty = 1u;
}

void LedStatus_SetMode(led_mode_t mode, led_flow_t flow)
{
    if (s_led.mode != mode)
    {
        s_led.tick_count = 0u;
        s_led.phase_1hz = 1u;
        s_led.phase_5hz = 1u;
    }
    if ((s_led.mode != mode) || (s_led.flow != flow))
    {
        s_led.mode = mode;
        s_led.flow = flow;
        s_led.rgb_dirty = 1u;
        LedStatus_DrivePins();
    }
}

static uint8_t LedStatus_PhaseUsed(void)
{
    if (s_led.mode == LED_MODE_BOOT)
    {
        return 1u;      /* phase_1hz */
    }
    if (s_led.mode == LED_MODE_FAULT)
    {
        return 2u;      /* phase_5hz */
    }
    if (s_led.mode != LED_MODE_NORM)
    {
        return 0u;
    }
    if (s_led.flow == LED_FLOW_STANDBY)
    {
        return 2u;      /* idle 5 Hz fast */
    }
    if (s_led.flow == LED_FLOW_CHARGING)
    {
        return 1u;      /* charge 1 Hz */
    }
    if (s_led.flow == LED_FLOW_LOW_BATT)
    {
        return 2u;      /* low battery 5 Hz red */
    }
    return 0u;          /* discharge/full/CAL steady */
}

void LedStatus_Tick(void)
{
    uint8_t phase_changed;

    s_led.tick_count++;

    s_led.phase_1hz = ((s_led.tick_count % LED_BLINK_1HZ_TICKS) <
                       (LED_BLINK_1HZ_TICKS / 2u)) ? 1u : 0u;
    s_led.phase_5hz = ((s_led.tick_count % LED_BLINK_5HZ_TICKS) <
                       (LED_BLINK_5HZ_TICKS / 2u)) ? 1u : 0u;

    LedStatus_DrivePins();

    phase_changed = 0u;
    if (s_led.phase_1hz != s_led.last_phase_1hz)
    {
        s_led.last_phase_1hz = s_led.phase_1hz;
        if (LedStatus_PhaseUsed() == 1u)
        {
            phase_changed = 1u;
        }
    }
    if (s_led.phase_5hz != s_led.last_phase_5hz)
    {
        s_led.last_phase_5hz = s_led.phase_5hz;
        if (LedStatus_PhaseUsed() == 2u)
        {
            phase_changed = 1u;
        }
    }
    if (phase_changed != 0u)
    {
        s_led.rgb_dirty = 1u;
    }
}

void LedStatus_Flush(void)
{
    uint8_t r = 0u;
    uint8_t g = 0u;
    uint8_t b = 0u;

    if (s_led.rgb_dirty == 0u)
    {
        return;
    }
    if (Ws2812_IsBusy())
    {
        return;
    }

    switch (s_led.mode)
    {
    case LED_MODE_BOOT:
        if (s_led.phase_1hz != 0u)
        {
            b = 64u;
        }
        break;

    case LED_MODE_FAULT:
        if (s_led.phase_5hz != 0u)
        {
            r = 128u;
        }
        break;

    case LED_MODE_NORM:
    default:
        switch (s_led.flow)
        {
        case LED_FLOW_STANDBY:
            if (s_led.phase_5hz != 0u)
            {
                if (s_led.soc_has_valid != 0u)
                {
                    LedStatus_Gradient(29u, &r, &g);
                }
                else
                {
                    r = 12u;
                    g = 12u;
                    b = 12u;
                }
            }
            break;
        case LED_FLOW_DISCHARGING:
            if (s_led.soc_has_valid != 0u)
            {
                LedStatus_Gradient(46u, &r, &g);
            }
            else
            {
                b = 48u;
            }
            break;
        case LED_FLOW_CHARGING:
            if (s_led.phase_1hz != 0u)
            {
                if (s_led.soc_has_valid != 0u)
                {
                    LedStatus_Gradient(46u, &r, &g);
                }
                else
                {
                    g = 48u;
                }
            }
            break;
        case LED_FLOW_CHARGE_FULL:
            r = 38u;
            g = 38u;
            b = 38u;
            break;
        case LED_FLOW_CAL:
            r = 48u;
            g = 24u;
            break;
        case LED_FLOW_LOW_BATT:
            if (s_led.phase_5hz != 0u)
            {
                r = 64u;
            }
            break;
        case LED_FLOW_FAULT:
        default:
            r = 12u;
            g = 12u;
            b = 12u;
            break;
        }
        break;
    }

    if ((s_led.last_sent_valid != 0u) &&
        (s_led.last_sent_rgb[0] == r) &&
        (s_led.last_sent_rgb[1] == g) &&
        (s_led.last_sent_rgb[2] == b))
    {
        s_led.rgb_dirty = 0u;
        return;
    }

    Ws2812_EncodeLed(s_led.rgb_frame, r, g, b);
    if (Ws2812_Send(s_led.rgb_frame) == 0)
    {
        s_led.rgb_send_count++;
        s_led.last_sent_valid = 1u;
        s_led.last_sent_rgb[0] = r;
        s_led.last_sent_rgb[1] = g;
        s_led.last_sent_rgb[2] = b;
        s_led.rgb_dirty = 0u;
    }
}

/**
 ******************************************************************************
 * @file    button.c
 * @brief   PB5/WAKE 按键：4 ms 采样、20 ms 消抖、按下/释放事件队列。
 ******************************************************************************
 */
#include "bsp/button.h"

#include "board.h"

#include <stddef.h>

#define BUTTON_SAMPLE_MS    4u
#define BUTTON_DEBOUNCE_MS  20u
#define BUTTON_QUEUE_SIZE   8u

static struct
{
    volatile uint8_t exti_flag;

    uint8_t raw_level;          /* 1 = 按下（PB5 低） */
    uint8_t stable_level;
    uint8_t pressed;
    uint8_t have_sample;
    uint32_t raw_change_ms;
    uint32_t press_start_ms;
    uint32_t last_sample_ms;

    button_event_t queue[BUTTON_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    uint32_t overflow;
} s_button;

static uint8_t Button_ReadPressed(void)
{
    return (HAL_GPIO_ReadPin(WAKE_EXTI5_GPIO_Port, WAKE_EXTI5_Pin) ==
            GPIO_PIN_RESET) ? 1u : 0u;
}

static void Button_Push(button_event_type_t type, uint8_t pressed,
                        uint32_t timestamp_ms, uint32_t held_ms,
                        uint8_t wake_origin)
{
    uint8_t next = (uint8_t)((s_button.tail + 1u) % BUTTON_QUEUE_SIZE);
    button_event_t *ev;

    if (next == s_button.head)
    {
        s_button.overflow++;
        return;
    }

    ev = &s_button.queue[s_button.tail];
    ev->type = type;
    ev->pressed = pressed;
    ev->timestamp_ms = timestamp_ms;
    ev->held_ms = held_ms;
    ev->wake_origin = wake_origin;
    s_button.tail = next;
}

void Button_Init(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t level = Button_ReadPressed();

    s_button.exti_flag = 0u;
    s_button.raw_level = level;
    s_button.stable_level = level;
    s_button.pressed = level;
    s_button.have_sample = 0u;
    s_button.raw_change_ms = now_ms;
    s_button.press_start_ms = now_ms;
    s_button.last_sample_ms = now_ms;
    s_button.head = 0u;
    s_button.tail = 0u;
    s_button.overflow = 0u;
}

void Button_NotifyExtiFromIsr(void)
{
    s_button.exti_flag = 1u;
}

void Button_PrepareForLp(void)
{
    s_button.exti_flag = 0u;
    s_button.raw_level = 0u;
    s_button.stable_level = 0u;
    s_button.pressed = 0u;
    s_button.have_sample = 0u;
    s_button.raw_change_ms = HAL_GetTick();
    s_button.press_start_ms = 0u;
    s_button.head = 0u;
    s_button.tail = 0u;
}

void Button_OnWake(uint8_t wake_button, uint8_t pressed_now, uint32_t now_ms)
{
    if (wake_button == 0u)
    {
        return;
    }

    s_button.have_sample = 0u;
    s_button.last_sample_ms = now_ms;

    if (pressed_now != 0u)
    {
        s_button.raw_level = 1u;
        s_button.stable_level = 1u;
        s_button.pressed = 1u;
        s_button.press_start_ms = now_ms;
        s_button.raw_change_ms = now_ms;
        Button_Push(BUTTON_EVENT_DOWN, 1u, now_ms, 0u, 1u);
    }
    else
    {
        /* 唤醒键在 30 ms 时钟恢复期间已释放：合成一次短按，保证它仍计入
         * 第一次短按（三连击/放电配对都依赖这一点）。 */
        s_button.raw_level = 0u;
        s_button.stable_level = 0u;
        s_button.pressed = 0u;
        s_button.press_start_ms = 0u;
        s_button.raw_change_ms = now_ms;
        Button_Push(BUTTON_EVENT_DOWN, 1u,
                    (now_ms >= 20u) ? (now_ms - 20u) : 0u, 0u, 1u);
        Button_Push(BUTTON_EVENT_UP, 0u, now_ms, 20u, 1u);
    }
}

void Button_Poll(uint32_t now_ms)
{
    uint8_t raw;

    if ((s_button.have_sample != 0u) &&
        ((now_ms - s_button.last_sample_ms) < BUTTON_SAMPLE_MS))
    {
        return;
    }
    s_button.have_sample = 1u;
    s_button.last_sample_ms = now_ms;

    raw = Button_ReadPressed();
    if (raw != s_button.raw_level)
    {
        s_button.raw_level = raw;
        s_button.raw_change_ms = now_ms;
        s_button.exti_flag = 0u;
        return;
    }

    if ((raw == s_button.stable_level) ||
        ((now_ms - s_button.raw_change_ms) < BUTTON_DEBOUNCE_MS))
    {
        return;
    }

    s_button.stable_level = raw;
    s_button.exti_flag = 0u;

    if (raw != 0u)
    {
        s_button.pressed = 1u;
        s_button.press_start_ms = now_ms;
        Button_Push(BUTTON_EVENT_DOWN, 1u, now_ms, 0u, 0u);
    }
    else
    {
        uint32_t held_ms = now_ms - s_button.press_start_ms;

        s_button.pressed = 0u;
        Button_Push(BUTTON_EVENT_UP, 0u, now_ms, held_ms, 0u);
    }
}

uint8_t Button_GetEvent(button_event_t *out)
{
    if ((out == NULL) || (s_button.head == s_button.tail))
    {
        return 0u;
    }

    *out = s_button.queue[s_button.head];
    s_button.head = (uint8_t)((s_button.head + 1u) % BUTTON_QUEUE_SIZE);
    return 1u;
}

uint8_t Button_IsPressed(void)
{
    return s_button.pressed;
}

uint32_t Button_GetPressStartMs(void)
{
    return s_button.press_start_ms;
}

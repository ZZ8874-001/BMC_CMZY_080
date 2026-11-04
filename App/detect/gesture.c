/**
 ******************************************************************************
 * @file    gesture.c
 * @brief   PB5 按键手势时序引擎（纯 C，无 HAL）。
 ******************************************************************************
 */
#include "detect/gesture.h"

#include <stddef.h>

#define GESTURE_QUEUE_SIZE          8u
#define GESTURE_SHORT_MIN_MS        20u
#define GESTURE_SHORT_MAX_MS        600u
#define GESTURE_HOLD3_MS            3000u
#define GESTURE_HOLD5_MS            5000u
#define GESTURE_RELEASE_WINDOW_MS   500u
#define GESTURE_PAIR_WINDOW_MS      2000u
#define GESTURE_TRIPLE_WINDOW_MS    1000u

static struct
{
    uint8_t pressed;
    uint32_t press_start_ms;
    uint8_t pair_candidate;
    uint8_t hold3_emitted;
    uint8_t hold5_emitted;
    uint32_t hold5_ms;

    uint8_t short_count;
    uint32_t first_short_press_ms;
    uint8_t have_short;
    uint32_t last_short_release_ms;

    gesture_event_t queue[GESTURE_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint32_t overflow;
} s_gesture;

static void Gesture_Push(gesture_event_t event)
{
    uint8_t next = (uint8_t)((s_gesture.tail + 1u) % GESTURE_QUEUE_SIZE);

    if (next == s_gesture.head)
    {
        s_gesture.overflow++;
        return;
    }
    s_gesture.queue[s_gesture.tail] = event;
    s_gesture.tail = next;
}

static void Gesture_ResetPress(void)
{
    s_gesture.pressed = 0u;
    s_gesture.press_start_ms = 0u;
    s_gesture.pair_candidate = 0u;
    s_gesture.hold3_emitted = 0u;
    s_gesture.hold5_emitted = 0u;
    s_gesture.hold5_ms = 0u;
}

void Gesture_Init(void)
{
    s_gesture.pressed = 0u;
    s_gesture.press_start_ms = 0u;
    s_gesture.pair_candidate = 0u;
    s_gesture.hold3_emitted = 0u;
    s_gesture.hold5_emitted = 0u;
    s_gesture.hold5_ms = 0u;
    s_gesture.short_count = 0u;
    s_gesture.first_short_press_ms = 0u;
    s_gesture.have_short = 0u;
    s_gesture.last_short_release_ms = 0u;
    s_gesture.head = 0u;
    s_gesture.tail = 0u;
    s_gesture.overflow = 0u;
}

void Gesture_OnButtonEvent(const button_event_t *ev)
{
    uint32_t held_ms;

    if (ev == NULL)
    {
        return;
    }

    if (ev->type == BUTTON_EVENT_DOWN)
    {
        s_gesture.pressed = 1u;
        s_gesture.press_start_ms = ev->timestamp_ms;
        s_gesture.hold3_emitted = 0u;
        s_gesture.hold5_emitted = 0u;
        s_gesture.hold5_ms = 0u;
        s_gesture.pair_candidate =
            ((s_gesture.have_short != 0u) &&
             ((ev->timestamp_ms - s_gesture.last_short_release_ms) <=
              GESTURE_PAIR_WINDOW_MS)) ? 1u : 0u;
        return;
    }

    if (ev->type != BUTTON_EVENT_UP)
    {
        return;
    }

    held_ms = ev->held_ms;
    s_gesture.pressed = 0u;

    if (s_gesture.hold5_emitted != 0u)
    {
        if ((ev->timestamp_ms - s_gesture.hold5_ms) <=
            GESTURE_RELEASE_WINDOW_MS)
        {
            Gesture_Push(GESTURE_RELEASE_AFTER_HOLD5);
        }
        Gesture_ResetPress();
        s_gesture.have_short = 0u;
        s_gesture.short_count = 0u;
        return;
    }

    if (s_gesture.hold3_emitted != 0u)
    {
        /* 3 s 长按事件已经在 Poll 中发出；释放不再补短按。 */
        Gesture_ResetPress();
        s_gesture.have_short = 0u;
        s_gesture.short_count = 0u;
        return;
    }

    if ((held_ms < GESTURE_SHORT_MIN_MS) ||
        (held_ms > GESTURE_SHORT_MAX_MS))
    {
        /* 中间时长（0.6–3 s）不是短按，也不触发长按事件。 */
        Gesture_ResetPress();
        s_gesture.have_short = 0u;
        s_gesture.short_count = 0u;
        return;
    }

    Gesture_Push(GESTURE_SHORT);

    if ((s_gesture.short_count == 0u) ||
        ((s_gesture.press_start_ms - s_gesture.first_short_press_ms) >
         GESTURE_TRIPLE_WINDOW_MS))
    {
        s_gesture.short_count = 1u;
        s_gesture.first_short_press_ms = s_gesture.press_start_ms;
    }
    else
    {
        s_gesture.short_count++;
        if (s_gesture.short_count >= 3u)
        {
            Gesture_Push(GESTURE_TRIPLE);
            s_gesture.short_count = 0u;
        }
    }

    s_gesture.have_short = 1u;
    s_gesture.last_short_release_ms = ev->timestamp_ms;
    Gesture_ResetPress();
}

void Gesture_Poll(uint32_t now_ms)
{
    uint32_t held_ms;

    if (s_gesture.pressed != 0u)
    {
        held_ms = now_ms - s_gesture.press_start_ms;

        if ((held_ms >= GESTURE_HOLD5_MS) && (s_gesture.hold5_emitted == 0u))
        {
            s_gesture.hold5_emitted = 1u;
            s_gesture.hold3_emitted = 1u;
            s_gesture.hold5_ms = now_ms;
            Gesture_Push(GESTURE_HOLD5);
        }
        else if ((held_ms >= GESTURE_HOLD3_MS) &&
                 (s_gesture.hold3_emitted == 0u))
        {
            s_gesture.hold3_emitted = 1u;
            Gesture_Push((s_gesture.pair_candidate != 0u) ?
                         GESTURE_PAIR_HOLD3 : GESTURE_HOLD3);
        }
    }

    if ((s_gesture.short_count > 0u) && (s_gesture.pressed == 0u) &&
        ((now_ms - s_gesture.first_short_press_ms) > GESTURE_TRIPLE_WINDOW_MS))
    {
        s_gesture.short_count = 0u;
    }
}

uint8_t Gesture_GetEvent(gesture_event_t *out)
{
    if ((out == NULL) || (s_gesture.head == s_gesture.tail))
    {
        return 0u;
    }

    *out = s_gesture.queue[s_gesture.head];
    s_gesture.head = (uint8_t)((s_gesture.head + 1u) % GESTURE_QUEUE_SIZE);
    return 1u;
}

uint8_t Gesture_IsBusy(void)
{
    return ((s_gesture.pressed != 0u) || (s_gesture.short_count != 0u)) ? 1u : 0u;
}

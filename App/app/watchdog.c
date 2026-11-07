/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   IWDG1 (LSI) 直接寄存器配置：boot guard、NORM 死机保险、LP 动态周期。
 *
 * IWDG 一旦启动就不能停止；超时只能产生系统复位，没有早唤醒中断。
 * 本模块负责在复位后的 boot guard 与 NORM/LP 的不同 timeout 之间切换。
 ******************************************************************************
 */
#include "app/watchdog.h"

#include "main.h"

#define WATCHDOG_KEY_UNLOCK   0x5555u
#define WATCHDOG_KEY_RELOAD   0xAAAAu
#define WATCHDOG_KEY_START    0xCCCCu
#define WATCHDOG_PRESCALER_BOOT  6u    /* /256, boot guard ~32.8 s */
#define WATCHDOG_PRESCALER_NORM  4u    /* /64,  NORM ~8.2 s */
#define WATCHDOG_RELOAD       4095u
#define WATCHDOG_WINDOW       4095u    /* 0xFFF = window disabled */

static uint8_t s_started;

static uint8_t Watchdog_WaitUpdate(void)
{
    uint32_t guard = 0u;

    while (((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)) != 0u) &&
           (guard < 100000u))
    {
        guard++;
    }
    return (guard < 100000u) ? 1u : 0u;
}

static uint8_t Watchdog_Program(uint8_t pr, uint16_t rlr)
{
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;

    IWDG->KR = WATCHDOG_KEY_UNLOCK;
    if (Watchdog_WaitUpdate() == 0u)
    {
        return 0u;
    }
    IWDG->PR = (uint32_t)(pr & 0x07u);
    IWDG->RLR = (uint32_t)(rlr & 0x0FFFu);
    IWDG->WINR = WATCHDOG_WINDOW;
    if (Watchdog_WaitUpdate() == 0u)
    {
        return 0u;
    }

    IWDG->KR = WATCHDOG_KEY_RELOAD;
    IWDG->KR = WATCHDOG_KEY_START;
    s_started = 1u;

    return ((IWDG->PR & 0x07u) == (uint32_t)(pr & 0x07u)) &&
           ((IWDG->RLR & 0x0FFFu) == (uint32_t)(rlr & 0x0FFFu));
}

void Watchdog_Init(void)
{
    /* 复位后的 boot guard：给 RTC 读取、BQ sniff、LP session 判定留足时间。 */
    (void)Watchdog_Program(WATCHDOG_PRESCALER_BOOT, WATCHDOG_RELOAD);
}

void Watchdog_InitNorm(void)
{
    /* NORM 死机保险：8.2 s 量级。 */
    (void)Watchdog_Program(WATCHDOG_PRESCALER_NORM, WATCHDOG_RELOAD);
}

uint8_t Watchdog_SetTimeoutMs(uint32_t ms, uint32_t lsi_hz,
                              uint32_t *actual_ms)
{
    uint8_t pr;
    uint32_t div = 4u;
    uint32_t ticks = 4096u;
    uint64_t num;
    uint64_t den;

    if (lsi_hz == 0u)
    {
        lsi_hz = 32000u;
    }
    if (ms == 0u)
    {
        ms = 1u;
    }

    /* 选“能装进 12 位 RLR 的最小分频”，让短 interval 也有更细分辨率。 */
    for (pr = 0u; pr <= 6u; pr++)
    {
        div = 4u << pr;
        num = (uint64_t)ms * (uint64_t)lsi_hz;
        den = 1000ull * (uint64_t)div;
        ticks = (uint32_t)((num + (den / 2u)) / den);
        if (ticks <= 4095u)
        {
            break;
        }
    }
    if (pr > 6u)
    {
        pr = 6u;
        div = 256u;
        ticks = 4095u;
    }
    if (ticks < 1u)
    {
        ticks = 1u;
    }

    if (actual_ms != NULL)
    {
        *actual_ms = (uint32_t)(((uint64_t)ticks * (uint64_t)div *
                                 1000ull) / (uint64_t)lsi_hz);
    }

    return Watchdog_Program(pr, (uint16_t)(ticks - 1u));
}

void Watchdog_Feed(void)
{
    if (s_started != 0u)
    {
        IWDG->KR = WATCHDOG_KEY_RELOAD;
    }
}

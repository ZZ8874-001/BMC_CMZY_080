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
    volatile uint8_t smba_alert_flag;

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

void Button_WakeInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio.Pin = WAKE_EXTI5_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WAKE_EXTI5_GPIO_Port, &gpio);

    /* HAL_GPIO_Init() 负责 GPIO/EXTI 复用；这里再显式确认一次 PB5 的
     * SYSCFG 映射、下降沿和 IMR，避免生成代码没有打开 EXTI 时静默失效。 */
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &
                         ~(0x0Fu << 4)) |
                        SYSCFG_EXTICR2_EXTI5_PB;
    EXTI->RTSR1 &= (uint32_t)~EXTI_RTSR1_RT5;
    EXTI->FTSR1 |= EXTI_FTSR1_FT5;
    EXTI->IMR1 |= EXTI_IMR1_IM5;
    EXTI->PR1 = EXTI_PR1_PIF5;

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0u, 0u);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

uint8_t Button_TakeExtiFlag(void)
{
    uint8_t f = s_button.exti_flag;

    s_button.exti_flag = 0u;
    return f;
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(WAKE_EXTI5_Pin);
}

/* PB5 EXTI5 的唯一回调 owner。ISR 只置标志，不做消抖/入队；LP 轻量路径
 * 直接读 s_button.exti_flag/PB5 电平，NORM 由 Button_Poll 消费。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == WAKE_EXTI5_Pin)
    {
        s_button.exti_flag = 1u;
    }
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

void Button_NotifySmbaFromIsr(void)
{
    s_button.smba_alert_flag = 1u;
}

void Button_SmbaEnable(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 先保留 EXTI5 回退，再把 PB5 切到 AF4。若 AF 模式下 EXTI 仍能采样
     * 引脚，按键仍可唤醒；若不能，IWDG 复位会兜底。 */
    Button_WakeInit();

    if (hi2c1.State == HAL_I2C_STATE_RESET)
    {
        hi2c1.Init.Timing = BQ_I2C_TIMINGR_HSI16_100K;
        (void)HAL_I2C_Init(&hi2c1);
    }

    gpio.Pin = WAKE_EXTI5_Pin;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(WAKE_EXTI5_GPIO_Port, &gpio);

    /* RM0394 的 I2C STOP 唤醒条件要求 I2CCLK=HSI16；测试窗口把 I2C1
     * 内核时钟切到 HSI16（同为 16 MHz），PE 先关再改选择。 */
    __HAL_I2C_DISABLE(&hi2c1);
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C1SEL_Msk, RCC_CCIPR_I2C1SEL_1);
    __HAL_I2C_ENABLE(&hi2c1);

    I2C1->CR1 = (I2C1->CR1 & ~(I2C_CR1_DNF_Msk | I2C_CR1_NOSTRETCH)) |
                I2C_CR1_SMBHEN | I2C_CR1_ALERTEN | I2C_CR1_ERRIE |
                I2C_CR1_WUPEN;
    I2C1->ICR = I2C_ICR_ALERTCF;
    s_button.smba_alert_flag = 0u;

    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1u, 0u);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void Button_SmbaDisable(void)
{
    I2C1->CR1 &= ~(I2C_CR1_SMBHEN | I2C_CR1_ALERTEN | I2C_CR1_ERRIE |
                   I2C_CR1_WUPEN);
    I2C1->ICR = I2C_ICR_ALERTCF;
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    s_button.smba_alert_flag = 0u;
    Button_WakeInit();
}

uint8_t Button_SmbaTakeAlert(void)
{
    uint8_t f = s_button.smba_alert_flag;

    s_button.smba_alert_flag = 0u;
    return f;
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

/**
 ******************************************************************************
 * @file    buzzer.c
 * @brief   TIM1 互补通道 PWM 蜂鸣器：PSC=0，ARR 决定音调，CCR=ARR/2。
 *
 * PA7 是 TIM1_CH1N，必须用 HAL_TIMEx_PWMN_Start 且显式开 MOE
 * （HAL 会在 PWMN_Start 内使能 MOE，这里再显式置一次，避免依赖实现细节）。
 * 音型推进只在主循环做；TIM1 无中断、无 NVIC。
 ******************************************************************************
 */
#include "bsp/buzzer.h"

#include "board.h"

#include <stddef.h>

typedef struct
{
    uint16_t autoreload;    /* 0 = 静音步 */
    uint16_t duration_ms;
} buzzer_step_t;

/* 结束标记：duration_ms == 0。 */
static const buzzer_step_t s_pattern_boot_ok[] =
{
    {BUZZER_ARR_4K0, 120u},
    {0u,             100u},
    {BUZZER_ARR_4K0, 120u},
    {0u,             0u}
};

static const buzzer_step_t s_pattern_fault[] =
{
    {BUZZER_ARR_2K0, 400u},
    {0u,             180u},
    {BUZZER_ARR_2K0, 400u},
    {0u,             180u},
    {BUZZER_ARR_2K0, 400u},
    {0u,             0u}
};

static const buzzer_step_t s_pattern_lp_enter[] =
{
    {BUZZER_ARR_2K5, 150u},
    {0u,             0u}
};

static const buzzer_step_t s_pattern_wake[] =
{
    {BUZZER_ARR_5K0, 80u},
    {0u,             0u}
};

static struct
{
    const buzzer_step_t *pattern;
    uint8_t step_index;
    uint32_t step_start_ms;
    uint8_t output_on;
    uint16_t current_autoreload;
    uint32_t pattern_start_count;
} s_buzzer;

/* PA7（TIM1_CH1N）：NORM 用 AF1 推挽输出 PWM；LP 静态拉低使 Q8 可靠关断。 */
static void Buzzer_PinConfigureAlternate(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = BUZ_TIM1_CH1_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(BUZ_TIM1_CH1_GPIO_Port, &gpio);
}

static void Buzzer_PinConfigureLow(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = BUZ_TIM1_CH1_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZ_TIM1_CH1_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(BUZ_TIM1_CH1_GPIO_Port, BUZ_TIM1_CH1_Pin, GPIO_PIN_RESET);
}

static const buzzer_step_t *Buzzer_GetPattern(buzzer_pattern_t pattern)
{
    switch (pattern)
    {
    case BUZZER_PATTERN_BOOT_OK:
        return s_pattern_boot_ok;
    case BUZZER_PATTERN_FAULT:
        return s_pattern_fault;
    case BUZZER_PATTERN_LP_ENTER:
        return s_pattern_lp_enter;
    case BUZZER_PATTERN_WAKE:
        return s_pattern_wake;
    default:
        return NULL;
    }
}

static void Buzzer_OutputStart(uint16_t autoreload)
{
    htim1.Instance->PSC = 0u;    /* 80 MHz 定时器时钟 → 全表 ARR 有效 */
    __HAL_TIM_SET_AUTORELOAD(&htim1, autoreload);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(autoreload / 2u));
    __HAL_TIM_SET_COUNTER(&htim1, 0u);
    htim1.Instance->EGR = TIM_EGR_UG;   /* 立即装载新 ARR/CCR */

    if (s_buzzer.output_on == 0u)
    {
        (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        s_buzzer.output_on = 1u;
    }
    __HAL_TIM_MOE_ENABLE(&htim1);
    s_buzzer.current_autoreload = autoreload;
}

static void Buzzer_OutputStop(void)
{
    if (s_buzzer.output_on != 0u)
    {
        (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        s_buzzer.output_on = 0u;
    }
    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
}

static void Buzzer_ApplyStep(void)
{
    const buzzer_step_t *step = &s_buzzer.pattern[s_buzzer.step_index];

    s_buzzer.step_start_ms = HAL_GetTick();

    if (step->autoreload == 0u)
    {
        Buzzer_OutputStop();
    }
    else
    {
        Buzzer_OutputStart(step->autoreload);
    }
}

void Buzzer_Init(void)
{
    s_buzzer.pattern = NULL;
    s_buzzer.step_index = 0u;
    s_buzzer.step_start_ms = 0u;
    s_buzzer.output_on = 0u;
    s_buzzer.current_autoreload = 0u;
    s_buzzer.pattern_start_count = 0u;

    Buzzer_PinConfigureAlternate();
    Buzzer_OutputStop();
}

void Buzzer_ReInit(void)
{
    Buzzer_Init();
}

void Buzzer_DeInitForLp(void)
{
    s_buzzer.pattern = NULL;
    Buzzer_OutputStop();
    Buzzer_PinConfigureLow();
}

void Buzzer_StartTone(uint16_t autoreload)
{
    s_buzzer.pattern = NULL;
    if (autoreload == 0u)
    {
        Buzzer_OutputStop();
        return;
    }
    Buzzer_OutputStart(autoreload);
}

void Buzzer_StartPattern(buzzer_pattern_t pattern)
{
    const buzzer_step_t *table = Buzzer_GetPattern(pattern);

    if (table == NULL)
    {
        Buzzer_Stop();
        return;
    }

    s_buzzer.pattern = table;
    s_buzzer.step_index = 0u;
    s_buzzer.pattern_start_count++;
    Buzzer_ApplyStep();
}

void Buzzer_Poll(void)
{
    const buzzer_step_t *step;

    if (s_buzzer.pattern == NULL)
    {
        return;
    }

    step = &s_buzzer.pattern[s_buzzer.step_index];
    if ((HAL_GetTick() - s_buzzer.step_start_ms) < step->duration_ms)
    {
        return;
    }

    s_buzzer.step_index++;
    step = &s_buzzer.pattern[s_buzzer.step_index];
    if (step->duration_ms == 0u)
    {
        Buzzer_Stop();
        return;
    }

    Buzzer_ApplyStep();
}

void Buzzer_Stop(void)
{
    s_buzzer.pattern = NULL;
    s_buzzer.step_index = 0u;
    Buzzer_OutputStop();
}

uint8_t Buzzer_IsActive(void)
{
    return (s_buzzer.pattern != NULL) ? 1u : 0u;
}

/**
 ******************************************************************************
 * @file    reset_cause.c
 * @brief   复位原因采集：RTC BKP0R 保留异常复位标记，RCC->CSR 补充硬件复位源。
 *
 * 移植自 Armor_Cmzy26_081/App/app/reset_cause（按本板保留位调整）。
 ******************************************************************************
 */
#include "app/reset_cause.h"

#include "main.h"

#define RESET_CAUSE_BKP_MAGIC      0x42520000u   /* "BR" */
#define RESET_CAUSE_BKP_MAGIC_MASK 0xFFFF0000u

static struct
{
    uint8_t latched;
    uint32_t csr_raw;
} s_reset_cause;

static void ResetCause_EnableBackupAccess(void)
{
    RCC->APB1ENR1 |= (RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN);
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;
    __DSB();
}

static void ResetCause_RecordAndReset(uint8_t cause) __attribute__((noreturn));

static void ResetCause_RecordAndReset(uint8_t cause)
{
    ResetCause_EnableBackupAccess();
    RTC->BKP0R = RESET_CAUSE_BKP_MAGIC | (uint32_t)cause;
    __DSB();
    NVIC_SystemReset();
    while (1)
    {
    }
}

void ResetCause_Init(void)
{
    uint32_t backup;

    s_reset_cause.latched = 0u;
    s_reset_cause.csr_raw = RCC->CSR;

    ResetCause_EnableBackupAccess();
    backup = RTC->BKP0R;
    if ((backup & RESET_CAUSE_BKP_MAGIC_MASK) == RESET_CAUSE_BKP_MAGIC)
    {
        s_reset_cause.latched |= (uint8_t)(backup & 0xFFu);
        RTC->BKP0R = 0u;   /* 消费一次，避免正常复位重复上报 */
    }

    if ((s_reset_cause.csr_raw & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0u)
    {
        s_reset_cause.latched |= RESET_CAUSE_WATCHDOG;
    }
    if ((s_reset_cause.csr_raw & RCC_CSR_SFTRSTF) != 0u)
    {
        s_reset_cause.latched |= RESET_CAUSE_SOFTWARE;
    }
    if ((s_reset_cause.csr_raw & RCC_CSR_PINRSTF) != 0u)
    {
        s_reset_cause.latched |= RESET_CAUSE_PIN;
    }
    if ((s_reset_cause.csr_raw & (RCC_CSR_BORRSTF | RCC_CSR_LPWRRSTF |
                                  RCC_CSR_OBLRSTF)) != 0u)
    {
        s_reset_cause.latched |= RESET_CAUSE_BOR;
    }

    RCC->CSR |= RCC_CSR_RMVF;
}

uint8_t ResetCause_GetLatched(void)
{
    return s_reset_cause.latched;
}

void ResetCause_RecordHardFaultAndReset(void)
{
    ResetCause_RecordAndReset(RESET_CAUSE_HARDFAULT);
}

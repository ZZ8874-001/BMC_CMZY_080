/**
 ******************************************************************************
 * @file    self_test.h
 * @brief   上电自检：LSE、BQ 链路/首帧数据、COMP2 运行、黑匣子区。
 *
 * 非阻塞状态机，由主循环轮询；完成后给出通过/失败位图。
 ******************************************************************************
 */
#ifndef APP_SELF_TEST_H
#define APP_SELF_TEST_H

#include <stdint.h>

#define SELFTEST_FAIL_LSE        (1u << 0)
#define SELFTEST_FAIL_BQ_LINK    (1u << 1)
#define SELFTEST_FAIL_BQ_DATA    (1u << 2)
#define SELFTEST_FAIL_COMP       (1u << 3)
#define SELFTEST_FAIL_BLACKBOX   (1u << 4)

void SelfTest_Init(void);
void SelfTest_Request(void);
void SelfTest_Poll(uint32_t now_ms);

/* 0 = 运行中，1 = 完成。 */
uint8_t SelfTest_IsDone(void);
uint8_t SelfTest_IsPassed(void);
uint16_t SelfTest_GetFailMask(void);

#endif /* APP_SELF_TEST_H */

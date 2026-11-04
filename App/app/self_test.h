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

/* TakeEvent 返回值；事件只用于一次性的蜂鸣/日志，状态本身可随时查询。 */
#define SELFTEST_EVENT_NONE      0u
#define SELFTEST_EVENT_PASS      1u
#define SELFTEST_EVENT_FAIL      2u
#define SELFTEST_EVENT_RECOVERED 3u

void SelfTest_Init(void);
void SelfTest_Request(void);
void SelfTest_Poll(uint32_t now_ms);

/* 取走并清除一次性事件。 */
uint8_t SelfTest_TakeEvent(void);

/* done 完成后仍会运行 MONITOR：BQ 动态项持续异常会重新置位，
 * 恢复后自动清除。IsPassed 始终反映当前结果。 */
uint8_t SelfTest_IsDone(void);
uint8_t SelfTest_IsPassed(void);
uint16_t SelfTest_GetFailMask(void);

#endif /* APP_SELF_TEST_H */

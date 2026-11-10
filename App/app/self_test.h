/**
 ******************************************************************************
 * @file    self_test.h
 * @brief   上电自检：RTC LP 时基（LSE/LSI 由宏选择）、BQ 链路/首帧数据、COMP2 运行、黑匣子区。
 *
 * 非阻塞状态机，由主循环轮询；完成后给出通过/失败位图。
 ******************************************************************************
 */
#ifndef APP_SELF_TEST_H
#define APP_SELF_TEST_H

#include <stdint.h>

#define SELFTEST_FAIL_TIMEBASE   (1u << 0)   /* RTC LP 时基不可用 */
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
/* 主循环每次 SelfTest_Poll 前调用。partial 仅能来自同批双次确认的
 * PARTIAL_VC5_PACK；defer=1 才延长启动 BQ 动态宽限。不能用旧快照或单次
 * reject 推断 partial。静态自检和真实保护/通信故障始终正常判失败。
 * 此接口不构造全量好帧，也不生成 60 s 异常报警（由主编排负责）。 */
void SelfTest_SetVoltageAnomaly(uint8_t partial, uint8_t defer);
void SelfTest_Poll(uint32_t now_ms);

/* 首次完成的 HAL ms（Request 后重置）；仅在 IsDone!=0 时有效，
 * 冷/PIN 启动可据此额外保持 2 s BOOT 蓝闪。 */
uint32_t SelfTest_GetCompletionMs(void);

/* 取走并清除一次性事件。 */
uint8_t SelfTest_TakeEvent(void);

/* done 完成后仍会运行 MONITOR：BQ 动态项持续异常会重新置位，
 * 恢复后自动清除。IsPassed 始终反映当前结果。 */
uint8_t SelfTest_IsDone(void);
uint8_t SelfTest_IsPassed(void);
uint16_t SelfTest_GetFailMask(void);

#endif /* APP_SELF_TEST_H */

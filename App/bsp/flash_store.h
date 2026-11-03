/**
 ******************************************************************************
 * @file    flash_store.h
 * @brief   通用 Flash 擦/写原语（黑匣子等持久化复用）。
 *
 * 规则：擦除 ~25 ms、编程按 8 字节双字，期间取指停摆——只允许初始化/主循环
 * 调用，严禁中断上下文。本模块只做机械擦写，不解释内容。
 *
 * 移植自 Armor_Cmzy26_081/App/bsp/flash_store（语义一致）。
 ******************************************************************************
 */
#ifndef APP_FLASH_STORE_H
#define APP_FLASH_STORE_H

#include <stdint.h>

/* 擦除 page_addr 所在的 2 KB 页。返回 0 成功，1 擦除失败，3 解锁失败。 */
int Fs_ErasePage(uint32_t page_addr);

/* 向 addr 写入 len 字节（len 必须为 8 的倍数），自动解锁/上锁。
 * 返回 0 成功，2 编程失败，3 解锁失败，-1 参数错误。 */
int Fs_ProgramDwords(uint32_t addr, const uint8_t *src, uint32_t len);

#endif /* APP_FLASH_STORE_H */

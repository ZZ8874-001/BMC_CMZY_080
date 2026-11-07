/**
 ******************************************************************************
 * @file    main_app.h
 * @brief   应用入口：App_Init（全部 MX_*_Init 之后）/ App_Loop（while(1) 内）。
 ******************************************************************************
 */
#ifndef APP_MAIN_APP_H
#define APP_MAIN_APP_H

#include <stdint.h>

void App_Boot(void);       /* main() 唯一入口：LP/IWDG 分流后进入 App_Init */
void App_Init(void);
void App_Loop(void);

/* 供 power_mode 使用：250 Hz 时基启停与运行时间/故障快照。 */
void App_TickStart(void);
uint32_t App_GetUptimeSeconds(void);
uint16_t App_GetFaultFlags(void);

#endif /* APP_MAIN_APP_H */

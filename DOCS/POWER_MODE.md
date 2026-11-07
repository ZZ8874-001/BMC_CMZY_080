
# BMC_CMZY_080 电源模式与切换说明（L0 + M1 基础）

依据（2026-11-07 复位式 LP 收束，默认 LSI + SMBA=0，`make -j2 all` 通过，text 68760 B，`_image_end`=0x08010D40）：

- `App/app/power_mode.c`（时钟切换、LP 进入/唤醒、RTC/PB5 中断 owner）
- `App/app/main_app.c`（App_Init / App_Loop / 250 Hz tick 编排）
- `App/bsp/bq_ic.c`、`App/bsp/ws2812_uart.c`、`App/bsp/buzzer.c`、`App/bsp/flash_store.c`
- `App/comm/app_log.c`、`App/app/faults.c`、`App/app/black_box.c`、`App/app/state_machine.c`、
  `App/app/state_store.c`、`App/detect/soc_est.c`、`App/detect/bms/*`、`App/app/led_status.c`、
  `App/app/self_test.c`、`App/app/reset_cause.c`
- `Core/Src/main.c`（生成初始化）、`BMC_CMZY_080.ioc`

> **当前实现（2026-11-07 收束，代码为准）**：LP 是 **IWDG 超时复位式**。NORM 空闲 5 s 后 `PowerMode_RequestLp()` 写 `START` 令牌并 `NVIC_SystemReset()`；下一次 boot 走 `App_Boot()` 的轻量 `lp_cycle` 路径（不跑 `App_Init`）读 RTC epoch，做 BQ 状态/充电器/PB5/PA8 检查，跨过 30 min 点则做 `BQ_LpMonitorRun()` 并写一条黑匣子 periodic 记录，然后按 RTC 剩余时间配置校准后的 IWDG timeout（基础 30 s）并进 STOP1。IWDG 超时复位；下一次 boot 由 `LpCycle_ClassifyBoot()` 校验 `STOPPED` 令牌与 RTC 间隔后继续。真正唤醒（PB5/充电/ALERT/监测失败）写 `WAKE` 令牌 + 软件复位进入完整 `App_Init()`，再由 `LpCycle_ApplyWake()` 注入唤醒原因/策略。
>
> - **时基宏**：`LP_RTC_USE_LSE=0`（默认，当前硬件 LSE 不起振）= RTC/LP 参考走 **LSI**，不启动 LSE、不置 `FAULT_LSE`；LSI 同时驱动 IWDG，30 min 只是相对时间，绝对时间会漂。`LP_RTC_USE_LSE=1` = RTC 走 LSE；LSE 起振失败时 `RtcTime_MspClockSource()` 回退 LSI 保 boot，但 `PowerMode_LpTimebaseReady()` 仍要求 LSE，因此 **LP 被禁用**并置 LSE/TIMEBASE 故障，NORM/5 V 继续。切换宏后第一次 boot 若 RTCSEL 改变会触发一次 `BDRST`，清空 BKP 会话/日历，之后不得每个 boot 反复切换。
> - **SMBA 备用**：`LP_SMBA_WAKE_TEST=0`（默认）时 PB5 只走 EXTI5；置 1 时第一个 LP 周期临时使用 I2C1_SMBA host alert，仍有 EXTI5 回退，记录 `pb5_mode`/`smba_result` 后自动恢复 EXTI5。该构建仅台架使用。
> - **LP 期间外设**：5 V 关、PA1 高（停 OSC1/PACK_SC）、HSE/PLL 关、SYSCLK 由 STOP1 停止；`STOPWUCK=1` 指定 HSI16 为退出时钟；RTC 常开只作绝对时间；PB4 每次轻量 boot 打 `LP_HEARTBEAT_MS=10 ms` 高脉冲（生产测电流可改 0）；LPTIM1 和 RTC WUT 均不参与 LP。
> - **BQ**：轻量路径用 `BQ_LpBootPrepare()`/`BQ_LpMonitorRun()` 的有界阻塞 I2C；30 min 监测临时开 ADC/TEMP+CC，不改 FET；会话期间 BQ 保持 `SYS_CTRL1=0x00`、`SYS_CTRL2` 由 LP FET mask 决定（默认 0x00）。NORM 的 500 ms 轮询、ADC 恢复和 SOC/SOH 不参与 LP tick。
> - **BKP 会话**：BKP1..10 = CRC 保护的 `lp_session_t`（LP03 布局），BKP20..23 保留 HardFault 现场。没有每 tick/每唤醒的调试计数；最小诊断只保留 `s_power.boot_reset_cause`、`wake_cause`、`wake_count` 和 session `seq`。
> - **复位分类优先级**：HardFault/NMI → 时基门控 → 软件 `START`/`WAKE` 令牌 → 合法 `WATCHDOG + STOPPED + RTC 间隔`（RESUME）→ PIN/BOR 使其失效 → 其余 WATCHDOG 视为 FAULT → NORMAL。该板会把软件/IWDG 复位额外报告为 PIN，因此合法会话必须先于 PIN/BOR 判断。
>
> **历史（仅作记录，不代表当前行为）**：2026-11-06 之前曾使用 RTC WUT 2 s、LPTIM1/LSI 2 s 和 4 s RTC 兜底等 STOP1 内周期中断方案；这些方案在 STOP1 唤醒路径上不稳定，相关 `timebase.c`、`lp_test_mode`、WUT 重布防、BKP stage 诊断和 SRAM 面包屑均已从当前代码删除。旧版本本节及正文中的 WUT/LPTIM/2 s 嗅探描述均属该历史阶段。

## 1. 外设状态总表


| 外设 / 板级器件       | 上电（生成代码跑完）                                             | NORM 常态                                                                                                                                                                                              | LP 保持态                                                                                                                                                  |
| ----------------------- | ------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| SYSCLK | HSI16，Scale2，2WS（`SystemClock_Config()` 为 HSI-only 补丁） | HSE bypass 12 MHz → PLL 80 MHz，Scale1，4WS | STOP1 停振；每次 IWDG 轻量 boot 以 HSI16 重启 |
| HSI16 / MSI | HSI16 开；MSI 不用 | HSI16 开；MSI 不用 | `STOPWUCK=1` 指定 HSI16 为 STOP 退出时钟；轻量 boot 全程 HSI16 |
| HSE / PLL / OSC1 | 关；`MX_GPIO_Init` 把 PA1 拉低 | 开；PA1=0（OSC1 + PACK_SC 使能） | 关；PA1 高。唤醒走复位，由完整 boot 重新切 HSE/PLL |
| LSE / RTC | **默认 `LP_RTC_USE_LSE=0`：RTC 走 LSI**，不启动 LSE；`LP_RTC_USE_LSE=1` 时由 `RtcTime_MspClockSource()` 有界启动 LSE，失败回退 LSI 但禁用 LP | RTC 运行（异步 127/同步 255），只作绝对时间；WUT 未布防 | RTC 常开作 epoch 参考；无 WUT/无 RTC 周期唤醒；LP tick 是 IWDG 超时复位 |
| SysTick | 1 ms HAL 时基 | 1 ms | STOP1 中不用；轻量 boot 在 HSI16 下 `HAL_InitTick` 后使用 |
| TIM2 | 占位配置，未启动 | 覆写 PSC319/ARR999 → 250 Hz，prio2 | 轻量 boot 不初始化/不启动 |
| TIM1 | 初始化未启动 | 蜂鸣器 PWM | 轻量 boot 不初始化；5 V 关 |
| I2C1 | 生成初始化；`BQ_Init()` 重配 95 kHz + DMA | 95 kHz；TX DMA1_Ch6、RX DMA1_Ch7；500 ms 轮询 | 轻量 boot 用 `BQ_LpSniffInit()` 重建无 DMA 阻塞 I2C，结束后 DeInit；30 min 监测同路径 |
| USART1 | 初始化 115200（生成值） | BRR=694（115200），日志 TX DMA1_Ch4 | 轻量 boot 不初始化；5 V 关 |
| USART2 | 初始化（生成值） | 重配 3.8095 MBd，IT 发送驱动 WS2812 | 轻量 boot 不初始化；5 V 关 |
| COMP2 | 初始化未启动 | 启动；250 Hz + 32 样本消抖 | 轻量 boot 的充电检查：PA1 短暂拉低 → `HAL_COMP_Start` → 30 ms → 读电平 → Stop → PA1 高 |
| DAC1 | 初始化（内部模式、未启动） | 不参与 | 不参与 |
| CAN1 | 初始化（无业务） | 初始化态 | 不初始化；TJA1044 断电 |
| PVD | 未开 | 开：2.8 V、prio4 | 轻量 boot 不跑 `Fault_Init`，PVD 不配置 |
| EXTI PA8（BQ ALERT） | 事件模式、上升沿，无 IRQ/挂起位 | 500 ms 轮询 SYS_STAT 消化 | **不能唤醒 WFI/STOP1**；每次轻量 boot 按电平检查，最坏延迟一个 IWDG 周期 |
| EXTI PB5（WAKE 按键） | `.ioc` 为 `GPIO_Input` + 内部上拉；EXTI5 由 `Button_WakeInit()` 在轻量 boot 配置（下降沿，prio0） | 4 ms 轮询 | **生产唤醒源 EXTI5**：ISR 只置标志，LP 路径写 `WAKE` 令牌并软件复位；SMBA 测试由宏启用并保留 EXTI5 回退 |
| GPIO PA1 | 低（OSC1 + PACK_SC） | 低 | 高（OSC1 tri-state、PACK_SC 关），仅充电检查窗口短暂拉低 |
| GPIO PB0 | 低（5 V 关） | 高（5 V 开） | 低（5 V 关） |
| GPIO PB1 | 低；App 做 2 ms 高脉冲再回低 | 低（BQ 已 boot；高会短路 NTC） | 低 |
| GPIO PB3/PB4 | 低（LED 灭） | 状态灯效（NORM PB4 常亮） | PB3 灭；PB4 每次轻量 boot 打 `LP_HEARTBEAT_MS=10 ms` 高脉冲作为存活心跳，其余时间灭 |
| GPIO PA7 | AF1 占位 | AF1 PWM | 轻量 boot 不初始化；5 V 关 |
| BQ7692003PWR | SHIP；App boot 脉冲 + 配置 | NORMAL：`SYS_CTRL1=0x18`、`SYS_CTRL2=0x40|FET`、保护配置/回读、300 ms SETTLE、500 ms 轮询、ADC 异常受控恢复 | LP 会话：`SYS_CTRL1=0x00`（ADC/TEMP 关）、`SYS_CTRL2` 按 LP FET mask（默认 0）；无电压/电流/温度轮询，30 min 点临时开 ADC/TEMP+CC 采一轮并恢复 |
| 5 V 轨负载 | 关 | 开：TJA1044、RGB、蜂鸣器、U18 +5 V | 关（全部断电） |
| SWD 调试口 | 可用 | 可用 | 仅编译期 `POWER_LP_KEEP_SWD` 控制 STOP1 下 `DBG_STOP`（无运行期覆盖字段）；测 LP 电流应物理断开 SWD |
| Flash 黑匣子 | 头扫描（最大有效 seq） | 30 min/条 + 唤醒事件 | **每次跨 30 min 的轻量 boot 写 1 条 periodic**；普通 IWDG tick 不写 Flash |

## 2. 上电 → NORM 流程

### 2.1 硬件复位与生成代码（main.c）

1. 复位向量 → `HAL_Init()`（Flash 接口、SysTick 1 ms）。
2. `SystemClock_Config()`：**HSI16-only**（PLL 关，Scale2，2WS），不等 LSE；这是每次 CubeMX 重新生成后必须重打的补丁。PA1 复位期间高阻、R16 4k7 上拉，但 boot 不依赖它。
3. `MX_GPIO_Init()`：PA1/PB0/PB1/PB3/PB4 输出低；PA8 = EXTI 事件上升沿（无 IRQ）；PB5 = `GPIO_Input` + 内部上拉，**不生成 EXTI**（由 `Button_WakeInit()` 在轻量 LP boot 配置）。
4. `MX_DMA_Init()` → `MX_CAN1_Init()`、`MX_COMP2_Init()`、`MX_DAC1_Init()`、`MX_I2C1_Init()`、`MX_USART1_UART_Init()`、`MX_USART2_UART_Init()`、`MX_TIM1_Init()`、`MX_RTC_Init()`（RTC 源由 `RtcTime_MspClockSource()` 按宏选择）、`MX_TIM2_Init()`（占位）、`MX_LPTIM1_Init()`（不使用）、`MX_IWDG_Init()`（USER CODE 启动 boot guard/256）。全部外设**只初始化不启动**；除生成代码外不主动开 NVIC。

### 2.2 App_Init()（App/app/main_app.c，顺序即依赖序，无时间预算）


| 步骤 | 动作                                      | 说明                                                                                                                                                                                                                                                                 |
| ------ | ------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1    | 清零 `s_app`                              | 全部运行状态置初值                                                                                                                                                                                                                                                   |
| 2    | `ResetCause_Init()`                       | RCC->CSR + BKP0R 魔数（hardfault 记录），消费后清 RMVF                                                                                                                                                                                                               |
| 3    | `Fault_Init()` → `Fault_PvdEnable()`     | 故障图清零；PVD 2.8 V 布防（prio4）                                                                                                                                                                                                                                  |
| 4    | `PowerMode_Init()`                        | RTC 源严格校验（LSI 模式不置 LSE 故障；LSE 模式失败置 live `FAULT_LSE`，LSERDY 连续 100 ms 后由 `PowerMode_LseHealthTask()` 清 live/sticky，但 RTCSEL 不热切换）；按 `PowerMode_LpTimebaseReady()` 置/清 `FAULT_BIT_TIMEBASE`；清遗留 WUT/LPTIM；**时钟切换**（见下）                                                                                                                  |
| 5    | `BlackBox_Init()`                         | 扫描 1020 槽取最大有效 seq；`_image_end > 0x08018000` → sticky `FAULT_FLASH_LOG`；恢复 `uptime_base`                                                                                                                                                                |
| 6    | 降级检查                                  | 时钟切换失败 → PB0 保持低、**直接 return**：只跑降级循环（PB3 以 250 ms 相位翻转 ≈2 Hz 闪烁），通信/BQ/tick 全部不启动                                                                                                                                             |
| 7    | PB0 = 1 + `HAL_Delay(20)`                 | 5 V 软启动余量（CAN 收发器/RGB/蜂鸣器/U18 +5 V）                                                                                                                                                                                                                     |
| 8    | `App_Log_Init()` + `[BOOT]` 日志          | USART1 覆写 115200、DMA1_Ch4、NVIC prio4；打印复位原因                                                                                                                                                                                                               |
| 9    | `BQ_Init()`                               | I2C1 95 kHz（`Init.Timing` 在 `HAL_I2C_Init` 前写入）+ DMA 重链（TX DMA1_Ch6 / RX DMA1_Ch7）+ NVIC prio3；**PB1 高脉冲 ≥2 ms**（强制退出 SHIP）→ 等 20 ms → 配置脚本（CC_CFG=0x19 写/读回、ADCGAIN1/2/ADCOFFSET 读、SYS_CTRL1=0x18、清 SYS_STAT、SYS_CTRL2=0x40）→ 保护配置（PROTECT1/2/3、OV_TRIP/UV_TRIP 写后逐项回读，失败重试配置） |
| 10   | `Ws2812_Init()`                           | USART2 时钟/波特率重配（USART2SEL=PCLK1、BaudRate=3809523、UE=0 写 BRR=21=3.8095 MBd）+ TXINV/推挽/清半双工；RGB 帧中断发送，USART2 IRQ prio1，不使用 DMA                                                                                                            |
| 11   | `LedStatus_Init()`                        | BOOT 灯效；PB3/PB4 先灭                                                                                                                                                                                                                                              |
| 12   | `Buzzer_Init()`                           | PA7 配 AF1，PWMN 停止 + MOE 关（静默布防）                                                                                                                                                                                                                           |
| 13   | `SelfTest_Init()` + `SelfTest_Request()`  | 自检启动（见 §3）                                                                                                                                                                                                                                                   |
| 14   | `StateMachine_Init()`                     | BOOT 状态、STANDBY 流动                                                                                                                                                                                                                                              |
| 15   | `HAL_COMP_Start(&hcomp2)`                 | 比较器启动（PA3/PA4 有效）                                                                                                                                                                                                                                           |
| 16   | `App_TickTimerInit()` + `App_TickStart()` | ★ 最后：TIM2 覆写 PSC319/ARR999、清 UIF、NVIC prio2，启动 250 Hz 中断                                                                                                                                                                                               |

**时钟切换 `PowerMode_ClkSwitchToHsePll()`**（唯一 HSE/PLL 入口，唤醒路径复用；App 代码禁止调 `SystemClock_Config()`）：

PA1=0（OSC1/PACK_SC 使能）→ `HAL_Delay(30)` 起振余量 → HSE **bypass** 12 MHz ON → PLL ON（M3/N40/R2）→ Scale1 → SYSCLK=PLL、AHB/APB1/APB2=DIV1、4WS → 80 MHz。整个序列重试 3 次；全失败 → sticky `FAULT_CLK` + 降级（步骤 6）。

### 2.3 上电自检（SelfTest）

非阻塞：IDLE → WAIT_BQ（超时 3000 ms；包含 300 ms ADC settle 与首帧校验）→ DONE。检查项：`PowerMode_LpTimebaseReady()`（RTC 源为宏选择且就绪，LSI 模式下不要求 LSE）、BQ 状态 == RUN、BQ 数据新鲜（`fresh && valid`）、COMP2 == BUSY、黑匣子无错误。完成后主循环触发：通过 → 蜂鸣器 `BOOT_OK`（4 kHz × 2 短）+ `[TEST] pass`；失败 → `FAULT` 音型（2 kHz × 3 长）+ `[TEST] fail=0xXXXX`，灯切 FAULT 模式。`s_power.diag_self_test_fail`/`diag_blackbox_error` 保存失败位图。首个有效电芯数据约在 boot 脉冲后 300 ms 出现。

## 3. NORM 常态运行

- **时钟**：HSE bypass 12 MHz → PLL → 80 MHz，Scale1，4WS；RTC 1 Hz 时基默认来自 LSI（`LP_RTC_USE_LSE=0`），切到 LSE 后来自 32.768 kHz；HSI16 保持开启。
- **250 Hz TIM2 中断（ISR 内只读寄存器/置标志/数数）**：COMP2 采样 + 32 样本消抖提交充/放方向；LED 相位推进（PB3/PB4 + RGB dirty 标志）；500 ms → `flag_bq_read_request`；1 s → `uptime_s++` + `flag_diag_request`；30 min → `flag_blackbox_request`。
- **主循环（每圈固定顺序）**：`PowerMode_Loop()`（消费调试器写的 LP 请求）→ 按键/手势 → BQ 读请求 + `BQ_Poll()`（事务状态机）→ `SocEst` → 自检轮询 → 1 s 诊断（BQ 新鲜度 >2 s → `FAULT_BQ_STALE`；每 10 s 一条 `[DIAG]`/`[POL]` 日志）→ `StateMachine_Update` → `ChargePolicy_Task` + `CalStream_Task` + 灯效 → 自动 LP 判断 → 黑匣子 30 min 记录 → `Buzzer_Poll()` → `LedStatus_Flush()` → `App_Log_Poll()` + `FlushSmall(4/32)`。
- **BQ**：每 500 ms 一轮 10 个事务（读 SYS_STAT → 清 CC_READY/XREADY → 读 VC1/VC2/VC5/VC3/VC4/BAT/TS1/CC，每字节 CRC-8）；配置完成时写并回读保护阈值（见 §1 表），随后进入 `BQ_ST_SETTLE` 等 300 ms；通信故障 3 次重试 → sticky `FAULT_BQ_COMM` + 总线重建 + 重跑配置序列；事务 100 ms 看门狗。SCD/OCD 在每次状态读取时更新 live；OV/UV/XREADY 只在电压帧通过 `valid` 校验后更新；假 OV/UV 仅在稳定帧中三相电压安全时用一次 `SYS_STAT` 写清除。无效帧 `fresh=0`，不进入 SOC/黑匣子/状态机。CC 换算：`s_bq.current_ma = cc_raw × 844 / 150`（1 LSB=8.44 µV、分流 1.5 mΩ、250 ms 平均，带符号）。MOS 开关标志：`s_bq.request_fet_mask`（bit0=CHG，bit1=DSG，由 `ChargePolicy_Task` 唯一驱动）、`fet_mask_applied`；run script 完成后若不同，用一次性 DMA 写 `SYS_CTRL2=0x40|mask`（保留 CC_EN）。`BQ_DEFAULT_FET_MASK` 当前被手动设为 `BQ_FET_MASK_ALL`，但 `ChargePolicy_Init()` 会请求 0，IDLE/自动 LP 保持 0；CHARGE/DISCHARGE 才请求 `0x03`。配置脚本最后只写 `0x40`（CC on、FET off）；复位式 LP 的轻量路径由 `BQ_LpBootPrepare()`/`BQ_LpMonitorRun()` 管理 BQ，`BQ_PrepareForLpMask()`/`s_power.lp_fet_mask` 只是旧的兼容 API，不再参与当前 LP 周期。
- **状态机**：BOOT →（数据新鲜）→ NORM；关键故障（BQ_COMM/XREADY/OV/UV/SCD/OCD/POWER/CLK）→ FAULT（故障消失回 BOOT 重判）。流动分类：充电（COMP2 高）/放电（低）/静置（数据不新鲜）。
- **灯效**：BOOT = PB4 1 Hz 慢闪 + RGB 蓝；NORM = PB4 常亮 + RGB（充电绿 48 / 放电蓝 48 / 静置白 12）；FAULT = PB3 5 Hz 快闪 + RGB 红。
- **蜂鸣器**：音型表（BOOT_OK/FAULT/LP_ENTER/WAKE），主循环推进相位，TIM1 无中断。
- **日志**：USART1 115200 非阻塞，1 KB 环形 + 64 B 暂存，TX DMA1_Ch4，100 ms DMA 看门狗。
- **PVD**：3.3 V 轨 < 2.8 V → 中断 → sticky `FAULT_POWER`。
- **黑匣子**：每 30 min 一条 24 B 记录（seq/uptime/3 节电压/总压/CC/TS1/种类/CRC16）；跨页首槽先擦除整页（~25 ms 取指停摆，约每 42.5 h 一次）。
- **LP 请求**：主循环 5 s 空闲自动调用 `PowerMode_RequestLp()`；调试器仍可写 `s_power.request_lp = 1`。自动 LP 不会被调试器连接阻止；看 `s_app.auto_lp_block` 判断未进入的原因（bit0=充电，bit1=按键，bit2=自检未完成/失败，bit3=状态机非 NORM，bit4=BQ 不新鲜/恢复中，bit5=RTC LP 时基不可用，bit6=电源模式非 NORM，bit7=策略 CHARGE/DISCHARGE/CAL 或 CAL pending）。bit5 只看 `PowerMode_LpTimebaseReady()`：默认 LSI 模式不因 LSE 缺失置位；`LP_RTC_USE_LSE=1` 且 LSE 失败时才阻止 LP。

### 3.1 L1 充放电策略与 CAL

- `ChargePolicy_Task()` 在 `StateMachine_Update()` 之后运行，状态优先级：`FAULT > CAL > CHARGE(充电器) > DISCHARGE(手势) > IDLE`。
- `comp_charging`（COMP2 消抖）上升立即置充电器存在；连续低 1000 ms 才撤销。充电器插入自动进入 CHARGE，双管全开；移除后回 IDLE，双管关。
- `GESTURE_PAIR_HOLD3`（LP 短按唤醒后 2 s 内第二次按住 3 s）切换 DISCHARGE/IDLE；CHARGE/FAULT/CAL/充电器存在时拒绝。
- 关键故障强制请求 0、进入策略 FAULT；CAL 自动结束，不自动恢复。
- IDLE 双管关；CHARGE/DISCHARGE 请求 `BQ_FET_MASK_ALL`；仅 CAL/FAULT 在 `BQ_ST_RUN && BQ_IsIdle()` 时 best-effort 立即写，正常充放转换最多等一个 500 ms BQ 轮询。
- CAL 由调试器写 `s_policy.cal_fet_mask`（0x00..0x03）再写 `s_policy.cal_mode=1` 进入；写 `cal_mode=0` 退出。CAL 期间自动 LP 被 bit7 阻止，BQ 不 fresh/非 RUN/LP 唤醒/关键故障都会结束 CAL。
- `[CAL]` 由 `cal_stream.c` 输出：BEGIN/HDR + 每个新 BQ 快照一行（约 2 Hz）+ END；黑匣子只写 `BB_EVENT_CAL_BEGIN/END`（0x12/0x13），不逐样本写 Flash。CAL 日志只用现有快照字段（mV、cc_raw、current_ma、ts1_raw、SocEst 的 ts1_mv/temp/SOC/SOH），不恢复 raw_cell/raw_pack。END 的 `reason`：1=user，2=fault，3=notready，4=lp。

## 4. NORM → LP：写 START 令牌后复位

自动 LP 只在 `App_Loop()` 的固定门控全部满足、且 5 s 无活动时调用 `PowerMode_RequestLp()`。阻止位图 `s_app.auto_lp_block` 的位定义：

| 位 | 宏 | 条件 |
| --- | --- | --- |
| 0 | `APP_LP_BLOCK_CHARGING` | COMP2 连续判定充电中 |
| 1 | `APP_LP_BLOCK_BUTTON` | PB5 当前按下 |
| 2 | `APP_LP_BLOCK_TEST` | 自检未完成或失败 |
| 3 | `APP_LP_BLOCK_STATE` | 状态机不是 NORM |
| 4 | `APP_LP_BLOCK_BQ` | BQ 不 fresh、ADC 恢复中或恢复已失败 |
| 5 | `APP_LP_BLOCK_TIMEBASE` | `PowerMode_LpTimebaseReady()==0`（RTC 源/就绪不匹配） |
| 6 | `APP_LP_BLOCK_POWER` | `PowerMode_GetState()!=POWER_STATE_NORM` |
| 7 | `APP_LP_BLOCK_POLICY` | CHARGE/DISCHARGE/CAL 或 CAL pending |

`s_power.lp_disable_auto=1`（IWDG 故障复位或调试器写入）会让 `PowerMode_RequestLp()` 直接拒绝；调试器仍可直接写 `s_power.request_lp=1`，但入口仍受时基门控。

`PowerMode_Loop()` 消费 `request_lp` 并调用 `LpCycle_RequestStart()`：

1. `PowerMode_LpTimebaseReady()==0` → 保持 NORM，下一轮再试；
2. 等 `BQ_IsIdle()` 且 I2C1 不 BUSY，最多 100 ms（每 1 ms 喂 IWDG）；
3. `RtcTime_Read()` 取 epoch；
4. `LpSession_Load()`：无效/IDLE/FAULT 时清空会话，设 `next_mark_epoch = now + 1800 s`、LSI 初值 32 kHz、`last_pr=6`、`last_rlr=4095`、boot overhead 初值 300 ms；有效会话则保留 mark 并推进到未来；
5. 关闭遗留 RTC WUT（旧固件可能留下，会写 BKP6 破坏会话）；
6. `s_lp.seq++`、`LpSession_Save()` 写 BKP1..10，喂狗，`__DSB()`，`NVIC_SystemReset()`。

这个函数**不直接关 5 V**；复位后的新 boot 在生成初始化完成后由 `App_Boot()` 先把 PA1 拉高、5 V 关，再判 LP。这样避免在 NORM 中提前断电导致 HSE/BQ 半状态。

## 5. 轻量 LP boot 与 STOP1

### 5.1 App_Boot() 分流

`main()`：`HAL_Init()` → `SystemClock_Config()`（HSI16，不等 LSE）→ 各 `MX_*_Init()` → `MX_IWDG_Init()`（USER CODE 调 `Watchdog_Init()`，/256 ≈32.8 s boot guard）→ `App_Boot()`。

`App_Boot()`：

1. `ResetCause_Init()` 锁存复位原因并消费 `RMVF`；
2. PA1 高、PB0 低（关 PACK_SC/HSE 前端和 5 V）；
3. `RtcTime_Init()`：按 `LP_RTC_USE_LSE` 选择 RTC 源，修复/校验 RTCEN，必要时播种 2001-01-01；
4. `LpCycle_ClassifyBoot()`；
5. `LP_BOOT_START`/`LP_BOOT_RESUME` → `LpCycle_Run(boot)`，正常不返回；
6. 其它 → `App_Init()`；`LP_BOOT_FAULT` 在完整 boot 后再 `PowerMode_ForceDisableAutoLp()` + 会话失效。

### 5.2 LpCycle_Run() 顺序

1. 会话必须已加载、`PowerMode_LpTimebaseReady()` 必须为 1、RTC 可读，否则 `LpCycle_ForceFullWake(LP_WAKE_FAULT)`；
2. `LP_BOOT_RESUME` 先跑 `LpCycle_Calibrate()`；连续 2 次间隔异常 → 完整唤醒（`LP_WAKE_FAULT`）；
3. 会话置 `PREP`、清 FULL_WAKE/FAULT 标志并保存；
4. `PowerMode_LpDbgStopApply()` 按编译期 `POWER_LP_KEEP_SWD` 设置 `DBGMCU_CR.DBG_STOP`；
5. 5 V 关、PB3/PB4 灭、PA1 高、COMP2 停；DeInit USART1/USART2/CAN/DAC/TIM1/TIM2，停 LPTIM1；`Button_WakeInit()` 独占配置 PB5 EXTI5；
6. 关闭遗留 WUT；PB5 已按下 → 直接完整唤醒；
7. `LP_SMBA_WAKE_TEST=1` 时，若上一轮 SMBA 测试仍无结果则判失败；否则本轮置 `pb5_mode=1` 并在 BQ sniff 后启用 SMBA；
8. BQ sniff：`BQ_LpSniffInit()` + `BQ_LpBootPrepare()`，失败重试一次；仍失败 → 完整唤醒（FAULT）；
9. 若 `LOAD_PRESENT=1 && CHG_ON=0`，做充电器电平检查：PA1 短暂拉低 → `HAL_COMP_Start` → 30 ms → 读 COMP2 → Stop → PA1 高；高 = 充电器 → `LP_WAKE_CHARGER`；
10. PA8 仍高 → `LP_WAKE_ALERT`；
11. PB4 打 `LP_HEARTBEAT_MS=10 ms` 高脉冲（存活心跳；改 0 可省电）；
12. 在 30 min 监测之前测 boot overhead；
13. 若 `next_mark_epoch==0` 则设 `now+1800 s`；剩余时间 ≤`LP_MARK_TOLERANCE_MS=1000 ms` 或已过点 → `LpCycle_DoMark()`；
14. 重读 RTC，计算 `target_ms`：若剩余 ≤30 s + overhead，则睡到 mark 前（`remaining - overhead`）；否则基础 30 s；再减 overhead，最小 `LP_MIN_SLEEP_MS=1 s`；
15. `LpCycle_EnterStop(target_ms)`。

### 5.3 LpCycle_EnterStop()

- `Watchdog_SetTimeoutMs(target_ms, lsi_hz, NULL)` 从校准后的 LSI 频率选最小分频并写 IWDG PR/RLR；
- 读 RTC；最后检查 PB5/PA8，已发生则完整唤醒，不带着唤醒请求进 STOP；
- 保存 `last_pr`、`last_rlr`、`last_arm_epoch/ms`，状态置 `STOPPED` 并写 BKP；
- 保持 PWR/RTCAPB 的 APB 与 Sleep 时钟；GPIOA/GPIOB/SYSCFG Sleep 时钟保持（退出后要读 PB5/PA8）；SMBA 测试时额外保持 I2C1；
- `HAL_SuspendTick()`、清 PendST；`RCC_CFGR.STOPWUCK=1`（退出到 HSI16）、`PWR_CR1.LPMS=STOP1`、`SCB.SCR.SLEEPDEEP=1`、`__DSB()`、`__WFI()`。

正常路径是 IWDG 超时**复位**：CPU 不执行 WFI 之后的代码，直接进入下一次 `App_Boot()`。若 PB5/ALERT 先把 CPU 唤醒，则 WFI 之后恢复 `SystemCoreClock=16 MHz`/`HAL_InitTick()`、喂狗，再检查 PB5/PA8：确认按键 → `LP_WAKE_BUTTON`，确认 ALERT → `LP_WAKE_ALERT`；其余未知唤醒 → `LP_WAKE_FAULT`。所有真实唤醒都是 `LpCycle_ForceFullWake()`：写 `PAUSED + FULL_WAKE` 令牌，软件复位到完整 `App_Init()`。

## 6. LP → NORM：唤醒注入

完整 boot 的 `App_Init()` 末尾调用 `LpCycle_ApplyWake()`：

- 读会话，无 FULL_WAKE 标志则直接返回；
- `s_lp.wake_cause` 映射为 `POWER_WAKE_BUTTON/CHARGER/BQ_ALERT/MONITOR`，未知则回退 `POWER_WAKE_RTC`；
- `PowerMode_NoteLpFullWake(cause)` 只更新 `s_power.wake_cause` 和 `wake_count`；
- `Button_OnWake()`、`ChargePolicy_OnLpWake()`：按键唤醒回 IDLE，放电仍需 LP 短按后 2 s 内再 `PAIR_HOLD3`；充电唤醒由策略层等 BQ fresh 后进 CHARGE；
- FAULT 唤醒强制 `PowerMode_ForceDisableAutoLp()`；
- 非 MONITOR 唤醒写一条黑匣子 wake 记录（wake mask）；
- 清 FULL_WAKE、状态置 PAUSED、`seq++` 保存。

PB5 的 EXTI5 回调只置标志；`LpCycle_EnterStop()` 退出 WFI 后读取标志/引脚并决定唤醒原因。PA8 是 EXTI **事件**模式，不能唤醒 STOP1，只在每次轻量 boot 和进入 STOP 前按电平检查，最坏延迟一个 IWDG 周期。

## 7. 30 min 监测与 LSI 校准

### 7.1 30 min 点

- `s_lp.next_mark_epoch` 跨复位保存在 BKP 会话中；新会话从 `now + 1800 s` 开始；
- 每次轻量 boot 若剩余 ≤1 s 或已过点，调用 `LpCycle_DoMark()`：
  1. `BQ_LpSniffInit()`；
  2. `BQ_LpMonitorRun()`：临时开 ADC/TEMP+CC 采一轮电压/电流/温度，不改 FET；
  3. `BQ_LpSniffDeInit()`，喂狗；
  4. `BlackBox_Init()` 扫描头指针；
  5. `BlackBox_WritePeriodicSnapshot(&snap, now.sec)` 写一条 24 B periodic 记录；
  6. 喂狗，`next_mark_epoch += 1800` 直到大于当前 epoch，再读 RTC。
- 任一步失败 → `LP_WAKE_MONITOR | LP_WAKE_FAULT` 完整唤醒，并禁止本 boot 自动 LP；监测/记录失败不会静默丢掉 30 min 点。
- LP 内**不运行** SOC/SOH、`StateStore`、NORM 的 500 ms BQ 轮询或 ADC 恢复状态机；这些只在 NORM 运行。

### 7.2 IWDG 校准

`LpCycle_Calibrate()` 只在 `LP_BOOT_RESUME` 上运行：

1. 用上一次的 `last_pr/last_rlr` 和当前 `lsi_hz_q16` 计算预期 timeout；
2. 预期 <5 s 的短段跳过（RTC 秒级分辨率误差过大）；
3. `elapsed = RtcTime_DiffMs(now, last_arm)`；必须落在预期的 75%..125%，否则 `anomaly_count++`；
4. `span = elapsed - boot_overhead_ms`；`sample = IWDG_ticks / span`，只接受 26..34 kHz；
5. `lsi_hz_q16` 做 7/8 旧值 + 1/8 新值的 EMA；
6. `boot_overhead_ms` 在每次轻量 boot 的 sniff/心跳之后、30 min 监测之前用 RTC 差分测，并做 EMA；
7. `anomaly_count >= 2` → 完整唤醒（`LP_WAKE_FAULT`），避免在时钟异常时盲睡。

因此 30 min 点是“RTC 绝对时间目标 + 校准后的 IWDG 睡眠段”：IWDG 只负责把它叫醒，RTC 决定还差多少时间，下一次 IWDG timeout 按剩余时间缩短以踩准下一个 30 min 点。LSI 模式下 RTC 和 IWDG 同源，这套校准让相对周期自洽，但绝对时间仍会随 LSI 漂移。

## 8. LSE / LSI 切换

| 宏 | 默认 | 行为 |
| --- | --- | --- |
| `LP_RTC_USE_LSE` | `0` | RTC/MSP 时钟源走 LSI；不启动 LSE、不等 LSERDY、不置 LSE 故障；LP 正常 |
| | `1` | `RtcTime_MspClockSource()` 有界启动 LSE；成功 → RTC=LSE，LP 正常；失败 → RTC 回退 LSI 保 boot，但 `PowerMode_LpTimebaseReady()` 失败，**LP 禁用**并置 `FAULT_LSE`/`FAULT_BIT_TIMEBASE`，NORM/5 V/BQ 继续 |
| `LP_SMBA_WAKE_TEST` | `0` | PB5 只走 EXTI5（生产） |
| | `1` | 仅台架：第一个 LP 周期用 I2C1_SMBA host alert，EXTI5 回退；记录 `pb5_mode`/`smba_result` 后自动恢复 EXTI5 |

构建时可直接用 make 变量（`app.mk` 转发为 `-D`）：

```sh
make clean && make -j2 all                       # LSI + SMBA off（默认）
make clean && make -j2 all LP_RTC_USE_LSE=1      # LSE 模式（等硬件修好再用）
make clean && make -j2 all LP_SMBA_WAKE_TEST=1   # 台架 SMBA 实测
```

也可以直接改 `App/board.h` 的默认值；不指定 make 变量时以 board.h 为准，指定时命令行 `-D` 优先。两者不要给相反值。改宏后必须 `make clean`（否则目标文件不会重编）。RTCSEL 改变会触发一次 HAL 备份域复位：BKP 会话、日历和 HardFault 现场清空一次，之后保持。**不要**在运行中热切换 RTCSEL，也不要每 boot 切换，否则每次复位都会清会话。

## 9. 调试要点

- `s_power`：`state`、`request_lp`、`wake_cause`、`boot_reset_cause`、`lse_ok`、`degraded`、`wake_count`、`lp_disable_auto`；没有旧的 HSE/PLL 分阶段诊断、每类唤醒计数或运行期 SWD 覆盖字段。
- `s_lp`（`lp_session.c` 私有）：`state`、`seq`、`next_mark_epoch`、`last_pr/last_rlr`、`last_arm_epoch/ms`、`lsi_hz_q16`、`boot_overhead_ms`、`anomaly_count`、`wake_cause`、`flags`、`pb5_mode`、`smba_result`。
- `s_app.auto_lp_block`：自动 LP 未进入的原因位图（见 §4）。
- LP 中 SWD 调试：默认 `POWER_LP_KEEP_SWD=0` 关 `DBG_STOP`，STOP1 下调试器会失去同步且可能本身改变唤醒行为；测 LP 电流必须物理断开 SWD。需要观察时用 `make clean && make -j2 all POWER_LP_KEEP_SWD=1`。
- PB4：每次轻量 LP boot 闪 10 ms。看到约 30 s 周期短闪 = IWDG 周期在跑；完全不闪 = 没进 LP/复位后卡住；常亮 = NORM/降级。
- 历史 CSV（`dbglog/*.csv`、`Ozone_Watched_Data_1_*.csv`）只记录当时的 watch 集，不代表当前字段；当前以 `s_power`/`s_lp`/`s_app.auto_lp_block` 为准。

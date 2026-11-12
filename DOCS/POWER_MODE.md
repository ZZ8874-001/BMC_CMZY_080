
# BMC_CMZY_080 电源模式与切换说明（L0 + M1 基础）

> **2026-11-11/12 更新说明**：控制/显示在真实 VC5/BAT 可信时优先采用实测，异常时回退为 VC1/VC2 估算；保护/SOC/充满参考节正常取三节最低值，VC5/BAT 异常时只用实测 VC2。BQ OV/UV 状态过滤正常只用 VC1/VC2 实测，异常时只比较 VC2。记录只写真实可信值，异常时 V3/包压置 0；估算不落盘。VC3/VC4 仅作 raw 诊断。`bq_anomaly.c/.h` 与 partial/2 A OCP/9 帧/2 min/5 min 锁存已删除；均衡永久关闭。LP monitor 只校验 cell1/2，`-2` 电压异常返回码已删除；LP ALERT 唤醒只用 SCD/OCD/XREADY。下文相关旧描述视为历史。

依据（当前工作树：复位式 LP、M2 保护监督层；具体行为与参数以当前源码为准，旧镜像体积及构建记录不代表本次改动的验证结果）：

- `App/app/power_mode.c`（时钟切换、LP 进入/唤醒、RTC/PB5 中断 owner）
- `App/app/main_app.c`（App_Init / App_Loop / 250 Hz tick 编排）
- `App/app/protection.c`、`App/app/lp_cycle.c`（FET 监督、LP 监测）
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
> - **BQ**：轻量路径用 `BQ_LpBootPrepare()`/`BQ_LpMonitorRun()` 的有界阻塞 I2C；30 min 监测临时开 ADC/TEMP+CC、不改 FET，正常两批真实确认，异常经完整唤醒归档。会话期间 BQ 保持 `SYS_CTRL1=0x00`、`SYS_CTRL2=0x00`（ADC/TEMP/CC/双 FET 全关）。NORM 的 500 ms 轮询、ADC 恢复与 SOC/SOH 不参与 LP tick。
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
| EXTI PA8（BQ ALERT） | 事件模式、上升沿，无 IRQ/挂起位 | 500 ms 轮询 SYS_STAT 消化，清除 `CC_READY/XREADY/OVRD_ALERT` | **不能唤醒 WFI/STOP1**；轻量 boot 先清非保护位，只有 SYS_STAT 的 OV/UV/SCD/OCD/XREADY 才作为 ALERT 唤醒，最坏延迟一个 IWDG 周期 |
| EXTI PB5（WAKE 按键） | `.ioc` 为 `GPIO_Input` + 内部上拉；EXTI5 由 `Button_WakeInit()` 在轻量 boot 配置（下降沿，prio0） | 4 ms 轮询 | **生产唤醒源 EXTI5**：ISR 只置标志，LP 路径写 `WAKE` 令牌并软件复位；SMBA 测试由宏启用并保留 EXTI5 回退 |
| GPIO PA1 | 低（OSC1 + PACK_SC） | 低 | 高（OSC1 tri-state、PACK_SC 关），仅充电检查窗口短暂拉低 |
| GPIO PB0 | 低（5 V 关） | 高（5 V 开） | 低（5 V 关） |
| GPIO PB1 | 低；App 做 2 ms 高脉冲再回低 | 低（BQ 已 boot；高会短路 NTC） | 低 |
| GPIO PB3/PB4 | 低（LED 灭） | 状态灯效（NORM PB4 常亮） | PB3 灭；PB4 每次轻量 boot 打 `LP_HEARTBEAT_MS=10 ms` 高脉冲作为存活心跳，其余时间灭 |
| GPIO PA7 | AF1 占位 | AF1 PWM | 轻量 boot 不初始化；5 V 关 |
| BQ7692003PWR | SHIP；App boot 脉冲 + 配置 | NORMAL：`SYS_CTRL1=0x18`、`SYS_CTRL2=0x40|FET`、保护配置/回读、300 ms SETTLE、500 ms 轮询；严格部分异常有条件放电时继续 ADC/CC 读取、输出关后间隔至少 10 s 才可 ADC off 800 ms 重启 | LP 会话：`SYS_CTRL1=0x00`、`SYS_CTRL2=0x00`（ADC/TEMP/CC/双 FET 全关）；30 min 标记临时开 ADC/TEMP/CC 做两批实测，不改 FET，结束后关回 |
| 5 V 轨负载 | 关 | 开：TJA1044、RGB、蜂鸣器、U18 +5 V | 关（全部断电） |
| SWD 调试口 | 可用 | 可用 | 仅编译期 `POWER_LP_KEEP_SWD` 控制 STOP1 下 `DBG_STOP`（无运行期覆盖字段）；测 LP 电流应物理断开 SWD |
| Flash 黑匣子 | 头扫描（最大有效 seq） | RTC 30 min 标记：九个新真实发布帧的同源候选；60 s 未达标则带质量位降级记录（缺项写 0），与 StateStore 去重/同步 | **正常**轻量 boot 跨过 30 min 点写一条两批确认的实测 periodic；异常监测保留标记并完整唤醒，不在每次 IWDG tick 写 Flash |

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
- **主循环（每圈固定顺序）**：`PowerMode_Loop()`（消费调试器写的 LP 请求）→ 按键/手势 → BQ 读请求 + `BQ_Poll()`（事务状态机）→ `SocEst` → 自检轮询 → 1 s 分支：先 `App_Telemetry_Task()` 发 1 Hz D 帧，再进入 10 s 诊断（BQ 新鲜度 >2 s → `FAULT_BQ_STALE`；每 10 s 一条 `[DIAG]`/`[POL]` 日志）→ `Protection_Task(now, snap, comp, temp_valid, temp_dc)` → `StateMachine_Update` → `ChargePolicy_Task` → `Protection_Service` + 保护事件落盘 → `CalStream_Task` + 灯效 → 自动 LP 判断 → 黑匣子 30 min 记录 → `Buzzer_Poll()` → `LedStatus_Flush()` → `App_Log_Poll()` + `FlushSmall(4/32)`。
- **保护监督层（M2）**：`Protection_Task()` 在 `StateMachine_Update()` 之前评估 BQ OV/UV/SCD/OCD、boot/LP 锁存、软件 OCP、NTC 温度、数据有效性和 FET 允许掩码；`Protection_Service()` 在 BQ RUN/idle、FET 全关、`|I| <= 500 mA`、数据有效且连续 2 s 安全时，一次性清除本次 boot/LP 唤醒首次配置前已锁存的 SCD/OCD。运行期 BQ 锁存和软件 OCP 不自动清除。软件 OCP 门槛为充电 **5.5 A/1 s**、放电 **33 A/500 ms**，至少 2 个新发布帧确认后锁存；`Protection_RequestLatchClear()` 需人工调用，充电 OCP 还要求充电器移除。温度：包温度 **4.0–54.0 °C** 允许，充电和放电都收紧，2 °C 回差/2 s 去抖，恢复窗口 6.0–52.0 °C；`PROT_TEMP_ENABLE=1`，TS1 有效才放行。`ChargePolicy` 仍是唯一 FET 请求 owner，允许掩码变化时立即收紧（含部分掩码）。BQ 的 OCD/SCD 只覆盖放电，BQ 没有充电过流或自主温度保护；软件 OCP 的方向用 COMP2、幅值用 CC，不依赖 CC 极性；CC 符号仍由 `BQ_CC_DISCHARGE_SIGN` 单点定义供 SOC/诊断，上板必须先用已知充放电方向确认。原 SOC 5%/0% 截止未实现；2.8 V 硬欠压、3.1 V/SOC10 低电充电已接入（见下），均衡和 die 温仍是后续阶段。
- **BQ**：每约 500 ms 正常全量读取（同批顺序读、逐寄存器 CRC；不是 BQ 原子 ADC 帧）。`BQ_GetBatchQuality()` 返回 `BQ_BATCH_FULL_CANDIDATE`（合理但尚未真实发布）、`BQ_BATCH_FULL_REAL_OK`（完成双帧确认并真正发布）、`BQ_BATCH_PARTIAL_VC5_PACK`（连续两批独立确认仅 VC5 和/或 BAT 不可信，VC1/VC2、VC3/VC4、CC、控制/保护状态及通信可信）或 `BQ_BATCH_INVALID`（其余，包括无法判断 BAT 与 VC5 谁错的包压残差）。部分模式中的第三节/包压估算仅供显示和带质量标志的存档；不回写 `bq_snapshot_t.valid/fresh`，不清真实 OV/UV 锁存，也不作为放行/关断依据。`BQ_GetCcSample()` 使用同一 BQ 的新 CC 转换和 CRC 证据，与电压发布解耦，**不是冗余硬件电流通道**。真 OV/UV/SCD/OCD/XREADY/PVD、通信/保护读回故障始终优先。
- **异常分级时序**：冷/PIN/BOR 启动仅在已确认是电压异常时延长对应自检宽限；自检完成后蓝闪至少再保持 2 s，且须有真实完整好帧才能离开 BOOT 蓝灯。仅启动电压异常累计 60 s 未恢复才进入声光报警；外部 LP 真唤醒先记录部分异常，未到期不蜂鸣（PB3/HOT 仍闪）。本次完整唤醒只有曾取得真实好帧并通过自检/原保护、最后好帧未满 20 s、BQ ADC/CC 持续开且新 CC/同批证据可靠、COMP2 方向可信，才能在部分模式维持或重新请求有条件放电；两次不同的新 CC 转换达到 2 A 则锁存软件 OCP。CC/状态/保护证据过期或失效立即关，20 s 无新真实好帧到期关并报警。充电**不新增绕过原自检/fresh/保护门控的特例**；BQ 硬件无充电 OCP，也不能凭两节估算证明第三节安全。正常完整帧仍按现有 **5.5 A/1 s 充、33 A/500 ms 放**软件 OCP 门槛。输出开时 10 s 只是额外复判/限频日志间隔，500 ms 轮询继续，不能 ADC off→on；**仅输出关且严格部分异常**可通过 `BQ_RequestPartialRetry()` 按至少 10 s 冷却调用受控重启（ADC/CC/FET off 800 ms → CONFIG → ADC settle 300 ms → 双帧确认），真实保护、通信及 XREADY 故障不受此退避豁免。
- **归档/报警**：NORM 每 30 min 标记以同一窗口连续 9 个真实新发布帧形成同源候选（与该帧的 SOC/温度一致）；到点未达标可再等最多 60 s；先达标后坏帧不丢候选。未达标的前 1–3 个连续异常标记使用带质量标志的降级周期记录（缺失电芯和包压字段写 0，SOC 仅估算/无效），不单凭这些标记响蜂鸣。连续第 4 个异常标记或窗口末 VC1/VC2 不可信进入 2 min HOT/PB3＋蜂鸣器报警；2 min 内 9 个新真实好帧可撤销，否则经策略关双管并将禁止输出/自动 LP 锁存到 StateStore（关断确认需 BQ 回读，Flash 失败不能当已持久成功）。锁存只能经人工安全确认入口解除，不随 PIN/掉电自动消失；连续实际报警满 5 min 只置未来 STOP2/SHIP 预留 trigger，**不**进入 STOP2/SHIP。黑匣子沿用 24 B 记录和累计运行 uptime，RTC epoch 仅用于跨 NORM/LP 标记 ID 去重；估算电压不得作为实测入档。
- **状态机**：BOOT →（真实完整帧、自检等门控通过）→ NORM；当前只有 BOOT/NORM/FAULT。critical 掩码是 BQ_COMM/XREADY/SCD/OCD/ADC_STUCK/PROTECTION/POWER/CLK；**BQ OV/UV 不直接进 critical**——OV/SCD/OCD 及“UV+其它硬锁存”由 Protection 汇总为 `FAULT_BIT_PROTECTION`，UV-only 保持 NORMAL 且 allow=CHG。2.8 V/低电状态叠加也不进 FAULT UI。流动分类由 COMP2/策略决定，失去必要证据则保持/转回保守态。
- **灯效（2026-11-12）**：BOOT = PB4 1 Hz 慢闪 + RGB 蓝 64（启动电压异常须经真实好帧且自检完成后再等至少 2 s）；真实关键 FAULT = PB3 5 Hz + RGB 红 128；低电状态 = `LED_FLOW_LOW_BATT` 红色 64/5 Hz（NORM，不进入 FAULT）；CAL = RGB 琥珀 (48,24)；LP STOP1 中 PB3/蜂鸣/RGB 均关闭。
- **NORM RGB SOC 渐变**：`led_status.c` 按 SOC 0% 红 → 50% 黄 → 100% 绿，满量程仍为 48。STANDBY 5 Hz 快闪、亮度约 60%（最大 29）；DISCHARGE 常亮、95%（最大 46）；CHARGE 1 Hz 慢闪、95%（最大 46）；`charge_full` 时白色 (38,38,38) 常亮（约 80%），不再随 SOC 渐变。SOC 显示门限是 `frame_valid && quality==REAL`，不要求 `cal_valid`；SOC 无效时保持最后一次有效颜色，从未有效则回退 STANDBY 白 12 / DISCHARGE 蓝 48 / CHARGE 绿 48。常亮模式只在颜色变化时发帧，相位切换不重复发送。PB3/PB4 仍按原模式驱动。
- **蜂鸣器**：音型表（BOOT_OK/FAULT/LP_ENTER/WAKE），主循环推进相位，TIM1 无中断。
- **日志**：USART1 115200 非阻塞，2 KB 环形 + 64 B 暂存，TX DMA1_Ch4，100 ms DMA 看门狗；1 Hz `D,...` SOC 遥测与 `[DIAG]`/`[CAL]` 共用此通路（见 `DOCS/SOC_TELEMETRY.md`）。
- **PVD**：3.3 V 轨 < 2.8 V → 中断 → sticky `FAULT_POWER`。
- **黑匣子**：NORM 的 RTC 30 min 标记依上述九帧/60 s 策略归档 24 B 周期记录并同步 StateStore：FULL 使用第九个认证帧及同源 SOC/温度，PARTIAL 将不可信电芯/包压写 0 并标质量；LP 正常标记保持两批独立实测确认、只写一条黑匣子周期记录，不要求九帧或在轻量 boot 运行 SOC/StateStore 存档，但须同步清持久四连计数。页轮转时可能触发整页擦除；不得将 RTC epoch 直接写入黑匣子既有 uptime 字段。
- **LP 请求**：空闲约 5 s 后才申请自动 LP；短暂严格部分电压异常在**无输出、无待归档/实际报警/锁存/真实故障**时允许单独放宽不 fresh/非 NORM/部分保护门控，其它 block（时基、自检、充电、策略、按键、电源等）必须仍成立为 0。START token 软件复位后的轻量 LP 会关闭 ADC/CC/FET，睡眠期不保证 10 s 复判、PB3 闪烁或维持任何输出；复位会清本次真实好帧放电资格，四连归档计数及锁存独立持久。`s_app.auto_lp_block` 仍逐位诊断；LSE 模式下时基未就绪不得进入 LP。

### 3.1 L1 充放电策略与 CAL

- `ChargePolicy_Task()` 在 `StateMachine_Update()` 之后运行，状态优先级：`FAULT > CAL > CHARGE(充电器) > DISCHARGE(手势) > IDLE`。
- `comp_charging`（COMP2 消抖）上升立即置充电器存在；CHARGE 中充电电流 >30 mA 也可保持存在（COMP2 在 MOS 导通后差模太小）；COMP2 低且无电流支撑时连续 **5200 ms** 才撤销。平衡充会故意短暂关断输出，电流先降到 0，1 s 去抖会误判为充电器移除并造成 CHARGE→IDLE→CHARGE 反复，因此退出充电判定放宽到 5.2 s；代价是真实拔充电器后最多 5.2 s 才回 IDLE，期间策略仍为 CHARGE、双管保持请求全开（Protection/数据/温度/故障仍即时）。近满（显示 SOC>90 且 −20 mA < 充电电流 < 220 mA）只请求 CHG，利用 DSG 体二极管 + COMP2 判充电器；电流 >320 mA 或 SOC<85% 恢复双管。充电器插入沿用既有 BQ fresh、自检和保护门控，部分异常**不保证持续充电**；移除后回 IDLE，双管关。
- `GESTURE_PAIR_HOLD3` 切换 DISCHARGE/IDLE；严格部分异常时若本次唤醒已取得真实好帧、20 s 和新 CC/保护/方向门控均成立，异常期已关输出亦可重新按键请求**有条件**放电；其它情况下不得因按键绕过关断或持久锁存。充电/真实 FAULT/CAL/充电器存在时仍拒绝。
- DISCHARGE 进入后的**前 10 s**内，若同一 signed 充电电流（净电池电流）≥100 mA，则置外部充电闩锁并立即转 CHARGE；≤90 mA 释放（10 mA 回差）。该判据不替代 COMP2：10 s 后 COMP2 仍可按原逻辑切 CHARGE。切换后清 `user_discharge_enabled`，按正常 CHARGE 电流保持/近满体二极管/移除逻辑运行。净电流会被放电负载抵消，BQ 500 ms 采样带来最多约一个读周期延迟。
- 关键故障强制请求 0、进入策略 FAULT；CAL 自动结束，不自动恢复。
- `CHARGE/DISCHARGE` 常规模式请求 `BQ_FET_MASK_ALL`，最终仍需与 `Protection_GetAllowedFetMask()` 和异常锁存门控相与；有条件部分放电**不代表双管无条件全开**。输出关断优先即时申请、RUN/idle 时核实实际 BQ FET 状态，不把请求影子当作已确认关断。
- **低压分级（2026-11-12）**：有效 BQ 帧参考节 `<2.8 V` 时 Protection 立即把 allow 裁成 CHG-only；`>=3.0 V` 连续 2 帧恢复。参考节 `<3.1 V` 或 SOC `<10%` 连续 2 帧进入 `CHARGE_POLICY_LOW_BATTERY_CHARGE`，默认 CHG-only 且不依赖充电器；低电状态内 DSG 只在充电电流 >200 mA/2 帧且参考节 `>=2.8 V` 时打开，<120 mA 或电流无效立即关闭；3.1 V+SOC≥10%/2 帧退出。无充电器/电流时允许自动 LP，按键唤醒后红灯 5 Hz；旧 `STATE_FLAG_CUTOFF_LATCH` 不恢复。正常参考节为三节最低值；`log_cell3_valid==0 || log_pack_valid==0` 时保护/SOC 只使用实测 VC2。`PAIR_HOLD3` 在低电/欠压期间拒绝。
- CAL 由调试器写 `s_policy.cal_fet_mask`（0x00..0x03）再写 `s_policy.cal_mode=1` 进入；写 `cal_mode=0` 退出。CAL 期间自动 LP 被 bit7 阻止，BQ 不 fresh/非 RUN/LP 唤醒/关键故障都会结束 CAL。
- `[CAL]` 由 `cal_stream.c` 输出：BEGIN/HDR + 每个新 BQ 快照一行（约 2 Hz）+ END；黑匣子只写 `BB_EVENT_CAL_BEGIN/END`（0x12/0x13），不逐样本写 Flash。CAL 日志只用现有快照字段（mV、cc_raw、current_ma、ts1_raw、SocEst 的 ts1_mv/temp/SOC/SOH），不恢复 raw_cell/raw_pack。END 的 `reason`：1=user，2=fault，3=notready，4=lp。

## 4. NORM → LP：写 START 令牌后复位

自动 LP 只在 `App_Loop()` 的固定门控全部满足、且 5 s 无活动时调用 `PowerMode_RequestLp()`。阻止位图 `s_app.auto_lp_block` 的位定义：

| 位 | 宏 | 条件 |
| --- | --- | --- |
| 0 | `APP_LP_BLOCK_CHARGING` | COMP2 连续判定充电中 |
| 1 | `APP_LP_BLOCK_BUTTON` | PB5 当前按下 |
| 2 | `APP_LP_BLOCK_TEST` | 自检未完成或失败 |
| 3 | `APP_LP_BLOCK_STATE` | 状态机不是 NORM；仅无输出/待归档/报警/锁存/真实故障的严格部分电压异常可单独例外 |
| 4 | `APP_LP_BLOCK_BQ` | BQ 不 fresh、ADC 恢复中/失败；部分异常仅在无输出等完整例外条件成立时可放宽，不可豁免通信/XREADY/真实保护 |
| 5 | `APP_LP_BLOCK_TIMEBASE` | `PowerMode_LpTimebaseReady()==0`（RTC 源/就绪不匹配） |
| 6 | `APP_LP_BLOCK_POWER` | `PowerMode_GetState()!=POWER_STATE_NORM` |
| 7 | `APP_LP_BLOCK_POLICY` | CHARGE/DISCHARGE/CAL 或 CAL pending |
| 8 | `APP_LP_BLOCK_PROTECTION` | `Protection_IsLpBlocked()`：真实保护/温度/锁存仍阻止 LP；严格部分电压异常且无输出、待归档/报警/锁存时才可定向例外 |
| 9 | `APP_LP_BLOCK_FLASH_MARK` | 有待写 LP mark 或 30 min 归档 |

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
8. BQ sniff：`BQ_LpSniffInit()` + `BQ_LpBootPrepare()`，失败重试一次；仍失败 → 完整唤醒（FAULT）；prepare 必须在 300 ms 等待后重新读 SYS_STAT，再清 `CC_READY/XREADY/OVRD_ALERT`；
9. 若 `LOAD_PRESENT=1 && CHG_ON=0`，做充电器电平检查：PA1 短暂拉低 → `HAL_COMP_Start` → 30 ms → 读 COMP2 → Stop → PA1 高；高 = 充电器 → `LP_WAKE_CHARGER`；
10. 只有 post-clear SYS_STAT 的 OV/UV/SCD/OCD/XREADY 才 → `LP_WAKE_ALERT`；PA8 仅因 `CC_READY/OVRD_ALERT` 为高不再唤醒；
11. PB4 打 `LP_HEARTBEAT_MS=10 ms` 高脉冲（存活心跳；改 0 可省电）；
12. 在 30 min 监测之前测 boot overhead；
13. 若 `next_mark_epoch==0` 则设 `now+1800 s`；剩余时间 ≤`LP_MARK_TOLERANCE_MS=1000 ms` 或已过点 → `LpCycle_DoMark()`；
14. 重读 RTC，计算 `target_ms`：若剩余 ≤30 s + overhead，则睡到 mark 前（`remaining - overhead`）；否则基础 30 s；再减 overhead，最小 `LP_MIN_SLEEP_MS=1 s`；
15. `LpCycle_EnterStop(target_ms)`。

### 5.3 LpCycle_EnterStop()

- `Watchdog_SetTimeoutMs(target_ms, lsi_hz, NULL)` 从校准后的 LSI 频率选最小分频并写 IWDG PR/RLR；
- 读 RTC；最后检查 PB5，以及 PA8 高时再读 SYS_STAT 确认；只有真实保护位才完整唤醒，`CC_READY/XREADY/OVRD_ALERT` 清掉后继续进 STOP；
- 保存 `last_pr`、`last_rlr`、`last_arm_epoch/ms`，状态置 `STOPPED` 并写 BKP；
- 保持 PWR/RTCAPB 的 APB 与 Sleep 时钟；GPIOA/GPIOB/SYSCFG Sleep 时钟保持（退出后要读 PB5/PA8）；SMBA 测试时额外保持 I2C1；
- `HAL_SuspendTick()`、清 PendST；`RCC_CFGR.STOPWUCK=1`（退出到 HSI16）、`PWR_CR1.LPMS=STOP1`、`SCB.SCR.SLEEPDEEP=1`、`__DSB()`、`__WFI()`。

正常路径是 IWDG 超时**复位**：CPU 不执行 WFI 之后的代码，直接进入下一次 `App_Boot()`。若 PB5/ALERT 先把 CPU 唤醒，则 WFI 之后恢复 `SystemCoreClock=16 MHz`/`HAL_InitTick()`、喂狗，再检查 PB5；PA8 高时读 SYS_STAT 并清非保护位：确认按键 → `LP_WAKE_BUTTON`，确认真实保护位 → `LP_WAKE_ALERT`；其余未知唤醒 → `LP_WAKE_FAULT`。所有真实唤醒都是 `LpCycle_ForceFullWake()`：写 `PAUSED + FULL_WAKE` 令牌，软件复位到完整 `App_Init()`。

## 6. LP → NORM：唤醒注入

完整 boot 的 `App_Init()` 末尾调用 `LpCycle_ApplyWake()`：

- 读会话，无 FULL_WAKE 标志则直接返回；
- `s_lp.wake_cause` 映射为 `POWER_WAKE_BUTTON/CHARGER/BQ_ALERT/MONITOR`，未知则回退 `POWER_WAKE_RTC`；
- `PowerMode_NoteLpFullWake(cause)` 只更新 `s_power.wake_cause` 和 `wake_count`；
- `Button_OnWake()`、`ChargePolicy_OnLpWake()`：按键唤醒回 IDLE，放电仍需 LP 短按后 2 s 内再 `PAIR_HOLD3`；充电唤醒由策略层等 BQ fresh 后进 CHARGE；
- FAULT 唤醒强制 `PowerMode_ForceDisableAutoLp()`；
- 非 MONITOR 唤醒写一条黑匣子 wake 记录（wake mask）；
- 清 FULL_WAKE、状态置 PAUSED、`seq++` 保存。

PB5 的 EXTI5 回调只置标志；`LpCycle_EnterStop()` 退出 WFI 后读取标志/引脚并决定唤醒原因。PA8 是 EXTI **事件**模式，不能唤醒 STOP1，只在每次轻量 boot 和进入 STOP 前作为触发；触发后读 SYS_STAT，只有 OV/UV/SCD/OCD/XREADY 才真正唤醒，`CC_READY/XREADY/OVRD_ALERT` 清掉继续睡，最坏延迟一个 IWDG 周期。

## 7. 30 min 监测与 LSI 校准

### 7.1 30 min 点

- `s_lp.next_mark_epoch` 跨复位保存在 BKP 会话中；新会话从 `now + 1800 s` 开始；
- 每次轻量 boot 若剩余 ≤1 s 或已过点，调用 `LpCycle_DoMark()`：
  1. `BQ_LpSniffInit()`；
  2. `BQ_LpMonitorRun()`：临时开 ADC/TEMP+CC，读取三节/包压/VC3/VC4/状态/电流/温度，做两批独立实测确认，不改 FET；
  3. `BQ_LpSniffDeInit()`，喂狗；
  4. `BlackBox_Init()` 扫描头指针；
  5. 正常监测 `BlackBox_WritePeriodicSnapshot(&snap, uptime_s)` 写 24 B 实测周期记录（`uptime_s` 为累计运行秒，**不是** RTC `now.sec`）；同步持久化该标记为 FULL 并清四连异常计数，不做 LP SOC/StateStore 电池快照；
  6. 成功确认标记后将 `next_mark_epoch += 1800` 推进到当前 epoch 之后；若失败仍保留原标记，交由完整唤醒归档。
- 任一步失败：若 `BQ_LpMonitorRun()` 返回 `-2`（CRC/控制/保护及 VC3/VC4 可信，仅电压合理性异常），保留原 RTC 标记与档案上下文，以 `LP_WAKE_MONITOR` **不附加 `LP_WAKE_FAULT`** 完整唤醒；进入 NORM 后按连续 9 个新真实发布帧及最多 60 s 归档策略补办，不把这次数据异常永久禁用 auto-LP。真实通信/CRC/保护故障或黑匣子失败仍走 FAULT/full wake 安全路径；标记不能静默跳过，也不能重复计数。
- 正常 LP 标记仅要求两批独立实测确认（包括 VC3/VC4 诊断），写一条黑匣子记录，不运行 NORM 的九帧/SOC/SOH/StateStore 电池快照；真实成功标记可清已持久的四连异常计数。黑匣子 `uptime_s` 仍是累计运行秒，不等于 `now.sec` RTC epoch；RTC 标记仅作去重/时限。

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

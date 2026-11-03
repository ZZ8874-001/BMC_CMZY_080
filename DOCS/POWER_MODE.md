# BMC_CMZY_080 电源模式与切换说明（L0）

依据（2026-11-02 版本，`make -j2 all` 通过，`_etext`=0x08009BA8）：

- `App/app/power_mode.c`（时钟切换、LP 进入/唤醒、RTC/PB5 中断 owner）
- `App/app/main_app.c`（App_Init / App_Loop / 250 Hz tick 编排）
- `App/bsp/bq_ic.c`、`App/bsp/ws2812_uart.c`、`App/bsp/buzzer.c`、`App/bsp/flash_store.c`
- `App/comm/app_log.c`、`App/app/faults.c`、`App/app/black_box.c`、`App/app/state_machine.c`、
  `App/app/led_status.c`、`App/app/self_test.c`、`App/app/reset_cause.c`
- `Core/Src/main.c`（生成初始化）、`BMC_CMZY_080.ioc`

## 1. 外设状态总表

| 外设 / 板级器件 | 上电（生成代码跑完） | NORM 常态 | LP 保持态 |
| --- | --- | --- | --- |
| SYSCLK | HSI16，Scale2，2WS | HSE bypass 12 MHz → PLL ×80 MHz，Scale1，4WS | 停（STOP1）；唤醒瞬间 MSI 4 MHz |
| HSI16 / MSI | HSI16 开，MSI 关 | HSI16 开，MSI 关 | 进入前开 MSI 并设 STOPWUCK=MSI；STOP 中两者皆停，唤醒先供 MSI |
| HSE / PLL / OSC1 | 关；`MX_GPIO_Init` 已把 PA1 拉低 | 开；PA1=0（OSC1 使能 + PACK_SC 前端使能） | 关：先切 MSI、关 PLL、关 HSE，再抬 PA1（OSC1 tri-state） |
| LSE / RTC | LSE 开、RTC 时基就绪（异步127/同步255 → 1 Hz） | 运行；WUT 未布防，RTC_WKUP NVIC 关 | **唯一运行外设**：WUT 20 s 倒计时（保底自动唤醒） |
| SysTick | 1 ms HAL 时基 | 1 ms | `HAL_SuspendTick()`；唤醒后在 MSI 下 `HAL_InitTick` 修复 |
| TIM2 | 占位配置（PSC0/ARRmax/OC CH1），NVIC 已使能 prio0，**未启动** | 覆写 PSC319/ARR999 → **250 Hz**，prio2，已启动 | 停止、NVIC 关 |
| TIM1 | 初始化未启动 | 蜂鸣器音型按需 `HAL_TIMEx_PWMN_Start` + MOE | 停止；PA7 改 GPIO 推挽输出并拉低（Q8 可靠关断） |
| I2C1 | 初始化（生成 ~103.6 kHz）；`hdma_i2c1_tx`/`hdma_i2c1_rx` 已 Init | 95 kHz：`Init.Timing` 在 `HAL_I2C_Init` **之前**写 0x1424A4FF（硬件读回 0x1024A4FF 是 HAL 清保留位的正常结果）；TX DMA1_Ch6、RX **DMA1_Ch7**；每 500 ms 跑 8 个事务 | DeInit；SDA/SCL 空闲高；BQ 保持 LP 寄存器态 |
| USART1 | 初始化 115200（生成值） | 覆写 BRR=694（115200）；日志 TX DMA1_Ch4，1 KB 环形缓冲 | DeInit；PB6 空闲高；U18 断电（5 V 关） |
| USART2 | 初始化（半双工/开漏/115200，生成值） | deinit/reinit：USART2SEL=PCLK1（80 MHz）、`BaudRate=3809523` 先于 `HAL_UART_Init`；再 UE=0 写 BRR=21（**3.8095 MBd**）+ TXINV + 清 HDSEL + PA2 推挽；RGB 帧用 **USART2 中断发送**（prio1，不使用 DMA） | DeInit；PA2 空闲低；U20 断电 |
| COMP2 | 初始化未启动 | 启动；250 Hz 采样 + 32 样本（128 ms）消抖 → 充/放方向 | 停止；PA3/PA4 被 330 kΩ 拉至 ~3.3 V（PA1 已高） |
| DAC1 | 初始化（内部模式、未启动） | 不参与（永远不启动、不接 COMP2） | 停 |
| CAN1 | 初始化（无 NVIC、无业务） | 初始化态；TJA1044 由 5 V 供电但不收发 | TJA1044 断电（5 V 关） |
| PVD | 未开 | 开：阈值 2.8 V、prio4，跌落 → sticky `FAULT_POWER` | 关（省电） |
| EXTI PA8（BQ ALERT） | 事件模式、上升沿（无中断、无挂起位） | 不产生中断；由 500 ms 轮询 SYS_STAT 消化 | **唤醒源**（事件唤醒；无标志 → 归因 BQ ALERT） |
| EXTI PB5（WAKE 按键） | 中断、下降沿、prio0 | 只置 `flag_button`（主循环消费） | **唤醒源**（按下即唤醒） |
| GPIO PA1 | 低（OSC1 + PACK_SC 使能） | 低 | 高（OSC1 tri-state、PACK_SC 前端关） |
| GPIO PB0 | 低（5 V 关） | 高（5 V 开，软启动 ≥20 ms） | 低（5 V 关） |
| GPIO PB1 | 低；App 做 2 ms 高脉冲再回低 | 低（BQ 已 boot；高会短路 NTC） | 低 |
| GPIO PB3/PB4 | 低（LED 灭） | 状态机灯效（NORM 常亮 PB4 等） | 低（灭） |
| GPIO PA7 | AF1 占位 | AF1 PWM（2.0/2.5/3.0/4.0/5.0 kHz 音型） | GPIO 输出低 |
| BQ7692003PWR | SHIP（POR 默认）；App boot 脉冲 + 配置 | NORMAL：SYS_CTRL1=0x18（ADC_EN\|TEMP_SEL）、SYS_CTRL2=0x43（CC_EN+DSG/CHG ON）；每 500 ms 读 SYS_STAT/VC1-3/BAT/TS1/CC | NORMAL，SYS_CTRL1=0x00（**ADC/TEMP 关**）、SYS_CTRL2=0x03（CC 关、DSG/CHG 保持）；无电压/电流/温度检测，OV/UV 停、SCD/OCD 仍有效；ALERT 可唤醒 |
| 5 V 轨负载 | 关 | 开：TJA1044、U20 RGB、蜂鸣器、U18 +5 V | 关（全部断电） |
| SWD 调试口 | 可用 | 可用 | **不可访问**（DBG_STOP=0，无时钟）；SRAM 内容保持，唤醒后重连可看 |
| Flash 黑匣子 | 头扫描（找最大有效 seq） | 30 min/条 + LP 进入/唤醒转换记录 | 无写入（Flash 不擦不写） |

## 2. 上电 → NORM 流程

### 2.1 硬件复位与生成代码（main.c）

1. 复位向量 → `HAL_Init()`（Flash 接口、SysTick 1 ms）。
2. `SystemClock_Config()`：**HSI16** + LSE ON，PLL 关，Scale2，2WS。复位后不依赖 OSC1——PA1 复位期间高阻被 R16 4k7 拉高（OSC1 tri-state），MCU 照样在 HSI16 上跑。
3. `MX_GPIO_Init()`：PA1/PB0/PB1/PB3/PB4 输出低；PA8 = EXTI 事件上升沿；PB5 = EXTI 中断下降沿；EXTI9_5 NVIC 使能（prio 0）。
4. `MX_DMA_Init()` → `MX_CAN1_Init()`、`MX_COMP2_Init()`、`MX_DAC1_Init()`、`MX_I2C1_Init()`、`MX_USART1_UART_Init()`、`MX_USART2_UART_Init()`、`MX_TIM1_Init()`、`MX_RTC_Init()`（LSE 已开，RTC 就绪）、`MX_TIM2_Init()`（占位 + NVIC prio0 使能）。全部外设**只初始化不启动**；NVIC 此时已使能的有：EXTI9_5、TIM2、DMA1_Ch6、DMA1_Ch7。

### 2.2 App_Init()（App/app/main_app.c，顺序即依赖序，无时间预算）

| 步骤 | 动作 | 说明 |
| --- | --- | --- |
| 1 | 清零 `s_app` | 全部运行状态置初值 |
| 2 | `ResetCause_Init()` | RCC->CSR + BKP0R 魔数（hardfault 记录），消费后清 RMVF |
| 3 | `Fault_Init()` → `Fault_PvdEnable()` | 故障图清零；PVD 2.8 V 布防（prio4） |
| 4 | `PowerMode_Init()` | LSE/RTC 状态核对（失败 → sticky `FAULT_LSE`）；PB5 下降沿幂等覆写；**时钟切换**（见下） |
| 5 | `BlackBox_Init()` | 扫描 1020 槽取最大有效 seq；`_etext ≥ 0x0801A000` → sticky `FAULT_FLASH_LOG`；恢复 `uptime_base` |
| 6 | 降级检查 | 时钟切换失败 → PB0 保持低、**直接 return**：只跑降级循环（PB3 以 250 ms 相位翻转 ≈2 Hz 闪烁），通信/BQ/tick 全部不启动 |
| 7 | PB0 = 1 + `HAL_Delay(20)` | 5 V 软启动余量（CAN 收发器/RGB/蜂鸣器/U18 +5 V） |
| 8 | `App_Log_Init()` + `[BOOT]` 日志 | USART1 覆写 115200、DMA1_Ch4、NVIC prio4；打印复位原因 |
| 9 | `BQ_Init()` | I2C1 95 kHz（`Init.Timing` 在 `HAL_I2C_Init` 前写入）+ DMA 重链（TX DMA1_Ch6 / RX DMA1_Ch7）+ NVIC prio3；**PB1 高脉冲 ≥2 ms**（强制退出 SHIP）→ 等 20 ms → 配置脚本（CC_CFG=0x19 写/读回、ADCGAIN1/2/ADCOFFSET 读、SYS_CTRL1=0x18、清 SYS_STAT、SYS_CTRL2=0x43） |
| 10 | `Ws2812_Init()` | USART2 时钟/波特率重配（USART2SEL=PCLK1、BaudRate=3809523、UE=0 写 BRR=21=3.8095 MBd）+ TXINV/推挽/清半双工；RGB 帧中断发送，USART2 IRQ prio1，不使用 DMA |
| 11 | `LedStatus_Init()` | BOOT 灯效；PB3/PB4 先灭 |
| 12 | `Buzzer_Init()` | PA7 配 AF1，PWMN 停止 + MOE 关（静默布防） |
| 13 | `SelfTest_Init()` + `SelfTest_Request()` | 自检启动（见 §3） |
| 14 | `StateMachine_Init()` | BOOT 状态、STANDBY 流动 |
| 15 | `HAL_COMP_Start(&hcomp2)` | 比较器启动（PA3/PA4 有效） |
| 16 | `App_TickTimerInit()` + `App_TickStart()` | ★ 最后：TIM2 覆写 PSC319/ARR999、清 UIF、NVIC prio2，启动 250 Hz 中断 |

**时钟切换 `PowerMode_ClkSwitchToHsePll()`**（唯一 HSE/PLL 入口，唤醒路径复用；App 代码禁止调 `SystemClock_Config()`）：

PA1=0（OSC1/PACK_SC 使能）→ `HAL_Delay(30)` 起振余量 → HSE **bypass** 12 MHz ON → PLL ON（M3/N40/R2）→ Scale1 → SYSCLK=PLL、AHB/APB1/APB2=DIV1、4WS → 80 MHz。整个序列重试 3 次；全失败 → sticky `FAULT_CLK` + 降级（步骤 6）。

### 2.3 上电自检（SelfTest）

非阻塞：IDLE → WAIT_BQ（超时 1500 ms）→ DONE。检查项：LSERDY、BQ 状态 == RUN、BQ 数据新鲜、COMP2 == BUSY、黑匣子无错误。完成后主循环触发：通过 → 蜂鸣器 `BOOT_OK`（4 kHz × 2 短）+ `[TEST] pass`；失败 → `FAULT` 音型（2 kHz × 3 长）+ `[TEST] fail=0xXXXX`，灯切 FAULT 模式。首个有效电芯数据约在 boot 脉冲后 250 ms 出现。

## 3. NORM 常态运行

- **时钟**：HSE bypass 12 MHz → PLL → 80 MHz，Scale1，4WS；LSE 32.768 kHz 供 RTC 1 Hz 时基；HSI16 保持开启。
- **250 Hz TIM2 中断（ISR 内只读寄存器/置标志/数数）**：COMP2 采样 + 32 样本消抖提交充/放方向；LED 相位推进（PB3/PB4 + RGB dirty 标志）；500 ms → `flag_bq_read_request`；1 s → `uptime_s++` + `flag_diag_request`；30 min → `flag_blackbox_request`。
- **主循环（每圈固定顺序）**：`PowerMode_Loop()`（消费调试器写的 LP 请求）→ BQ 读请求 + `BQ_Poll()`（事务状态机）→ 自检轮询 → 1 s 诊断（BQ 新鲜度 >2 s → `FAULT_BQ_STALE`；每 10 s 一条 `[DIAG]` 日志）→ `StateMachine_Update` + 灯效 → 黑匣子 30 min 记录 → `Buzzer_Poll()` → `LedStatus_Flush()`（RGB 帧，USART2 在飞则本圈跳过）→ `App_Log_Poll()` + `FlushSmall(4)`。
- **BQ**：每 500 ms 一轮 8 个事务（读 SYS_STAT → 清 CC_READY/XREADY → 读 VC1/VC2/VC3/BAT/TS1/CC，每字节 CRC-8）；通信故障 3 次重试 → sticky `FAULT_BQ_COMM` + 总线重建 + 重跑配置序列；事务 100 ms 看门狗。SYS_STAT 保护位（OV/UV/SCD/OCD/XREADY）→ live 故障位 → 状态机 FAULT。CC 换算：`s_bq.current_ma = cc_raw × 844 / 150`（1 LSB=8.44 µV、分流 1.5 mΩ、250 ms 平均，带符号）。MOS 开关标志：`s_bq.request_fet_mask`（bit0=DSG，bit1=CHG，默认 0x03）、`fet_mask_applied`；run script 完成后若不同，用一次性 DMA 写 `SYS_CTRL2=0x40|mask`（保留 CC_EN）。配置/LP 仍固定写 `0x43`/`0x03`（两颗 MOS 全开）。
- **状态机**：BOOT →（数据新鲜）→ NORM；关键故障（BQ_COMM/XREADY/OV/UV/SCD/OCD/POWER/CLK）→ FAULT（故障消失回 BOOT 重判）。流动分类：充电（COMP2 高）/放电（低）/静置（数据不新鲜）。
- **灯效**：BOOT = PB4 1 Hz 慢闪 + RGB 蓝；NORM = PB4 常亮 + RGB（充电绿 48 / 放电蓝 48 / 静置白 12）；FAULT = PB3 5 Hz 快闪 + RGB 红。
- **蜂鸣器**：音型表（BOOT_OK/FAULT/LP_ENTER/WAKE），主循环推进相位，TIM1 无中断。
- **日志**：USART1 115200 非阻塞，1 KB 环形 + 64 B 暂存，TX DMA1_Ch4，100 ms DMA 看门狗。
- **PVD**：3.3 V 轨 < 2.8 V → 中断 → sticky `FAULT_POWER`。
- **黑匣子**：每 30 min 一条 24 B 记录（seq/uptime/3 节电压/总压/CC/TS1/种类/CRC16）；跨页首槽先擦除整页（~25 ms 取指停摆，约每 42.5 h 一次）。
- **LP 请求**：调试器写 `s_power.request_lp = 1`（`volatile`，主循环消费）；NORM 永续、无超时。

## 4. NORM → LP 切换序列（PowerMode_EnterLp）

前置：`s_power.state == NORM`。任何一步失败都**留在 NORM**（或降级），不会出现“无唤醒源”的 STOP。

| 步骤 | 动作 | 说明 / 时序 |
| --- | --- | --- |
| 1 | 排空在飞通信 | 等 `!Ws2812_IsBusy() && !App_Log_TxBusy() && BQ_IsIdle()`，最多 100 ms；超时也继续进入 LP（在飞事务由各外设的 LP 停机路径中止） |
| 2 | BQ 准备（有界阻塞，每步 50 ms） | 写 SYS_CTRL1=0x00（**关 ADC/TEMP**，LP 期间无电压/电流/温度检测，BQ 60→40 µA）→ 写 SYS_CTRL2=0x03（**CC_EN=0**、DSG/CHG 保持）→ 读 SYS_STAT → 写 0xFF 清状态（含 CC_READY，防 ALERT 唤醒风暴）。失败 → sticky `FAULT_BQ_COMM`，仍继续进 LP（唤醒后自愈） |
| 3 | 黑匣子 LP 记录 | `BlackBox_WriteTransition(BB_KIND_LP_ENTER)`（此时仍在 NORM，Flash 可写） |
| 4 | **布防 RTC WUT 20 s** | prio1 + 清挂起 + NVIC 使能 → `HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20, ..._CK_SPRE_16BITS)`。**失败 → 放弃进 LP**：清 request_lp、回 NORM、sticky `FAULT_LSE`、撤 WUT、关 NVIC |
| 5 | 停外设、灭灯 | `App_TickStop` → `App_Log_DeInitForLp` → `Ws2812_DeInitForLp` → `BQ_DeInitForLp` → `HAL_COMP_Stop` → `Buzzer_DeInitForLp`（PA7 拉低）→ `Fault_PvdDisable` → PB3/PB4 灭 |
| 6 | 清挂起唤醒源 | 清 WUTF、EXTI PR1（PIF5/PIF8）、EXTI9_5 挂起——保证唤醒后判定只可能是 RTC/PB5/PA8 |
| 7 | MSI 就绪 | MSI 开 + 等 MSIRDY；STOPWUCK=MSI（确定唤醒后时钟） |
| 8 | SYSCLK → MSI 4 MHz | 等 SWS==MSI；`SystemCoreClock=4 MHz`；降 Scale2（仍 4WS，合法） |
| 9 | 关 PLL、关 HSE | 等 PLLRDY=0、HSERDY=0（HSEBYP 仅在 HSEON=0 时可写，唤醒后重配 HSE 的前置条件） |
| 10 | 抬 PA1、关 PB0 | OSC1 tri-state + PACK_SC 前端关；5 V 轨断电 |
| 11 | 进入 STOP1 | `state=LP`、`lp_enter_count++`、`HAL_SuspendTick()`、`HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI)` |

## 5. LP 保持态

- **运行中**：只有 LSE 32.768 kHz + RTC（WUT 20 s 倒计时）；PA8 EXTI 事件与 PB5 EXTI 中断作为唤醒源；SRAM 全部内容保持（`s_power`/`s_app`/`s_bq` 等可调试）。SYSCLK 停。
- **引脚**：PA1=1（OSC1 tri-state、PACK_SC 前端关——SC 节点被 330 kΩ 拉至 ~3.3 V）；PB0=0（5 V 关，TJA1044/RGB/蜂鸣器/U18 全部断电）；PB1=0；PB3/PB4=0（灭）；PA7=0（Q8 关断）；PA2 空闲低、PB6 空闲高、SDA/SCL 空闲高（外设已 DeInit，引脚保持 AF 配置）；PA3/PA4 约 3.3 V（COMP 停，无意义）。
- **BQ76920**：仍由电池供电，处于 NORMAL：SYS_CTRL1=0x00（**ADC 与 TS1/NTC 偏置关闭**）、CC 关、DSG/CHG 保持导通（两只 MOS 导通、分流电阻通路保持）。**电压/电流/温度检测仅在 NORM 运行**；LP 期间 OV/UV 保护随 ADC 停用，但 SCD/OCD 比较器不可关、仍然有效——短路/过流仍会拉高 ALERT → 唤醒 MCU（唤醒后读 SYS_STAT 消化，配置序列恢复 SYS_CTRL1=0x18）。
- **SWD**：`DBG_STOP=0` → 调试口无时钟，**调试器不可访问内存**；唤醒（≤20 s）后重连即可看全部状态。
- **电流预算（待上板实测）**：MCU STOP1 + LSE/RTC ~4.4–5.3 µA（典型，DS11451）+ BQ NORMAL ADC/CC 均关 **40 µA 典型 / 60 µA 最大**（SLUSBK2I）+ 板级静态（PACK_SC 分压、上拉等）。相比改动前（ADC 开 = 60 µA 典型），LP 电流约降低 20 µA，且不再随 NTC 温度变化。
- **uptime 语义**：250 Hz tick 已停 → LP 时长不计入 `uptime_s`（黑匣子文档已注明）。

## 6. LP → NORM 唤醒序列（PowerMode_EnterLp 的后半段）

**唤醒原因判定**（进入前已清全部挂起）：

| 原因 | 判据 | 计数 |
| --- | --- | --- |
| RTC 20 s 超时（保底） | `flag_rtc`（WUT IRQ 置位并关 WUTIE） | `wake_rtc_count` |
| PB5 按键按下 | `flag_button`（EXTI5 下降沿） | `wake_button_count` |
| BQ ALERT（PA8 事件） | 前两者都没有（事件模式无标志），或 PR1.PIF8 挂起 | `wake_alert_count` |

**恢复步骤**（`state=WAKING`）：

1. 读标志/挂起 → 组成 `wake_cause` → 清 PR1、清标志、`wake_count++`。
2. `SystemCoreClock=4 MHz` + `HAL_InitTick(TICK_INT_PRIORITY)`——先把 SysTick 修成 1 ms（此时在 MSI 4 MHz 上跑）。
3. 撤 RTC WUT：`HAL_RTCEx_DeactivateWakeUpTimer` + 清 WUTF + 关 RTC_WKUP NVIC。
4. **重切 HSE/PLL 80 MHz**：`PowerMode_ClkSwitchToHsePll()`（内部先 PA1=0 再 30 ms 起振余量；HSEON=0 → HSEBYP 可写）。失败 → 降级：sticky `FAULT_CLK`、`wake_clock_fail_count++`、只跑故障灯。
5. PB0=1 + `HAL_Delay(2)`（5 V 软启动）。
6. 外设重建（顺序敏感）：`App_Log_ReInit` → `BQ_ReInitAfterLp`（HwInit + 重跑完整配置脚本，约 250 ms 后数据有效）→ `Ws2812_ReInit`（**重设 USART2SEL=PCLK1、BaudRate=3809523、UE=0 写 BRR=21**）→ `Buzzer_ReInit` → `HAL_COMP_Start` → `Fault_PvdEnable` → `App_TickStart`。
7. 黑匣子 `BB_KIND_WAKE` 记录（含唤醒原因）→ `request_lp=0` → `state=NORM`，回到常态循环。

## 7. 调试要点

- **进 LP**：NORM 下调试器写 `s_power.request_lp = 1`；主循环下一圈消费。唤醒后 `request_lp` 已清零，想再进再写。
- **观测字段**：`s_power.state / lp_enter_count / wake_count / wake_rtc_count / wake_button_count / wake_alert_count / wake_cause / clock_fail_count / lp_prepare_fail_count / wake_clock_fail_count`；`s_app.tick_250hz_count / uptime_s / loop_count`；`s_bq.state / cc_raw / current_ma / request_fet_mask / fet_mask_applied`。
- **电流/MOS 控制**：`[DIAG]` 的 `CC=` 为原始 CC，`I=` 为 `cc_raw×844/150`（mA，带符号，250 ms 平均）；写 `s_bq.request_fet_mask` 为 0x00/0x01/0x02/0x03（全关/仅 DSG/仅 CHG/全开），下一轮 500 ms run script 完成后同步 `SYS_CTRL2=0x40|mask`，`fet_mask_applied` 随之更新。LP 准备和配置仍强制两颗 MOS 全开；BQ 硬件保护（OV/UV/SCD/OCD）可能覆盖手动写。
- **LP 电流测量**：必须断开调试器测；LP 期间 SWD 本来就连不上（DBG_STOP=0），测量流程 = 断开 → 复位 → 写 `request_lp=1` → 观察 5 V/HSE 关断 → µA 表读数 → 等 RTC 20 s 唤醒或按 PB5 触发。
- **黑匣子核对**：每次进/出 LP 各有一条转换记录（flags 带唤醒原因），唤醒后回读 Flash 或看 `s_bb.writes` 即可确认循环发生。
- **RGB/USART2 核对**：冷启动和 LP 唤醒后都应满足 `USART2->BRR == 21`、`RCC->CCIPR & RCC_CCIPR_USART2SEL == PCLK1`、`USART2->CR1 & UE == 1`；若 BRR 不是 21，说明 `Ws2812_HwInit` 的时钟/波特率重配没有执行或写 BRR 时 UE 仍为 1。
- 若需在 LP 中保持调试口可用，可临时置 `DBGMCU->CR |= DBGMCU_CR_DBG_STOP`（进出 STOP1 前后），但会破坏低功耗指标，只用于调试构建。

## 8. 后续待机策略方向（本次未实现）

L0 目前由调试器写 `request_lp` 手动进入 LP，且 LP 中不做任何检测。后续可演进为**周期待机**：

1. 大部分时间处于 LP（STOP1 + BQ ADC/TEMP 关）；
2. RTC WUT 周期唤醒（周期为可调参数，当前固定 20 s）；
3. 唤醒后短暂回到 NORM：重建外设、读取 BQ 电压/电流/温度、写一条黑匣子记录；
4. 再次休眠。

该策略需要把 `PowerMode_EnterLp()` 的“进入-唤醒”从一次性调用改为状态机循环，并决定唤醒周期、日志写入频率与 30 min 周期记录的关系。本次只完成 LP 关断（ADC/TEMP off），未实现周期待机逻辑。

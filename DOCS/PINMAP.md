# BMC_CMZY_080 MCU 引脚映射与比较器分析

依据（均为本仓库 `DOCS/` 和 `Core/` 内文件，未做猜测）：

- `BMC_CMZY_080.ioc`（STM32CubeMX 6.18.1，STM32Cube FW_L4 V1.18.2）
- `DOCS/SCH_SCHEM_BTT_CTRL_081_2026-10-30.pdf`（4 页原理图，BTT_CTRL_081 / SHOOT_cmzy，V0.80，2026-10-30）
- `DOCS/32L432.pdf`（DS11451 Rev 4，STM32L432KB，UFQFPN32）
- `DOCS/bq769X0.pdf`（BQ76920/30/40，SLUSBK2I，2022-03 修订）
- `Core/Src/main.c`、`Core/Src/stm32l4xx_hal_msp.c`、`Core/Inc/main.h`

MCU：**STM32L432KBU6**，UFQFPN32 + 裸露焊盘（EP，原理图 pin 33）。

## 0. 一句话结论

- 3 节锂电池（BTT_0P_0V / 1P_4V / 2P_8V / 3P_12V），电池负极 BTT_N 经 1.5 mΩ 分流电阻
  （R2‖R26）和背靠背 N-MOS（Q2/Q3）到输出负极 PACK_N_0V；BQ76920 监控电芯、电流和 FET。
- MCU 用 **I2C1（PA9/PA10）** 访问 BQ76920，**CAN1（PA11/PA12）** 对外通信，
  **USART1（PB6/PB7）** 作调试/上位机口，**USART2（PA2）** 驱动 5 V 可寻址 RGB 灯。
- **COMP2**：同相输入 PA3 = `SC_BTTN`（电池负极取样），反相输入 PA4 = `SC_PACK`（输出负极取样）。
  比较器输出 **高 = 充电，低 = 放电**（详见第 3 节）。L0 固件在 `App_Init()` 中 `HAL_COMP_Start()`，
  由 250 Hz tick 采样、32 样本（约 128 ms）一致性消抖。
- 硬件已画、固件已接的点（L0）：5 V 由 PB0 门控（`App_Init()` 拉高，LP 拉低）、
  BQ 由 PB1 boot 脉冲唤醒并走 I2C1 CRC+DMA、TIM1 CH1N 由 `buzzer.c` 以 2–5 kHz 可调频率驱动、
  LSE/RTC 由 CubeMX 初始化（LP 用 WUT 20 s 自动唤醒）、PA8 为 EXTI **事件**模式（仅唤醒 STOP1）、
  PB5 为下降沿中断（按下唤醒）。仍存的硬件风险：I2C 只有 MCU 内部上拉（无外部上拉）。

## 1. MCU 引脚总表（UFQFPN32）

| 封装引脚 | 端口 | .ioc 信号 / 标签 | 原理图网络 | 连接器件与功能 | 配置 / 固件备注 |
| --- | --- | --- | --- | --- | --- |
| 1 | VDD | — | `SUP_3V3D` | 3.3 V 数字电源 | U22 HT7533 由 `VBAT_12V` 产生；C95–C101 去耦 |
| 2 | PC14-OSC32_IN | `RCC_OSC32_IN` | X1 32.768 kHz + C14 5 pF | LSE 晶振输入 | 生成的 `SystemClock_Config()` 打开 LSE，`MX_RTC_Init()` 配置 RTC（异步 127 / 同步 255 → ck_spre 1 Hz）；LP 进入时布防 RTC WUT 20 s 作为自动唤醒源 |
| 3 | PC15-OSC32_OUT | `RCC_OSC32_OUT` | X1 + C15 5 pF | LSE 晶振输出 | 同上 |
| 4 | NRST | — | `RST` | 复位：R12 10 kΩ 上拉 3.3 V，C16 100 nF 到 REF_GND，KEY1 按键到 REF_GND | 低有效复位；内部上拉 |
| 5 | VDDA/VREF+ | — | `SUP_3V3D` 经 L9 | 模拟电源/参考 | L9 磁珠；VREFBUF 关闭，COMP2 参考即 VDDA |
| 6 | PA0 | `HSE-External-Clock-Source-for-LittleOrca` | `HSE_12M` | OSC1 12 MHz 输出 → R14 22 Ω → PA0 | HSE **bypass**；PLL 输入；不是 GPIO。复位后 MCU 先跑 HSI16，`App_Init()` 拉低 PA1 后由 `PowerMode_ClkSwitchToHsePll()` 切到 HSE+PLL 80 MHz |
| 7 | PA1 | `GPIO_Output` / `HSE_DIS` | `HSE_DIS` + `CHGSC_DIS`（同一网络） | Q5 栅极（OSC1 tri-state 使能）+ Q6/Q7 栅极（PACK_SC 前置） | 推挽输出，`MX_GPIO_Init` 后为低；`PowerMode_ClkSwitchToHsePll()` 显式拉低，LP 中（已切 MSI、关 PLL/HSE 之后）拉高，唤醒后重配置 HSE 前再拉低。低 = Q5 导通（OSC1 使能）+ Q6/Q7 导通（PACK_SC 有效）；高 = Q5 关 + PACK_SC 关。硬件复位期间 PA1 高阻、R16 4k7 上拉为高，但启动时钟已改 HSI16，因此不再影响开机（见 §5）。**运行中（HSE/PLL 为源时）拉高会关 PACK_SC 并使 PLL 失锁，只允许在切换序列中操作** |
| 8 | PA2 | `USART2_TX` / `RGB_USART2_TX` | `RGB_TX` | U20 `GL5050RGB01H-T` DIN（5 V 可寻址 RGB） | AF7 USART2。IOC 初始配置（半双工/开漏/115200）在运行时被 `ws2812_uart.c` 覆写：**deinit/reinit、USART2SEL=PCLK1（80 MHz）、BaudRate=3809523、UE=0 时写 BRR=21（3.8095 MBd）**、TXINV、清 HDSEL、PA2 推挽，按 WS2812 时序发 1 灯 12 B（GRB）；**USART2 中断发送（prio1），不使用 DMA**（DMA1_Ch7 留给 I2C1_RX）。冷启动 NORM 和 LP 唤醒 NORM 都重配。硬件风险：5 V 灯 VIH≈3.5 V 高于 3.3 V 输出，需上板实测 |
| 9 | PA3 | `COMP2_INP` / `COMP_BTTN` | `SC_BTTN` | PACK_SC 分压，取自电池负极 BTT_N | 模拟输入；COMP2 同相输入。约 `2.50 V + 0.266 × V(BTT_N)`；`App_Init()` 启动比较器，250 Hz tick 采样 + 32 样本消抖 |
| 10 | PA4 | `COMP2_INM` / `COMP_PACK` | `SC_PACK` | PACK_SC 分压，取自输出负极 PACK_N_0V | 模拟输入；COMP2 反相输入。约 `2.50 V + 0.266 × V(PACK_N)`。DAC1_OUT1 为内部模式（`DAC_OUT1_Int`），不驱动 PA4、不接 COMP2（INM=IO4），且未启动 |
| 11 | PA5 | — | — | 未连接（原理图打 X） | — |
| 12 | PA6 | — | — | 未连接（原理图打 X） | — |
| 13 | PA7 | `TIM1_CH1N` / `BUZ_TIM1_CH1` | `BUZ_TIM1_CH1` | Q8 AO3400A 栅极 → BUZZER1（4 kHz、5 V）；D16 续流 | AF1 TIM1 互补通道 CH1N。IOC 的 PSC=0 / ARR=65535 只是占位；`buzzer.c` 按音调表改 ARR（2.0/2.5/3.0/4.0/5.0 kHz）、CCR=ARR/2，用 `HAL_TIMEx_PWMN_Start()` + MOE 启停；LP 前停 PWM 并把 PA7 拉低 |
| 14 | PB0 | `GPIO_Output` / `SUP_5V_EN` | `SUP_5V_EN` | U21 LMR54410 的 EN（pin 4） | 推挽输出，`MX_GPIO_Init` 后为低。低 = 5 V 关；高 = 5 V 开。5 V 负载：TJA1044 VCC、U20 RGB、蜂鸣器、U18 的 +5 V |
| 15 | PB1 | `GPIO_Output` / `BQ_BOOT` | `BQ_BOOT` | Q4 AO3400A 栅极；漏极 = TS1(`TSEXT_H`)，源极 = REF_GND；R10 100 kΩ 下拉 | 推挽输出，`MX_GPIO_Init` 后为低。高 = 把 TS1 拉低；低 = 释放 TS1，靠**外部上拉**越过 BQ 的 boot 门限（V0.80 板上必须外加 TS1 上拉）。BQ 上电后处于 SHIP 模式 |
| 16 | VSS | — | `REF_GND` | 数字地 | 与 EP(33)、BQ VSS 同网络 |
| 17 | VDD | — | `SUP_3V3D` | 3.3 V 数字电源 | 同 pin 1 |
| 18 | PA8 | `GPXTI8` / `BQ_ALT_EXTI8` | `BQ_ALT` | BQ76920 `ALERT`（pin 20） | 输入，CubeMX 生成为 **EXTI 事件模式、上升沿**（`GPIO_MODE_EVT_RISING`）：不产生中断、没有 EXTI 挂起位，只作为 STOP1 的唤醒源；LP 唤醒后若无 RTC/PB5 标志，即归因为 BQ ALERT。ALERT 为高有效输出（带弱下拉），是 SYS_STAT 所有位的 OR；对应状态位写 1 清除后 ALERT 才撤销 |
| 19 | PA9 | `I2C1_SCL` / `BQ_IIC1_SCL` | `BQ_SCL` | RN4 22 Ω → BQ76920 SCL（pin 5） | AF4 开漏；固件在 `HAL_I2C_Init` **之前**写 `hi2c1.Init.Timing=0x1424A4FF`（PRESC 1 / SCLDEL 2 / SDADEL 4 / SCLH 164 / SCLL 255 → **95 kHz**，tHIGH=4.125 µs）；HAL 会清掉保留位，硬件 TIMINGR 读回 `0x1024A4FF` 属正常。IOC 生成的 ~103.6 kHz / tHIGH 3.93 µs 不满足 BQ 的 tHIGH≥4.0 µs。MSP 为 `GPIO_PULLUP`（内部上拉约 40 kΩ），**板上无外部 I2C 上拉**（见第 6 节）。I2C1_TX 用 DMA1_Ch6（生成句柄 `hdma_i2c1_tx`） |
| 20 | PA10 | `I2C1_SDA` / `BQ_IIC1_SDA` | `BQ_SDA` | RN4 22 Ω → BQ76920 SDA（pin 4） | AF4 开漏；同上。I2C1_RX 用生成的 **DMA1_Ch7**（`hdma_i2c1_rx`，Req3）；全部 DMA 只用 DMA1 |
| 21 | PA11 | `CAN1_RX` | `CAN1_RX` | RN4 22 Ω → TJA1044 RXD（pin 4） | AF9；约 833 kbit/s（Prescaler 16、BS1=4TQ、BS2=1TQ，采样点 5/6） |
| 22 | PA12 | `CAN1_TX` | `CAN1_TX` | RN4 22 Ω → TJA1044 TXD（pin 1） | AF9；TJA1044 VIO=3.3 V、VCC=5 V（需 PB0 高）、STB 接 GND（正常模式） |
| 23 | PA13 | `SYS_JTMS-SWDIO` | `SWDIO` | CN5 `TC-225-4PWT` SWD 口 | SWDIO；调试/下载，勿复用 |
| 24 | PA14 | `SYS_JTCK-SWCLK` | `SWCLK` | CN5 SWD 口 | SWCLK |
| 25 | PA15 | — | — | 未连接（原理图打 X） | — |
| 26 | PB3 | `GPIO_Output` / `IND_ROHT` | `IND_ROHT` | R11 510 Ω → LED1 → REF_GND | 推挽输出，`MX_GPIO_Init` 后为低。**高 = 点亮**（高有效）；由 `led_status.c` 驱动：NORM 常灭、故障时 5 Hz 快闪、LP 灭 |
| 27 | PB4 | `GPIO_Output` / `IND_NORM` | `IND_NORM` | R13 510 Ω → LED2 → REF_GND | 推挽输出，`MX_GPIO_Init` 后为低。高 = 点亮；`led_status.c`：NORM 常亮（调试期兼作心跳）、LP 灭 |
| 28 | PB5 | `GPXTI5` / `WAKE_EXTI5` | `WAKE_EXTI5` | SW1 B3F-3152 到 REF_GND；R24 10 kΩ 上拉到 3.3 V | 输入，EXTI **下降沿中断**（`.ioc` 已改；`power_mode.c` 幂等覆写）。按下即触发。EXTI9_5 NVIC 优先级 0，生成的 `EXTI9_5_IRQHandler` 只派发 `WAKE_EXTI5_Pin`；`HAL_GPIO_EXTI_Callback` 的唯一 owner 是 `app/power_mode.c`，只置唤醒标志 |
| 29 | PB6 | `USART1_TX` | `UART1_TX` | U18 `WAFFER-GH1.25-4PWB`（+5 V/GND/RX/TX）；22 Ω + 100 pF EMC | AF7；`app_log.c` 覆写 BRR=694（80 MHz 下 115200 8N1），TX 走 DMA1_Ch4（`hdma_log_usart1_tx`）；调试/上位机通道 |
| 30 | PB7 | `USART1_RX` | `UART1_RX` | U18；22 Ω + 100 pF EMC | AF7，115200 8N1 |
| 31 | PH3-BOOT0 | — | `BOOT0` | R15 10 kΩ 到 REF_GND | 从 Flash 启动；无 BOOT 按键 |
| 32 | VSS | — | `REF_GND` | 数字地 | 同 pin 16 |
| 33 | EP | — | `REF_GND` | 裸露焊盘 | 必须焊到地平面（电气 + 散热） |

## 2. 未使用/保留的引脚

- **PA5、PA6、PA15**：原理图打 X，NC。
- **PC14/PC15**：32.768 kHz 晶振已装（X1 + 5 pF）。生成的 `SystemClock_Config()` 打开 LSE，`MX_RTC_Init()` 配置 RTC；LP 进入时布防 RTC WUT（20 s）作为保底唤醒源。
- **DAC1**：CubeMX 配置了 CH1 为内部模式（`DAC_OUT1_Int`，`DAC_ConnectOnChipPeripheral=ENABLE`，不占引脚），
  代码 `MX_DAC1_Init()` 只初始化、从未 `HAL_DAC_Start()`；COMP2 的反相输入选的是 PA4（`COMP_INPUT_MINUS_IO4`），
  不是 DAC，因此 DAC 当前完全不参与比较器，也不影响 PA4。

## 3. 比较器（COMP2）与充放电判定

### 3.1 硬件通路

```
电池负极 BTT_N ──[ R2‖R26 = 3mΩ‖3mΩ = 1.5mΩ 分流 ]──[ Q2/Q3 SP40N01AAGTO 背靠背 ]── 输出负极 PACK_N_0V
        │                                                    ↑
        └── BQ_SRP (经 R6 100Ω)                              └── BQ_SRN (经 R7 100Ω) —— BQ76920 测分流电阻电压
```

PACK_SC 前置电路（左右两路完全对称）：

```
SUP_3V3D ── R17/R21 330k ──┬── SC_PACK / SC_BTTN ──→ PA4(COMP2_INM) / PA3(COMP2_INP)
                           │
                           ├── C17/C18 100nF ── REF_GND
                           │
                  Q6/Q7 AO3401A (P-MOS，源极在 SC 节点，栅极 = CHGSC_DIS)
                           │
                     R19/R22 910k
                           │
                     D4 / D5 (1N5819WS)
                           │
              PACK_N_0V (左) / BTT_N (右)
```

- Q6/Q7 的栅极由 **`CHGSC_DIS`** 驱动，而 `CHGSC_DIS` 与 PA1 的 `HSE_DIS` 在原理图上是**同一网络**（PA1 引脚处有连接点）。
  - PA1 = 低（`MX_GPIO_Init` 后）：Q6/Q7 导通，`SC_PACK` / `SC_BTTN` 有效。
  - PA1 = 高：Q6/Q7 关闭，两个 SC 节点被 330 kΩ 拉到约 3.3 V，比较器输入失效。
- 栅极还有 R20/R23 4.7 kΩ 上拉、D6/D7 到 REF_GND 的钳位；D4/D5 用来承受负极轨相对 REF_GND 的负压。
- 注意：**比较器输入是差分结构**，判据只取决于 `V(BTT_N) − V(PACK_N)`；REF_GND 的实际接点不影响“充电/放电”的符号，
  只影响共模点。BQ 数据手册明确要求 VSS 接分流电阻靠 SRP 的电池负极侧，本板 REF_GND = BQ VSS = MCU 地，
  因此共模在 2.5 V 附近，处于 COMP2 的 0…VDDA 范围内。

### 3.2 传递函数

设 `Vf` 为 1N5819WS 在约 2.4 µA 下的正向压降（约 0.2–0.3 V），则

```
V(SC) ≈ VDD3 − (VDD3 − V(neg) − Vf) × 330k/(330k+910k)
      ≈ 2.48…2.50 V + 0.266 × V(neg)
```

其中 `V(neg)` 是相应负极轨相对 REF_GND 的电压（BTT_N 或 PACK_N_0V）。因为：

```
V(SC_BTTN) − V(SC_PACK) = 0.266 × [ V(BTT_N) − V(PACK_N_0V) ]
```

- 斜率 = 330/(330+910) = **0.2661**。
- 直流共模 ≈ **2.48–2.50 V**。
- 滤波时间常数：节点看到的等效电阻 ≈ 330k‖(910k + 二极管) ≈ **242 kΩ**，C = 100 nF，
  因此 **τ ≈ 24 ms**（约 5τ ≈ 120 ms 才稳定到 1%）。这是“充放电状态”级别的慢信号，不能当快速过流保护。
- 比较器看到的差模信号 = `0.266 × I × (R分流 + Rds(on)Q2/Q3)`。
  10 A 时约几十 mV；1 A 时只有几 mV，接近/低于 COMP2 的输入失调与噪声。
  **精确电流/荷电状态请用 BQ76920 的 SRP/SRN 库仑计读数，COMP2 只做粗方向判定。**

### 3.3 极性（结论）

| 工况 | 负极通路电流方向 | 电压关系 | COMP2 输出 |
| --- | --- | --- | --- |
| **充电** | BTT_N → 分流电阻 → Q2/Q3 → PACK_N_0V | `V(BTT_N) > V(PACK_N)` | **高（1）** |
| **放电** | PACK_N_0V → Q2/Q3 → 分流电阻 → BTT_N | `V(BTT_N) < V(PACK_N)` | **低（0）** |
| 静置 / 零电流 | 无 | 两者近似相等 | 不确定（无迟滞，会抖） |

推导：充电时充电器把电流灌入电池正极，回路由电池负极 BTT_N 流出，经分流电阻和 FET 到输出负极 PACK_N_0V；
电流从高电位流向低电位，所以 `BTT_N` 电位高于 `PACK_N`。`SC_BTTN` 接同相端、`SC_PACK` 接反相端，
故 `COMP2 = 高`。放电时方向相反，`COMP2 = 低`。

### 3.4 软件使用要点

1. **必须启动**：`MX_COMP2_Init()` 只做 `HAL_COMP_Init(&hcomp2)`；`App_Init()` 末尾补了
   `HAL_COMP_Start(&hcomp2)`，LP 唤醒后的 `PowerMode_RestorePeripherals()` 也会重启它。删除这一步会让
   `HAL_COMP_GetOutputLevel()` 永远读到无效/0。
2. **读数**：`HAL_COMP_GetOutputLevel(&hcomp2)`（内部读 `COMP_CSR_VALUE`，bit 30）。
   COMP2 没有输出引脚、没有 TIM 触发、没有 EXTI；L0 在 250 Hz TIM2 tick 中轮询。
3. **消抖**：无迟滞 + 24 ms 模拟滤波，软件连续 32 次（250 Hz 下约 128 ms ≈ 5τ）一致才提交方向；
   电流接近 0 时保持上一个稳定状态（状态机用 BQ 新鲜度区分 standby）。
4. **PA1 运行中必须为低、LP 中为高**：`HSE_DIS/CHGSC_DIS` 为高会关掉 PACK_SC 前置，两个 SC 节点被拉到约 3.3 V，
   比较器结果无意义；同时 Q5 关停 12 MHz HSE。L0 只在进入 LP（已切 MSI、关 PLL/HSE 后）拉高 PA1，唤醒时先拉低再重配 HSE。
5. **BQ 电流符号相反**：按 SRP/SRN 引脚定义（SRP 靠 VSS 的负端、SRN 正端），放电时 `SRN > SRP`，
   CC 读数应为**正**、充电为**负**；即 **COMP2 高（充电）↔ CC 为负**。首次联动时用已知充电/放电方向实测确认符号后再定映射。
6. **DAC1 是内部模式**：`DAC_OUT1_Int`，没有外部引脚，也不在 COMP2 的 INM 路径（INM = IO4），启动它不会影响 PA4；
   但不要改成外部引脚输出或把 COMP2_INM 改为 DAC——那会覆盖 `SC_PACK`。保持现状或关闭即可。

## 4. BQ76920 引脚/网络对照（U1，BQ7692003PWR）

器件变体（数据手册 Device Comparison Table）：

- **BQ7692003PWR：3–5 节，I2C 7 位地址 0x08（HAL 用 0x10），LDO = 3.3 V，CRC = Yes。**
- 本板用 VC0–VC3 共 3 节；VC4/VC5 未用（NC，原理图打 X）。
- CRC 使能意味着 I2C 读写每个字节后要跟 CRC-8；不能按普通无 CRC 器件访问。

| BQ 引脚 | 名称 | 原理图网络 | 连接 / 功能 |
| --- | --- | --- | --- |
| 1 | DSG | `BQ_DSG` | 放电 FET 驱动，进 PACK_SWITCH 的 Q1/Q2/Q3 驱动网络 |
| 2 | CHG | `BQ_CHG` | 充电 FET 驱动 |
| 3 | VSS | `REF_GND` | IC 地；按 TI 要求接分流电阻靠 SRP 的电池负极侧 |
| 4 | SDA | `BQ_SDA` | 经 RN4 22 Ω → MCU PA10（I2C1_SDA） |
| 5 | SCL | `BQ_SCL` | 经 RN4 22 Ω → MCU PA9（I2C1_SCL） |
| 6 | TS1 | `TSEXT_H` | 外部 NTC（CN3）+ R8/R9 10 Ω + C13 10 nF；同时是 SHIP→NORMAL 的 boot 检测脚；Q4 可把它拉低。**V0.80 原理图没有 TS1 外部上拉**，boot 必须另加上拉（见 §4 末） |
| 7 | CAP1 | — | 内部 3.3 V LDO 电容到 VSS（BMS 块去耦网络，具体电容见原理图） |
| 8 | REGOUT | — | 外部 LDO 输出（003 变体 3.3 V，boot 后才有输出）；可作为 I2C 上拉电源（不建议在 SHIP 时依赖它） |
| 9 | REGSRC | `VBAT_12V`（应为） | 外部 LDO 输入；**V0.80 原理图只接了 C4 1 µF，未接 VBAT_12V**，必须飞线到 VBAT_12V（BAT/pin 10 节点），否则 REGOUT 不输出、CHG/DSG 栅极驱动无电源 |
| 10 | BAT | `VBAT_12V` | 电池组最高端 |
| 11 | NC | — | 未连接 |
| 12 | VC5 | — | 未连接（3 节，打 X） |
| 13 | VC4 | — | 未连接（3 节，打 X） |
| 14 | VC3 | `BQ_VC3` | R29 100 Ω / C6 100 nF → `BTT_3P_12V`（第 3 节正端） |
| 15 | VC2 | `BQ_VC2` | R30 100 Ω / C7 100 nF → `BTT_2P_8V` |
| 16 | VC1 | `BQ_VC1` | R31 100 Ω / C8 100 nF → `BTT_1P_4V` |
| 17 | VC0 | `BQ_VC0` | R32 100 Ω / C9 100 nF → `BTT_0P_0V`（第 1 节负端） |
| 18 | SRP | `BQ_SRP` | R6 100 Ω → BTT_N 侧（靠近 VSS）的分流电阻端；数据手册定义为“负电流检测” |
| 19 | SRN | `BQ_SRN` | R7 100 Ω → 分流电阻的 FET/PACK 侧；数据手册定义为“正电流检测” |
| 20 | ALERT | `BQ_ALT` | → MCU PA8；高有效输出（带弱下拉），是 SYS_STAT 所有位的 OR；对应状态位写 1 清除后 ALERT 才撤销；数据手册也把它作 override 输入 |

BQ 在 **POR 后总是进入 SHIP 模式**；退出 SHIP 需要 TS1 出现**上升沿**并越过 VBOOT（>1.0 V 才保证，
0.3–1.0 V 不确定，<0.3 V 不 boot）。BQ 内部 10 kΩ TS1 上拉只在 NORMAL/TEMP_SEL=1 测温时有效，
**SHIP 下不工作**；V0.80 板上 Q4 只把 TS1 拉低，release 后 TS1 仍被 NTC 拉到 0 V，因此必须在 TS1
外部提供上拉（例如 10 kΩ 到 SUP_3V3D 或 VC1，或可切换的高边偏置）才能 boot。推荐时序：
PB1 高（Q4 拉低 TS1）→ PB1 低（释放，TS1 靠外部上拉升高）→ 等 ≥10 ms（tBOOTREADY）再访问 I2C；
首次电芯电压数据约需 250 ms。boot 完成后 PB1 必须保持低，否则 Q4 会把 NTC 短路、温度测量失效。
REGSRC 必须接 VBAT_12V（V0.80 漏接），否则 REGOUT 与 CHG/DSG 栅极驱动都没有电源。

## 5. 电源域与上电默认

| 电源轨 | 来源 | 默认状态 | 负载 |
| --- | --- | --- | --- |
| `VBAT_12V` | 3S 电池 `BTT_3P_12V` 经 R1 100 Ω + TVS/滤波 | 接电池即有 | BQ 的 BAT/REGSRC；U21 5 V buck；U22 3.3 V LDO |
| `SUP_3V3D` | U22 HT7533（从 VBAT_12V） | **常开**（无使能） | MCU VDD/VDDA、BQ I2C 上拉（MCU 内部，无外部）、LED、PACK_SC 分压、SWD |
| `SUP_5VD` | U21 LMR54410（从 VBAT_12V），EN = PB0 | **默认关**（`MX_GPIO_Init` 后 PB0 = 低） | TJA1044 VCC、U20 RGB LED、BUZZER1、U18 的 +5 V |
| `REF_GND` | 控制地 | — | MCU VSS/EP、BQ VSS、所有去耦/按键/LED/比较器钳位 |

上电默认（`MX_GPIO_Init` 之后、应用代码之前）：

- PA1 = 低 → OSC1 12 MHz 工作，PACK_SC 前置使能。
- PB0 = 低 → **5 V 关**；CAN 收发器、RGB 灯、蜂鸣器、UART 连接器 +5 V 都不可用。
- PB1 = 低 → Q4 关，TS1 由上拉/NTC 决定，BQ 可自动从 SHIP boot。
- PB3/PB4 = 低 → LED1/LED2 灭（高有效）。
- COMP2 未启动；DAC1 未启动；TIM1 未启动；TIM2 已初始化但未启动；I2C/CAN/USART 外设已初始化；
  EXTI9_5 已使能，`EXTI9_5_IRQHandler` 只派发 PB5。

`App_Init()` 之后（NORM）：

- 时钟 HSI16 → HSE bypass + PLL 80 MHz（失败重试 3 次 → `FAULT_CLK` 降级）；TIM2 覆写为 250 Hz 并启动。
- PB0 = 高（5 V 开，20 ms 软启动余量）；PB1 先 2 ms 高脉冲（强制 BQ 退出 SHIP）后保持低。
- BQ 走 I2C1（95 kHz、CRC、DMA）配置；COMP2 启动；TIM1 由蜂鸣器模块按需启停；PB3/PB4 由灯效驱动。
- LP 中：PA1 = 高、PB0 = 低、SYSCLK = MSI 4 MHz、PLL/HSE 关、TIM2/I2C/USART/DMA 全部停止；
  只有 RTC（LSE + WUT）和 PA1/PB0 保持。唤醒后按相反顺序恢复。

硬件复位期间（`MX_GPIO_Init` 之前）：PA1 为高阻，R16 4k7 把 `HSE_DIS` 拉高、Q5 关；但
`SystemClock_Config()` 现在只启动 HSI16，HSE 的使能由 App 在 PA1 已驱动为低之后执行，因此
OSC1 的 tri-state 默认电平不再影响能否开机。上板仍应确认 PA0 上 12 MHz 波形存在。

推荐的 bring-up 顺序（按需）：

1. 上电观察：PB4 灯亮（NORM）、USART1 115200 收到 `[BOOT]` 日志；示波器测 PA0 12 MHz。
2. 读 BQ：I2C 95 kHz、tHIGH≥4.0 µs；内部上拉已使能，若边沿/速率不足补 4.7–10 kΩ 外部上拉。
   若 BQ 不响应，PB1 高→低 boot 脉冲（固件已自动执行一次）后等 ≥10 ms。
3. COMP2 已在 `App_Init()` 启动；用已知充/放电方向核对高/低与 BQ CC 符号。
4. 蜂鸣器/WS2812 由 App 驱动，需要时按状态/自检结果验证（WS2812 注意 5 V VIH 风险）。

## 6. 核对清单（按优先级）

1. **COMP2 已在 `App_Init()` 启动**（`HAL_COMP_Start`）：若被人为删除或 LP 唤醒后未重建，
   PA3/PA4 的采样结果无效；自检用 `HAL_COMP_GetState()==HAL_COMP_STATE_BUSY` 检查。
2. **I2C 上拉/时序**：PA9/PA10 经 RN4 22 Ω 到 BQ，MSP 为 `GPIO_PULLUP`（内部上拉约 40 kΩ），
   板上没有外部上拉；建议各加 4.7–10 kΩ 到 SUP_3V3D（或 REGOUT）留余量。按 **CRC=Yes** 的格式访问
   （地址 0x08）。固件在 `HAL_I2C_Init` **之前**写 `hi2c1.Init.Timing=0x1424A4FF`（95 kHz、tHIGH 4.125 µs）；
   HAL 会清掉保留位，硬件读回 `0x1024A4FF` 是正常的。I2C1_RX 使用生成的 DMA1_Ch7/Req3。
3. **5 V 默认关**：不先拉高 PB0，CAN（TJA1044 VCC）、RGB 灯、蜂鸣器、UART 连接器 +5 V 都不会工作。
4. **PA1 双功能**：`HSE_DIS` 与 `CHGSC_DIS` 同一网络；PA1 高会关 PACK_SC，且设计意图是停 HSE
   （实际是否停振取决于 OSC1 的 tri-state 默认电平；若 HSE 是 PLL 源，停振会失锁）。不要把 PA1 当普通 GPIO
   在运行中翻转。硬件复位期间 PA1 高阻、R16 4k7 上拉使 HSE_DIS 为高；`SystemClock_Config()` 在
   `MX_GPIO_Init()` 之前运行，HSE bypass 能否起振取决于 OSC1 的 tri-state 默认电平——上电前必须确认。
5. **BQ boot**：POR 后必在 SHIP；TS1 需要**上升沿**越过 VBOOT（>1.0 V 保证，0.3–1.0 V 不确定）。
   BQ 内部 10 kΩ TS1 上拉在 SHIP 下不工作，V0.80 板必须外加 TS1 上拉（10 kΩ 到 SUP_3V3D/VC1 或可切换
   高边偏置）；用 PB1 高（拉低）→ 低（释放）产生上升沿。boot 后等 ≥10 ms，首次电芯数据约 250 ms；
   boot 完成后 PB1 保持低，否则 NTC 被短路。
6. **EXTI 分工**：PA8（BQ ALERT）是**事件模式**（上升沿），不产生中断、没有挂起位，只唤醒 STOP1；
   PB5（WAKE）是**下降沿中断**，按下即触发。EXTI9_5 的 `EXTI9_5_IRQHandler` 只派发 PB5；
   `HAL_GPIO_EXTI_Callback` 的唯一 owner 是 `app/power_mode.c`，只置唤醒标志。LP 唤醒后若
   RTC 标志和 PB5 标志都没有，就归因为 BQ ALERT。不要把 PA8 改回中断模式（会引入未消费的中断/挂起位）。
7. **USART2 与 RGB 灯**：用户已确认 U20 为 WS2812 类。`ws2812_uart.c` 在运行时 deinit/reinit USART2：
   USART2SEL=PCLK1、BaudRate=3809523、UE=0 时写 BRR=21（3.8095 MBd）、TXINV、推挽；冷启动 NORM 和
   LP 唤醒 NORM 都重配。RGB 帧用 **USART2 中断发送**（prio1，不使用 DMA；DMA1_Ch7 留给 I2C1_RX），
   不再使用 IOC 的 115200/开漏配置。硬件风险仍在：5 V 灯的 VIH≈3.5 V 高于 3.3 V 输出，若点不亮/闪烁，
   记录并降级用 PB3/PB4 指示。
8. **TIM1 蜂鸣器**：PA7 是 TIM1_CH1N（互补输出）。`buzzer.c` 按音调表写 ARR/CCR 并用
   `HAL_TIMEx_PWMN_Start()` + MOE 启停；IOC 的 ARR=65535 只是占位。LP 前停 PWM 并把 PA7 拉低。
9. **CAN 终端**：TJA1044 的 STB 接 GND（正常模式），板上只看到共模电感 L13、TVS D3，**未见 120 Ω 终端电阻**；
   确认整车线束两端终端由其他节点提供，否则通信反射/误码。
10. **LSE/RTC**：32.768 kHz 晶振已装；`.ioc` 已含 RTC，生成的 `SystemClock_Config()` 打开 LSE，
    `MX_RTC_Init()` 配置 1 Hz 时基。LP 进入时固件布防 RTC WUT 20 s；布防失败会放弃进入 LP（防止无唤醒源）。
11. **Flash 容量**：链接脚本已改为 128 K（实际 STM32L432KBU6 为 128 K）；**CubeMX 重生成会恢复成 256 K，必须手动改回**，
    否则大镜像会链接通过但烧录失败。黑匣子占用 `0x0801A000–0x0801FFFF`，`_etext` 必须低于 `0x0801A000`。
12. **DAC1**：当前为内部模式（`DAC_OUT1_Int`），不占 PA4、不接 COMP2（INM=IO4）；不要改成外部引脚输出
    或把 COMP2_INM 改为 DAC，否则会干扰 `SC_PACK`。
13. **LED 极性**：PB3/PB4 经 510 Ω 驱动 LED 阳极，LED 阴极到 REF_GND → **高电平点亮**；`MX_GPIO_Init` 后为低 = 灭。
14. **比较器精度**：无迟滞、模拟 τ≈24 ms、差模仅 0.266×，1 A 级别接近失调；
    精确电流/充放电判据优先用 BQ 的 SRP/SRN 库仑计，COMP2 只作粗方向/冗余判断。
15. **LP 唤醒路径**：进入 STOP1 前固件先写 BQ `SYS_CTRL1=0x00`（关 ADC/TEMP，LP 期间无电压/电流/温度检测）、
    写 `CC_EN=0` 并清 `SYS_STAT`（否则 CC_READY 每 250 ms 拉 ALERT 造成唤醒风暴）、
    清 EXTI 挂起，再切 MSI/关 PLL/HSE、抬 PA1、关 PB0。唤醒源为 RTC 20 s（保底）、PB5 按下、PA8 ALERT（事件，归因）。
    调试 LP 时须断开调试器，否则 STOP1 电流和唤醒行为不可信。

## 7. 快速查阅：比较器链路

```
PA3 (COMP2_INP) ← SC_BTTN ← R21 330k/Q7/R22 910k/D5 ← BTT_N (电池负极)
PA4 (COMP2_INM) ← SC_PACK ← R17 330k/Q6/R19 910k/D4 ← PACK_N_0V (输出负极)
COMP2 输出: SC_BTTN > SC_PACK → 高 → 充电
            SC_BTTN < SC_PACK → 低 → 放电
```

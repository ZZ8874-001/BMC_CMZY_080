# BMC_CMZY_080 L1 电池组管理开发方案（既定计划）

> 本文按此前确认的 L1 目标、硬件边界和设计决策整理，描述“要做什么、如何划分、如何验收”。  
> 本文不按当前仓库代码实现状态修订；文中的“当前/现有”指 L0 基线，不代表本文写作时的代码已经完成。

---

## 0. 方案摘要

目标产品是一块 **3S LiFePO4 1500 mAh 电池组控制板**：

- MCU：STM32L432KBU6，Cortex-M4F，最高 80 MHz；
- AFE：BQ7692003PWR，I2C1，7 位地址 0x08，每字节 CRC；
- 主回路：1.5 mΩ 负端分流 + 背靠背 N 沟道 MOSFET；
- 充电最大 5 A，放电最大 30 A；
- 板载 5 V 受控电源、RGB 灯、蜂鸣器、PB5 按键、PA8 BQ ALERT、COMP2 充放电方向检测；
- 电池组通过负端 MOSFET 与外部 PACK 隔离，MCU 由电池侧 3.3 V 常供电。

L1 要完成的电池组行为：

1. **空闲自动进 LP**；充电或放电时保持 NORM；
2. **充电器插入自动唤醒**，进入充电状态；
3. **放电必须由用户手势显式激活**，不允许上电即放电；
4. 充电、放电都 **双管全开**，不依赖体二极管；
5. 充电/放电期间持续采样、估算 SOC/SOH、均衡、保护；
6. 放电 SOC 低于 5% 报警并切断，锁存到充电且 SOC 回到 10% 以上；
7. 长期储存每 30 min 监测并存档；SOC 到 0% 且电压过低进入死存储，BQ SHIP；
8. 支持换电检测、换电模式、三连击清除换电报错；
9. 使用已 clone 的 `li-ion-soc-soh-toolkit` 完成 SOC/SOH 集成；
10. 所有保护在失效时回到“双管全关”的安全状态。

### 0.1 已确认的关键决策

| 编号 | 决策 |
|---|---|
| D1 | 充电和放电都必须 **CHG=1、DSG=1 双管全开**，不允许依赖体二极管 |
| D2 | 放电最大 30 A，充电最大 5 A；BQ 硬件阈值和软件 OCP 都按此上限设计 |
| D3 | LP、1 s 充电嗅探、30 min 储存监测使用 **HSI16**；NORM 使用 HSE/PLL 80 MHz |
| D4 | 充电检测采用 1 s RTC 轮询：BQ `LOAD_PRESENT` 初判，PA1/COMP2 复核方向 |
| D5 | BQ FET 默认全关，可由状态机/调试变量显式打开；配置结束状态为“CC 开、双管关” |
| D6 | 均衡“常开”指充电/放电期间持续启用自动均衡；按 BQ 相邻节规则单节轮换；BQ die 温 65 °C 超限停止 |
| D7 | 外部 I2C 上拉已具备；REGSRC 已接 12 V；OCD/SCD 已实测 |
| D8 | LiFePO4 参数标定推迟到 M3 前完成；标定前 SOC 只用于显示/日志，不作为 5%/0% 的唯一依据 |
| D9 | 30 A/5 A、双管全开、BQ die 温 65 °C、SCD/OCD 均需在 M6 复测 |
| D10 | CAN1 保持初始化但不实现业务；L1 的 NORM 80 MHz 时基已经满足后续 CAN 需求 |

---

## 1. 硬件与 L0 基线

### 1.1 主要硬件

| 项目 | 说明 |
|---|---|
| MCU | STM32L432KBU6，UFQFPN32，Cortex-M4F |
| AFE | BQ7692003PWR，3S，I2C1 0x08，CRC=Yes，内部 3.3 V LDO |
| 主回路 | 负端 BTT_N → 1.5 mΩ 分流 → 背靠背 N-FET → PACK_N_0V |
| 电池 | 3S LiFePO4，1500 mAh，NMC 参数表只可临时占位 |
| 5 V | LMR54410，PB0 使能；CAN 收发器、RGB、蜂鸣器、UART 连接器 |
| 3.3 V | HT7533，由电池侧 VBAT_12V 供电，常开 |
| 充电方向 | COMP2：PA3 `SC_BTTN` 对 PA4 `SC_PACK`；COMP2 高 = 充电 |
| BQ ALERT | PA8，EXTI 事件/轮询共用 |
| 按键 | PB5，低有效，下降沿 EXTI，外部 10 kΩ 上拉 |
| 12 MHz HSE | 外部 OSC1，经 Q5 三态控制，PA1=0 使能 |
| RTC | LSE 32.768 kHz；LP 周期唤醒 |

### 1.2 L0 已具备

- HSE bypass 12 MHz → PLL 80 MHz；HSI16 冷启动；
- I2C1 + DMA + 每字节 CRC 的 BQ 驱动；
- BQ 电压/电流/温度读取，CC 250 ms 平均；
- COMP2 充放电方向检测；
- RGB（USART2 3.8095 MBd）、蜂鸣器、状态灯、黑匣子；
- 250 Hz TIM2 应用时基；
- RTC/LP 基础：手动请求后进入 STOP1，RTC WUT 唤醒；
- 调试日志、自检、故障位、复位原因。

### 1.3 L1 不重复实现

- 不重写 HAL/CubeMX 外设初始化；
- 不在 `Core/` 里写业务代码；
- 不使用 RTOS、动态内存或阻塞式长任务；
- 不把 CAN 业务、上位机协议作为 L1 前置条件。

### 1.4 L1 非目标

- 不实现 CAN 报文协议、主机工具或云连接；
- 不实现充电器 CC/CV 控制，只通过 CHG/DSG 允许或禁止充放电；
- 不实现主动均衡，只使用 BQ 内部被动均衡；
- 不增加电池 ID/EEPROM 硬件，换电识别只做电压启发式；
- 不实现无线升级/Bootloader；
- 不在 LFP 参数标定完成前把 SOC 作为唯一硬保护依据；
- 不追求 µA 级整板静态电流，只对 MCU + BQ 可控部分设定目标并实测。

---

## 2. 软件分层与模块

沿用参考工程的 `App/` 分层：

```text
App/
├─ board.h                 外设句柄唯一 extern 入口、板级常量、Flash 分区
├─ bsp/
│  ├─ bq_ic.c/.h           BQ 驱动、FET、均衡、SHIP、保护、LP 嗅探
│  ├─ button.c/.h          PB5 采样/消抖/事件队列
│  ├─ flash_store.c/.h     Flash 擦写封装
│  ├─ ws2812_uart.c/.h     RGB
│  └─ buzzer.c/.h          蜂鸣器
├─ comm/
│  └─ app_log.c/.h         调试日志（CAN 业务不实现）
├─ detect/
│  ├─ gesture.c/.h         短按/长按/配对/三连击
│  ├─ soc_est.c/.h         BQ 快照 → SOC/SOH 适配
│  └─ bms/                 li-ion-soc-soh-toolkit 核心
├─ app/
│  ├─ main_app.c/.h        初始化、主循环、250 Hz tick
│  ├─ power_mode.c/.h      NORM/LP、时钟、STOP1、唤醒原因
│  ├─ state_machine.c/.h   业务状态与 FET/均衡/灯/蜂鸣器决策
│  ├─ shelf_monitor.c/.h   LP 周期监测、换电比较、30 min 存档
│  ├─ state_store.c/.h     跨掉电状态（48 B 记录，页 50–51）
│  ├─ black_box.c/.h       24 B 黑匣子记录（页 52–63）
│  ├─ led_status.c/.h      RGB/PB3/PB4
│  ├─ faults.c/.h          故障位/ sticky
│  ├─ self_test.c/.h       上电自检
│  ├─ reset_cause.c/.h     复位原因
│  ├─ charge_policy.c/.h   充放电/空闲策略（L1 策略层）
│  └─ cal_stream.c/.h      调试 CSV 流
└─ app.mk
```

依赖方向：

- `app/` → `detect/`、`comm/`、`bsp/`
- `detect/`、`comm/` → `bsp/`
- `bsp/` → HAL + `board.h`
- `detect/` 不依赖 `comm/`
- ISR 只置标志、计数、启动 DMA；Flash、协议、状态机、蜂鸣器都在主循环

---

## 3. 运行模式

### 3.1 NORM

- HSE bypass 12 MHz → PLL 80 MHz，Scale1，4WS；
- 5 V 打开；
- BQ NORMAL；
- TIM2 250 Hz、BQ 轮询、SOC/SOH、状态机、灯、蜂鸣器、日志运行；
- 充电或放电时保持 NORM，不自动 LP；
- 无充放电、无按键 5 s 后请求 LP。

### 3.2 LP

- STOP1；
- SYSCLK 切换 HSI16；
- PLL/HSE 关闭，OSC1 三态关闭；
- 5 V 关闭；
- BQ 进入低功耗配置，FET 全关（除非调试指定）；
- RTC WUT 1 s 周期；
- 每次唤醒在 HSI16 下读 BQ `SYS_CTRL1/SYS_STAT/SYS_CTRL2`；
- 充电器、PB5、BQ ALERT 或 30 min 维护唤醒时完整唤醒；
- 1 s 嗅探不写 Flash、不启动 5 V；
- 调试构建可用编译开关在 STOP1 保持 SWD；量产构建关闭以获得最低 LP 电流。

### 3.3 时钟与切换原则

- 冷启动：HSI16 → 外设初始化 → `PowerMode_ClkSwitchToHsePll()`；
- NORM → LP：先排空通信，再停外设/5 V，之后切 HSI16、关 PLL/HSE；
- LP → NORM：先修 SysTick（HSI16），再恢复 HSE/PLL，最后开 5 V、重建外设；
- App 代码不得直接调用生成的 `SystemClock_Config()`；
- 8 MHz 以下时基与 LP 超时必须在 HSI16 下重新标定。

### 3.4 启动顺序

1. `HAL_Init()`，HSI16 冷启动；
2. `MX_GPIO_Init()` 和各 `MX_*_Init()`；
3. `App_Init()`：
   - 看门狗/故障/复位原因；
   - 电源模式与 HSE/PLL 80 MHz；
   - 黑匣子、StateStore、BMS NVM 扫描；
   - 5 V 上电并等待软启动；
   - 日志、BQ 初始化、SOC/SOH 初始化；
   - RGB、灯、蜂鸣器、按键/手势；
   - 自检；
   - 状态机、充电策略、保护监督；
   - COMP2、250 Hz TIM2 最后启动；
   - 若从 LP 唤醒，恢复外设并应用唤醒原因；
4. 自检完成前不进入自动 LP；
5. 任何初始化失败必须进入 FAULT/降级，不允许带故障继续充放电。

### 3.5 RTC 时基

- 生产配置使用 LSE 32.768 kHz；异步预分频 127、同步 255 → 1 Hz；
- RTC 只作为绝对时间/WUT 时基，不承担日历业务；
- 调试阶段若 LSE 未起振，可用 LSI 做相对时间诊断，但必须明确：
  - LSI 未校准，不能作为 15 天储存时间基准；
  - LSE 故障时置 `FAULT_LSE`，禁止自动 LP/30 min 存档，或仅允许人工调试 LP；
- RTC 时钟源切换会触发备份域复位并清 BKP 状态，禁止在运行中热切换；
- 进入 STOP1 前必须确认 WUT 的 `WUTE` 和 `WUTIE` 同时有效；
- RTC WUT 中断服务只允许置标志，不允许在回调里清 WUTIE。

---

## 4. 业务状态

`IDLE/CHARGE/DISCHARGE/...` 是业务状态；NORM/LP 是电源模式，二者叠加，不混为一谈。

| 状态 | 含义 | DSG | CHG | 均衡 | 备注 |
|---|---|---:|---:|---:|---|
| BOOT | 自检/读状态/首批 BQ | 0 | 0 | 0 | 不充不放 |
| IDLE | 正常空闲/储存 | 0 | 0 | 0 | 5 s → LP |
| CHARGE | 充电器存在 | 1 | 1 | 开 | 双管全开；充满不断电 |
| DISCHARGE_ARM | 第一次短按后等待第二次长按 | 0 | 0 | 0 | 2 s 配对窗口 |
| DISCHARGE | 用户激活放电 | 1 | 1 | 开 | 双管全开；5% 切断 |
| CUTOFF_LATCH | 放电 SOC<5% 锁存 | 0 | 0 | 0 | 充电 SOC>10% 才解 |
| SWAP_ERROR | 换电/电压跳变错误 | 0 | 0 | 0 | 三连击清除 |
| REPLACE_READY | 5 s 长按确认换电 | 0 | 0 | 0 | 下次上电必报错 |
| DEAD_STORAGE | SOC=0/过放死存储 | BQ SHIP | — | 0 | 仅 RTC + 最小充电嗅探 |
| FAULT | 关键保护/硬件故障 | 0 | 0 | 0 | 安全优先 |

### 4.1 FET 策略

- 充电和放电：**DSG=1、CHG=1**，双管全开，方向只由电流/COMP2 判断；
- 其余状态：双管全关；
- 配置和 LP 安全默认由状态机显式写入，不允许固定 `0x43/0x03`；
- FET 状态可调，默认全关；调试变量可以单独验证 CHG/DSG；
- 任何保护动作优先于手动 FET 请求。

### 4.2 事件优先级

```text
关键故障/DEAD > SWAP_ERROR > 充电器 > 按键手势 > 空闲 5 s
```

- 充电器出现时，终止 ARM/DISCHARGE，进入 CHARGE；
- 充电器插入正在放电时，保持双管全开，转 CHARGE；
- 充电器移除后，用户需重新做放电手势；
- 不允许充电中通过长按进入换电模式，避免误操作。

---

## 5. 状态转换

| 起点 | 事件 / 守卫 | 动作 | 终点 |
|---|---|---|---|
| BOOT | 状态存储有 `expected_swap/swap_error` | 双管关、错误提示 | SWAP_ERROR |
| BOOT | 状态存储有 `cutoff_latch` | 双管关、提示 | CUTOFF_LATCH |
| BOOT | 状态存储有 `dead_storage` 且电压仍低 | BQ SHIP | DEAD_STORAGE |
| BOOT | 充电器存在且包合法 | 双管开、充电开始音 | CHARGE |
| BOOT | 包合法、无充电器 | 双管关、5 s 计时 | IDLE |
| IDLE | 充电器出现 | 双管开、充电开始音 | CHARGE |
| IDLE | 第一次短按 | 记录时间、开 2 s 配对窗口 | DISCHARGE_ARM |
| IDLE | 按住 5 s → 0.5 s 内释放 | 5 s 提示音、H-L-H-L、写 expected_swap | REPLACE_READY → LP |
| IDLE | 5 s 无操作 | 请求 IDLE LP | IDLE(LP) |
| IDLE | 换电压差超限 | 双管关、报错、写状态 | SWAP_ERROR |
| IDLE | SOC=0 且最低节 ≤2.5 V | BQ SHIP、写 dead | DEAD_STORAGE |
| DISCHARGE_ARM | 第二次按下且保持 3 s | L-M-H 三声、双管开 | DISCHARGE |
| DISCHARGE_ARM | 2 s 窗口到/第二次短按/充电器 | 取消 arm → IDLE/CHARGE | IDLE/CHARGE |
| DISCHARGE | 同样的短按+3 s 长按 | 双管关、H-M-L 三声、5 s 计时 | IDLE |
| DISCHARGE | SOC≤5%（连续 2 帧） | 3 声长鸣、双管关、写 cutoff | CUTOFF_LATCH → LP |
| DISCHARGE | 充电器出现 | 双管开、充电开始音 | CHARGE |
| CHARGE | 充满判据 | 充满提示音、保持双管开 | CHARGE |
| CHARGE | 充电器移除 5 s | 双管关、5 s 计时 | IDLE |
| CUTOFF_LATCH | 充电器出现 | 双管开（允许充电） | CHARGE |
| CUTOFF_LATCH | 充电 SOC>10% | 清 cutoff、写状态 | CHARGE |
| CUTOFF_LATCH | 按键唤醒、无充电器 | 显示锁存、禁止放电、5 s→LP | CUTOFF_LATCH |
| SWAP_ERROR | 1 s 内三次短按且包合法 | 清 SOC NVM/StateStore、重置 SOC/SOH | IDLE |
| SWAP_ERROR | 三连击但包非法 | 错误音、保持 | SWAP_ERROR |
| SWAP_ERROR | 5 s 无操作 | LP，错误保持 | SWAP_ERROR(LP) |
| REPLACE_READY | 下一次上电 | expected_swap 强制报错 | SWAP_ERROR |
| DEAD_STORAGE | 检测到充电器 | PB1 boot BQ、重配、双管开 | CHARGE |
| FAULT | 故障消失且条件安全 | 清 BQ 锁存、安全 mask | BOOT/IDLE |

### 5.1 充满判据（暂定）

- `SOC >= 98%` 或最低节电压 `>= 3.55 V`；
- 且充电电流降到 `<= 0.05C`（1500 mAh → 75 mA）持续 30 s；
- 充满只置 `charge_full` 并鸣叫，**不关闭 CHG**；
- BQ OV 3.70 V/cell 是最后硬件保护。

---

## 6. 按键手势

PB5 低有效、外部上拉、EXTI 下降沿。

| 手势 | 定义 |
|---|---|
| 消抖 | 电平稳定 20 ms（4 ms 采样 5 次） |
| 短按 | 按下 20–600 ms 后释放 |
| 长按 3 s | 持续 3000 ms，事件只发一次 |
| 长按 5 s | 持续 5000 ms，事件一次；第 5 s 立即提示音 |
| 换电确认 | 5 s 事件后 0–500 ms 内释放 |
| 三连击 | 从第一次按下开始 1000 ms 内完成 3 次短按 |
| 放电配对 | 第一次短按释放后 2000 ms 内第二次按下，并保持 3000 ms |

关键规则：

- LP 唤醒键算第一次短按；若唤醒后仍按住，按下时间从唤醒时刻开始；
- 唤醒后一直按到 3 s/5 s，发 HOLD3/HOLD5，不再补发短按；
- DISCHARGE_ARM 中禁止 HOLD5，避免误进换电模式；
- 5 s 换电只在 IDLE 有效；充电/放电/ARM/SWAP 不响应换电；
- 任何按键事件刷新 5 s 空闲计时；手势进行中不自动 LP；
- 三连击要等第三次释放后判定；包电压非法时只报错，不清状态。

---

## 7. LP、充电检测与储存监测

### 7.1 LP 类型

| LP 类型 | BQ 配置 | FET | WUT | 用途 |
|---|---|---|---|---|
| IDLE | NORMAL，ADC/TEMP 开，CC 关 | 全关 | 1 s | 充电检测、换电压差、30 min 监测 |
| CUTOFF | 同上 | 全关 | 1 s | 充电恢复检测 |
| SWAP_ERROR | 同上 | 全关 | 1 s | 按键唤醒/充电检测 |
| REPLACE_READY | 同上 | 全关 | 1 s | 等断电/按键取消 |
| DEAD_STORAGE | SHIP | 全关 | 20 s | 仅 RTC + COMP2 嗅探充电 |

说明：

- 若实测 LP 电流不达标，可把 IDLE 的 ADC/TEMP 改为“嗅探/30 min 时临时打开”；但 1 s 和 30 min 都不能切 HSE；
- LP 期间双管默认全关；需要保持导通时由调试变量显式指定；
- LP 进入前必须确认 BQ 空闲；BQ 准备失败不得进入 LP。

### 7.2 1 s 嗅探流程

1. RTC WUT 1 s 唤醒，MCU 保持 HSI16，不开 5 V、不切 HSE/PLL；
2. 用 HSI16 I2C 时序有界读 BQ `SYS_CTRL1`；
3. 检查 `LOAD_PRESENT`（该位只在 `SYS_CTRL2.CHG_ON=0` 时有效；LP 默认双管全关满足条件）：
   - `0`：读 `SYS_STAT`/`SYS_CTRL2` 做诊断，回 STOP1；
   - `1`：说明 CHG 引脚被外部拉高，继续复核；
4. 临时 `PA1=0` 使能 PACK_SC 前端，等约 30 ms；
5. `HAL_COMP_Start`，再等约 30 ms，读 COMP2：
   - 高 = 充电：完整唤醒，进入 CHARGE；
   - 低 = 负载/噪声：停 COMP2、PA1=1，回 STOP1；
6. 嗅探只使用有界阻塞 I2C（≤50 ms/次），不改 NORM 的 DMA 事务；
7. 1 s 嗅探不写 Flash、不启动 5 V/RGB/蜂鸣器；
8. 若台架发现 `LOAD_PRESENT` 对充电器不可靠，退化为每个 1 s 窗口都做 PA1/COMP2 嗅探；
9. 每次嗅探后必须重新确认 WUT 使能，禁止在无唤醒源状态下进入 STOP1。

### 7.3 30 min 储存监测

- 每 30 min 由 LP 服务执行；
- 在 HSI16 下临时打开 BQ ADC 和 CC：
  - 等约 250–300 ms；
  - 读三节电压、BAT、TS1、CC；
  - 更新 SOC/SOH（M3 后）；
  - 写 StateStore 和一条黑匣子周期记录；
  - 关闭 CC，恢复 LP 寄存器态；
- 不在 1 s 嗅探中写 Flash；
- 黑匣子页擦除只在 30 min 记录跨页时发生；
- SOC 只在新 BQ 快照时间戳变化时更新，避免同一帧重复积分；
- 30 min 监测期间若按键/ALERT/充电器到来，立即中止监测走正常唤醒。

### 7.4 空闲 5 s 回 LP

- IDLE 每 4 ms/每圈检查 `last_activity`；
- 按键、充电、放电启动/停止、状态变化都刷新活动时间；
- 无充电、无放电、无手势、无故障，5 s 后请求 LP；
- DISCHARGE_ARM、5 s 长按过程中不超时；
- CHARGE/DISCHARGE 不自动 LP；
- CUTOFF/SWAP_ERROR 显示 5 s 后进入对应 LP，错误状态保持；
- 充电器移除后 5 s 无操作 → IDLE，再入 LP。

### 7.5 死存储

- 进入条件：SOC==0（库输出钳位）且最低节 ≤2.5 V，或包压 ≤7.5 V，持续 10 s；
- 动作：双管关、均衡关、5 V 关、LOG/RGB/蜂鸣器停；
- 写 `dead_storage` 到 StateStore；
- BQ 写 SHIP：`SYS_CTRL1=0x01` 后再写 `0x02`；
- MCU STOP1 + RTC 20 s；
- 每 20 s：PA1 短时拉低 + COMP2 嗅探充电；无充电回 STOP1；
- 有充电：PB1 高 ≥2 ms 启动 BQ，重配，清 dead，转 CHARGE；
- 3.3 V LDO 仍由电池供电，死存储不是整板零电流，需实测 LDO/分压静态。

### 7.6 LP 调试

- 调试开关 `POWER_LP_KEEP_SWD` 的默认值为 **1**：STOP1 期间置 `DBGMCU_CR_DBG_STOP`，SWD 保持可连；
- 量产低功耗构建把开关设为 0，关闭 STOP1 调试时钟；
- 调试开关只影响调试，不改变唤醒源、BQ 配置或 FET 策略；
- LP 中调试可见：`s_power.state/lp_stage/wake_cause/wake_count`，`s_lp_sniff.*`，BQ 快照，StateStore 视图。

### 7.7 BQ 配置与数据有效性

#### 3S 电压映射

- CELL1 = VC1 − VC0；
- CELL2 = VC2 − VC1；
- CELL3 = VC5 − VC4；
- VC4−VC3、VC3−VC2 必须短接；
- BAT 为电池组总压。

电压有效范围：

| 量 | 有效范围 | 说明 |
|---|---|---|
| 单节 | 1.0–5.0 V | OV 3.70 V 仍在范围内 |
| BAT | 3–15 V | 3S 包压通常 6–11 V |
| 控制用 cell3/BAT | 需与 VC1/VC2 估计一致 | 不一致时不参与控制/保护 |

#### 上电配置

1. PB1 高脉冲 ≥2 ms 退出 SHIP，随后回低并等待 ≥20 ms；
2. 写 `CC_CFG=0x19`；
3. 读 `ADCGAIN1/ADCGAIN2/ADCOFFSET`，换算增益/偏移；
4. 写 `SYS_CTRL1=0x18`（ADC_EN|TEMP_SEL），清 `SYS_STAT`；
5. 写 `SYS_CTRL2=0x40`（CC_EN，FET 全关）；
6. 写保护：
   - `PROTECT1=0x89`（RSNS=1，SCD 67 mV，约 44.7 A/100 µs）；
   - `PROTECT2=0x57`（OCD 56 mV，约 37.3 A/320 ms，放电方向）；
   - `PROTECT3=0x40`（OV delay 1 s，UV delay 4 s）；
   - `OV_TRIP/UV_TRIP` 按 3.70 V/2.50 V 和 GAIN/OFFSET 反算；
7. 每项写后读回；失败重试配置，连续失败置 `FAULT_BQ_COMM`；
8. ADC 开启后等待 ≥300 ms 再取第一帧；
9. 只清 `CC_READY/XREADY/OVRD_ALERT`（0xB0），不清 OV/UV/SCD/OCD 保护锁存。

#### 数据有效性

- `fresh=0` 的帧不更新 SOC/StateStore/状态机；
- 单节超出 1.0–5.0 V、BAT 超出 3–15 V、CRC 错误、I2C 超时均判无效；
- 连续 8 个无效帧请求一次受控 ADC 重启；
- 每个上电周期最多 2 次 ADC 重启；每次重启有 500 ms 判定窗；
- 两次失败置 `FAULT_BQ_ADC_STUCK`，停止自动重启，仅保留采样；
- 连续 4 个有效发布帧后清 ADC 故障；
- 启动/LP 唤醒/ADC 重启后的前两帧要求相邻一致（单节 ≤±4 raw LSB，BAT ≤±6 mV），否则退回粗确认，避免把真实电压松弛误判为 ADC 故障；
- 运行期重新确认可用 250 mV（单节）/300 mV（BAT）粗阈值，避免负载跳变导致数据长期不发布；
- OV/UV 等保护只从有效帧映射；虚假 OV/UV 只能在安全电压和连续好帧后清除，真实 OV/UV 必须保持锁存。

#### CC 换算

- BQ CC 1 LSB = 8.44 µV；
- 分流 1.5 mΩ；
- `current_ma = cc_raw × 844 / 150`（250 ms 平均）；
- 电流方向在台架上用充电/放电各一次确认，并在 SOC 适配层统一；
- CC 零点/增益标定在 M0/M3 完成，未标定前记录原始 `cc_raw`。

---

## 8. 充电、放电、均衡与保护

### 8.1 充电

- 进入 CHARGE：DSG=1、CHG=1，双管全开，CC/ADC 开启；
- 充电开始：3.0 kHz × 150 ms 一声；
- 采样：BQ 250 ms，SOC 500 ms；均衡 5 s 复评；
- RGB：SOC 绿→黄→红渐变 + 快闪；
- 充满：4 kHz × 2 短鸣，保持双管开，不主动断电；
- 充电器移除：双管关，清 charge_full，5 s 后 IDLE/LP；
- 充电中按键只唤醒/显示，不启动放电；长按不进入换电；
- 充电最大 5 A：软件充电 OCP 5.5 A/1 s 作为唯一充电过流保护（BQ 无充电方向 OCP）；
- 充电温度 0–45 °C，超限停充；2 °C 回差、2 s 驻留。

### 8.2 放电

- 激活：短按唤醒/IDLE → 2 s 内第二次按下并保持 3 s；
- 动作：DSG=1、CHG=1 双管全开；蜂鸣器低-中-高（2.0/3.0/4.0 kHz × 3）；
- RGB 同充电渐变快闪；
- 采样/BQ/SOC/均衡同充电；
- 退出：同样短按 + 2 s 内二次长按 3 s → 双管关；蜂鸣器高-中-低（4.0/3.0/2.0 kHz × 3）；5 s 后 LP；
- 5% 切断：SOC ≤5% 连续 2 帧，或最低节 ≤2.8 V 兜底；
  - 3 声长鸣（2.0 kHz/500 ms，间隔 200 ms）；
  - 双管关，写 `cutoff_latch`，进 LP；
  - 禁止放电直到充电且 SOC>10%；
- 充电器优先：放电中插充电器立即保持双管开、转 CHARGE；拔掉后需重新手势才能放电；
- 放电最大 30 A：软件放电 OCP 33 A/500 ms；BQ 硬件 OCD 设在 30 A 以上。

### 8.3 均衡

- 只在 CHARGE/DISCHARGE；
- 选择最高节；`max(cell) ≥3.35 V`、`max-min ≥30 mV` 时开对应 `CELLBAL1` 位；
- 一次只开一节，30 s 后重新评估；单节轮换，不同时开三节；
- BQ die 温是主判据：
  - `TEMP_SEL=0` 读内部 die 温度；
  - 建议每 10 s 切到 die 温一次，测完恢复外部 NTC；
  - die 温度 >65 °C 立即清 `CELLBAL1=0`，回差 5 °C；
  - die 温读数无效/超时按超限处理；
- 低于 3.0 V、外部 NTC 超限、故障、SWAP/CUTOFF/DEAD/LP 时清均衡；
- BQ 在 DEVICE_XREADY/SHIP→NORMAL 后会自动清均衡位，恢复后必须重写；
- 均衡期间电压读数可能偏高，SOC 使用未均衡前快照或做补偿；
- 均衡电流约 30 mA（100 Ω + 内部 FET），需实测温升。

### 8.4 电流/短路保护

| 项目 | 设计值 | 说明 |
|---|---|---|
| 分流电阻 | 1.5 mΩ | 30 A 时约 45 mV，耗散约 1.35 W |
| BQ OCD | RSNS=1，56 mV / 320 ms（≈37 A） | 放电方向；必须高于 30 A 正常电流 |
| BQ SCD | RSNS=1，67 mV / 100–200 µs（≈44.7 A） | 硬件短路保护；触发后双管关、FAULT、锁存 |
| 软件放电 OCP | 33 A / 500 ms | 后备，按负载/电芯最终标定 |
| 软件充电 OCP | 5.5 A / 1 s | BQ 无充电方向 OCP |
| 最大放电 | 30 A | 连续能力、FET/分流温升需实测 |
| 最大充电 | 5 A | 充电器 CV/CC 与电芯标定 |

- BQ SCD 是 µs 级硬件比较器；CC 250 ms 平均不能替代短路保护；
- SCD 触发后固件读 SYS_STAT，双管关，进入 FAULT 并锁存，不自动重合闸；
- 软件 OCP/SCD 恢复必须由主机显式清 `SYS_STAT` 并按安全条件重开 FET；
- 外部保险丝仍作为后备。

### 8.5 保护阈值（暂定，M0/M3 标定后固化）

| 项目 | 暂定值 |
|---|---|
| 单节满充 | 3.55 V + taper 75 mA/30 s，或 SOC≥98% |
| BQ OV | 3.70 V/cell，delay 1 s |
| BQ UV | 2.50 V/cell，delay 4 s |
| 包合法范围 | 6.0–11.0 V；单节 2.0–3.8 V |
| 充电温度 | 0–45 °C |
| 放电温度 | −20–60 °C |
| 均衡 die 温 | 65 °C，5 °C 回差 |
| 5% 放电切断 | SOC≤5% + 最低节≤2.8 V 兜底 |
| 0% 死存储 | SOC=0 + 最低节≤2.5 V，持续 10 s |

> 注意：`SOC_EST_CAL_VALID=0` 时，5%/0% 保护必须同时使用电压兜底；SOC 不能单独作为硬切断依据。

### 8.6 BQ SHIP / 恢复

- SHIP 入口：双管已关，写 `SYS_CTRL1=0x01`，再写 `0x02`；中间不得插其他 SYS_CTRL1 写；
- CRC/超时失败重试；成功后 I2C 可能不再响应；
- 退出：PB1 高 ≥2 ms → 低 → 等 20 ms → 完整 BQ 配置；
- PB1 之后必须保持低，避免短接 NTC；
- 外部上拉已确认；REGSRC 已确认接 12 V；
- SHIP 不会切断 MCU 3.3 V，需实测整板静态电流。

### 8.7 故障管理与安全状态

故障分为 live（实时）和 sticky（锁存）：

| 类别 | 例子 | 动作 |
|---|---|---|
| BQ 通信 | CRC 错、I2C 超时、配置失败 | `FAULT_BQ_COMM`，重试/总线恢复；持续失败→FAULT |
| BQ 器件 | DEVICE_XREADY | 双管关、FAULT、等待安全恢复 |
| BQ 保护 | OV/UV/SCD/OCD | 双管关；软件监督清锁存后才允许重开 |
| ADC/数据 | 连续无效、ADC 重启失败 | 数据不发布；锁存 ADC 故障 |
| 温度 | NTC 无效/超限 | 禁止充电或放电，2 °C 回差 |
| 电源 | PVD 跌落 | sticky `FAULT_POWER`；恢复后清 sticky |
| 时钟 | HSE/PLL 切换失败 | 降级只闪故障灯，禁止充放电 |
| RTC/LSE | LSE 未起振/WUT 布防失败 | 禁止自动 LP/30 min 存档 |
| Flash | 黑匣子/StateStore 写失败 | 置故障位，禁止继续依赖该存储 |
| 软件 OCP | 充电 5.5 A/1 s、放电 33 A/500 ms | 双管关、硬件锁存；需显式清 |
| 硬件 SCD | 短路尖峰 | 双管关、FAULT 锁存、不自动重合闸 |

BQ ALERT 语义：

- ALERT = `SYS_STAT` 所有位的 OR；
- 清 ALERT 必须写 1 清除对应源位，不能只清 EXTI；
- PA8 是电平/事件输入，若进入 LP 前 ALERT 已为高，不会产生新的上升沿；需要读引脚电平或在嗅探中处理；
- 保护锁存（OV/UV/SCD/OCD）不能为了“消 ALERT”而随手清掉。

统一原则：

- 任何关键故障 → 双管关、均衡关、充电/放电状态退出；
- FAULT 状态不自动进入 LP，保留 5 V/灯/蜂鸣器用于提示；
- 关键故障消失且条件安全后，才允许清 BQ 锁存和恢复 FET；
- 不允许在故障状态下依赖体二极管继续导通；
- 故障事件写黑匣子；故障位图可按需要上报；
- L1 需要评估 IWDG：NORM 喂狗，LP 进入前处理；若无法在 LP 保持看门狗，则至少在 NORM 提供死机保护；
- HardFault/NMI 必须有确定的复位/安全状态，不允许无提示自旋。

---

## 9. SOC/SOH 工具包集成

### 9.1 文件

复制 `li-ion-soc-soh-toolkit/算法库` 核心到 `App/detect/bms/`：

```text
bms_algo.h  bms_config.h  bms_port.h/.c
bms_cal.h/.c  soc.h/.c  soh.h/.c  bms_nvm.h/.c
```

不复制：

- `移植示例/` 多个文件定义同一函数，会链接冲突；
- `bms_tune.*`；
- Python 标定脚本；
- Windows GUI；
- 任何 `app.mk` 之外的测试工程。

### 9.2 配置

- `BMS_USE_TICK=1`
- `BMS_USE_LIBM_EXPF=0`
- `BMS_USE_NVM=1`
- `BMS_USE_TUNE=0`
- `SOC_CAPACITY_NOMINAL_MAH=1500`
- `BMS_REF_CUR_MA=300`（0.2C）
- `BMS_UPDATE_PERIOD_MS=500`
- `SOC_QUIET_DEBOUNCE=2`
- `SOH_IDLE_DEBOUNCE=2`
- `SOH_R0_TAIL_N=2`

### 9.3 适配层

`SocEst_Init()`：

```text
BMS_CAL_Init → SOH_Init → BMS_NVM_Init
```

`SocEst_OnSnapshot()`：

- 只在新 BQ 快照（`last_update_ms` 变化）时调用；
- 输入 `v_mv = min(cell_mv[0..2])`，不能传 pack_mv；
- 输入 `i_ma = current_ma`；电流符号在台架核对一次；
- 温度由 TS1 NTC 换算；无效时标记温度无效，不带温度修正；
- 顺序：`BMS_CAL_Update(temp_dc)` → `SOC_AhUpdate(v_mv, i_ma)` → `SOH_Update(v_mv, i_ma, temp_dc)`；
- 提供 `SocEst_GetPercent01()`、`SocEst_IsValid()`、`SocEst_ResetForNewPack()`、`SocEst_Task()`。

### 9.4 标定门槛

- 当前 LFP OCV/R0/R1/τ 为占位值，必须用 1500 mAh 实测表替换；
- 标定完成前 `SOC_EST_CAL_VALID=0`，SOC 只显示/记录，不单独触发 5%/0% 硬保护；
- 5%/0% 使用 SOC + 最低节电压双条件；
- BQ CC 零点/增益、TS1 NTC 参数、均衡补偿在 M0/M3 完成。

### 9.5 BMS NVM

- 132 B 记录不是 8 B 倍数，不能直接按 132 B 双字写；
- 132 B 拷入 136 B 缓冲，尾部 0xFF，再按双字写；
- 两页槽（页 48/49），擦页后写；读仍读 132 B；
- 写失败保留旧槽；
- 只在主循环 NORM 低频调用；LP 进入前若 dirty，先保存。

---

## 10. Flash、持久化、换电与黑匣子

### 10.1 Flash 分区

| 页 | 地址 | 用途 |
|---:|---|---|
| 0–47 | `0x08000000–0x08017FFF` | 应用代码，96 KB |
| 48–49 | `0x08018000–0x08018FFF` | BMS SOC/SOH NVM |
| 50–51 | `0x08019000–0x08019FFF` | L1 StateStore |
| 52–63 | `0x0801A000–0x0801FFFF` | 黑匣子 |

- 链接脚本 `FLASH LENGTH=96K`；
- `_image_end` 必须 <0x08018000；
- 黑匣子必须 <0x0801A000；
- CubeMX 重生成后恢复 96K、`-include app.mk` 和链接断言。

### 10.2 StateStore

48 B 记录：

```text
magic/version/seq, flags, last_cell_mv[3], last_pack_mv,
last_soc01, last_soh01, last_temp_dc, battery_epoch, crc16
```

flags：

- `expected_swap`
- `swap_error`
- `cutoff_latch`
- `dead_storage`
- `charge_full`
- `soc_valid`
- `pack_valid`

- 两页环形，跨页先擦；上电扫描最高有效 seq；
- 主循环写；
- 转换/30 min 监测时写；
- 1 s 嗅探不写；
- 换电清除时重置 `battery_epoch`。

### 10.3 换电检测

- 参考值：StateStore 上次稳定电芯/包压（30 min 或状态转换更新）；
- LP/唤醒时，BQ 新鲜且 `|I|<50 mA`，连续 2 帧满足任一：
  - 任一节与参考差 >300 mV；
  - 包压差 >600 mV；
  - 三节最大差 >600 mV；
  - 包压超出 6.0–11.0 V；
- 立即 `SWAP_ERROR`，双管关，写状态；
- 整板断电后上电，用 StateStore 最后记录与首批 BQ 比较；
- `expected_swap` 无条件报错；
- 局限：无电池 ID/EEPROM，无法区分电压几乎相同的不同电池；若要求 100% 识别需增加 ID 触点。

### 10.4 换电清除

- `SWAP_ERROR` 中 1 s 内三次短按，且包合法；
- 擦 BMS NVM 页 48/49；
- `SocEst_ResetForNewPack()`；
- 清 swap/expected/cutoff；
- 写新 `battery_epoch`；
- 转 IDLE。

### 10.5 换电模式（长按 5 s）

- 仅 IDLE 有效；
- 按住 5 s：第 5 s 蜂鸣器 1 声；
- 5.0–5.5 s 内释放：写 `expected_swap`，蜂鸣器高-低-高-低，双管关，LP；
- 超过 5.5 s 释放：取消，不进入换电模式；
- 下一次上电必报 `SWAP_ERROR`；
- 未真正断电时，可用三次短按取消 `expected_swap`（需文档化）。

### 10.6 黑匣子

- 保留 24 B 周期记录：seq/uptime/三节/pack/CC/TS1/flags/CRC；
- 事件类型：
  - 充电开始/充满；
  - 放电开始/停止；
  - 5% 切断；
  - 换电报错/换电模式；
  - 死存储；
  - LP 进入/唤醒；
  - 保护触发/清除；
- 30 min 一条周期记录；状态转换一条；不在 1 s 嗅探写；
- 1020 条容量需覆盖 ≥15 天；如果增加事件记录，必须重新核算写频率；
- 提供 `ReadNewest/ReadSlot/GetHead` 供台架和启动诊断；
- `POWER_WAKE_*` 与 `BB_WAKE_*` 必须统一映射，避免 RTC+BUTTON 被记成 BQ ALERT。

---

## 11. LED 与蜂鸣器

### 11.1 RGB / PB3/PB4

| 状态/事件 | RGB | PB3/PB4 |
|---|---|---|
| BOOT | 蓝慢闪 | PB4 慢闪 |
| IDLE | 暗白/按最终 UI | PB4 常亮 |
| CHARGE | SOC 绿→黄→红，快闪 | PB4 常亮 |
| DISCHARGE | 同 CHARGE | PB4 常亮 |
| CUTOFF | 红快闪 | PB3 快闪 |
| SWAP_ERROR | 红/蓝交替快闪 | PB3 快闪 |
| REPLACE_READY | 琥珀/蓝慢闪 | PB4 |
| DEAD_STORAGE | 灭 | 灭 |
| FAULT | 红 5 Hz | PB3 5 Hz |
| LP | 灭 | 灭 |

- SOC 100% 绿，50% 黄，0% 红；
- 充电/放电共用 SOC 渐变快闪；
- 充满显示满电绿，不断电；
- RGB 只在 5 V/80 MHz NORM 可用；LP 全关。

### 11.2 蜂鸣器

| 事件 | 音型 |
|---|---|
| 充电开始 | 3.0 kHz ×1，150 ms |
| 充满 | 4.0 kHz ×2 短 |
| 放电激活 | 2.0→3.0→4.0 kHz，各 200 ms |
| 放电退出 | 4.0→3.0→2.0 kHz，各 200 ms |
| SOC<5% | 2.0 kHz 长鸣 ×3（500 ms 响/200 ms 停） |
| 长按 5 s | 3.0 kHz ×1，150 ms |
| 换电模式 | 4.0/2.0/4.0/2.0 kHz，各 150 ms |
| 换电报错 | FAULT 长鸣 ×3 |

优先级：故障/低 SOC/换电错误 > 换电模式 > 充放电转换 > 启动/唤醒。

---

## 12. 调度与时序

| 任务 | 周期 | 上下文 |
|---|---:|---|
| TIM2 tick | 4 ms | ISR：COMP2 采样、灯相位、秒计数、置标志 |
| 按键采样/消抖 | 4 ms | 主循环 |
| BQ 读 | 充电/放电 250 ms；IDLE 500 ms | 主循环 DMA 状态机 |
| SOC/SOH | 500 ms（新快照） | 主循环 |
| 均衡复评 | 5 s | 主循环 |
| 空闲 → LP | 5 s | 主循环 |
| LP 嗅探 | 1 s | RTC WUT + HSI16 |
| 30 min 监测 | 30 min | HSI16 |
| BMS NVM | 1 s 轮询 + 60 s 去抖 | 主循环 |
| 黑匣子 | 30 min + 状态转换 | 主循环 |

主循环顺序：

1. `PowerMode_Loop()` / LP 服务；
2. `Button_Poll()` → `Gesture_Poll()`；
3. BQ 调度 + `BQ_Poll()`；
4. `SocEst_OnSnapshot()`；
5. `ShelfMonitor_Task()`（换电/死存储/存档）；
6. `StateMachine_Task()`（FET/均衡/LED/蜂鸣器/LP 请求）；
7. `Buzzer_Poll()` → `LedStatus_Flush()`；
8. StateStore/BMS NVM/黑匣子低频任务；
9. `App_Log_Poll()`。

LP 电流目标：

| 场景 | 目标 |
|---|---|
| IDLE LP（1 s 嗅探） | <100 µA 平均 |
| DEAD（SHIP + STOP1） | <10 µA（需实测 LDO/分压） |
| 30 min 监测平均增量 | 可忽略 |
| 完整唤醒 | HSE 30 ms + BQ 配置 250 ms + 5 V 软启动 |

---

## 13. 文件与 API 规划

### 13.1 新增

- `App/bsp/button.c/.h`
- `App/detect/gesture.c/.h`
- `App/detect/soc_est.c/.h`
- `App/detect/bms/*`（toolkit 核心）
- `App/detect/bms_port_stm32.c`
- `App/detect/bms_nvm_port.c`
- `App/app/state_store.c/.h`
- `App/app/shelf_monitor.c/.h`
- `App/app/charge_policy.c/.h`
- `App/app/cal_stream.c/.h`

### 13.2 修改

- `App/app/state_machine.c/.h`：L1 状态、事件、FET/均衡/LED/蜂鸣器决策；
- `App/app/power_mode.c/.h`：LP 类型、嗅探、HSI16 I2C、DEAD/SHIP、唤醒延迟；
- `App/app/main_app.c/.h`：初始化顺序、主循环、BQ 速率、按钮/SOC/StateStore 调度；
- `App/bsp/bq_ic.c/.h`：FET mask、均衡、SHIP、保护、温度源、LP 嗅探；
- `App/app/led_status.c/.h`：SOC 渐变、快闪、L1 状态；
- `App/bsp/buzzer.c/.h`：新音型/优先级；
- `App/app/black_box.c/.h`：事件类型、读取 API、wake 映射；
- `App/board.h`、`App/app/faults.h`：Flash/温度/换电/死存储常量；
- `app.mk`、`STM32L432xx_FLASH.ld`、`compile_commands.json`；
- `DOCS/POWER_MODE.md`、`DOCS/PINMAP.md`、`CLAUDE.md`。

### 13.3 关键 API（规划名）

```c
/* BQ */
void BQ_RequestFetMask(uint8_t mask);
void BQ_ApplyFetMaskNow(uint8_t mask);
int  BQ_SetBalanceMask(uint8_t mask);
int  BQ_ShipMode(void);
void BQ_ClearProtection(uint8_t sys_stat_mask);
int  BQ_ReadDieTempDeciC(int16_t *temp_dc);
int  BQ_LpSniffRead(uint8_t *sys_ctrl1, bq_snapshot_t *snap);

/* Button / Gesture */
void Button_Init(void);
void Button_Poll(uint32_t now_ms);
void Button_NotifyExtiFromIsr(void);
uint8_t Gesture_GetEvent(gesture_event_t *ev);

/* SOC */
void SocEst_Init(void);
void SocEst_OnSnapshot(const bq_snapshot_t *s, uint32_t now_ms);
void SocEst_ResetForNewPack(void);
void SocEst_Task(void);

/* State */
void StateStore_Init(void);
const state_store_view_t *StateStore_Get(void);
void StateStore_Commit(const state_store_view_t *st);
void StateStore_ClearLatch(uint16_t flags);

/* Power */
void PowerMode_RequestLp(void);
void PowerMode_ServiceLp(void);
uint8_t PowerMode_GetWakeEvents(void);
```

---

## 14. 分阶段实施计划

### M0 — 硬件/标定冻结（1–2 周）

目标：把硬件风险和标定输入关闭，避免在错误参数上写策略。

工作：

- 确认 3S LiFePO4 1500 mAh、充电器 CV/CC、NTC BOM、FET/分流功率余量；
- **30 A/5 A 连续温升**：1.5 mΩ 分流约 1.35 W，Q1/Q2/Q3/PCB 温升；
- 外部上拉确认；REGSRC→VBAT_12V 飞线确认；
- **HSI16 下 BQ I2C 实测**：SCL/SDA 波形、CRC 成功率、1 s/30 min 可用性；失败先修 HSI16 时序，不能改用 HSE 做储存监测；
- BQ die 温度读取（`TEMP_SEL=0`、约 2 s 调度延迟、TS1 换算）和 65 °C 阈值；
- BQ SCD 67 mV/100–200 µs（RSNS=1）与 30 A 正常电流不误触发；
- 采集 LFP OCV/R0/R1/τ、BQ CC 零点/增益、TS1 温度表；
- 确认 5%/0% 电压、包合法范围、充电器插入时序；
- 输出硬件风险清单、测试记录、标定 CSV。

退出：30 A/5 A/SCD/die 温/HSI16 I2C 通过，标定 CSV 就绪。

### M1 — 安全基础（1–2 周）

- 96K 链接、`_image_end` guard、Flash 分区；
- StateStore（页 50–51）、BMS NVM（页 48–49）；
- BQ FET/均衡/SHIP/保护/温度源 API；
- Button/Gesture 基础；
- 黑匣子读取/事件类型；
- 不改变 L0 日常行为；验证构建、BQ 寄存器读回、FET 真值表、掉电状态恢复。

### M2 — LP 与充电检测（1–2 周）

分为两个可验证小步：

**M2a — LOAD_PRESENT 诊断**

- BQ 配置结束为 CC 开、双管关；
- LP 1 s 嗅探读 `SYS_CTRL1.LOAD_PRESENT`；
- 增加 `load_present_count/last_load_present` 等诊断；
- 验证 FET 全关时 LOAD_PRESENT 对充电器/负载的行为；
- 不启用自动充电唤醒、不自动 LP。

**M2b — 自动 LP + 充电唤醒 + 30 min 监测**

- NORM 5 s 空闲自动 LP；
- LP 1 s 嗅探 + `LOAD_PRESENT` → PA1/COMP2 复核；
- 充电器完整唤醒，PB5/ALERT 保留；
- 30 min HSI16 监测；
- 5 s 空闲计时、充电中禁止 LP；
- 验证充电插入延迟、纯负载不误唤醒、LP 电流、无 Flash 写入。

退出：充电器插入 ≤1.5 s 唤醒；5 s 回 LP；LP 电流达标；30 min 记录正确。

### M3 — SOC/SOH（2–3 周）

- toolkit 核心移植、LFP 占位表、min-cell 输入、NTC 温度；
- 500 ms SOC 更新、NVM pad、掉电 SOC 恢复；
- 换电重置；
- PC 工具回放/离线对比；
- 标定后替换 LFP OCV/R0/R1/τ，打开 `SOC_EST_CAL_VALID`。

退出：0.2C/0.5C/1C 充放电 SOC 单调、5%/10%/0% 阈值、SOH 有效性、掉电恢复。

### M4 — 充放电策略（1–2 周）

- CHARGE/FULL；
- DISCHARGE_ARM/DISCHARGE；
- 按键手势、FET 双管全开、均衡；
- 5% cutoff/10% 恢复；
- RGB/蜂鸣器；
- 30 A/5 A 软件 OCP、BQ OCD/SCD 联合；
- BQ die 温均衡；
- 充电器优先、充电器移除、放电退出。

退出：双管栅极波形、30 A/5 A 温升、5%/10%、充放电转换全部通过。

### M5 — 换电与死存储（1–2 周）

- 换电电压启发式、expected_swap、三连击清除；
- 5 s 换电模式；
- DEAD/SHIP、充电恢复；
- 换电/整板断电/同电压电池/非法包压测试；
- StateStore/黑匣子 500 次写入/掉电中断测试。

退出：换电错误可复现/可清除；SHIP 电流、PB1 boot、充电恢复正确。

### M6 — 系统验证（1–2 周）

- 故障注入：I2C 掉线、OV/UV/OCD/SCD、PVD、HSE、WUT、5 V、按键抖动；
- **30 A 放电、5 A 充电、短路尖峰、双管全开**复测；
- BQ die 温 65 °C 停均衡；
- HSI16 I2C 1 s/30 min 复测；
- LP 电流、充放电温升、EMC 初测、长期循环；
- 状态转换覆盖表、测试记录、最终参数表。

退出：全项验收通过，遗留风险关闭或记录。

---

## 15. 验证与验收

### 15.1 构建/静态

- `make clean && make -j2 all` 零警告零错误；
- `_image_end < 0x08018000`；
- 黑匣子/StateStore/NVM 地址不重叠；
- `app.mk` 包含所有新源；`compile_commands.json` 同步；
- CubeMX 重生成后恢复 96K、`-include app.mk`、RTC 选择补丁；
- 运行 toolkit C/PC 常量一致性检查。

### 15.2 按键/状态

- 20 ms 消抖、短按、3 s/5 s、0.5 s 释放窗口、2 s 配对、1 s 三连击；
- 每个状态转换至少注入一次；
- LP 唤醒键计入第一次短按；
- 手势进行中不自动 LP；充电中不换电。

### 15.3 LP/充电

- 断开调试器测 IDLE LP、DEAD SHIP 电流；
- HSI16 BQ I2C 连接/CRC 成功率；
- 充电器/电阻负载/空载矩阵验证 LOAD_PRESENT/COMP2；
- 1 s 嗅探不写 Flash、不产生 ALERT 风暴；
- 5 V 只在完整唤醒时打开；
- 充电插入延迟 ≤1.5 s。

### 15.4 充放电/保护

- 30 A/5 A 持续：DSG/CHG 均高、Vgs 足够、体二极管不承担电流、温升；
- SCD 67 mV/100–200 µs：30 A 不误触发，短路立即双管关；
- OCD 56 mV/320 ms 与软件 OCP 配合；
- 5%/0% 切断/锁存/10% 恢复；
- 均衡 die 温 65 °C 停、外部 NTC 冻结/恢复；
- 30 A 分流耗散/R2/R26/Q1-Q3 温升。

### 15.5 SOC/SOH

- 0.2C/0.5C/1C 充放电记录 V/I/T；
- 离线对比 SOC、5%/10%/0%、OCV 静置重同步、温度修正、SOH；
- CC 零点/增益标定；
- 掉电 SOC 恢复、换电重置。

### 15.6 换电/死存储

- 长按 5 s 提示音、0.5 s 释放窗口；
- expected_swap 后上电必报错；
- 三次短按（含唤醒短按）清除；
- 热插拔/整板断电/同电压不同电池/非法包压/单节开路；
- 500 次 StateStore 写入/掉电中断；
- SHIP 电流、PB1 时序、充电恢复。

### 15.7 保护/故障

- BQ OV/UV/OCD/SCD、锁存清除、FET 安全 mask、软件 OCP；
- I2C 超时/CRC、HSE 失败、WUT 失败、PVD、5 V 掉电、按键卡死；
- 拔电池/插充电器/组合按键；
- 确认两 FET 不会非预期同时导通。

---

## 16. 风险与待办

1. **LFP 标定是硬阻塞**：当前只有占位表；M3 前 SOC 不能作为 5%/0% 唯一依据。
2. **1 s 充电检测不是“立即”**：约 1–1.5 s；若产品要求 <100 ms，需要改板拆分 PA1/HSE_DIS 与 CHGSC_DIS，用 COMP2/EXTI 直接唤醒。
3. **LOAD_PRESENT 需台架确认**：负载/充电器都可能拉高 PACK−；必须 COMP2 复核，否则误唤醒。
4. **REGSRC 必须接 12 V**：否则 REGOUT、栅极驱动、SHIP 恢复不可信。
5. **BQ SHIP 不切 3.3 V**：DEAD 是 SHIP + MCU STOP1，不是整板零功耗；需实测 LDO/分压。
6. **30 A/5 A 热与短路**：1.5 mΩ 30 A≈45 mV、1.35 W；OCD/SCD 必须高于 30 A；分流/FET/PCB 温升需实测。
7. **无电池 ID**：电压启发式无法识别电压几乎相同的不同电池；100% 识别需增加 ID 触点。
8. **Flash 磨损**：1 s 嗅探绝不写 Flash；StateStore/黑匣子只在状态/30 min 写；核算页寿命。
9. **看门狗**：L1 需要 IWDG 或等效死机保护；LP 周期与喂狗策略必须单独设计。
10. **5 V 唤醒延迟**：当前规划 2 ms 偏短，建议实测后改为 20 ms 或按负载调整。
11. **均衡热量/电压读数**：100 Ω + 内部 FET 约 30 mA；单节轮换测温升；SOC 用未均衡快照。
12. **SOC<0 语义**：toolkit 输出钳位 0..100；DEAD 用 SOC=0 + 最低节≤2.5 V + 10 s 定义；必要时适配层维护原始积分余量。
13. **HSI16 BQ I2C**：1 s/30 min 都依赖；M0 必须实测；失败先修时序，不把储存监测切 HSE。
14. **双管全开/30 A**：必须 DSG=1/CHG=1；VGS/温升/体二极管不导通/老化影响需记录。
15. **BQ die 温与外部 NTC 冲突**：`TEMP_SEL=0` 才能读 die 温，切换约 2 s；均衡期间外部 NTC 短暂冻结，充放电高温保护要用最后有效 NTC 兜底。
16. **BQ SCD 恢复**：SCD 后必须主机清 SYS_STAT、双管关、FAULT 锁存，不自动重合闸；软件 OCP 只作后备。
17. **BQ 无充电 OCP**：充电 5 A 上限只能靠软件 OCP + 充电器自身限流。
18. **CAN 未实现**：L1 不实现 CAN 业务；NORM 80 MHz 已满足后续需求，但不得因此把 1 s/30 min 监测切到 HSE。
19. **PACK_SC 分压/COMP2 只用于方向判定**：不能作为电压测量或主保护。
20. **SWD 与 LP 电流**：`POWER_LP_KEEP_SWD` 默认 1，调试期保持 SWD 会显著抬高 STOP1 电流；量产构建必须设为 0。

---

## 17. 交付物

- 本文档：L1 开发方案；
- 更新后的 `DOCS/POWER_MODE.md`、`DOCS/PINMAP.md`、`CLAUDE.md`；
- L1 源码、`app.mk`、链接脚本、`compile_commands.json`；
- LFP 标定表/CSV、SOC 对比报告、BQ CC 零点/增益记录；
- 状态转换表、按键时序记录、LP 电流记录；
- 30 A/5 A/SCD/die 温测试记录；
- 换电/死存储测试记录、故障注入记录；
- 最终验收清单与遗留风险清单。

---

## 18. 完成定义

L1 完成需同时满足：

1. 空闲 5 s 自动 LP；充电器插入 1 s 级唤醒并进入 CHARGE；
2. 放电必须手势激活，双管全开；退出/5% 切断/10% 恢复正确；
3. 充电充满只鸣叫不断电；充电器移除回 IDLE/LP；
4. SOC/SOH 在标定后可用于显示与辅助策略；5%/0% 有电压兜底；
5. 均衡按单节轮换，die 温 65 °C 停止；
6. 30 min 储存监测/存档在 HSI16 下运行，不切 HSE；
7. 换电/换电模式/三连击清除完整；
8. 死存储 BQ SHIP + MCU STOP1 + 充电恢复完整；
9. 30 A/5 A/SCD/双管全开/温升通过；
10. 所有故障路径最终回到双管全关的安全状态。

---

## 附录 A：关键默认参数速查

| 参数 | 默认值 | 备注 |
|---|---|---|
| 电池 | 3S LiFePO4，1500 mAh | 标定前 SOC 仅显示 |
| 放电最大 | 30 A | 双管全开 |
| 充电最大 | 5 A | 软件 OCP |
| 分流 | 1.5 mΩ | 30 A ≈45 mV/1.35 W |
| BQ OV | 3.70 V/cell，1 s | 最终按电芯标定 |
| BQ UV | 2.50 V/cell，4 s | 最终按电芯标定 |
| BQ OCD | RSNS=1，56 mV/320 ms ≈37 A | 放电方向 |
| BQ SCD | RSNS=1，67 mV/100–200 µs ≈44.7 A | 硬件短路 |
| 软件充电 OCP | 5.5 A/1 s | BQ 无充电 OCP |
| 软件放电 OCP | 33 A/500 ms | 后备 |
| 充电温度 | 0–45 °C | 2 °C 回差 |
| 放电温度 | −20–60 °C | 2 °C 回差 |
| 均衡 | max≥3.35 V，diff≥30 mV | 单节 30 s 轮换 |
| 均衡 die 温 | 65 °C，5 °C 回差 | 超限清 CELLBAL1 |
| 放电 5% | SOC≤5% + 最低节≤2.8 V 兜底 | 2 帧确认 |
| 10% 恢复 | 充电 SOC>10% | 清 cutoff |
| 0% 死存储 | SOC=0 + 最低节≤2.5 V，10 s | BQ SHIP |
| LP 嗅探 | 1 s | HSI16 |
| 储存监测 | 30 min | HSI16 |
| 空闲 LP | 5 s | 无充放电/无按键 |
| WUT | 1 s，自由运行 | WUTE+WUTIE 必须为 1 |
| 代码 Flash | 96 KB | `_image_end<0x08018000` |
| BMS NVM | 页 48–49 | 132/136 B |
| StateStore | 页 50–51 | 48 B |
| 黑匣子 | 页 52–63 | 24 B ×1020 |
| SWD in LP | `POWER_LP_KEEP_SWD=1` | 默认保持；量产设 0 |

---

## 附录 B：主要缩写

| 缩写 | 含义 |
|---|---|
| BQ | BQ7692003PWR AFE |
| NORM | 正常 80 MHz 模式 |
| LP | 低功耗 STOP1 模式 |
| WUT | RTC wake-up timer |
| FET mask | bit0 CHG、bit1 DSG |
| OV/UV | 过压/欠压 |
| OCD/SCD | 放电过流/放电短路 |
| OCP | 软件过流保护 |
| SOC/SOH | 荷电状态/健康状态 |
| StateStore | 跨掉电状态记录 |
| expected_swap | 已确认即将换电，下次上电报错 |
| cutoff_latch | 5% 放电锁存 |
| dead_storage | 0% 死存储 |

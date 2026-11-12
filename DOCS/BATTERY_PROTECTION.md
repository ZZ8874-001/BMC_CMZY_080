# 电池保护逻辑：作用范围与当前实现

> **2026-11-11/12 更新说明（优先于下文旧描述）**：控制/显示在真实 VC5/BAT 可信时优先采用实测，异常时回退为 VC1/VC2 估算（`cell3=(c1+c2+1)/2`、`pack=c1+c2+c3`）；**保护/SOC/充满参考节不采用该估算，正常用三节最低值，`log_cell3_valid==0 || log_pack_valid==0` 时只用实测 VC2**。BQ 硬件 OV/UV 在正常情况下用 cell1/cell2 实测过滤，异常时只比较 VC2，并在本批判定最终后清假锁存。记录只写真实可信值，异常时 V3/包压置 0 并置 missing；估算不落盘。VC3/VC4 仅作 raw 诊断；均衡永久关闭（`CELLBAL1=0` 回读）。`bq_anomaly.c/.h`、partial/2 A OCP、9 帧认证、2 min/5 min 锁存和 0x20-0x23 事件已删除。下文涉及这些路径的段落视为历史。

> 依据：当前工作树中的 `App/` 源码（`protection.c/.h`；`bq_anomaly.c/.h` 已删除）。本文描述**已编码的实际路径**，不是已完成上板安全认证的规格；与旧文档或注释不一致时以源码及 BQ769x0 数据手册 `DOCS/bq769X0.pdf` 为准。编译成功不等于阈值、方向、时序、开关管动作已经实测。

## 0. 2026-11-12 低压分级、BQ UV 与充满（当前实现）

> 本节优先于下文所有旧描述。FET 仍由 `ChargePolicy` 唯一请求，Protection 只给 `allow_mask`；所有低压动作都是“允许掩码叠加”，不能放宽 BQ OV/SCD/OCD、软件 OCP 或温度已经关断的通路。

> **VC2-only 降级（2026-11-12）**：已发布帧 `log_cell3_valid==0 || log_pack_valid==0` 时，`BQ_GetRefCellMv()` 只返回实测 VC2，MCU 2.8 V、低电状态、充满电压腿、SOC/SOH 和 D 帧都使用它；正常帧仍取三节最低值。`Bq_FilterHwOvUv()` 在该模式只比较 VC2，并在本批 VC5/BAT 判定最终确认后安排假 OV/UV 清除；VC2 无效则保留 raw OV/UV、保持 `allow=NONE`。这是软件/状态过滤，BQ 硬件比较器仍会动作，真 VC5 过压会再次锁存。

> **亏电充电状态（2026-11-12）**：参考节 `<3100 mV` **或** 有效 SOC `<10%` 连续 2 个新 BQ 帧进入低电状态；默认 CHG-only，**不依赖先检测到充电器**，因此不会再因 CHG 关断而无法探测充电器。DSG 只在该状态内、参考节 `>=2800 mV` 且充电电流 `>200 mA` 连续 2 个新样本后允许；电流 `<120 mA` 或无效立即关闭 DSG，`<2.8 V` 绝对不允许 DSG。退出条件为参考节 `>=3100 mV` 且 SOC `>=10%` 连续 2 帧（SOC 无效时只看 3100 mV）。低电状态不设 `FAULT_BIT_PROTECTION`/`SM_STATE_FAULT`；无充电器时可自动 LP，按键唤醒后红灯 5 Hz。MCU 复位不恢复旧 CUTOFF_LATCH，按实时参考节/SOC 重建。

> **TI UV 复位边界**：BQ769x0 手册 §8.3.1.3.1/§8.5.1 说明 UV 是锁存状态，主机必须写 `SYS_STAT` 清除并重新使能 FET；条件仍存在时 UV 可能立即重新锁存。§8.4 CAUTION 的“完整复位”只针对电源低于 POR/VSHUT，不适用于普通 2.8 V 单节 UV。本固件不强制 SHIP/TS1 全复位；BQ UV 只影响 DSG，CHG 可在低电状态恢复，电压恢复后由 `pending_sysstat_clear` 清状态。


| 功能              | 触发                                                                                                                        | 动作/恢复                                                                                                                                                          | 关键边界                                                                                                                                                                                                                          |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| MCU 单节硬欠压    | **有效 BQ 帧**参考节 `<2800 mV`，单帧即触发，不依赖 SOC；正常为三节最低值，VC5/BAT 异常时仅为实测 VC2                                                                     | 立即清 `DSG`，`CHG` 保持；2 个有效 BQ 帧参考节 `>=3000 mV` 后恢复 DSG                                                                                              | 2.8–3.0 V 保持原状态（回差）；不置 `FAULT_BIT_PROTECTION`、不改变 `SM_STATE_FAULT`；事件 0x26/0x27；VC2-only 降级时只使用实测 VC2，不使用合成 cell3                                                                                             |
| BQ 硬件 UV-only   | `SYS_STAT.UV` 存在，且无 OV/SCD/OCD/XREADY、无软件 OCP、无 clear-failure                                                    | 状态仍 `PROTECTION_NORMAL`，`allow = CHG`；UV 位清除且电压恢复后由正常路径放宽                                                                                     | UV 与任一硬锁存同时存在时仍 `allow=0` 并置 `FAULT_BIT_PROTECTION`；BQ 只关 DSG，固件不再把 UV-only 扩大到双管关。VC5/BAT 不可信时 BQ OV/UV 过滤只比较 VC2，并在本批判定最终后清假锁存                                                                                                                   |
| 亏电充电状态     | 参考节 `<3.1 V`（正常最低节，VC5/BAT 异常时 VC2）**或** 有效 SOC `<10%`，连续 2 个新 BQ 帧；不限制 IDLE/DISCHARGE/CHARGE | 进入 `CHARGE_POLICY_LOW_BATTERY_CHARGE`，默认 CHG-only、无需先检测充电器；无充电器/电流时允许自动 LP；DSG 只在 >200 mA/2 帧且参考节≥2.8 V 时开，<120 mA 或无效立即关；3.1 V+SOC≥10%/2 帧退出 | 不置 PROTECTION fault/SM FAULT；不持久化，复位按实时电压重建；BQ UV-only 只关 DSG，CHG 可恢复；TI UV 锁存需主机清 SYS_STAT，不强制全 BQ 复位 |
| 充满/过充提示     | 充电中且 `comp_charging=1`、策略为 CHARGE；参考节 `>=4.15 V` 或（标定有效）SOC `>=98%`；充电电流在 `[-20, 75] mA` 持续 30 s | 置 RAM-only `charge_full`、事件 0x28、RGB 白灯常亮；**不关 CHG**                                                                                                   | 充电器/COMP2 移除时立即清除；电流 `>150 mA` 连续 2 帧清除；复位不恢复；BQ OV 4.25 V 仍是最终硬件保护；SOH 无独立阈值                                                                                                              |
| 原 5%/0% SOC 截止 | —                                                                                                                          | **未按原样实现**                                                                                                                                                   | 改为 3.1 V/SOC10 低电充电 + 2.8 V 电压硬兜底；不用 SOC 单独切断；DSG 在低电状态内仅按 >200/<120 mA 窗口开放 |

`protection_snapshot_t` 新增 `soc01/min_cell_mv/cell_uv_active/soc_early_latched（低电锁存兼容名）/charge_full` 及 streak/count 字段，可在 Ozone 展开 `s_protection` 查看。`[PROT]` 事件行给出 16 位 `ev`、reason、allow 和 OCP/温度状态；低压/充满事件也写黑匣子 0x24–0x29。

## 1. 系统边界与职责

板上是 **3S 电池、1.5 mΩ 负端分流器、BQ7692003PWR 和背靠背 N-MOS**。BQ 的 CELL1/2/3 实测通道分别是 VC1、VC2、**VC5−VC4**；VC3/VC4 是 3S 接法中短接的诊断通道。BQ 的 `SYS_CTRL2` bit0 为 `CHG_ON`、bit1 为 `DSG_ON`。**充电与放电许可是两个逻辑位，不等同于 MCU 的 5 V 辅助电源 PB0。**来源：`App/bsp/bq_ic.h:29-63`、`App/board.h:70-81`、`DOCS/PINMAP.md`。


| 层                                                      | 实际职责                                                                                                       | 不应混淆的事                                                                              |
| --------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| BQ 硬件                                                 | 用其寄存器阈值/延迟检测单节 OV/UV 和**放电方向** OCD/SCD；锁存 `SYS_STAT` 并控制相关 FET。                     | BQ **无硬件充电过流保护**；芯片的温度读数不自动执行本文的软件温控。                       |
| `App/bsp/bq_ic.c`                                       | I²C1 逐字节 CRC、DMA 读、配置/回读硬件保护、真实帧确认、按帧更新故障位、FET 寄存器写入/恢复和 LP 阻塞式监测。 | `BQ_RequestFetMask()` 只改请求影子；`fet_mask_applied` 是软件同步记录，不是栅极电气测量。 |
| `App/app/protection.c`                                  | 读取真实 BQ 状态、数据时效、COMP2 电流方向、CC 幅值及温度；计算**允许掩码**、软件过流锁存与安全清除。          | 不直接负责用户工作模式或 FET 请求。                                                       |
| `App/app/charge_policy.c`                               | 独占常规 FET 请求：IDLE=关、CHARGE/DISCHARGE=按许可请求、部分异常只可限放电；处理手势、充电器和 CAL。          | 请求始终与 `Protection_GetAllowedFetMask()` 做 AND；只在 BQ RUN/空闲时尝试即时写。        |
| `state_machine` / `self_test` / `faults` | 故障优先级、上电准入、live/sticky 故障位。                                             | “异常提示”、`SM_STATE_FAULT`、BQ 硬件锁存和**跨掉电档案锁存**是不同概念。               |

完整 NORM 主循环在 `App/app/main_app.c:1112-1416` 依次推进 BQ→SOC→自检→`Protection_Task()`→`StateMachine_Update()`→`ChargePolicy_Task()`→`Protection_Service()`→异常报警/档案/LP 门控；250 Hz TIM2 ISR 只对 COMP2 做 32 次同向采样消抖（约 128 ms）并按 500 ms 置读取请求。FET 收紧在下一次主循环执行；RUN/空闲时尝试立即写 `SYS_CTRL2`，否则等后续脚本同步，**不能声称任意状态下立即完成物理关断**。`App/app/charge_policy.c:80-108,233-271`、`App/app/main_app.c:164-209`。

## 2. BQ 硬件阈值、有效期与动作

配置来源为 `App/bsp/bq_ic.c:50-62,1504-1568`：CONFIG 时 ADC/CC/FET/均衡先关闭并回读，写入保护寄存器，开启 ADC/CC 后核对状态；配置信息不一致按配置失败重试。OV/UV 的 8 位 trip 码按读得的 `ADCGAIN/ADCOFFSET` 换算，并非固定寄存器字节。


| 项目                        | 当前配置                                                                                 | 作用及动作                                                                                                     |
| ----------------------------- | ------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| SCD 短路                    | `PROTECT1=0x89`，RSNS=1，67 mV / 1.5 mΩ ≈ **44.7 A**；按数据手册码值延迟 **100 µs**。 | **放电**短路；BQ 锁存 SCD、关 DSG。源码上方“70 µs”注释与寄存器编码不符，应按数据手册核查实物。              |
| OCD 过流                    | `PROTECT2=0x57`，56 mV / 1.5 mΩ ≈ **37.3 A**，延迟 **320 ms**。                        | **放电**过流；BQ 锁存 OCD、关 DSG。不是软件充电 5.5 A 或放电 33 A 阈值。                                       |
| OV 单节过压                 | `OV_TRIP` 对应 **4.25 V/节**，`PROTECT3=0x40` 对应约 **1 s**。                           | BQ 锁存 OV，关 CHG。真实阈值受增益/偏移校准及 trip 码量化影响。                                                |
| UV 单节欠压                 | `UV_TRIP` 对应 **2.80 V/节**，`PROTECT3=0x40` 对应约 **4 s**。                           | BQ 锁存 UV，关 DSG。                                                                                           |
| XREADY、外部 ALERT override | BQ 状态位；驱动清 `CC_READY/XREADY/OVRD_ALERT` 等非保护中断原因。                        | XREADY 是芯片异常；OVRD_ALERT 不是 OV/UV/SCD/OCD 保护位，但可强制双管关。不能把单纯 `ALERT` 电平当作电池过压。 |

这些是 BQ 的硬件动作。固件 `Protection_Task()` 对 **OV/SCD/OCD**（以及 UV 与任一硬锁存同时存在的情况）把允许掩码置 0、置 `FAULT_BIT_PROTECTION` 并让状态机进入 FAULT；**只有 UV 单独存在时是例外**：状态保持 `PROTECTION_NORMAL`、`allow=CHG`，便于充电恢复。硬件锁存清除**不会自动重新开管**，仍需策略重新授权。详细的 UV-only 和 MCU 低压叠加见 §0。来源：`App/app/protection.c`、`App/app/state_machine.c`。

BQ 的 OV/UV 依赖 ADC：NORM 配置、受控 ADC 重启期间以及 reset-based LP 中 `ADC_EN=0` 时无法宣称存在连续的硬件 OV/UV 监测；放电 OCD/SCD 是独立比较器路径。LP 强制 FET/ADC/TEMP/CC 关闭，约每 30 s 的 IWDG 复位轻量启动执行 sniff，正常 30 min 标记临时打开 ADC 并监测两批，随后关回。来源：`App/bsp/bq_ic.c:1605-1659,3191-3350`、`App/app/lp_cycle.c:373-459,738-830`。

## 3. 正常 NORM 准入、软件过流与保护锁存

### 3.1 故障位和动作层级

`Fault_GetAll()` 是 live|sticky 的 16 位并集，但不同位不意味着同一种关断动作：`App/board.h` 定义 BQ_COMM/XREADY/OV/UV/SCD/OCD、TEMP、LSE、FLASH_LOG、PROTECTION、POWER、CLK、BQ_STALE、FLASH_STATE、BQ_ADC_STUCK、TIMEBASE；`App/app/faults.c` 实现置位与清除。`StateMachine_Update()` 的 **critical 掩码只包含** BQ_COMM、XREADY、SCD、OCD、ADC_STUCK、PROTECTION、POWER、CLK；**BQ 的 OV/UV 位本身不直接进入该掩码**，BQ OV/SCD/OCD 与“UV+其它硬锁存”由 `Protection_Task()` 汇总成 `FAULT_BIT_PROTECTION` 后再映射到 FAULT。因此 UV-only 可以保持 NORMAL 并允许 CHG。BQ_STALE 不直接进入 FAULT，但使正常数据准入失效；TEMP/FLASH_STATE/TIMEBASE/LSE/FLASH_LOG 并非这一 critical 掩码成员，却可能经自检或 LP 门控阻止工作。`Fault_Init()` 在完整启动中清 RAM live/sticky；不要把 sticky 一律理解为跨电源保持。`App/app/state_machine.c`、`App/app/self_test.c`、`App/app/main_app.c`。


| 状态/信号                                 | 直接作用                                                          | 持久性                                                                                     |
| ------------------------------------------- | ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| BQ `SYS_STAT` 锁存                        | BQ 按故障类型控制相关 FET；固件保护层再评估并可同时禁止 CHG/DSG。 | 由 BQ 芯片保持，不能把 MCU RAM 复位等同于 BQ 清零。                                        |
| `FAULT_BIT_*` 与 `PROTECTION_*`、软件 OCP | 影响状态机、自检、允许掩码和策略。                                | 当前 RAM 状态；`Fault_Init()`/`Protection_Init()` 在完整 boot 重新初始化。                 |
| `STATE_ARCHIVE_ALARM_PENDING/LATCHED`     | 四次异常归档的报警和跨掉电禁止输出，策略层处理归档锁存。          | 48 B StateStore v2 记录 CRC 校验后可在重新上电时加载；Flash 写失败不可保证掉电后的新锁存。 |

### 3.2 正常准入和软件 OCP

**正常准入**要求 `SelfTest_IsDone/IsPassed`、状态机 NORM、BQ RUN、`BQ_IsFresh(now_ms)` 和电源模式 NORM；充电器由消抖 COMP2 高电平指示，或 CHARGE 中充电电流 >30 mA 保持；COMP2 低且无电流支撑时还需 **5.2 s** 才撤销（`CHARGE_POLICY_CHARGER_LOW_MS=5200`；平衡充会短暂关断输出，1 s 去抖会误判为移除并造成充放电状态抖动）。近满（显示 SOC>90 且 −20 mA < 充电电流 < 220 mA）时只请求 CHG（`0x01`），用 DSG 体二极管压降让 COMP2 判充电器；电流 >320 mA 或 SOC<85% 恢复双管。SOC 只作策略启发（近满子模式），不是硬保护；硬保护看 §0 的 2.8 V 电压腿。IDLE 请求双管关；CHARGE、DISCHARGE 在通常路径请求 `BQ_FET_MASK_ALL=0x03`，再与保护掩码相与。**2.8 V** 硬欠压或低电状态存在时，`ChargePolicy_Task()` 会进入 `CHARGE_POLICY_LOW_BATTERY_CHARGE`，默认 CHG-only、不依赖先检测到充电器；无充电器/电流时允许自动 LP。只有低电状态内 >200 mA/2 帧且参考节≥2.8 V 才请求双管，<120 mA 或电流无效立即回 CHG-only；`PAIR_HOLD3` 在低电/欠压期间被拒绝。**来源：`App/app/charge_policy.c`、`App/bsp/bq_ic.c`。


| 软件项目     | 触发条件                                                                                                                                                            | 处置/解除                                                                                                                                              |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 正常充电 OCP | BQ **旧发布快照判据** `valid/fresh/protection_ok` 成立、`last_update_ms` 是新时间戳时，CC 幅值 **≥5.5 A** 且 COMP2 为充电，持续 **1 s**，且至少 **2 个新发布帧**。 | 软件锁存充电过流，允许掩码归零、双 FET 请求关，记事件；需显式 `Protection_RequestLatchClear()`，且安全清除时充电器必须移走。BQ 硬件并无充电 OCP 兜底。 |
| 正常放电 OCP | 同上，COMP2 为放电、CC 幅值 **≥33 A**，持续 **500 ms**，且至少 **2 个新发布帧**。                                                                                  | 软件锁存、双管请求关；不因电流变小自动解除。                                                                                                           |
| CC 极性诊断  | `BQ_CC_DISCHARGE_SIGN` 默认 -1，CC 与消抖 COMP2 方向不一致累计三帧。                                                                                                | `PROT_RSN_CURRENT_SIGN` 诊断/日志，不单独作为跳闸条件；OCP 的方向由 COMP2、幅值由 `abs(current_ma)` 决定。                                             |

正常 OCP 仅在旧快照判据 `Protection_DataReady()` 为真、`last_update_ms` 是新值且没有硬件保护锁存时累计；该路径**没有单独再检查** `batch.quality==FULL_REAL_OK`，也不能证明 `snap.current_ma` 与 `snap.cc_raw` 来自同一批。`|I|>60 A` 被软件评价器视为不可信并**排除**软件 OCP（SCD/OCD 硬件另行处理），不是一个“60 A 软件断开”阈值。CC 换算为 `cc_raw × 844 / 150` mA，1 LSB 为 8.44 µV、分流器 1.5 mΩ；平均窗口约 250 ms，主循环通常每 500 ms 读一次。来源：`App/app/protection.h:54-68`、`App/app/protection.c:277-285,332-445,675-781`、`App/board.h:70-81`。

### 3.3 锁存清除与数据复核

**安全清锁存**：启动/LP 唤醒时捕获的 SCD/OCD 可走一次自动验证式清除；运行期 SCD/OCD 和软件 OCP 需显式请求，OV/UV 不通过这一接口清除。必须 BQ RUN、数据有效/fresh、总线 idle、请求及软件已应用 FET 掩码均为 0、`|I|≤500 mA` 持续 2 s；充电 OCP 另要求 COMP2 不再指示充电。失败标记 `clear_failed`，继续禁开管并记事件；再次尝试仍需显式请求。**此处 FET=0 判断依赖请求/已应用影子，非再次物理测栅极；立即写接口也不是每次都回读控制寄存器。**`App/app/protection.c:931-1060`、`App/bsp/bq_ic.c:3386-3420`。

驱动有独立的“ADC 建立期假 OV/UV”处理：当前使用 20 mV 安全余量（OV 低 20 mV、UV 高 20 mV），在有效 VC1/VC2 证据下安排对应锁存清除；不触碰 SCD/OCD。BQ 帧的 VC5/BAT 判定最终确认后才会安排清除，读过程中的暂态不会清真实锁存。该写入/影子更新不是对物理 FET 状态的独立验证。`App/bsp/bq_ic.c`。

## 4. 温度、供电和数据失效

温度来自 BQ TS1 的外接 NTC 经 `SocEst` 换算。当前包温度门控窗口为 **4.0–54.0 °C**，充电和放电都收紧；约 2 s 越界确认，恢复边界为 6.0–52.0 °C（2 °C 回差）。初始化时 `temp_invalid=1`；运行期由有效转为无效约 2 s 后同时禁充/禁放，恢复需连续有效约 2 s。**`PROT_TEMP_ENABLE=1`，`Protection_TempAllowMask()` 据此收紧 FET 允许掩码；`temp_invalid` 也会置 `FAULT_BIT_TEMP`，自检将 TEMP 当作真实故障。**TS1 有效上拉模型仍建议用 4 °C/54 °C 两点复核，避免读数偏差导致误禁。`App/app/protection.h`、`App/app/protection.c`、`App/app/self_test.c`、`App/detect/soc_est.c`。

MCU 的 PVD 配置为 3.3 V 电源轨约 **2.8 V** 检测，IRQ 设置 `FAULT_BIT_POWER`（sticky），与电池三节 UV 是不同保护；80 MHz HSE/PLL 时钟失败进入降级路径，只留故障灯。`FAULT_BIT_BQ_STALE`（无新完整数据约 2 s）本身不在状态机 critical 掩码中，但会让正常策略/BQ 就绪条件失败并拒绝 FET；真正 critical 包括 BQ_COMM/XREADY/OV/UV/SCD/OCD/ADC_STUCK/PROTECTION/POWER/CLK。`App/app/faults.c:13-16,122-132`、`App/app/state_machine.c:14-20`、`App/app/charge_policy.c:50-77`、`App/board.h:36-40`。

## 5. VC5/BAT 部分电压异常：诊断、限时放电与 ADC 退避

BQ 以顺序的 CRC I²C 寄存器读取构成一批，不是芯片内部原子 ADC 帧。数字合理性窗口为单节 **1–5 V**、BAT **3–15 V**、`|BAT−三节之和|≤300 mV`，短接的 VC3/VC4 raw 各不超过 128。**这些是数据发布的合理性门槛，不是 BQ 4.25/2.80 V 的电池保护门槛。**启动/受控重启后首个有效候选需第二次确认（首次每原始寄存器 ±4 LSB；三次不合后允许回退 250/300 mV 粗确认）；常规拒帧后的确认使用粗容差。仅同一读取批次真正完成发布的 `BQ_BATCH_FULL_REAL_OK` 是归档/异常时钟的“真实全包好帧”；`frame_ok_count` 包含待双帧确认的候选，不等同于发布次数。`App/bsp/bq_ic.c:71-93,1107-1502`、`App/bsp/bq_ic.h:101-162`。

`BQ_BATCH_PARTIAL_VC5_PACK` 要求两批独立读取（开始间隔至少一次约 250 ms 转换、VC1/VC2 差 ≤250 mV），VC1/VC2/VC3/VC4、状态、控制、保护、TS1/CC 等字段满足该路径校验，且 **VC5 和/或 BAT 字段超出合理范围**；两批不要求坏的是同一个字段，也可能两字段同时坏。如果只出现 BAT 与三节之和不符、却无法确定哪个通道错，则归为 INVALID 而不是部分可信。该分类不证明芯片脚电位实际安全，也不证明两批必然同一个通道失真；同一颗 BQ 的 CC 不是第二个独立电流传感器。只有用于显示/诊断时，VC5 不可信才取 `估算CELL3=(VC1+VC2)/2`，BAT 不可信才取两实测/估算电芯和；**估算值不回写正常 BQ `valid/fresh`、不清锁存、不直接决定 FET，也不会填入降级档案的实测电压字段**。`App/bsp/bq_ic.c:1204-1325,1336-1385`。

部分异常放电不是“保证可用 20 s”：必须本次 MCU 启动已在真实好包、自检和原保护允许条件下取得放电资格、最后完整好帧距今 **<20 s**；持续确认 BQ RUN、ADC/CC 开、两次状态读与保护/控制字段可靠、消抖 COMP2 指向放电、**同批 CRC 正确的新 CC** 未超过 1 s、无 OV/UV/SCD/OCD/XREADY/COMM/ADC_STUCK/PVD/温度等阻断，才可令保护允许掩码最多为 `DSG`。两次不同转换、时间至少差 250 ms 的 CC 幅值 **≥2 A** 将软件放电 OCP 锁存；CC 失效或其它条件先失效则提前关。期间可以在上述条件持续成立时重新请求放电；充电**没有**坏电压情况下绕过原自检/fresh/保护门控的路径。`App/app/protection.c:85-268,675-929`、`App/app/charge_policy.c:51-77,328-379`。

每约 500 ms 的读取照常执行：输出**开**时 10 s 仅增加复判/限频请求，不在此路径关 ADC；输出**关**且严格分类的部分异常可在至少 10 s 冷却后请求受控 ADC off→on（关闭 ADC/CC/FET、约 800 ms off、重新配置、约 300 ms settle 和双帧确认）。通用 OOR 持续 8 帧、受控重启最多 2 次，后可能锁存 `FAULT_BIT_BQ_ADC_STUCK`；四次真正好帧可清故障/额度。局部异常 10 s 重试与通用 OOR 恢复是分支，不能视为“放电时无中断的 ADC 重启”。`App/bsp/bq_ic.c:81-95,922-1004,2500-2570,2732-2786`、`App/app/bq_anomaly.c:17-30,494-531,827-837`。

## 6. 电压异常报警、归档锁存和 LP


| 事件                  | 编码行为                                                                                                                                                                                                                                                                  | 范围/限制                                                                                                                                   |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| 冷/PIN 启动无真实好包 | 上电自检蓝灯保持；若是**已确认的局部电压异常**且硬件/链路仍正常，满 60 s 进入电压异常报警。自检完成后蓝灯另保持 2 s（以主循环自检完成时间为准）。                                                                                                                         | 不因为其它真实故障而延期；不将局部估算帧当自检 PASS。                                                                                       |
| 输出中的失帧          | 满 20 s 没有新真实好包时条件放电门控先失效；协调器只在认为输出仍开且 COMP2 指放电时启动报警。                                                                                                                                                                             | OCP、CC 超时、真实保护故障可在 20 s 前先关管；失帧并非一定等满 20 s。                                                                       |
| NORM 30 min 存档      | 优先使用本窗口连续 **9 个真实新发布帧**及同源 SOC/温度候选；到点不足最多再等 60 s。其后允许降级档案：两节有可信实测则第三节与包压在 Flash 中写 **0** 并带质量标识，SOC 由两节较低者和可信 CC 继续估算；前两节也不能可信则记 CRITICAL，缺失电压写 0，SOC 标无效。          | 归档质量 0 不代表真实 0 V；估算 SOC 不用作硬保护，部分期间不做 SOH 学习。黑匣子 24 B 周期记录与 StateStore 48 B v2 分别写、非跨区原子事务。 |
| 连续异常标记          | StateStore 用 RTC 标记 ID 去重：DEGRADED 累计，FULL 清计数，**第 4 个**异常标记或 CRITICAL 直接将计数置 4、置报警 pending。                                                                                                                                               | “连续四次”是**四个 30 min 标记结果**，不是四个坏采样帧。正常 LP 的两帧成功标记也可清计数，但不跑轻路径 SOC。                              |
| 第 4 次报警 / 解锁    | HOT/PB3 闪 + 蜂鸣最多 **2 min**；在锁存前取得 9 个连续真实好帧可取消并持久清 pending/count；否则请求双管关，归档禁输出/禁自动 LP 锁存跨复位/掉电保存。人工请求仍须真实好帧、可信 CC、电流安全、无硬件告警、FET 控制回读确认及成功持久清除；代码没有已接入的自动换电解锁。 | `StateStore` 写失败只可在本次 RAM 中 fail-closed，**不能保证掉电后保存尚未写入的锁存**。请求关断与寄存器回读不构成物理栅极测量。            |
| 其它报警 / 5 min      | 20 s 放电或 60 s 冷启动报警本身不自动满足“四标记锁存”；所有实际持续报警满 **5 min** 仅置 `stop2_ship_trigger` 诊断预留标志。                                                                                                                                            | **没有实现 STOP2、BQ SHIP 或不可恢复死锁**。报警状态在 MCU 复位后若持久 pending，计时由启动时重新开始，不代表真实掉电时长。                 |

电压异常时 PB3/HOT 可独立闪烁而 RGB 不因该原因红闪；真实 critical FAULT 仍用原 PB3 快闪 + RGB 红优先级。`BlackBox_WriteEvent()` 使用 0x18–0x1F 记录保护/温度相关事件，0x20–0x23 记录电压异常报警、取消、锁存和人工清除；`BlackBox_WritePeriodicQualified()` 对缺失电压/CC/TS1 写 0 并附质量位，**事件和质量位只用于追溯，不替代保护执行**。源码：`App/app/black_box.h:25-68`、`App/app/main_app.c:466-518`。**LP 的 5 V/RGB/蜂鸣及 PB3 正常关闭，睡眠中没有 10 s 复判/声光持续运行**。LP 正常 30 min 点由轻量路径读两批实测并只写黑匣子周期记录，StateStore 元数据会清四连计数，但不更新 SOC/SOH 快照；监测失败可保存 RTC mark 并完整唤醒 NORM，只有针对电压类别的 `-2` 返回不附带 LP_FAULT。真实通讯/硬件保护错误继续走故障唤醒。源码：`App/app/bq_anomaly.c:17-30,241-349,420-820,847-1035`、`App/app/main_app.c:609-765,1192-1412`、`App/app/lp_cycle.c:373-459,601-655,1004-1163`、`App/app/led_status.c:36-76`、`App/app/state_store.c:405-585`。

## 7. 实现边界、已知缺口与验证状态

以下事项以当前代码**实际能保证到的程度**措辞，避免把配置值误写为认证结果：

1. **硬件与软件分工**：BQ 硬件无充电过流保护；温度门控由 `PROT_TEMP_ENABLE=1` 启用；SOC 的 NMC 表仍含占位 OCV/tau，但 `SOC_EST_CAL_VALID=1`，SOC/SOH 用于低电判断和显示。当前**没有**原计划的 5%/0% SOC 截止；硬欠压是 §0 的 2.8 V 电压保护，3.1 V/SOC10 低电状态负责 CHG-only 恢复，充满只是提示，均衡永久关闭。低电状态是 RAM-only，MCU 复位不恢复旧 `STATE_FLAG_CUTOFF_LATCH`；`STATE_FLAG_CHARGE_FULL` 仍未使用。换电/死存储/自动安全解锁未接入。`BQ_SetBalanceMask()` 已删除；不能把任何 balance API 当作已实现平衡充。`App/detect/soc_est.h`、`App/app/state_store.h`、`App/app/protection.c`。
2. **新批质量与旧快照并非同一安全门**：`Bq_FinishRunBatch()` 可以得到 INVALID（状态/配置/格式等不可信），但 `Bq_FrameCommit()` 在未确认 PARTIAL 时仍可能仅因数字电压合理并完成双帧确认而发布 `bq_snapshot_t.valid/fresh`，甚至发布序号递增。严谨的 FULL 准入与归档使用批质量；`Protection_DataReady()` 仍仅以旧 `valid/fresh/protection_ok` 门控正常路径，不能声称所有下游都只消费完整合格批次。此差异是当前保护覆盖上的**待修复缺口**，不能靠文档将其描述为已解决。`App/bsp/bq_ic.c:1107-1502`、`App/app/protection.c:108-125,277-285`。
3. **状态位/快照时序**：OV/UV/XREADY 的固件 live 故障更新主要依赖数值有效的发布帧；无效极端电压时仍应看 BQ 硬件锁存与原始 `SYS_STAT`。`BQ_GetSnapshot()` 中的 raw CC/TS1 与已发布电压/`current_ma` 不一定同一发布时刻；保护部分异常用单独的带批号和年龄的 CC。RUN 批虽回读 PROTECT1/2/3，但未每次复读 `OV_TRIP/UV_TRIP/CC_CFG`；I²C 校验逐寄存器，不等于同一芯片 ADC 时刻。`App/bsp/bq_ic.c:561-600,1110-1332,2634-2675`。
4. **电流与温度实际覆盖**：部分模式 `SocEst_OnPartialSample()` 令温度无效，而 `Protection_Task()` 在无效温度约 2 s 后仍置 `FAULT_BIT_TEMP`（若此前已无效，故障可能早已存在），部分放电严格门控会拒绝 TEMP。因此尽管设了 `<20 s` 的最终时限，部分放电可能在约 2 s 提前关闭；CAL 则可绕过自检且默认温度 FET gate 为 0，不能把 CAL 当作与正常模式同等的温度安全准入。`App/detect/soc_est.c:350-451`、`App/app/protection.c:128-163,311-329,447-475,921-929`、`App/app/charge_policy.c:61-77,381-410`。
5. **清锁存与持久化边界**：清 SCD/OCD 的 `Protection_Service()`检查的是软件请求/已应用 FET 影子，不是管脚实际电压；BQ 的通用 `BQ_ClearProtection()`、`BQ_WriteProtectionRaw()` 等低层 API 未附带同等级安全门，不能用调试器直接调用它们代替保护层。LP 的 `-2` 表示**两组数值局部越界**，不能证明 VC5 或 BAT 固定损坏，更不能排除硬件 OV/UV 延时窗口内的真实异常；LP 完整唤醒后要重新测量。StateStore v1 可读取、v2 存档带 CRC；黑匣子和 StateStore 不构成一个原子事务，写盘间断电可能留下单侧记录；无有效 RTC 标记则不能凭 uptime 推断跨 LP 的 30 min 时间。`App/app/protection.c:931-1060`、`App/app/state_store.c:156-190,405-585`。
6. **上板验证欠缺**：当前工程无自动测试/单测试命令；需在限流电源和受控负载下逐项测真实三节电压、COMP2/CC 方向、OV/UV/SCD/OCD 动作与延迟、两 FET 实际栅极状态、温度信号、两分钟锁存及断电恢复、LP 30 min 标记去重。不要在未验证时把软件计算值当作实测保护能力。构建命令：`make clean && make -j2 all`；镜像应满足链接脚本 `_image_end <= 0x08018000`。
7. **2026-11-12 新增项的最低台架检查**：

   - 2.8 V 硬欠压：单节降到 2.79 V 内应在一个 BQ 帧内只关 DSG、CHG 仍可开；2.9 V 保持；≥3.0 V 两帧后恢复；观察 `PROT_RSN_CELL_UV`、事件 0x26/0x27，确认不出现 `FAULT_BIT_PROTECTION`/红灯。
   - BQ UV-only：触发后确认 BQ 的 UV 位与 `s_protection.allow_mask=0x01`，充电器可继续；与 OV/SCD/OCD 同时存在时必须回到 `allow=0`。
   - 低电充电：参考节<3.1 V 或 SOC<10% 进入，默认 CHG-only 且不要求先见充电器；>200 mA/2 帧且≥2.8 V 才双开，<120 mA/无效立即关 DSG；3.1 V+SOC≥10%/2 帧退出；复位后不恢复旧锁存；无充电器可自动 LP，按键唤醒红灯 5 Hz。
   - 充满：4.15 V/≤75 mA/30 s 或标定 SOC≥98% 后白灯常亮、事件 0x28，确认 CHG 不被固件关闭；移除充电器或 >150 mA 两帧清除。
   - 低压锁存期间 `PAIR_HOLD3` 被拒绝；无充电器时低电状态允许自动 LP；旧 StateStore CUTOFF_LATCH 不参与控制。
   - VC2-only 降级：`log_cell3_valid==0 || log_pack_valid==0` 时确认 `ref_cell2_only=1`，强制 VC1 虚假偏低但 VC2 正常，2.8 V/低电/SOC/D 帧均使用 VC2；BQ 假 OV/UV 被过滤并在本批最终判定后清除；VC2 无效时不得清除或放宽。

**代码入口索引：**硬件配置/读数/恢复 `App/bsp/bq_ic.c`、`App/bsp/bq_ic.h`；监督层 `App/app/protection.c/.h`；策略和状态 `App/app/charge_policy.c/.h`、`App/app/state_machine.c/.h`；存储 `App/app/black_box.c/.h`、`App/app/state_store.c/.h`；启动及 LP `App/app/main_app.c`、`App/app/lp_cycle.c`；NTC/SOC `App/detect/soc_est.c/.h`；故障登记 `App/app/faults.c/.h`、`App/board.h`。

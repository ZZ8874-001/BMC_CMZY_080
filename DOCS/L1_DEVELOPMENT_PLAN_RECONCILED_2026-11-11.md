# BMC_CMZY_080 L1 电池组管理开发方案（2026-11-11 实现核对版）

> **文件性质**：本文件是 `DOCS/L1_DEVELOPMENT_PLAN.md` 的副本与实现核对版，不是原计划的替代历史记录。  
> 原文件保持原样，SHA-256 `9a1d9e10ace8e0486f94e406ef4988b4e2ac4ac175ba7f1aea20cd729f4c8ed7`；完整原始计划仍以原文件为准。  
> 核对基线：2026-11-11 工作树；`git log -1` = `ce8d609`，工作树含未提交改动（见 G1.1）。  
> 本文把“已实现 / 部分实现 / 未实现”明确分开：已实现段按当前代码改写；未实现项保留原计划文字并加状态块；计划与实现的差异集中在 G3。  
> **证据等级**：E1=代码路径；E2=构建/静态检查；E3=台架记录；E4=未验证。没有 E3 记录的内容不得写成“已通过验证”。当前副本只确认 E1/E2，除非明确引用已有台架记录。  
> **安全警告**：当前固件仍使用 NMC 1500 mAh 参数和部分占位 OCV/tau，`SOC_EST_CAL_VALID=1` 允许 SOC/SOH 参与低电判断/显示；2.8 V 电压硬保护仍是独立兜底。VC5/BAT 还有板级降级策略。本文件不是 LFP 安全认证、量产放行或标定报告。

**状态标签**

| 标签 | 含义 |
|---|---|
| `[已实现]` | 当前代码有完整路径；本节按实际行为改写 |
| `[部分实现]` | 当前只实现子集；写出实际子集和剩余项 |
| `[未实现-未来]` | 保留原计划文字；只加状态说明，不改成已实现 |
| `[已替代]` | 原模块/API/机制被当前实现替代；写出实际名称/路径 |
| `[冲突-已裁决]` | 当前实现优先；G3 记录原计划与实际 |
| `[计划外新增]` | 原计划没有、当前代码新增的行为 |

**阅读规则**：下文原计划中的“当前/现有”指 2026-11-11 之前的 L0 基线，不等于当前工作树。凡标 `[未实现-未来]` 的功能，不能被当作已交付；凡只在 StateStore 头文件里定义、但没有写入者/读取者的 flag，也不能算实现。

---

## G1 当前实现基线

### G1.1 快照与证据

- 基线 commit：`ce8d609`（`fix:fuck烂掉的ADC，直接用正常的VC1，VC2值`）。
- 发布基线：`dd6fded`（`Fuck_ADC_2`，相对 `main` 5 个提交）；发布标签 `v0.0.11+hw0.80`，未合入 `main`。
- 构建（2026-11-14 clean build，含 SOC/低压/充满保护、低电充电状态、RGB SOC 指示、VC2-only 降级、BQ FET 回读影子校准与 BQ OV/UV 退避/VC2 软件 OV，以及 HW/SW 版本启动打印）：text 87640 B、data 168 B、bss 8200 B、`_image_end=0x08015700`；链接脚本 96 KB，`_image_end<=0x08018000` 断言通过。
- 文档优先级：当前代码 > `CLAUDE.md` 的 2026-11-10/11/12 当前状态段 > `DOCS/POWER_MODE.md`/`SYSTEM_STATES.md`/`BATTERY_PROTECTION.md` 顶部更新说明 > 本计划原文。`DOCS/BATTERY_PROTECTION.md` 正文仍残留已删除的 partial/`bq_anomaly` 历史段落，不能整篇当作当前状态；2026-11-12 新增行为见其 §0。

### G1.2 已落地的关键路径

- **时钟/NORM**：冷启动 `SystemClock_Config()` 只开 HSI16；`PowerMode_ClkSwitchToHsePll()` 在 `App_Init()` 内切 HSE bypass 12 MHz + PLL 80 MHz（PLLM=3/PLLN=40/PLLR=2，Scale1，4WS）；App 不得直接调用 `SystemClock_Config()`。
- **LP**：`PowerMode_RequestLp()` → `LpCycle_RequestStart()` 写 BKP START token → 软件复位；轻量 boot 在 HSI16 做 BQ sniff、充电器/PB5/PA8 检查、30 min 标记、校准 IWDG timeout，然后 STOP1；IWDG 超时复位下一次轻量 boot。`LP_BASE_CYCLE_MS=30000`。RTC WUT 和 LPTIM1 被显式关闭，`RTC_WKUP_IRQHandler` 只清 pending/WUTF。默认 `LP_RTC_USE_LSE=0` 走 LSI（板载 LSE 不起振）；`=1` 才启动 LSE、LSE 失败回退 LSI 并禁用 LP。
- **充电检测/唤醒**：每次轻量 boot 读 `LOAD_PRESENT`（要求 `!CHG_ON`）后，临时 PA1=0、等 30 ms、启动 COMP2、再等 30 ms 读方向；判定为充电则写 WAKE token + 软件复位，下一次完整 `App_Init` 进入 CHARGE。**该检查约每 30 s 一次，不是 1 s 一次**。PA8 是 EXTI event rising，不能唤醒 STOP1；轻量 boot 读 `SYS_STAT`，LP 唤醒只认 SCD/OCD/XREADY，OV/UV/CC_READY/OVRD_ALERT 被排除。
- **工作状态**：`state_machine` 只有 BOOT/NORM/FAULT；`charge_policy` 只有 IDLE/CHARGE/DISCHARGE/CAL/FAULT。CHARGE 和 DISCHARGE 正常请求 `BQ_FET_MASK_ALL`（CHG+DSG 双管全开）；IDLE 请求 0；CAL 是 Ozone 调试态。COMP2 消抖充电器自动进入 CHARGE；CHARGE 中充电电流 >30 mA 时用电流保持充电器存在，近满（显示 SOC>90 且 −20 mA < 电流 < 220 mA）切 CHG-only/DSG 体二极管；PAIR_HOLD3 切换 DISCHARGE/IDLE；FAULT 和 CAL 阻止自动 LP。没有 DISCHARGE_ARM/CUTOFF_LATCH/SWAP_ERROR/REPLACE_READY/DEAD_STORAGE。
- **按键**：`button` 4 ms 采样、20 ms 消抖；`gesture` 能产生 SHORT/HOLD3/PAIR_HOLD3/HOLD5/RELEASE_AFTER_HOLD5/TRIPLE。`main_app` 目前只消费 `GESTURE_PAIR_HOLD3`；其余事件没有业务动作。
- **SOC/SOH**：toolkit 核心已复制到 `App/detect/bms/`；`SocEst_OnSnapshot()` 在 500 ms 新快照上用参考节（正常三节最低，VC2-only 降级时 VC2）+ CC 电流 + TS1 温度；`SOC_EST_CURRENT_SIGN=BQ_CC_DISCHARGE_SIGN`；`SOC_EST_CAL_VALID=1`，SOC/SOH 参与低电判断和显示，但 2.8 V 电压仍是独立硬保护。当前没有原计划的 5%/0% SOC 截止；参考节<3.1 V 或 SOC<10% 进入低电充电，充满只是提示。NMC 1500 mAh：R0/R1 已按 25 °C 放电实测接入，OCV/tau 仍有占位成分；不是 LFP 标定表。
- **保护**：BQ `PROTECT1=0x89`（SCD 67 mV≈44.7 A/100 µs）、`PROTECT2=0x57`（OCD 56 mV≈37.3 A/320 ms，放电方向）、`PROTECT3=0x40`（OV 1 s/UV 4 s）；OV_TRIP/UV_TRIP 当前按 NMC **4.25 V/2.80 V** 计算。**2026-11-14 起**，首个 `ref_cell2_only=1` 完整批次立即进入 BQ OV/UV 退避：临时写端点 TRIP，MCU 用 VC2 软件 OV（4.25/4.15 V）和现有 2.8 V UV；清除只做一次；pending 阶段暂停 FET 同步，active 后 raw OV/UV 完全忽略（只记录诊断），不再重复清除，BQ 自行清位由 FET 影子回读重写；3 个正常发布批次后恢复 BQ TRIP。SCD/OCD、XREADY/OVRD_ALERT、温度和软件 OCP 不变；自动 LP 仍正常运行，假开关在退避期间仍启用：异常批一到就立即尝试同值重写，但同值重写不能覆盖 BQ 硬件 OV/UV 锁存。软件 OCP 为充电 **5.5 A/1 s**、放电 **33 A/500 ms**，至少 2 个新发布帧确认并锁存；`PROT_TEMP_ENABLE=1`，包温度 **4.0–54.0 °C** 充放电门控生效（2 °C 回差/2 s）。Protection 现在同时读有效 BQ 帧的参考节和 `soc_est_snapshot_t`：正常参考节为三节最低值；已发布帧 `log_cell3_valid==0 || log_pack_valid==0` 时只使用实测 VC2（`BQ_GetRefCellMv`），并据此判断 2.8 V、低电充电、充满、SOC/SOH 和 D 帧。BQ OV/UV 状态过滤在 VC2-only 模式只比较 VC2，并在本批最终判定后清假锁存；VC2 无效时保持 raw OV/UV。BQ UV-only 允许 CHG；这些低压叠加都不置 `FAULT_BIT_PROTECTION`（BQ OV/SCD/OCD 等硬锁存仍置）。
- **存储**：96 KB app；BMS NVM 页 48-49（132/136 B）；StateStore 页 50-51（48 B，v2）；黑匣子页 52-63（24 B×1020）。低电状态是 RAM-only；MCU 复位不恢复旧 `STATE_FLAG_CUTOFF_LATCH`，按实时参考节/SOC 重建。`expected_swap/swap_error/dead_storage/charge_full` 仍无写入者/读取者；`charge_full` 不持久化。黑匣子已增加低压/充满事件 0x24–0x29，但仍无充放电/换电/死存储事件；wake 记录只打包 3 bit，`POWER_WAKE_CHARGER=0x08`/`MONITOR=0x10` 会被截断。
- **模块**：实际树包含 `protection`、`lp_cycle`、`lp_session`、`rtc_time`、`watchdog`；**没有 `shelf_monitor.c/.h`**。30 min 归档在 `main_app.c` 的 `App_ArchiveTask()` 和 `lp_cycle.c` 的 `LpCycle_DoMark()` 中分担。
- **计划外新增（当前树）**：VC5/BAT 真实值可信时优先进入控制/显示；异常时控制/显示用 VC1/VC2 估算，保护/SOC/充满只用实测 VC2；记录只写真实可信值，异常写 0 + missing。VC5/BAT 异常首个完整批次立即进入 BQ OV/UV 退避：临时写端点 TRIP，由 VC2 软件 OV（4.25/4.15 V）+ 现有 2.8 V UV 接管，pending 阶段暂停 FET 同步，active 后 raw OV/UV 完全忽略，不再重复清除；3 个正常发布批次后恢复 BQ TRIP。`App_FakeSwitchTask()` 在退避期间仍启用，异常新批次一到就立即尝试同值重写；同值重写不能覆盖 BQ 硬件 OV/UV 锁存。SCD/OCD、温度、软件 OCP 保留。

### G1.3 未落地/不可宣称

- 完整的 L1 业务状态机（ARM/CUTOFF/SWAP/REPLACE/DEAD）、换电检测/清除/5 s 换电模式、死存储/SHIP 自动路径。
- **原计划的 5%/0% SOC cutoff 未按原样实现**：当前是 2.8 V 电压-only 硬欠压、参考节<3.1 V 或 SOC<10% 的亏电充电状态和充满提示；不是 SOC 单独硬保护。DSG 在低电状态内只按 >200/<120 mA 窗口开放，2.8 V 以下绝不开放。原 98%/3.55 V 充满判据已按当前 NMC 决策改为 4.15 V/98% 白灯提示。
- 均衡（永久关闭，`CELLBAL1=0` 回读；`BQ_SetBalanceMask` 已删除）、BQ die 温读取与 65 °C 停均衡。
- 1 s RTC WUT LP、≤1.5 s 充电唤醒、LP 多类型、PA8 ALERT 唤醒。
- LFP OCV/R0/R1/τ 标定、LFP 3.70/2.50 保护阈值；当前 NMC 参数不可用于 LFP 认证。
- StateStore 的 `CUTOFF_LATCH` 已不再恢复/写入；低电状态为 RAM-only。`expected_swap/swap_error/dead_storage/charge_full` 和 swap/cutoff 清除逻辑仍未接入。黑匣子仍无充放电/换电/死存储事件，wake 映射不完整。
- 蜂鸣器优先级队列、换电/死存储专用音型仍未实现；SOC 渐变灯效已实现（见 C-20）。
- M6 的 30 A/5 A/SCD/温升/LP 电流/EMC/长期循环验收。

### G1.4 模块/API 替代关系

| 原计划 | 当前实现 |
|---|---|
| `shelf_monitor.c/.h` | 无此模块；`App_ArchiveTask()` + `LpCycle_DoMark()` |
| `PowerMode_ServiceLp()` / `PowerMode_GetWakeEvents()` | `PowerMode_Loop()` / `LpCycle_Run()` / `LpCycle_GetWakeCause()` |
| `BQ_LpSniffRead(uint8_t *sys_ctrl1, bq_snapshot_t *snap)` | `BQ_LpSniffRead(uint8_t reg, uint8_t *out)` + `BQ_LpBootPrepare()` + `BQ_LpMonitorRun()` |
| `BQ_SetBalanceMask()` | 已删除；均衡永久关闭 |
| `BQ_ReadDieTempDeciC()` | 无；只有未接线的 `BQ_SetTempSource()` |
| `StateStore_Get()` / `StateStore_Commit()` / `StateStore_ClearLatch()` | `StateStore_GetView()` / `StateStore_Commit()`（签名不同）/ `StateStore_ClearLatch()`（存在但无调用者） |
| `SocEst_GetPercent01()` | `SocEst_GetSoc01()` / `SocEst_GetSocPercent()` |

---

## G2 里程碑状态

| 里程碑 | 状态 | 依据/缺口 |
|---|---|---|
| M0 硬件/标定冻结 | **未关闭** | 当前代码为 NMC 占位；无 LFP OCV/R0/R1/τ/CC/TS1 标定 CSV；30 A/5 A/SCD/die 温/HSI16 I2C 无正式记录 |
| M1 安全基础 | **大部分实现** | 96K/Flash 分区、StateStore、BMS NVM、黑匣子、button/gesture、FET/protection/temp-source API 已落地；均衡 API 删除、`BQ_ShipMode` 未接线、StateStore 生命周期 flags 未使用 |
| M2 LP 与充电检测 | **部分实现/冲突** | 5 s 自动 LP、充电器唤醒、30 min 监测存在；LP 是 IWDG 复位 ~30 s，不是 1 s RTC WUT；充电唤醒最坏约 30 s；30 min 不更新 SOC/SOH |
| M3 SOC/SOH | **部分实现** | toolkit/500 ms/NVM 已接线；NMC R0/R1 已实测、OCV/tau 部分占位、`SOC_EST_CAL_VALID=1`、换电重置无调用者；无原 5%/0% SOC 截止，但有 2.8 V 硬欠压 + 3.1 V/SOC10 低电充电 + 充满提示 |
| M4 充放电策略 | **部分实现** | CHARGE/DISCHARGE/双管/FET/PAIR_HOLD3/软件 OCP 存在；SOC 渐变、低电单管/200-120mA 双管窗口、充满提示已接入；ARM、原 5%/10% SOC 截止、均衡/die 温、音效优先级仍缺失 |
| M5 换电与死存储 | **未实现** | 只有 StateStore flag 定义；无 swap 检测、expected_swap、三连击动作、5 s 换电、DEAD/SHIP 路径 |
| M6 系统验证 | **未开始/未验证** | 无正式故障注入、30 A/5 A、die 温、LP 电流、EMC、循环记录 |

**结论**：当前树是 L0 + M1（大部分）+ M2（部分，架构不同）+ M3/M4（部分）的实现，不是 L1 完成态。

---

## G3 计划 vs 实现冲突表

| ID | 原计划 | 当前实现（摘要） |
|---|---|---|
| C-01 | LP 用 RTC WUT 1 s 周期唤醒 | 复位式 IWDG ~30 s tick；WUT/LPTIM1 显式关闭；`LP_BASE_CYCLE_MS=30000`；**D-LP1 已决定接受** |
| C-02 | 充电插入 ≤1.5 s 唤醒；每秒 LOAD_PRESENT+PA1/COMP2 | 仅每次 ~30 s 轻量 boot 嗅探一次；LOAD_PRESENT 后才做 PA1/COMP2；无 1 s 兜底；**D-LP1 已决定接受 ~30 s** |
| C-03 | 生产 RTC 用 LSE；LSE 故障禁 LP/存档 | 默认 `LP_RTC_USE_LSE=0` 走 LSI（板载 LSE 不起振）；`=1` 才有 LSE 门控 |
| C-04 | BQ ALERT/PA8 可唤醒 | PA8 仅 EXTI event/轮询，不能唤醒 STOP1；LP 唤醒位只保留 SCD/OCD/XREADY |
| C-05 | LP 分 IDLE/CUTOFF/SWAP/REPLACE/DEAD 类型，DEAD 走 SHIP | 只有一种复位 LP；无 LP 类型、无 DEAD；`BQ_ShipMode()` 无调用者 |
| C-06 | IDLE LP 保持 BQ ADC/TEMP 开 | LP 强制 `SYS_CTRL1=0`/`SYS_CTRL2=0`，ADC/TEMP/CC/FET 全关（计划自己的 fallback 条款） |
| C-07 | 30 min 维护完整唤醒并更新 SOC/SOH | 30 min 标记在轻量 boot 内联完成；不跑 SOC/SOH；只有监测失败才完整唤醒 |
| C-08 | BOOT/IDLE/CHARGE/ARM/DISCHARGE/CUTOFF/SWAP/REPLACE/DEAD/FAULT | SM 仅 BOOT/NORM/FAULT；Policy 仅 IDLE/CHARGE/DISCHARGE/CAL/FAULT；其余缺失 |
| C-09 | LFP 3S 1500 mAh；OV 3.70/UV 2.50 | NMC 1500 表（OCV/tau 部分占位）；BQ OV 4.25/UV 2.80；`SOC_EST_CAL_VALID=1`；无 5%/0% SOC 硬保护，但有独立 2.8 V 电压硬欠压和 3.1 V/SOC10 低电充电 |
| C-10 | CHARGE/DISCHARGE 期间均衡常开，die 温 65 °C 停 | **D-BAL1 已决：均衡永久关闭**；`CELLBAL1=0` 回读；`BQ_SetBalanceMask` 已删除；无 die 温 API |
| C-11 | 软件 OCP 充电 5.5 A/1 s、放电 33 A/500 ms | **已恢复**：充电 5.5 A/1 s、放电 33 A/500 ms，至少 2 个新发布帧；详见 D-PROT1 |
| C-12 | 充电 0–45 °C、放电 −20–60 °C 温度禁充/禁放生效 | **已按新决策改为包温度 4.0–54.0 °C，充放电都收紧；`PROT_TEMP_ENABLE=1`**，2 °C 回差/2 s 去抖 |
| C-13 | 充满判据 98%/3.55 V/75 mA/30 s，鸣叫但不断电 | **已按当前 NMC 决策实现为提示**：最低节≥4.15 V 或标定 SOC≥98%，充电电流 −20..75 mA/30 s；白灯 + 事件 0x28，不关 CHG；原 3.55 V/LFP 数值不适用 |
| C-14 | 放电 SOC≤5% 切断、10% 恢复、2 帧确认 | **未按原样实现**：改为 2.8 V 电压-only 硬欠压（只关 DSG，3.0 V/2 帧恢复）+ 参考节<3.1 V/SOC<10% 低电充电状态（3.1 V+SOC10/2 帧退出；>200/<120 mA DSG 窗口）；SOC 标定有效，低电状态 RAM-only。VC5/BAT 不可信时参考节改为 VC2 |
| C-15 | 换电检测、expected_swap、三连击清除、5 s 换电模式 | 手势检测能产生 HOLD5/RELEASE/TRIPLE，但 App 只消费 PAIR_HOLD3；swap flags 全为死定义 |
| C-16 | StateStore 在状态转换/30 min 写入并在启动恢复 | 30 min 归档仍写；低电状态为 RAM-only，`CUTOFF_LATCH` 不再写入/恢复；`expected_swap/swap_error/dead_storage/charge_full` 仍未使用；不会恢复电压/SOC |
| C-17 | 黑匣子记录充/放/充满/换电/死存储/LP 进入/唤醒 | 只有周期记录和部分保护/ADC/CAL 事件；无充放电/换电/死存储；`LP_ENTER` 只有旧解码；wake mask 只打包 3 bit，CHARGER/MONITOR 被截断 |
| C-18 | `shelf_monitor`、`BQ_SetBalanceMask`、`BQ_ReadDieTempDeciC`、`PowerMode_ServiceLp/GetWakeEvents`、旧 `BQ_LpSniffRead` | 无 `shelf_monitor`；上述 API 缺失或更名；实际为 `lp_cycle/lp_session/watchdog/protection`、`PowerMode_Loop/RequestLp`、`BQ_LpBootPrepare/BQ_LpMonitorRun`、`BQ_LpSniffRead(reg,out)` |
| C-19 | 短按/长按/三连击/5 s 全部有业务动作 | 检测齐全，但只消费 PAIR_HOLD3；SHORT/HOLD3/HOLD5/RELEASE/TRIPLE 未处理 |
| C-20 | 充放电 SOC 渐变快闪、CUTOFF/SWAP/REPLACE/DEAD 灯效；多专用音型及优先级 | **SOC 渐变已实现**：STANDBY 5 Hz/60%、DISCHARGE 常亮/95%、CHARGE 1 Hz/95%、充满白色常亮；CUTOFF/SWAP/REPLACE/DEAD 专用灯效仍缺。蜂鸣器仍只有 BOOT_OK/FAULT/LP_ENTER/WAKE，无优先级队列 |
| C-21 | `POWER_LP_KEEP_SWD` 默认 1 | 实际默认 0（`app.mk:20`、`power_mode.c:32-33`） |
| C-22 | BQ FET 默认全关 | 运行期 `ChargePolicy_Init()` 请求 0、IDLE 保持关；但宏 `BQ_DEFAULT_FET_MASK=BQ_FET_MASK_ALL`，注释/初始化窗口有误导 |
| C-23 | BQ 在充/放 250 ms、IDLE 500 ms 读；ADC 重启判定窗 500 ms | 固定 500 ms 读；ADC 重启 verdict 1500 ms；2 次尝试、4 好帧清 |
| C-24 | 5 V 唤醒延迟 2 ms（风险项） | `APP_5V_STARTUP_DELAY_MS=20`，旧风险已关闭；仍建议按负载实测 |
| C-25 | 原计划未包含 VC5/BAT 降级采样与假开关 | 当前控制值真实可信优先、异常 VC1/VC2 估算；记录 real-or-0；`App_FakeSwitchTask()` 同值重写 `SYS_CTRL2` + 同方向提示音，2.5 s 重试，不改 FET/保护 |
| C-26 | D7：外部 I2C 上拉已具备；PB5 外部 10 kΩ 上拉 | **已确认（2026-11-11）**：外部 I2C/PB5 上拉存在；D-PULL1 已决 |

---

## G4 未决事项

以下事项不能在文档中擅自“裁决”；当前实现采用左侧实际行为，但产品/硬件是否接受仍需确认：

- `D-CHEM1`：**仍未决**。目标电芯 LFP 与当前 NMC 占位参数冲突；保护阈值/OCV 表由谁、何时切换？切换前必须禁止把 SOC 当硬保护。
- `D-PULL1`：**已决（2026-11-11）**。外部 I2C/PB5 上拉确认存在；不再按“仅内部上拉”评估。
- `D-LP1`：**已决（2026-11-11）**。接受当前 ~30 s 复位式 LP 充电唤醒，不恢复 RTC WUT 1 s。
- `D-BAL1`：**已决（2026-11-11）**。均衡永久关闭，不恢复 BQ 被动均衡。
- `D-PROT1`：**已决（2026-11-11）**。软件 OCP 恢复为充电 5.5 A/1 s、放电 33 A/500 ms；包温度门控 4.0–54.0 °C，充放电都生效，2 °C 回差/2 s 去抖。
- `D-SWAP1`：换电/死存储/SOC cutoff 是 L1 必做还是后续里程碑。
- `D-WAKE1`：黑匣子 wake 3-bit 截断、CHARGER/MONITOR 丢失事件是否本轮修。
- `D-API1`：文档按实际 API 名，还是把计划 API 作为兼容别名补回。

---

## 0. 方案摘要

> **状态（2026-11-11）**：`[部分实现]`。目标仍是 3S 1500 mAh 电池组；当前固件只交付了 G1 列出的子集，完整 L1 行为见 G2/G3。下文“目标”保留原计划，不表示已实现。

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

**目标状态核对**

| 目标 | 状态 | 当前情况 |
|---|---|---|
| 1 空闲自动 LP；充放电保持 NORM | 部分 | 5 s 自动 LP 已实现；充/放保持 NORM；LP tick 是 IWDG ~30 s，见 C-01 |
| 2 充电器插入自动唤醒并进 CHARGE | 部分 | 可唤醒，但最坏约 30 s，不是 1.5 s，见 C-02 |
| 3 放电必须手势激活 | 部分 | 只有 PAIR_HOLD3 切换；无 ARM 状态，见 C-08/C-19 |
| 4 充放电双管全开 | 已实现 | CHARGE/DISCHARGE 都请求 `BQ_FET_MASK_ALL` |
| 5 采样/SOC/SOH/均衡/保护 | 部分 | 采样/SOC/SOH/保护子集有；均衡永久关闭，见 C-10 |
| 6 5% 报警切断/10% 恢复 | 改为低压方案 | 2.8 V 电压-only 硬欠压 + 3.1 V/SOC10 低电充电（>200/<120 mA DSG 窗口），见 C-14 |
| 7 30 min 监测/存档；0% 死存储/SHIP | 部分/未实现 | 30 min 有；DEAD/SHIP 自动路径无，见 C-05/C-07 |
| 8 换电检测/换电模式/三连击 | 未实现 | 手势检测有，业务动作未接线，见 C-15 |
| 9 toolkit SOC/SOH | 部分 | 已集成，`CAL_VALID=1`，但 OCV/tau 仍有占位，见 C-09 |
| 10 保护失效回双管全关 | 部分 | 现有保护路径会收紧；完整 L1 状态/路径未实现 |

### 0.1 已确认的关键决策

| 编号 | 决策 | 当前状态/裁决 |
|---|---|---|
| D1 | 充电和放电都必须 **CHG=1、DSG=1 双管全开**，不允许依赖体二极管 | 已实现于现有 CHARGE/DISCHARGE 策略 |
| D2 | 放电最大 30 A，充电最大 5 A；BQ 硬件阈值和软件 OCP 都按此上限设计 | 软件 OCP 已恢复为 5.5 A/1 s、33 A/500 ms 余量；BQ 阈值≈37/44.7 A；D-PROT1 已决 |
| D3 | LP、1 s 充电嗅探、30 min 储存监测使用 **HSI16**；NORM 使用 HSE/PLL 80 MHz | 时钟部分已实现；1 s 嗅探未实现，LP 为 ~30 s IWDG，见 C-01/C-02 |
| D4 | 充电检测采用 1 s RTC 轮询：BQ `LOAD_PRESENT` 初判，PA1/COMP2 复核方向 | 冲突：每次 ~30 s 轻量 boot 才检查一次，见 C-02 |
| D5 | BQ FET 默认全关，可由状态机/调试变量显式打开；配置结束状态为“CC 开、双管关” | 运行期已实现；宏 `BQ_DEFAULT_FET_MASK=ALL` 注释有误导，见 C-22 |
| D6 | 均衡“常开”指充电/放电期间持续启用自动均衡；按 BQ 相邻节规则单节轮换；BQ die 温 65 °C 超限停止 | 冲突：均衡永久关闭，API 删除，见 C-10 |
| D7 | 外部 I2C 上拉已具备；REGSRC 已接 12 V；OCD/SCD 已实测 | 外部 I2C/PB5 上拉已确认；OCD/SCD 软件配置存在；REGSRC 仍需按实物确认（D-PULL1 已决） |
| D8 | LiFePO4 参数标定推迟到 M3 前完成；标定前 SOC 只用于显示/日志，不作为 5%/0% 的唯一依据 | 部分：`CAL_VALID=1`，SOC 参与低电判断/显示但 2.8 V 独立兜底；当前是 NMC 表，LFP 标定未完成，见 C-09 |
| D9 | 30 A/5 A、双管全开、BQ die 温 65 °C、SCD/OCD 均需在 M6 复测 | 未验证：无 M6 记录；die 温/均衡路径不存在 |
| D10 | CAN1 保持初始化但不实现业务；L1 的 NORM 80 MHz 时基已经满足后续 CAN 需求 | 已保持初始化，无 CAN 业务 |

---

## 1. 硬件与 L0 基线

> **状态（2026-11-11）**：`[部分实现/冲突]`。3S/VC5 映射和主要外设已实现；外部 I2C/PB5 上拉已确认存在；LSE 和电芯化学体系仍与原计划不一致或待确认，见 C-03/C-09。

### 1.1 主要硬件

| 项目 | 说明 |
|---|---|
| MCU | STM32L432KBU6，UFQFPN32，Cortex-M4F |
| AFE | BQ7692003PWR，3S，I2C1 0x08，CRC=Yes，内部 3.3 V LDO |
| 主回路 | 负端 BTT_N → 1.5 mΩ 分流 → 背靠背 N-FET → PACK_N_0V |
| 电池 | **目标** 3S LiFePO4 1500 mAh；**当前固件参数**是 NMC 1500 mAh 占位表，标定前不能当 LFP 安全参数 |
| 5 V | LMR54410，PB0 使能；CAN 收发器、RGB、蜂鸣器、UART 连接器 |
| 3.3 V | HT7533，由电池侧 VBAT_12V 供电，常开 |
| 充电方向 | COMP2：PA3 `SC_BTTN` 对 PA4 `SC_PACK`；COMP2 高 = 充电 |
| BQ ALERT | PA8，EXTI **event rising**；不能唤醒 STOP1，只能在轻量 boot 轮询（C-04） |
| 按键 | PB5，低有效，下降沿 EXTI；外部上拉已确认存在（D-PULL1 已决） |
| 12 MHz HSE | 外部 OSC1，经 Q5 三态控制，PA1=0 使能 |
| RTC | 目标 LSE 32.768 kHz；当前板 LSE 不起振，默认 LSI；LP 由 IWDG 复位，不是 RTC WUT（C-01/C-03） |

> **I2C/PB5 上拉**：外部 I2C/PB5 上拉已确认存在（2026-11-11，D-PULL1 已决）；MCU 内部上拉仍使能，但不作为唯一上拉依赖。LP 嗅探/按键可靠性按外部上拉评估。

### 1.2 L0 已具备（按当前树更新）

- HSE bypass 12 MHz → PLL 80 MHz；HSI16 冷启动；
- I2C1 + DMA + 每字节 CRC 的 BQ 驱动；
- BQ 电压/电流/温度读取，CC 250 ms 平均；
- COMP2 充放电方向检测；
- RGB（USART2 3.8095 MBd）、蜂鸣器、状态灯、黑匣子；
- 250 Hz TIM2 应用时基；
- **LP 基础（与原文不同）**：`PowerMode_RequestLp()` 写 BKP START token 后软件复位；轻量 boot 在 HSI16 做 BQ sniff/充电器/PB5/PA8 检查/30 min 标记，按 IWDG timeout 进 STOP1；IWDG 超时复位继续，真正唤醒写 WAKE token + 软件复位。RTC WUT/LPTIM1 关闭，RTC 默认 LSI；
- 调试日志、自检、故障位、复位原因；
- 已实现的 StateStore/BMS NVM/保护监督/charge policy；未实现完整 L1 状态机、均衡、换电/死存储等（见 G3）。

### 1.3 L1 不重复实现

- 不重写 HAL/CubeMX 外设初始化；
- 不在 `Core/` 里写业务代码；
- 不使用 RTOS、动态内存或阻塞式长任务；
- 不把 CAN 业务、上位机协议作为 L1 前置条件。

### 1.4 L1 非目标

- 不实现 CAN 报文协议、主机工具或云连接；
- 不实现充电器 CC/CV 控制，只通过 CHG/DSG 允许或禁止充放电；
- 不实现主动均衡，只使用 BQ 内部被动均衡；**当前实现进一步永久关闭被动均衡（C-10）**；
- 不增加电池 ID/EEPROM 硬件，换电识别只做电压启发式；
- 不实现无线升级/Bootloader；
- 不在 LFP 参数标定完成前把 SOC 作为唯一硬保护依据；
- 不追求 µA 级整板静态电流，只对 MCU + BQ 可控部分设定目标并实测。

---

## 2. 软件分层与模块

> **状态（2026-11-11）**：`[部分实现/已替代]`。原计划的 `shelf_monitor` 不存在，30 min 归档由 `App_ArchiveTask()` 和 `LpCycle_DoMark()` 分担；`timebase.c`、`bq_anomaly.*` 已删除（见 C-18/C-21）。

沿用参考工程的 `App/` 分层。**当前实际树**：

```text
App/
├─ board.h                 外设句柄唯一 extern 入口、板级常量、Flash 分区
├─ bsp/
│  ├─ bq_ic.c/.h           BQ 驱动、FET、保护、LP 阻塞嗅探/监测
│  ├─ button.c/.h          PB5 采样/消抖/EXTI 唤醒
│  ├─ flash_store.c/.h     Flash 擦写封装
│  ├─ ws2812_uart.c/.h     RGB
│  └─ buzzer.c/.h          蜂鸣器
├─ comm/
│  └─ app_log.c/.h         调试日志（CAN 业务不实现）
├─ detect/
│  ├─ gesture.c/.h         短按/长按/配对/三连击检测
│  ├─ soc_est.c/.h         BQ 快照 → SOC/SOH 适配
│  └─ bms/                 li-ion-soc-soh-toolkit 核心（NMC 占位）
├─ app/
│  ├─ main_app.c/.h        初始化、主循环、250 Hz tick、30 min NORM 归档
│  ├─ power_mode.c/.h      NORM 时钟、LP 请求、唤醒原因、DBGMCU
│  ├─ lp_cycle.c/.h        复位式 IWDG LP、轻量 boot、30 min LP 标记
│  ├─ lp_session.c/.h      BKP LP 会话 token
│  ├─ rtc_time.c/.h        RTC 绝对时间（LSI 默认/LSE 可选）
│  ├─ watchdog.c/.h        IWDG 启动/运行/LP timeout
│  ├─ protection.c/.h      OV/UV/SCD/OCD、软件 OCP、温度、FET 允许 mask
│  ├─ state_machine.c/.h   BOOT/NORM/FAULT + COMP2 flow
│  ├─ state_store.c/.h     跨掉电状态（48 B v2，页 50–51）
│  ├─ black_box.c/.h       24 B 黑匣子（页 52–63）
│  ├─ charge_policy.c/.h   IDLE/CHARGE/DISCHARGE/CAL/FAULT + FET 请求
│  ├─ led_status.c/.h      RGB/PB3/PB4
│  ├─ faults.c/.h          故障位/ sticky
│  ├─ self_test.c/.h       上电自检
│  ├─ reset_cause.c/.h     复位原因
│  └─ cal_stream.c/.h      调试 CSV 流
└─ app.mk
```

**原计划中未出现/已替代**：

- `shelf_monitor.c/.h` — 不存在；30 min 归档由 `App_ArchiveTask()`（NORM）和 `LpCycle_DoMark()`（LP）分担。
- `app/timebase.c` — 已删除；TIM2 直接在 `main_app.c` 初始化。
- `bq_anomaly.c/.h` — 已删除；没有 partial/anomaly 旁路框架。
- 均衡 API（`BQ_SetBalanceMask`）— 已删除；均衡永久关闭。

依赖方向：

- `app/` → `detect/`、`comm/`、`bsp/`
- `detect/`、`comm/` → `bsp/`
- `bsp/` → HAL + `board.h`
- `detect/` 不依赖 `comm/`
- ISR 只置标志、计数、启动 DMA；Flash、协议、状态机、蜂鸣器都在主循环

---

## 3. 运行模式

> **状态（2026-11-11）**：`[部分实现/冲突]`。NORM 80 MHz、STOP1、5 V 开关、HSI16 轻量 boot 已实现；LP 不是 RTC WUT 1 s，而是 IWDG 复位式 ~30 s（C-01/C-02/C-05）。

### 3.1 NORM

- HSE bypass 12 MHz → PLL 80 MHz，Scale1，4WS；
- 5 V 打开（`APP_5V_STARTUP_DELAY_MS=20`；原计划 2 ms 已关闭，C-24）；
- BQ NORMAL；
- TIM2 250 Hz、BQ 轮询、SOC/SOH、状态机、灯、蜂鸣器、日志运行；
- **BQ 请求固定 500 ms 一次**（原计划“充/放 250 ms、IDLE 500 ms”未实现，C-23）；
- 充电或放电时保持 NORM，不自动 LP；
- 无充放电、无按键 5 s 后请求 LP。

### 3.2 LP（当前实现：复位式 IWDG）

> **原计划目标**：STOP1 + RTC WUT 1 s 周期嗅探。**当前实现不是这个架构**；本节按实际改写，原目标保留为未来项（C-01/C-02/C-05）。

- NORM 空闲 5 s 后 `PowerMode_RequestLp()` → `LpCycle_RequestStart()`：写 CRC 保护的 BKP `START` 会话 token，然后 `NVIC_SystemReset()`；不在原上下文里就地切时钟。
- 下一次 boot 先在 HSI16 分类：START/RESUME 走轻量 `LpCycle_Run()`，不执行 `App_Init()`；真实唤醒写 WAKE token + 软件复位，走完整 `App_Init()`。
- 轻量 boot：
  - 关 5 V、PA1 高（关 PACK_SC/HSE 前端）、LED 灭；PB4 每次轻量 boot 打 10 ms 心跳；
  - 读 RTC epoch（绝对时间参考）；默认 LSI，`LP_RTC_USE_LSE=1` 才走 LSE；
  - `BQ_LpBootPrepare()`：有界阻塞 I2C 读 BQ，默认 `SYS_CTRL1=0`、`SYS_CTRL2=0`（ADC/TEMP/CC/FET 全关）；
  - `LOAD_PRESENT && !CHG_ON` → PA1=0、30 ms、COMP2 start、30 ms、读方向；判定为充电则写 WAKE token + 软件复位；
  - PB5 EXTI 检查；PA8 只读 `SYS_STAT`，不清 OV/UV 保护；唤醒只用 SCD/OCD/XREADY；
  - 到 30 min 标记则内联执行 `BQ_LpMonitorRun()`：临时开 ADC/TEMP+CC、两帧独立确认、写黑匣子/StateStore，然后关回；
  - 按 RTC 剩余时间用 `Watchdog_SetTimeoutMs()` 配 IWDG（基础 30 s，扣除 boot 开销），进 STOP1（`STOPWUCK=1`）；
  - IWDG 超时复位；下一次 boot 用 `STOPPED` token + RTC 间隔识别 RESUME，继续 LP。
- **没有 RTC WUT/LPTIM1**：两者已显式停用/屏蔽；`RTC_WKUP_IRQHandler` 只清 pending/WUTF，且 WUT 从未布防。
- **没有 LP 类型**：IDLE/CUTOFF/SWAP/REPLACE/DEAD 未实现；DEAD/SHIP 也没有自动路径。
- **没有 1 s 嗅探**：充电器/PB5/PA8 检查只在每个 ~30 s 轻量 boot 执行；充电唤醒最坏约 30 s，不是 1.5 s。
- **低频/低功耗**：轻量路径只写 BKP 会话和 30 min 标记；每次轻量 boot 的检查不写 Flash；调试构建可用 `POWER_LP_KEEP_SWD=1`，默认 0。

### 3.3 时钟与切换原则（复位式）

- 冷启动：HSI16 → 外设初始化 → `PowerMode_ClkSwitchToHsePll()`；
- NORM → LP：不在原上下文切时钟；排空 BQ 后写 START token + 软件复位；
- LP 轻量 boot：HSI16；只关 HSE/PLL/5 V，不重新打开 HSE；
- LP → NORM：真实唤醒写 WAKE token + 软件复位 → 完整 boot → HSE/PLL 80 MHz → 5 V/外设；
- App 代码不得直接调用生成的 `SystemClock_Config()`；
- 每次 SYSCLK 变化后必须重建 SysTick；轻量路径在 STOP1 返回后先 `HAL_InitTick` 再使用 HAL_Delay/Flash；
- 原计划的“8 MHz 以下重新标定”在当前复位式架构中不适用。

### 3.4 启动顺序

> **状态**：`[已实现/复位式]`。整体顺序保留；LP 唤醒不是恢复原外设，而是完整软件复位后重新 `App_Init()`。

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
   - 若从 LP 唤醒，写 WAKE token 后走完整 `App_Init()`；`LpCycle_ApplyWake()` 注入唤醒原因/计数（不是原地恢复外设）；
4. 自检完成前不进入自动 LP；
5. 任何初始化失败必须进入 FAULT/降级，不允许带故障继续充放电。

### 3.5 RTC 时基（当前实现）

- RTC 只作绝对时间参考；异步 127、同步 255；**不承担 WUT 周期唤醒**。
- 当前板 LSE 不起振：默认 `LP_RTC_USE_LSE=0`，RTC 走 LSI；LSI 未校准，30 min/15 天只是相对时间，且与 IWDG 共享 LSI。
- `LP_RTC_USE_LSE=1` 时 `RtcTime_MspClockSource()` 启动 LSE；LSE 失败回退 LSI 保 boot，但 `PowerMode_LpTimebaseReady()` 要求 LSE，自动 LP 被禁用并置 LSE/TIMEBASE 故障。
- 第一次 backup-domain boot 种子 2001-01-01；RTC 写入/读取等待 RSF。
- RTCSEL 改变触发一次备份域复位，清 BKP 会话；禁止运行中热切换。改 RTC 源后必须接受 BKP 丢失。
- 原计划的 WUT `WUTE/WUTIE` 检查和“WUT 中断只置标志”在当前实现中**不适用**：WUT 从未布防；`RTC_WKUP_IRQHandler` 只清 pending/WUTF，LPTIM1 也关闭。

---

## 4. 业务状态

> **状态（2026-11-11）**：`[部分实现/冲突]`。当前没有完整的 L1 业务状态机；实际只有 SM 的 BOOT/NORM/FAULT 和策略层的 IDLE/CHARGE/DISCHARGE/CAL/FAULT（C-08）。下表保留为原计划目标，除已列出的实际状态外均为 `[未实现-未来]`。

`IDLE/CHARGE/DISCHARGE/...` 是业务状态；NORM/LP 是电源模式，二者叠加，不混为一谈。

**当前实际状态模型**

| 层 | 当前状态 | 行为 |
|---|---|---|
| `state_machine` | BOOT / NORM / FAULT | BOOT 等首批有效 BQ；NORM 正常；FAULT 由关键故障位进入，清故障后回 NORM/BOOT |
| `charge_policy` | IDLE / CHARGE / DISCHARGE / CAL / FAULT | IDLE 双管关；CHARGE/DISCHARGE 双管全开；CAL 是 Ozone 调试；FAULT 强制关 |
| 电源 | INIT / NORM / DEGRADED | DEGRADED 只闪故障灯，不跑业务 |

**原计划完整状态表（未来目标；当前只实现上表子集）**

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

> **状态**：`[已实现]`（现有状态范围内）。CHARGE/DISCHARGE 都请求 CHG+DSG；IDLE 请求 0；保护允许 mask 始终 AND；CAL 可用调试变量单独验证。原计划的 ARM/CUTOFF/SWAP/DEAD 状态未实现，因此没有这些状态的 FET 行为。

- 充电和放电：**DSG=1、CHG=1**，双管全开，方向只由电流/COMP2 判断；
- 其余现有状态：双管全关；
- 配置脚本以 `SYS_CTRL2=0x40`（CC 开、FET 关）结束，运行期由 `request_fet_mask` 同步；
- 宏 `BQ_DEFAULT_FET_MASK=ALL` 仍存在，但 `ChargePolicy_Init()` 请求 0，运行期默认关（C-22：建议改宏或注释）；
- 任何保护动作优先于手动 FET 请求。

### 4.2 事件优先级

> **状态**：`[部分实现]`。实际优先级为 FAULT > CAL > 充电器 > 用户放电 > IDLE；没有 DEAD/SWAP，也没有 ARM。PAIR_HOLD3 只在非充电、非 CAL、非 FAULT 且 BQ/自检/策略就绪时生效。

```text
当前：FAULT > CAL > 充电器 > PAIR_HOLD3 用户放电 > IDLE
原计划（未来）：关键故障/DEAD > SWAP_ERROR > 充电器 > 按键手势 > 空闲 5 s
```

- 充电器出现时，终止放电（当前直接清 `user_discharge_enabled`），进入 CHARGE；
- 充电器插入正在放电时，保持双管全开，转 CHARGE；
- 充电器移除后，用户需重新做放电手势；
- 原计划“充电中长按禁止换电”尚未实现，因为换电模式本身未实现。

---

## 5. 状态转换

> **状态（2026-11-11）**：`[部分实现/冲突]`。当前只实现下面的“实际转换子集”；原完整表是未来目标，不能当作现有行为。C-08/C-13/C-14/C-15/C-16。

**当前实际转换子集**

| 起点 | 事件/守卫 | 动作 | 终点 |
|---|---|---|---|
| BOOT | 首批 BQ fresh/valid | 进入 NORM；策略 IDLE，双管关 | NORM/IDLE |
| BOOT/NORM | 关键故障位 | 双管关、FAULT、阻止自动 LP | FAULT |
| FAULT | 关键故障消失且 BQ fresh | 回 NORM 或 BOOT | NORM/BOOT |
| IDLE | COMP2 消抖充电器（CHARGE 中净充电电流 >30 mA 可保持）+ 自检/BQ/电源就绪 | 双管开 | CHARGE |
| CHARGE | 充电器移除连续 5.2 s（平衡充短暂断流不误判） | 双管关 | IDLE |
| IDLE | PAIR_HOLD3（短按后 2 s 内二次 3 s 长按） | 双管开、WAKE 音 | DISCHARGE |
| DISCHARGE | 再次 PAIR_HOLD3 | 双管关、LP_ENTER 音 | IDLE |
| DISCHARGE | COMP2 充电器 | 双管开、WAKE 音 | CHARGE |
| DISCHARGE | 进入后前 10 s 净充电电流 ≥100 mA（≤90 mA 释放） | 双管开、WAKE 音 | CHARGE |
| 任意 NORM 空闲 | 5 s 且无 LP block | 写 START token、软件复位 | LP 轻量 boot |
| LP START/RESUME | 轻量 boot/IWDG 超时 | 继续 LP；真实唤醒写 WAKE token + 软件复位 | LP 继续/完整 boot |

> **DISCHARGE 前 10 s 充电检测（2026-11-11）**：仅作为“充电器存在”的补充判据，不是新状态。窗口从 `state_enter_ms` 起算；使用与 CHARGE 相同的“充电为正”净电流（`charge_current_ma`），置位 100 mA / 释放 90 mA，无效电流帧不置位也不清除。BQ 500 ms 读取 + 250 ms CC 平均，因此最多约一个读周期后生效。窗口结束后清零闩锁，之后仍可由 COMP2 触发 DISCHARGE→CHARGE。负载大于外部充电时净电流可能达不到 100 mA（漏检）。切换后 `user_discharge_enabled=0`，拔掉充电器走正常 CHARGE→IDLE，不自动回 DISCHARGE。

**原计划完整转换表（未来目标；当前未实现）**

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

> **状态**：`[已实现为提示]`（C-13，2026-11-12）。`charge_full` 只在 RAM，`STATE_FLAG_CHARGE_FULL` 仍未使用；无专用充满提示音，白灯指示由 `LED_FLOW_CHARGE_FULL` 完成。

- 最低节电压 `>= 4.15 V`（NMC；电压腿不依赖 SOC 标定）或标定有效时 `SOC >= 98%`；
- 且充电电流在 `[-20, 75] mA`（允许未标定 CC 零点偏置）持续 30 s；
- 只在 `comp_charging=1` 且策略为 `CHARGE` 时评估；充满只置 `charge_full`/白灯/事件 0x28，**不关闭 CHG**；
- 充电器/COMP2 移除后立即清除；电流 >150 mA 连续 2 帧清除；复位不恢复；
- BQ OV 4.25 V/cell 是最后硬件保护（原计划 3.70 V 是 LFP，不适用当前 NMC，见 C-09）。

---

## 6. 按键手势

> **状态（2026-11-11）**：`[部分实现]`。`button`/`gesture` 检测层已实现下表全部手势；**业务层只消费 `GESTURE_PAIR_HOLD3`**。SHORT/HOLD3/HOLD5/RELEASE_AFTER_HOLD5/TRIPLE 目前没有动作（C-19）。外部上拉已确认存在（D-PULL1 已决）。

PB5 低有效、EXTI 下降沿；外部上拉已确认存在（2026-11-11）。

| 手势 | 定义 |
|---|---|
| 消抖 | 电平稳定 20 ms（4 ms 采样 5 次） |
| 短按 | 按下 20–600 ms 后释放 |
| 长按 3 s | 持续 3000 ms，事件只发一次 |
| 长按 5 s | 持续 5000 ms，事件一次；第 5 s 立即提示音 |
| 换电确认 | 5 s 事件后 0–500 ms 内释放 |
| 三连击 | 从第一次按下开始 1000 ms 内完成 3 次短按 |
| 放电配对 | 第一次短按释放后 2000 ms 内第二次按下，并保持 3000 ms |

**当前实际动作**：

- `GESTURE_PAIR_HOLD3` → `ChargePolicy_OnPairHold3()`：在非充电/非 CAL/非 FAULT 且就绪时切换 DISCHARGE/IDLE；充电中拒绝。
- 其余事件在 `main_app.c` 的 `Gesture_GetEvent()` 分支中没有处理；HOLD5/TRIPLE 不会写 StateStore，也不会触发换电/清除。

关键规则（原计划；未实现部分见状态）：

- LP 唤醒键算第一次短按；若唤醒后仍按住，按下时间从唤醒时刻开始；
- 唤醒后一直按到 3 s/5 s，发 HOLD3/HOLD5，不再补发短按；
- DISCHARGE_ARM 中禁止 HOLD5，避免误进换电模式；
- 5 s 换电只在 IDLE 有效；充电/放电/ARM/SWAP 不响应换电；
- 任何按键事件刷新 5 s 空闲计时；手势进行中不自动 LP；
- 三连击要等第三次释放后判定；包电压非法时只报错，不清状态。

---

## 7. LP、充电检测与储存监测

> **状态（2026-11-11）**：`[部分实现/冲突]`。5 s 自动 LP、充电器/PB5 唤醒、30 min 监测存在；但 LP 是 IWDG 复位式 ~30 s，不是 RTC WUT 1 s；LP 类型/DEAD/SHIP 未实现（C-01/C-02/C-05/C-06/C-07）。

### 7.1 LP 类型

**当前只有一种复位式 LP 会话**，没有按业务状态分型：

| 当前 LP | BQ 配置 | FET | 周期 | 用途 |
|---|---|---|---|---|
| 统一 reset LP | `SYS_CTRL1=0`、`SYS_CTRL2=0`（ADC/TEMP/CC/FET 全关） | 全关 | IWDG ~30 s | 充电嗅探、PB5/PA8 检查、30 min 标记 |

原计划的 IDLE/CUTOFF/SWAP_ERROR/REPLACE_READY/DEAD_STORAGE 五种 LP 类型均为 `[未实现-未来]`；DEAD 的 SHIP 路径也未实现。

说明（按当前实现）：

- LP 电流不达标时，当前已采用“只 sniff/30 min 临时开 ADC”的方案，而不是保持 ADC/TEMP 常开；
- LP 期间双管全关；**没有**通过调试变量在 LP 保持 FET 导通的路径；`BQ_PrepareForLpMask()` 是遗留 API，当前 LP 路径不调用；
- LP 进入前 `LpCycle_RequestStart()` 等待 BQ idle/I2C 空闲最多 100 ms；轻量 boot 的 BQ 准备失败会完整唤醒 FAULT，不进入 STOP1。

### 7.2 当前嗅探流程（~30 s 轻量 boot；不是 1 s WUT）

1. IWDG 超时复位（或 NORM 的 START 软件复位）→ HSI16 轻量 boot；不开 5 V、不切 HSE/PLL；
2. `RTC` 读 epoch（绝对时间；默认 LSI）；
3. `BQ_LpBootPrepare()` 用 HSI16 有界阻塞 I2C 读 BQ，默认强制 `SYS_CTRL1=0`、`SYS_CTRL2=0`；
4. 检查 `LOAD_PRESENT`（只在 `CHG_ON=0` 时有效；LP 双管全关满足）：
   - `0`：继续 PB5/PA8/30 min 检查；
   - `1`：临时 `PA1=0` 使能 PACK_SC，等约 30 ms；`HAL_COMP_Start`，再等约 30 ms 读 COMP2；
5. COMP2 高 = 充电：写 WAKE token + 软件复位，下一次完整 boot 进 CHARGE；低 = 负载/噪声：停 COMP2、PA1=1，继续 LP；
6. PB5 EXTI 检查；PA8 只读 `SYS_STAT`，不清 OV/UV 保护；LP 唤醒只认 SCD/OCD/XREADY；
7. 嗅探只使用有界阻塞 I2C（≤50 ms/次），不改 NORM DMA；
8. 轻量 boot 不启动 5 V/RGB/蜂鸣器；PB4 只打 10 ms 心跳；Flash 只在 30 min 标记写；
9. 原计划“每秒 COMP2 兜底”未实现：只有 `LOAD_PRESENT=1` 才做 PA1/COMP2；若充电器不置 LOAD_PRESENT，当前固件不会检测到（D-LP1/C-02）。

> **原计划目标（未实现）**：RTC WUT 1 s 唤醒、每秒 LOAD_PRESENT 初判 + PA1/COMP2 复核、≤1.5 s 充电唤醒。参见 C-01/C-02。

### 7.3 30 min 储存监测

**当前实现**：

- 每个 IWDG ~30 s 轻量 boot 计算 RTC 是否跨过 30 min 标记；跨过则内联调用 `BQ_LpMonitorRun()`；
- 在 HSI16 下临时打开 BQ ADC/TEMP+CC，等待 300 ms，取两帧独立读数（间隔约 300 ms、±4 raw LSB 确认），然后恢复 LP 关断态；
- 写一条黑匣子周期记录和 StateStore 标记（同 mark id 去重）；不更新 SOC/SOH；
- 正常监测不完整唤醒；监测失败才 `LP_WAKE_MONITOR|LP_WAKE_FAULT` 完整唤醒；
- 监测期间不检查按键/ALERT/充电器，不会立即中止；
- 黑匣子页擦除只在 30 min 记录跨页时发生；
- SOC 只在 NORM 的新 BQ 快照上更新，LP 监测不喂 SOC。

> **原计划中“更新 SOC/SOH（M3 后）”和“监测期间立即中止走正常唤醒”为 `[未实现-未来]`**（C-07/C-20）。

### 7.4 空闲 5 s 回 LP

> **状态**：`[已实现]`。当前只有 IDLE/CHARGE/DISCHARGE/CAL/FAULT 策略状态；没有 DISCHARGE_ARM/CUTOFF/SWAP 专用 5 s 逻辑。

- 主循环每圈检查 `last_activity_ms`；
- 按键、充电、放电启动/停止、状态变化刷新活动时间；
- 无充电、无放电、无手势、无故障，且 `auto_lp_block==0` 时，5 s 后 `PowerMode_RequestLp()`；
- 充电/放电（`ChargePolicy_IsLpBlocked()`）不自动 LP；
- 未实现 DISCHARGE_ARM/5 s 换电/CUTOFF/SWAP 的专用超时；
- 充电器移除后充电策略回到 IDLE，再按 5 s 计时进入 LP。

### 7.5 死存储

> **状态**：`[未实现-未来]`（C-05/C-14）。`BQ_ShipMode()` 驱动 API 存在但无调用者；`STATE_FLAG_DEAD_STORAGE` 只有定义；没有 DEAD LP 类型、20 s WUT 或自动充电恢复。以下保留原计划供后续实现。

- 进入条件：SOC==0（库输出钳位）且最低节 ≤2.5 V，或包压 ≤7.5 V，持续 10 s；
- 动作：双管关、均衡关、5 V 关、LOG/RGB/蜂鸣器停；
- 写 `dead_storage` 到 StateStore；
- BQ 写 SHIP：`SYS_CTRL1=0x01` 后再写 `0x02`；
- MCU STOP1 + RTC 20 s；
- 每 20 s：PA1 短时拉低 + COMP2 嗅探充电；无充电回 STOP1；
- 有充电：PB1 高 ≥2 ms 启动 BQ，重配，清 dead，转 CHARGE；
- 3.3 V LDO 仍由电池供电，死存储不是整板零电流，需实测 LDO/分压静态。

### 7.6 LP 调试

> **状态**：`[部分实现/冲突]`。默认已从计划的 1 改为 **0**（C-21）；LP 诊断字段也与原计划不同。

- `POWER_LP_KEEP_SWD` 默认 **0**（`app.mk:20`、`power_mode.c:32-33`）；STOP1 期间不保留 SWD 时钟；
- 需要调试时显式 `make POWER_LP_KEEP_SWD=1`，并在改宏后 `make clean`；
- 调试开关只影响调试，不改变唤醒源、BQ 配置或 FET 策略；
- 当前可见：`s_power.state/request_lp/wake_cause/boot_reset_cause/lse_ok/degraded/wake_count/lp_disable_auto`，`s_lp.*` 会话字段，`s_app.*`，BQ 快照；**没有** `lp_stage` 或 `s_lp_sniff.*`。

### 7.7 BQ 配置与数据有效性

> **状态（2026-11-11）**：`[已实现/冲突]`。3S 映射、ADC 配置、ADC 恢复、CC 换算已实现；OV/UV 阈值是 NMC 占位（4.25/2.80），不是原计划的 LFP 3.70/2.50（C-09）。VC5/BAT 降级与假开关是计划外新增，见 7.7.1/7.7.2。

#### 3S 电压映射

- CELL1 = VC1 − VC0；
- CELL2 = VC2 − VC1；
- CELL3 = VC5 − VC4；
- VC4−VC3、VC3−VC2 必须短接；
- BAT 为电池组总压。

当前固件按 V0.80 飞线硬编码 `BQ_REG_CELL3=BQ_REG_VC5`，读取 VC1/VC2/VC5 及诊断 VC3/VC4；没有 4S/VC3 兼容开关。

#### 7.7.1 当前 VC5/BAT 可信/估算策略（`[计划外新增]`，C-25）

原计划没有这个策略；当前树为板级 VC5/BAT 间歇异常增加了以下规则：

- 发布门槛只看 `voltage_settled` 与 VC1/VC2 1.0–5.0 V；VC5/BAT/VC3/VC4 不阻塞发布。
- 真实 VC5 可信：VC5 换算值 1.0–5.0 V，且本批 VC5 字段 CRC/格式有效。
- 真实 BAT 可信：BAT 3.0–15.0 V、cell3 可信，且 `|BAT - (VC1+VC2+real_cell3)| <= 300 mV`。
- **控制/显示值逐字段选择**：真实 cell3 可信则用实测，否则 `(VC1+VC2+1)/2`；真实 BAT 可信且 cell3 可信则用实测，否则 `VC1+VC2+最终cell3`。SOC/SOH 与保护参考节不直接使用该控制值，而用 `BQ_GetRefCellMv()`：正常为三节最低值，`log_cell3_valid==0 || log_pack_valid==0` 时只用实测 VC2。
- **记录值**：只写真实可信值；异常字段写 0 并置 missing/quality；估算值不落盘。事件/唤醒/转换记录同样清空异常 cell3/pack。
- **BQ 硬件 OV/UV 过滤**：正常只用 VC1/VC2 实测；VC5/BAT 不可信时只用实测 VC2。Protection 的 2.8 V、低电和充满参考节走 `BQ_GetRefCellMv()`，正常时包含实测 cell3。
- 估算不持久化：复位后由下一帧 VC1/VC2 重新生成；没有“估算值”归档标志。

#### 7.7.2 VC5/BAT fake-switch 恢复（`[计划外新增]`，C-25）

- 触发：每个新的完整 BQ 批次且 VC1/VC2 有效、`log_cell3_valid==0 || log_pack_valid==0`。
- 动作：播放与当前输出方向相同的提示音（输出非零用 WAKE，全关用 LP_ENTER），并调用 `BQ_ApplyFetMaskIfUnchanged(current applied mask)` 同值重写 `SYS_CTRL2`。
- 外层触发不再因 `voltage_backoff_active/pending/failed`、`request != applied` 或 `batch.ctrl2` 与影子不一致而跳过；异常新批次一到就立即尝试。
- 安全条件：BQ RUN/idle、无 FET/SYS_STAT 同步、无 ADC 重启/恢复/verdict/resample、`request_fet_mask==fet_mask_applied`、实际 `SYS_CTRL2` 与期望一致且 `CC_ONESHOT=0`；绝不改变 FET 状态或保护。同值重写不能覆盖 BQ 硬件 OV/UV 锁存。
- 重试：动作后 2.5 s 内不重复；门控跳过用 50 ms 短重试；蜂鸣器忙/自检未完成/保护非 NORMAL/CAL/FAULT/按钮/30 min 标记/LP 临近 <3 s 时跳过。
- 诊断：`s_app.fake_switch_*`、`[VSW]`。这是板级临时缝补，不是 VC5 模拟通道修复。

#### 电压有效范围（当前）

| 量 | 有效范围 | 说明 |
|---|---|---|
| VC1/VC2 | 1.0–5.0 V | 发布门槛；OV 4.25/UV 2.80 均在范围内 |
| 真实 cell3 | 1.0–5.0 V | 可信才进控制/记录 |
| 真实 BAT | 3.0–15.0 V | 且与 `VC1+VC2+真实cell3` 残差 ≤300 mV |
| VC3/VC4 | raw 诊断 | 不参与控制/保护/发布 |
| 控制 cell3/pack | 真实可信或估算 | 保护不直接使用；记录 real-or-0 |

#### 上电配置（当前）

1. PB1 高脉冲 ≥2 ms 退出 SHIP，随后回低并等待 ≥20 ms；
2. 写 `CC_CFG=0x19`；
3. 读 `ADCGAIN1/ADCGAIN2/ADCOFFSET`，换算增益/偏移；
4. 写 `SYS_CTRL1=0x18`（ADC_EN|TEMP_SEL），清 `SYS_STAT`；
5. 写 `SYS_CTRL2=0x40`（CC_EN，FET 全关）；
6. 写保护：
   - `PROTECT1=0x89`（RSNS=1，SCD 67 mV，约 44.7 A/100 µs）；
   - `PROTECT2=0x57`（OCD 56 mV，约 37.3 A/320 ms，放电方向）；
   - `PROTECT3=0x40`（OV delay 1 s，UV delay 4 s）；
   - **当前** OV_TRIP/UV_TRIP 按 **4.25 V/2.80 V** 和 GAIN/OFFSET 反算（NMC 占位）；原计划 3.70/2.50 未切换；
7. 每项写后读回；失败重试配置，连续失败置 `FAULT_BQ_COMM`；
8. ADC 开启后等待 ≥300 ms 再取第一帧；
9. 只清 `CC_READY/XREADY/OVRD_ALERT`（0xB0），不清 OV/UV/SCD/OCD 保护锁存。

#### 数据有效性（当前）

- `fresh=0`/`valid=0` 的帧不更新 SOC/StateStore/状态机；
- 发布只校验 VC1/VC2 1.0–5.0 V；VC5/BAT/VC3/VC4 异常不阻塞；
- 连续 8 个 VC1/VC2 越界帧请求一次受控 ADC 重启；
- 每个上电周期最多 2 次 ADC 重启；每次 verdict 窗 **1500 ms**（原计划 500 ms，C-23）；
- 两次失败置 `FAULT_BQ_ADC_STUCK`，停止自动重启，仅保留采样；
- 连续 4 个有效发布帧后清 ADC 故障；
- 启动/LP 唤醒/ADC 重启后的前两帧要求 VC1/VC2 ≤±4 raw LSB；三次不匹配后退回粗确认；
- 运行期粗确认使用单节 250 mV/电池 300 mV；
- OV/UV 等保护只从有效帧映射；VC5 假 OV/UV 在 VC1/VC2 安全时过滤/延迟清除；真实 OV/UV 保持锁存。

#### CC 换算

- BQ CC 1 LSB = 8.44 µV；
- 分流 1.5 mΩ；
- `current_ma = cc_raw × 844 / 150`（250 ms 平均）；
- 电流方向由 `BQ_CC_DISCHARGE_SIGN` 统一（当前 `-1`，TI 充电为正 -> SOC 放电为正）；
- CC 零点/增益标定在 M0/M3 完成，未标定前记录原始 `cc_raw`。

---

## 8. 充电、放电、均衡与保护

> **状态（2026-11-12）**：`[部分实现/冲突]`。充/放双管、FET 策略、软件 OCP/温度门控已实现；充满提示、SOC 渐变、2.8 V 硬欠压和 3.1 V/SOC10 低电充电（>200/<120 mA 双管窗口）已接入；原 5%/10% SOC 截止未按原样实现，均衡永久关闭，NMC 参数仍有占位成分（C-09/C-10/C-11/C-12/C-13/C-14/C-20）。

### 8.1 充电（当前实现）

- 进入：COMP2 消抖检测到充电器，且自检 PASS、SM NORM、BQ RUN/fresh、电源 NORM；设置 `CHARGE_POLICY_CHARGE`，正常请求 CHG+DSG 双管全开；
- **MOS 导通后的 COMP2 兜底**：COMP2 差模只有 `0.266×I×(Rshunt+Rds_on)`，100 mA 时约 0.3 mV；CHARGE 中充电电流 >30 mA 时保持充电器存在，<20 mA 释放，20–30 mA 回差。该判据只影响充电器存在，不改 Protection OCP。
- **近满体二极管子模式**：显示 SOC>90 且 −20 mA < 充电电流 < 220 mA 时请求 CHG-only（DSG 关），利用 DSG 体二极管压降 + COMP2 判充电器；电流 >320 mA 或 SOC<85% 退出。SOC 为未标定启发值，不作保护输入。
- 充电开始音：当前使用通用 `BUZZER_PATTERN_WAKE`（5 kHz × 80 ms），不是计划中的 3 kHz × 150 ms；
- 采样：BQ 固定 500 ms（C-23）；SOC 500 ms；没有均衡；
- RGB：SOC 渐变（红→黄→绿，满量程 48），CHARGE 以 1 Hz 慢闪、亮度 95%；充满时改为白色 (38,38,38) 常亮；
- **充满提示**：最低节≥4.15 V 或标定 SOC≥98%，且充电电流 −20..75 mA/30 s → `charge_full`/白灯/事件 0x28；**不关 CHG**，近满体二极管子模式仍是独立策略；
- 充电器移除：正常 CHARGE 中 COMP2 低且无电流保持连续 **5.2 s** 才撤销存在（`CHARGE_POLICY_CHARGER_LOW_MS=5200`；原 1 s 会被平衡充的短暂输出关断误判为移除，造成 CHARGE/IDLE 反复）；电流 >30 mA 时先由电流保持；体二极管模式同样用 5.2 s；代价是真实拔充电器后最多 5.2 s 才回 IDLE，期间策略仍是 CHARGE、双管保持请求，Protection/数据/温度/故障仍即时；清 `user_discharge_enabled`，回 IDLE/双管关；
- 充电中按键：策略拒绝 PAIR_HOLD3；没有换电模式；
- 充电过流：软件 OCP **5.5 A/1 s**，至少 2 个新发布帧确认后锁存；BQ 无充电方向硬件 OCP；D-PROT1 已决；
- 充电温度：包温度门控 **4.0–54.0 °C**，充放电都收紧；2 °C 回差/2 s 驻留，`PROT_TEMP_ENABLE=1`。

> **原计划目标（未实现/未来）**：3.0 kHz 开始音、250 ms BQ、均衡 5 s 复评、SOC 渐变快闪、充满 4 kHz×2、充电器移除 5 s。参见 C-11/C-12/C-13/C-20。

### 8.2 放电（当前实现）

- 激活：IDLE 中 PAIR_HOLD3（第一次短按释放后 2 s 内第二次按下并保持 3 s）→ `CHARGE_POLICY_DISCHARGE`，请求 CHG+DSG 双管全开；
- 动作音：通用 WAKE（5 kHz×80 ms），不是低-中-高；
- RGB：SOC 渐变常亮（红→黄→绿，满量程 48，亮度 95%）；无有效 SOC 时回退蓝色 48；
- 退出：再次 PAIR_HOLD3 → IDLE/双管关，通用 LP_ENTER 音；
- 充电器优先：放电中插充电器立即转 CHARGE、保持双管全开；拔掉后需重新手势；
- 前 10 s 外部充电检测：进入 DISCHARGE 后 10 s 内净充电电流 ≥100 mA（释放 90 mA）也直接转 CHARGE，用于 COMP2 差分过小、无法识别的平衡充/小电流充电器；10 s 后仍只靠 COMP2。切换后按正常 CHARGE 电流保持（30/20 mA）与近满体二极管逻辑运行；
- **没有原计划的 5% 切断/10% 恢复**：改为 2.8 V 电压-only 硬欠压（只关 DSG，3.0 V/2 帧恢复）和 3.1 V/SOC10 低电充电状态；低电状态默认 CHG-only，>200 mA/2 帧且≥2.8 V 才开 DSG，<120 mA 或电流无效立即关，3.1 V+SOC10/2 帧退出；不持久化，复位重建；
- 放电过流：软件 OCP **33 A/500 ms**，至少 2 个新发布帧确认后锁存；BQ 硬件 OCD≈37.3 A/320 ms，SCD≈44.7 A/100 µs；D-PROT1 已决；
- 没有 DISCHARGE_ARM 状态；PAIR_HOLD3 之前没有 2 s 配对窗口的可视状态，只有 gesture 内部计时。

> **原计划目标（未实现/未来）**：ARM→DISCHARGE 独立状态、L-M-H/H-M-L 音型、SOC 渐变、5% 三声长鸣/双管关/cutoff 锁存、10% 恢复。软件 OCP 已恢复 33 A/500 ms。参见 C-11/C-14/C-20。

### 8.3 均衡（当前实现）

> **状态**：`[冲突-已裁决]`（C-10）。当前均衡**永久关闭**：每次开 ADC/CC、进 LP、SHIP 前写 `CELLBAL1=0` 并回读；`BQ_SetBalanceMask()` 已删除；`BQ_SetTempSource()` 存在但无调用者；没有 die 温读取 API。

- 当前没有任何 `CELLBAL1` 非零写入；没有 3.35 V/30 mV 判据、30 s 单节轮换、die 温 65 °C 保护。
- 原计划的均衡实现先保留为未来目标；恢复前必须解决 VC5 采样、均衡期间电压读数、die 温读取和板级温升。
- 均衡电流约 30 mA（100 Ω + 内部 FET）等原计划说明仍是未来约束，不是当前行为。

### 8.4 电流/短路保护（当前值 vs 原计划）

| 项目 | 原计划 | 当前实现 | 状态 |
|---|---|---|---|
| 分流电阻 | 1.5 mΩ | 1.5 mΩ | 已实现 |
| BQ OCD | 56 mV/320 ms≈37 A | `PROTECT2=0x57`，≈37.3 A/320 ms，放电方向 | 已实现 |
| BQ SCD | 67 mV/100–200 µs≈44.7 A | `PROTECT1=0x89`，≈44.7 A/100 µs | 已实现 |
| 软件放电 OCP | 33 A/500 ms | **33 A/500 ms，至少 2 个新发布帧** | 已恢复（D-PROT1） |
| 软件充电 OCP | 5.5 A/1 s | **5.5 A/1 s，至少 2 个新发布帧** | 已恢复（D-PROT1） |
| 最大放电 | 30 A | 30 A 目标；温升未验证 | 部分 |
| 最大充电 | 5 A | 5 A 目标；温升未验证 | 部分 |

> **当前余量**：软件 OCP 已恢复为 5.5 A/1 s 与 33 A/500 ms，高于 5 A/30 A 最大工作电流；阈值/时间仍需台架复核。方向由 COMP2 消抖 + CC 幅值判定；`|I|>60 A` 视为不可信帧。软件 OCP 锁存需显式 `Protection_RequestLatchClear()`；充电 OCP 还要求 `comp_charging==0`。

- BQ SCD 是 µs 级硬件比较器；CC 250 ms 平均不能替代短路保护；
- SCD 触发后固件读 `SYS_STAT`，双管关，进入 FAULT/Protection 锁存，不自动重合闸；
- 软件 OCP/SCD 恢复必须显式清锁存并满足安全条件；
- 外部保险丝仍作为后备。

### 8.5 保护阈值（计划 vs 当前）

| 项目 | 原计划 | 当前实现/说明 |
|---|---|---|
| 单节满充 | 3.55 V + taper 75 mA/30 s，或 SOC≥98% | **已实现为提示**：4.15 V 或 SOC≥98% + −20..75 mA/30 s（NMC），白灯/事件，不关 CHG（C-13） |
| BQ OV | 3.70 V/cell，1 s | **4.25 V/cell**，1 s（NMC 占位，C-09） |
| BQ UV | 2.50 V/cell，4 s | **2.80 V/cell**，4 s（NMC 占位，C-09） |
| 包合法范围 | 6.0–11.0 V；单节 2.0–3.8 V | 发布只要求 VC1/VC2 1.0–5.0 V；BAT 信任 3–15 V + 残差≤300 mV；无 6–11 V 控制窗 |
| 充电温度 | 0–45 °C（原计划） | **包温度 4.0–54.0 °C，充放电都收紧；`PROT_TEMP_ENABLE=1`** |
| 放电温度 | −20–60 °C（原计划） | **同上，4.0–54.0 °C；2 °C 回差/2 s** |
| 均衡 die 温 | 65 °C，5 °C 回差 | 未实现；均衡永久关闭 |
| 5% 放电切断 | SOC≤5% + 最低节≤2.8 V | **未按原样实现**：2.8 V 电压-only 硬欠压只关 DSG；参考节<3.1 V 或 SOC<10% 进入低电充电，>200/<120 mA 控制 DSG，3.1 V+SOC10/2 帧退出（C-14） |
| 0% 死存储 | SOC=0 + 最低节≤2.5 V，10 s | **未实现**（C-05） |

> **安全警告**：`SOC_EST_CAL_VALID=1`，SOC/SOH 参与低电判断/显示，但 2.8 V 电压硬保护仍是独立兜底；OCV/tau 仍有占位成分。当前 NMC 4.25/2.80 参数不能用于 LFP 安全认证。切 LFP 前必须替换 OCV/R0/R1/τ、OV/UV 和保护策略（D-CHEM1）。

### 8.6 BQ SHIP / 恢复

> **状态**：`[部分实现/未接线]`。`BQ_ShipMode()` 存在（关双管→`SYS_CTRL1=0x01`→`0x02`），但当前没有调用者；没有 DEAD 状态或自动恢复路径（C-05）。

- SHIP 入口：双管已关，写 `SYS_CTRL1=0x01`，再写 `0x02`；中间不得插其他 SYS_CTRL1 写；
- CRC/超时失败重试；成功后 I2C 可能不再响应；
- 退出：PB1 高 ≥2 ms → 低 → 等 20 ms → 完整 BQ 配置；
- PB1 之后必须保持低，避免短接 NTC；
- 外部 I2C/PB5 上拉已确认（D-PULL1 已决）；REGSRC 仍需按实物确认；
- SHIP 不会切断 MCU 3.3 V，需实测整板静态电流。

### 8.7 故障管理与安全状态

> **状态**：`[部分实现]`。当前故障位/FAULT/Protection 行为是 L0/M2 子集；没有完整 L1 状态机。

故障分为 live（实时）和 sticky（锁存）：

| 类别 | 例子 | 当前动作 |
|---|---|---|
| BQ 通信 | CRC 错、I2C 超时、配置失败 | `FAULT_BQ_COMM`，重试/总线恢复；持续失败→FAULT |
| BQ 器件 | DEVICE_XREADY | 双管关、FAULT、等待安全恢复 |
| BQ 保护 | OV/UV/SCD/OCD | 双管关；Protection 监督清锁存后才允许重开；boot/LP 唤醒 SCD/OCD 可在安全条件后清除 |
| ADC/数据 | 连续无效、ADC 重启失败 | 数据不发布；锁存 `FAULT_BIT_BQ_ADC_STUCK`；2 次重启/4 好帧清 |
| 温度 | NTC 无效/超限 | 包温度 4.0–54.0 °C 充放电都收紧；无效/超限置 `FAULT_BIT_TEMP`，`PROT_TEMP_ENABLE=1`（C-12） |
| 电源 | PVD 跌落 | sticky `FAULT_POWER`；`Fault_PowerRecoverTask()` 1 s 无新边沿后自动清 sticky，计数保留 |
| 时钟 | HSE/PLL 切换失败 | 降级只闪故障灯，禁止充放电；`FAULT_BIT_CLK` |
| RTC/LSE | LSE 未起振/WUT 布防失败 | 默认 LSI 不复位；`LP_RTC_USE_LSE=1` 时 LSE 失败回退 LSI 并禁用自动 LP；没有 WUT |
| Flash | 黑匣子/StateStore 写失败 | 置 `FAULT_BIT_FLASH_LOG/FLASH_STATE`；不继续依赖该存储 |
| 软件 OCP | 充电 5.5 A/1 s、放电 33 A/500 ms，至少 2 帧 | 双管关、Protection 锁存；需显式清 |
| 硬件 SCD | 短路尖峰 | 双管关、FAULT 锁存、不自动重合闸 |

**BQ ALERT / PA8（当前）**：

- ALERT 是 `SYS_STAT` 的 OR；清 ALERT 必须写 1 清源位；
- PA8 配置为 EXTI **event rising**，不能唤醒 STOP1；轻量 boot 读 `SYS_STAT`，只把 SCD/OCD/XREADY 作为唤醒原因；OV/UV/CC_READY/OVRD_ALERT 不作为 LP 唤醒（C-04）；
- 保护锁存不能为了“消 ALERT”随手清掉。

**统一原则**：

- 关键故障 → 双管关、退出充放电；当前没有均衡可关；
- FAULT 不自动进入 LP，保留 5 V/灯/蜂鸣器提示；
- 故障消失且条件安全后才允许清 BQ 锁存/恢复 FET；
- 不允许依赖体二极管继续导通；
- 故障事件写黑匣子（当前覆盖保护/ADC/CAL 子集，见 C-17）；
- **IWDG 已实现**：NORM `/64`≈8.2 s 死机保险；轻量 boot `/256`≈32.8 s 启动保护；LP 进入前写 calibrated timeout 并停止喂狗，让 IWDG 复位形成 LP tick；
- HardFault/NMI 有复位/故障路径，不允许无提示自旋。

---

## 9. SOC/SOH 工具包集成

> **状态（2026-11-12）**：`[部分实现/冲突]`。toolkit、适配层、NVM 已集成；NMC 1500 mAh 的 R0/R1 已按 25 °C 放电实测接入，OCV/tau 仍有占位，`SOC_EST_CAL_VALID=1`。SOC/SOH 用于低电判断、遥测、显示、归档和 LED 渐变；2.8 V 电压仍是独立硬保护；LP 监测不更新 SOC/SOH（C-09/C-14/C-20）。

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

> **状态**：`[已实现]`（数值匹配）。化学体系是 NMC；R0/R1 为 1500 mAh 放电实测，OCV/tau 仍为占位，不是 LFP。

- `BMS_USE_TICK=1`
- `BMS_USE_LIBM_EXPF=0`
- `BMS_USE_NVM=1`
- `BMS_USE_TUNE=0`
- `SOC_CAPACITY_NOMINAL_MAH=1500`
- `SOC_R0/R1_25C_MOHM`：1500 mAh NMC 放电实测（满电 + 每 130 mAh 一组，按 1500 mAh 映射 100/91.33/.../22%，线性插值到 10% 网格，0–20% 端点平延）；5/45 °C 暂复用 25 °C；R0 为放电方向，充电方向共用基线并另学 delta；
- `SOC_TAU_25C_S`：本次不改，保持原库占位；测量值 83–138 ms 与秒表/500 ms EKF 步长未对齐
- `BMS_REF_CUR_MA=300`（0.2C）
- `BMS_UPDATE_PERIOD_MS=500`
- `SOC_QUIET_DEBOUNCE=2`
- `SOH_IDLE_DEBOUNCE=2`
- `SOH_R0_TAIL_N=2`

### 9.3 适配层（当前 API）

`SocEst_Init()`：

```text
BMS_CAL_Init → SOH_Init → BMS_NVM_Init
```

`SocEst_OnSnapshot()`：

- 只在新 BQ 快照（`last_update_ms` 变化）时调用；
- 输入 `v_mv = min(cell_mv[0..2])`，不传 pack_mv；
- 输入 `i_ma = current_ma`，符号由 `SOC_EST_CURRENT_SIGN=BQ_CC_DISCHARGE_SIGN` 统一；
- 温度由 TS1 NTC 换算；无效时 SOC 仍按 25.0 °C 占位，**跳过 SOH_Update**；
- 额外 gate：`|pack - sum(cells)| > 300 mV` 时拒帧（原计划未写）；
- 顺序：`BMS_CAL_Update(temp_dc)` → `SOC_AhUpdate(v_mv, i_ma)` → `SOH_Update(v_mv, i_ma, temp_dc)`；
- 实际 getter：`SocEst_GetSoc01()`、`SocEst_GetSocPercent()`、`SocEst_GetSoh01()`、`SocEst_GetSnapshot()`、`SocEst_IsValid()`、`SocEst_IsTempValid()`、`SocEst_Task()`；
- `SocEst_ResetForNewPack()` 存在但**无调用者**；`SocEst_PrepareForLp()` 存在但**无调用者**（NVM dirty 不保证 LP 前落盘）。

### 9.4 标定门槛

> **状态**：`[冲突-未决]`（C-09/D-CHEM1）。

- 当前 `App/detect/bms/bms_config.h`：**R0/R1 是 NMC 1500 mAh 25 °C 放电实测**（满电开始每 130 mAh 一组，按 1500 mAh 映射到 100/91.33/.../22%，线性插值到 10% 网格；0–20% 端点平延）；**OCV 仍是 NMC 3350 mAh 占位，tau 未接入**（测量值为 83–138 ms，与秒表/500 ms EKF 步长未对齐）；不是 LFP 实测表；
- `SOC_EST_CAL_VALID=1`，`SocEst_IsValid()` 在真实帧/温度有效时为 1；SOC/SOH 参与低电判断/显示，但 2.8 V 电压仍是独立硬保护；
- 原 5%/0% SOC cutoff 未实现；当前是 2.8 V 电压硬欠压 + 3.1 V/SOC10 低电充电（C-14）。充满电压腿 4.15 V 也不依赖 SOC 标定；
- BQ CC 零点/增益、TS1 NTC 参数、均衡补偿在 M0/M3 完成；均衡已永久关闭，均衡补偿暂不适用；
- 30 min LP 监测不调用 `SocEst_OnSnapshot()`，LP 期间 SOC/SOH 不更新（C-20）。

### 9.5 BMS NVM

> **状态**：`[部分实现]`（格式/接线匹配）；`SocEst_PrepareForLp()` 无调用者，LP 前 dirty 保存未接线。

- 132 B 记录不是 8 B 倍数，不能直接按 132 B 双字写；
- 132 B 拷入 136 B 缓冲，尾部 0xFF，再按双字写；
- 两页槽（页 48/49），擦页后写；读仍读 132 B；
- 写失败保留旧槽；
- 只在主循环 NORM 低频调用；LP 进入前若 dirty，应先保存（接口存在但未调用）。

---

## 10. Flash、持久化、换电与黑匣子

> **状态（2026-11-11）**：`[部分实现/冲突]`。Flash 分区、BMS NVM、StateStore、黑匣子格式已实现；StateStore 生命周期 flags/启动恢复、换电检测/清除/模式、黑匣子完整事件与 wake 映射未实现或存在缺陷（C-15/C-16/C-17）。

### 10.1 Flash 分区

| 页 | 地址 | 用途 | 当前状态 |
|---:|---|---|---|
| 0–47 | `0x08000000–0x08017FFF` | 应用代码，96 KB | 已实现；当前 `_image_end=0x08015700` |
| 48–49 | `0x08018000–0x08018FFF` | BMS SOC/SOH NVM | 已实现；132/136 B，两槽 |
| 50–51 | `0x08019000–0x08019FFF` | L1 StateStore | 已实现；48 B v2，两页 |
| 52–63 | `0x0801A000–0x0801FFFF` | 黑匣子 | 已实现；24 B×1020 |

- 链接脚本 `FLASH LENGTH=96K`；
- `_image_end` 必须 ≤0x08018000；当前 0x08015700 通过；
- 黑匣子必须 <0x0801A000；
- CubeMX 重生成后恢复 96K、`-include app.mk` 和链接断言。

### 10.2 StateStore

> **状态**：`[部分实现]`（C-16）。48 B v2 记录、两页环形、CRC、30 min 归档已实现；但生命周期 flags 没有写入者/读取者，启动不恢复，状态转换不写。

48 B 记录（v2 在 reserved[16] 中带归档元数据）：

```text
magic/version/seq, flags, last_cell_mv[3], last_pack_mv,
last_soc01, last_soh01, last_temp_dc, battery_epoch, crc16
```

flags：

- `expected_swap` — 仅定义，无写入/读取；
- `swap_error` — 仅定义，无写入/读取；
- `cutoff_latch` — 低电状态为 RAM-only；复位不写入/恢复旧 flag，按实时参考节/SOC 重建；
- `dead_storage` — 仅定义，无写入/读取；
- `charge_full` — 仅定义，无写入/读取（充满状态是 RAM-only）；
- `soc_valid` / `pack_valid` / `soc_estimated` / `pack_partial` — 30 min 归档时更新。

当前实际：

- 两页环形，跨页先擦；上电扫描最高有效 seq；
- 主循环只在 30 min 归档（`App_ArchiveTask`/`LpCycle_DoMark`）写；
- 没有状态转换写；
- 轻量 boot 不写 Flash（30 min 标记除外）；
- 启动不消费 flags/电压/SOC，也没有换电清除路径；`StateStore_ClearLatch()` 存在但无调用者；
- `STATE_ARCHIVE_MARK_*`/`ANOMALY`/`ALARM` 是旧 v2 元数据；当前归档只用 FULL/DEGRADED 质量，不产生 L1 报警/锁存。

### 10.3 换电检测

> **状态**：`[未实现-未来]`（C-15）。没有参考电压比较、`SWAP_ERROR` 状态或 expected_swap 处理。

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

> **状态**：`[未实现-未来]`。`GESTURE_TRIPLE` 会被检测，但 App 不消费；`SocEst_ResetForNewPack()` 和 `StateStore_ClearLatch()` 都没有调用者。

- `SWAP_ERROR` 中 1 s 内三次短按，且包合法；
- 擦 BMS NVM 页 48/49；
- `SocEst_ResetForNewPack()`；
- 清 swap/expected/cutoff；
- 写新 `battery_epoch`；
- 转 IDLE。

### 10.5 换电模式（长按 5 s）

> **状态**：`[未实现-未来]`。`GESTURE_HOLD5`/`GESTURE_RELEASE_AFTER_HOLD5` 被检测，但没有提示音、expected_swap 写入或 LP 动作。

- 仅 IDLE 有效；
- 按住 5 s：第 5 s 蜂鸣器 1 声；
- 5.0–5.5 s 内释放：写 `expected_swap`，蜂鸣器高-低-高-低，双管关，LP；
- 超过 5.5 s 释放：取消，不进入换电模式；
- 下一次上电必报 `SWAP_ERROR`；
- 未真正断电时，可用三次短按取消 `expected_swap`（需文档化）。

### 10.6 黑匣子

> **状态**：`[部分实现/冲突]`（C-17）。24 B 环、1020 条、周期记录、读取 API 已实现；事件覆盖不完整，wake 映射有 3-bit 截断缺陷。

- 保留 24 B 记录：seq/uptime/三节/pack/CC/TS1/flags/CRC；当前记录类型包括 periodic、legacy LP enter/wake、new wake mask、event。
- **已写事件**：CAL BEGIN/END、BQ ADC 重启/故障/恢复、Protection 触发/清除/失败/锁存/软件 OCP/温度抑制/恢复、wake mask。
- **未写事件**：充电开始/充满、放电开始/停止、5% 切断、换电报错/换电模式、死存储；`BB_EVENT_WAKE_CHARGER/MONITOR` 定义了但从不写；`BlackBox_WriteTransition()` 无调用者。
- 30 min 一条周期记录；状态转换不写；轻量 boot 不写（30 min 标记除外）。
- 1020 条容量需覆盖 ≥15 天；增加事件后必须重新核算写频率。
- 提供 `ReadNewest/ReadSlot/GetHead` 供台架和启动诊断。
- **wake 映射缺陷**：`BlackBox_WriteWake()` 只打包 `power_wake_mask & 0x07`，`POWER_WAKE_CHARGER(0x08)`/`POWER_WAKE_MONITOR(0x10)` 被截断为 0；解码只返回 RTC/BUTTON/BQ_ALERT。当前 `POWER_WAKE_*` 与 `BB_WAKE_*` 没有完整统一。
- 周期记录在异常 VC5/BAT 时 cell3/pack 写 0 + missing；估算不落盘（见 7.7.1/7.7.2）。

---

## 11. LED 与蜂鸣器

> **状态（2026-11-11）**：`[部分实现/冲突]`（C-20）。BOOT/IDLE/FAULT/LP 基本匹配；充放电没有 SOC 渐变/快闪；CUTOFF/SWAP/REPLACE/DEAD 灯效不存在；蜂鸣器只有四个通用音型，没有原计划的专用音型/优先级。

### 11.1 RGB / PB3/PB4

**当前实际灯效**

| 状态/事件 | RGB | PB3/PB4 |
|---|---|---|
| BOOT / 等首批 BQ | 蓝色 1 Hz 闪 | PB4 1 Hz |
| NORM/IDLE | SOC 红→黄→绿渐变，5 Hz 快闪，最大 29（约 60%）；无效 SOC 回退白 12 | PB4 常亮 |
| CHARGE | SOC 渐变，1 Hz 慢闪，最大 46（约 95%）；无效 SOC 回退绿 48 | PB4 常亮 |
| CHARGE_FULL | 白色 (38,38,38) 常亮（约 80%） | PB4 常亮 |
| DISCHARGE | SOC 渐变常亮，最大 46（约 95%）；无效 SOC 回退蓝 48 | PB4 常亮 |
| CAL | 静态琥珀 48/24（调试态） | PB4 常亮 |
| FAULT | 红色 5 Hz | PB3 5 Hz |
| DEGRADED | 只闪故障灯 | PB3/PB4 按降级路径 |
| LP | 灭 | 灭；PB4 每 ~30 s 轻量 boot 10 ms 心跳 |

**原计划目标（未实现）**

| 状态/事件 | RGB | PB3/PB4 |
|---|---|---|
| CHARGE | SOC 绿→黄→红，快闪 | PB4 常亮 |
| DISCHARGE | 同 CHARGE | PB4 常亮 |
| CUTOFF | 红快闪 | PB3 快闪 |
| SWAP_ERROR | 红/蓝交替快闪 | PB3 快闪 |
| REPLACE_READY | 琥珀/蓝慢闪 | PB4 |
| DEAD_STORAGE | 灭 | 灭 |

- 原计划 SOC 100% 绿、50% 黄、0% 红；当前没有 SOC 颜色映射；
- 充/放共用 SOC 渐变快闪未实现；充满显示满电绿未实现；
- RGB 只在 5 V/80 MHz NORM 可用；LP 全关（已实现）。

### 11.2 蜂鸣器

**当前实际音型**

| 事件 | 音型 |
|---|---|
| 自检通过 / BOOT_OK | 4 kHz ×2 短 |
| 故障 / FAULT | 2 kHz ×3 长 |
| CHARGE/DISCHARGE/CAL 进入、部分唤醒 | `BUZZER_PATTERN_WAKE`：5 kHz ×1 短 |
| 退出到 IDLE / LP_ENTER | 2.5 kHz ×1 |
| fake-switch | 按当前输出方向选 WAKE 或 LP_ENTER |

**原计划目标（未实现）**

| 事件 | 原计划音型 |
|---|---|
| 充电开始 | 3.0 kHz ×1，150 ms |
| 充满 | 4.0 kHz ×2 短 |
| 放电激活 | 2.0→3.0→4.0 kHz，各 200 ms |
| 放电退出 | 4.0→3.0→2.0 kHz，各 200 ms |
| SOC<5% | 2.0 kHz 长鸣 ×3（500 ms 响/200 ms 停） |
| 长按 5 s | 3.0 kHz ×1，150 ms |
| 换电模式 | 4.0/2.0/4.0/2.0 kHz，各 150 ms |
| 换电报错 | FAULT 长鸣 ×3 |

当前没有音型优先级/队列：`Buzzer_StartPattern()` 会覆盖当前 pattern；`Buzzer_IsActive()` 只用于 fake-switch/LP 门控。原计划的优先级“故障/低 SOC/换电错误 > 换电模式 > 充放电转换 > 启动/唤醒”未实现。

---

## 12. 调度与时序

> **状态（2026-11-11）**：`[部分实现/冲突]`。实际 BQ 固定 500 ms、LP 是 ~30 s IWDG tick、主循环没有 `shelf_monitor`；原计划的 1 s WUT/250 ms BQ/均衡任务不成立（C-01/C-18/C-23）。

**当前实际调度**

| 任务 | 周期/触发 | 上下文 |
|---|---|---|
| TIM2 tick | 4 ms（250 Hz） | ISR：COMP2 采样/消抖、灯相位、秒计数、置 BQ/diag/30 min 标志 |
| 按键采样/消抖 | 4 ms / 20 ms 稳定 | 主循环 `Button_Poll` |
| BQ 读 | **固定 500 ms** | 主循环 `BQ_RequestRead` + `BQ_Poll` DMA 状态机 |
| SOC/SOH | 500 ms 新发布快照 | 主循环 `SocEst_OnSnapshot`/`SocEst_Task` |
| 均衡复评 | **无** | 均衡永久关闭 |
| 空闲 → LP | 5 s | 主循环 block 检查后 `PowerMode_RequestLp` |
| LP tick/嗅探 | **~30 s IWDG reset** | 轻量 boot `LpCycle_Run` |
| 30 min 监测 | 30 min RTC 标记 | 轻量 boot 内联 `BQ_LpMonitorRun` |
| BMS NVM | 1 s 轮询 + 60 s 去抖 | 主循环 `SocEst_Task`/`BMS_NVM_Task` |
| 黑匣子 | 30 min + 部分事件 | 主循环；无充放电/换电/死存储事件 |
| fake-switch | 新 BQ 批次；动作后 2.5 s | 主循环最低优先级 `App_FakeSwitchTask` |

**当前主循环顺序**（`App_Loop`）：

1. `Watchdog_Feed()` + `Fault_PowerRecoverTask()`；
2. `PowerMode_Loop()`；
3. `Button_Poll()` → `Gesture_Poll()`（只派发 PAIR_HOLD3）；
4. BQ 标志/`BQ_Poll`/快照 → `SocEst_OnSnapshot`/`SocEst_Task` → BQ ADC 事件；
5. `SelfTest_Poll` + 自检完成音；
6. 1 s 诊断；
7. `Protection_Task` → `StateMachine_Update` → `ChargePolicy_Task` → `Protection_Service`/事件；
8. `CalStream_Task` + LED 模式；
9. `App_ArchiveTask`（30 min/待处理 LP 标记）→ `App_FakeSwitchTask`；
10. 自动 LP block 判断/请求；
11. `Buzzer_Poll` → `LedStatus_Flush` → `App_Log_Poll/FlushSmall`。

> 原计划中的 `ShelfMonitor_Task()` 不存在；其 30 min 归档职责已拆分。

**LP 电流目标（未验证）**

| 场景 | 目标 | 当前状态 |
|---|---|---|
| IDLE LP（~30 s 嗅探） | <100 µA 平均 | 无正式测量记录 |
| DEAD（SHIP + STOP1） | <10 µA（需实测 LDO/分压） | DEAD/SHIP 未实现 |
| 30 min 监测平均增量 | 可忽略 | 未测量 |
| 完整唤醒 | HSE 30 ms + BQ 配置 250 ms + 5 V 软启动 | 5 V 等待 20 ms；整体未测量 |

---

## 13. 文件与 API 规划

> **状态（2026-11-11）**：`[部分实现/已替代]`。原计划的很多文件/API 已实现但名字不同；`shelf_monitor`、均衡/die 温 API 不存在；部分 API 存在但无调用者（C-18）。

### 13.1 当前实际新增/存在的文件

- `App/bsp/button.c/.h` — 已实现
- `App/detect/gesture.c/.h` — 已实现（动作消费不完整）
- `App/detect/soc_est.c/.h` — 已实现
- `App/detect/bms/*` + `bms_port_stm32.c` + `bms_nvm_port.c` — 已实现
- `App/app/state_store.c/.h` — 已实现（生命周期 flags 未使用）
- `App/app/charge_policy.c/.h` — 已实现
- `App/app/cal_stream.c/.h` — 已实现
- `App/app/protection.c/.h` — 已实现（原计划树中未单列）
- `App/app/lp_cycle.c/.h`、`lp_session.c/.h` — 已实现（复位式 LP）
- `App/app/rtc_time.c/.h`、`watchdog.c/.h` — 已实现
- `App/app/shelf_monitor.c/.h` — **不存在**，职责被 main_app/lp_cycle 分担

### 13.2 当前实际修改

- `App/app/state_machine.c/.h`：仍只 BOOT/NORM/FAULT + flow；没有完整 L1 状态；
- `App/app/power_mode.c/.h`：NORM 时钟、LP 请求、唤醒注入；LP 在 `lp_cycle`；
- `App/app/main_app.c/.h`：初始化、主循环、500 ms BQ、SOC、30 min NORM 归档、fake-switch；
- `App/bsp/bq_ic.c/.h`：FET mask、保护、LP 阻塞嗅探/监测、VC5/BAT 降级；**没有均衡 API**；
- `App/app/led_status.c/.h`：BOOT/NORM/CHARGE/DISCHARGE/CAL/FAULT 子集；无 SOC 渐变/CUTOFF/SWAP；
- `App/bsp/buzzer.c/.h`：四个通用音型；无优先级队列；
- `App/app/black_box.c/.h`：24 B 记录/读取 API；事件和 wake 映射不完整；
- `App/board.h`、`App/app/faults.h`：Flash/电压/故障常量；没有死存储/换电运行路径；
- `app.mk`、`STM32L432xx_FLASH.ld`、`compile_commands.json`：96K 分区、源列表、构建参数已同步；
- `DOCS/POWER_MODE.md`、`DOCS/PINMAP.md`、`CLAUDE.md`：已按当前树更新为主。

### 13.3 关键 API：计划名 vs 当前

| 计划 API | 当前实际 | 状态 |
|---|---|---|
| `BQ_RequestFetMask`, `BQ_ApplyFetMaskNow` | 同名 | 已实现 |
| `BQ_SetBalanceMask` | **无**；`CELLBAL1=0` 强制 | 未实现/冲突 |
| `BQ_ShipMode` | 同名，但无调用者 | 部分/未接线 |
| `BQ_ClearProtection` | 同名；运行时多用 `BQ_ClearProtectionVerified` | 已实现，旧接口少用 |
| `BQ_ReadDieTempDeciC` | **无**；只有未接线的 `BQ_SetTempSource` | 未实现 |
| `BQ_LpSniffRead(sys_ctrl1, snap)` | `BQ_LpSniffRead(reg, out)` + `BQ_LpBootPrepare` + `BQ_LpMonitorRun` | 已替代 |
| `Button_Init/Poll`, `Button_NotifyExtiFromIsr` | `Button_Init/Poll`, `Button_TakeExtiFlag`/`Button_WakeInit` | 已实现/更名 |
| `Gesture_GetEvent` | 同名 | 已实现 |
| `SocEst_Init/OnSnapshot/ResetForNewPack/Task` | 同名；Reset 无调用者 | 部分 |
| `SocEst_GetPercent01` | `SocEst_GetSoc01` / `SocEst_GetSocPercent` | 已更名 |
| `SocEst_IsValid` | 同名；`CAL_VALID=1`，真实帧/温度有效时为 1 | 部分 |
| `StateStore_Get` | `StateStore_GetView` | 已更名 |
| `StateStore_Commit` | 同名，返回 int，签名不同 | 已实现 |
| `StateStore_ClearLatch` | 同名，无调用者 | 未接线 |
| `PowerMode_RequestLp` | 同名 | 已实现 |
| `PowerMode_ServiceLp` | `PowerMode_Loop` + `LpCycle_Run` | 已替代 |
| `PowerMode_GetWakeEvents` | `LpCycle_GetWakeCause` + `PowerMode_NoteLpFullWake` | 已替代 |

---

## 14. 分阶段实施计划

> **状态（2026-11-11）**：`[部分实现/未验证]`。M0 未关闭；M1 大部分；M2 部分但 LP 架构不同；M3/M4 部分；M5 未实现；M6 未开始。详见 G2。

### M0 — 硬件/标定冻结（1–2 周）

> **状态**：`[未关闭]`。没有 LFP 标定 CSV、30 A/5 A 温升、die 温/均衡记录；当前代码是 NMC 占位。外部上拉已确认；REGSRC 仍需按实物确认。

目标：把硬件风险和标定输入关闭，避免在错误参数上写策略。

工作：

- 确认 3S LiFePO4 1500 mAh、充电器 CV/CC、NTC BOM、FET/分流功率余量；
- **30 A/5 A 连续温升**：1.5 mΩ 分流约 1.35 W，Q1/Q2/Q3/PCB 温升；
- 外部上拉已确认（完成）；REGSRC→VBAT_12V 飞线仍需确认；
- **HSI16 下 BQ I2C 实测**：SCL/SDA 波形、CRC 成功率、1 s/30 min 可用性；失败先修 HSI16 时序，不能改用 HSE 做储存监测；
- BQ die 温度读取（`TEMP_SEL=0`、约 2 s 调度延迟、TS1 换算）和 65 °C 阈值；
- BQ SCD 67 mV/100–200 µs（RSNS=1）与 30 A 正常电流不误触发；
- 采集 LFP OCV/R0/R1/τ、BQ CC 零点/增益、TS1 温度表；
- 确认 5%/0% 电压、包合法范围、充电器插入时序；
- 输出硬件风险清单、测试记录、标定 CSV。

退出：30 A/5 A/SCD/die 温/HSI16 I2C 通过，标定 CSV 就绪。

### M1 — 安全基础（1–2 周）

> **状态**：`[大部分实现]`。96K/分区/StateStore/NVM/黑匣子/button/gesture/protection 已实现；均衡 API 删除、SHIP 未接线、StateStore 生命周期 flags 死。

- 96K 链接、`_image_end` guard、Flash 分区；
- StateStore（页 50–51）、BMS NVM（页 48–49）；
- BQ FET/均衡/SHIP/保护/温度源 API；
- Button/Gesture 基础；
- 黑匣子读取/事件类型；
- 不改变 L0 日常行为；验证构建、BQ 寄存器读回、FET 真值表、掉电状态恢复。

### M2 — LP 与充电检测（1–2 周）

> **状态**：`[部分实现/冲突]`。5 s 自动 LP、充电器唤醒、30 min 监测已实现；但 LP 是 IWDG ~30 s，不是 RTC WUT 1 s；充电唤醒最坏约 30 s；30 min 不更新 SOC/SOH（C-01/C-02/C-07）。

分为两个可验证小步：

**M2a — LOAD_PRESENT 诊断**

- BQ 配置结束为 CC 开、双管关；
- LP 嗅探读 `SYS_CTRL1.LOAD_PRESENT`（原计划 1 s；当前实现 ~30 s 轻量 boot）；
- 增加 `load_present_count/last_load_present` 等诊断；
- 验证 FET 全关时 LOAD_PRESENT 对充电器/负载的行为；
- 不启用自动充电唤醒、不自动 LP。

**M2b — 自动 LP + 充电唤醒 + 30 min 监测**

- NORM 5 s 空闲自动 LP；
- LP 嗅探 + `LOAD_PRESENT` → PA1/COMP2 复核（原计划 1 s；当前 ~30 s 轻量 boot）；
- 充电器完整唤醒，PB5/ALERT 保留；
- 30 min HSI16 监测；
- 5 s 空闲计时、充电中禁止 LP；
- 验证充电插入延迟、纯负载不误唤醒、LP 电流、无 Flash 写入。

退出：充电器插入 ≤1.5 s 唤醒；5 s 回 LP；LP 电流达标；30 min 记录正确。

### M3 — SOC/SOH（2–3 周）

> **状态**：`[部分实现]`。toolkit/500 ms/NVM 已接线；NMC R0/R1 已实测、OCV/tau 仍有占位、`CAL_VALID=1`、换电重置无调用者；无原 5%/0% SOC cutoff，已改为 2.8 V 电压硬欠压 + 3.1 V/SOC10 低电充电 + 充满提示（C-09/C-14）。

- toolkit 核心移植、LFP 占位表、min-cell 输入、NTC 温度；
- 500 ms SOC 更新、NVM pad、掉电 SOC 恢复；
- 换电重置；
- PC 工具回放/离线对比；
- 标定后替换 LFP OCV/R0/R1/τ，打开 `SOC_EST_CAL_VALID`。

退出：0.2C/0.5C/1C 充放电 SOC 单调、5%/10%/0% 阈值、SOH 有效性、掉电恢复。

### M4 — 充放电策略（1–2 周）

> **状态**：`[部分实现]`。CHARGE/DISCHARGE/FET/PAIR_HOLD3/软件 OCP 已有；FULL、ARM、5%/10%、均衡/die 温、专用灯效/音效缺失（C-10/C-11/C-13/C-14/C-20）。

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

> **状态**：`[未实现]`。只有 StateStore flag 定义；没有 swap 检测/expected_swap/三连击动作/5 s 换电/DEAD/SHIP（C-05/C-15/C-16）。

- 换电电压启发式、expected_swap、三连击清除；
- 5 s 换电模式；
- DEAD/SHIP、充电恢复；
- 换电/整板断电/同电压电池/非法包压测试；
- StateStore/黑匣子 500 次写入/掉电中断测试。

退出：换电错误可复现/可清除；SHIP 电流、PB1 boot、充电恢复正确。

### M6 — 系统验证（1–2 周）

> **状态**：`[未开始/未验证]`。无正式故障注入、30 A/5 A、die 温、LP 电流、EMC/循环记录；`WUT` 已不存在，改为复位式 LP。

- 故障注入：I2C 掉线、OV/UV/OCD/SCD、PVD、HSE、WUT、5 V、按键抖动；
- **30 A 放电、5 A 充电、短路尖峰、双管全开**复测；
- BQ die 温 65 °C 停均衡；
- HSI16 I2C 1 s/30 min 复测；
- LP 电流、充放电温升、EMC 初测、长期循环；
- 状态转换覆盖表、测试记录、最终参数表。

退出：全项验收通过，遗留风险关闭或记录。

---

## 15. 验证与验收

> **状态（2026-11-11）**：`[部分验证]`。构建/静态 E2 已通过；已有台架记录只覆盖部分 LP/ADC/假开关；完整 L1 验收未开始。没有 E3 记录的项目不能标“已验证”。

**当前验证状态**

| 项 | 状态 |
|---|---|
| 清理构建/镜像尺寸/链接断言 | E2 通过：87640/168/8200 B，`_image_end=0x08015700` |
| 按键手势检测 | E1/E2；只消费 PAIR_HOLD3 |
| LP 周期/PB5/30 min/BQ sniff | 已有台架记录（E3，历史），但周期是 ~30 s，不是 1 s |
| 充电唤醒 ≤1.5 s | **未满足**，实际最坏约 30 s |
| 30 A/5 A/SCD/温升/EMC | 未验证 |
| 5%/0%/换电/死存储/均衡 | 原 5%/0% 未实现；2.8 V 硬欠压/低电充电/充满提示未上板验证；换电/死存储/均衡未实现 |
| SOC/SOH 标定 | 部分：NMC R0/R1 已实测，OCV/tau 未标定 |

### 15.1 构建/静态

> **状态**：`[已通过 E2]`（2026-11-11）。

- `make clean && make -j2 all` 零警告零错误；
- `_image_end <= 0x08018000`；当前 `0x08015700`；
- 黑匣子/StateStore/NVM 地址不重叠；
- `app.mk` 包含所有新源；`compile_commands.json` 同步；
- CubeMX 重生成后恢复 96K、`-include app.mk`、RTC 选择补丁；
- toolkit C/PC 常量一致性检查：数值配置匹配，但化学体系是 NMC 占位，不能当 LFP 已校验。

### 15.2 按键/状态

> **状态**：`[部分实现]`。检测层可用；ARM/CUTOFF/SWAP/REPLACE/DEAD 转换不存在，无法做完整“每个状态转换注入”。

- 20 ms 消抖、短按、3 s/5 s、0.5 s 释放窗口、2 s 配对、1 s 三连击——检测层已实现；
- 当前只测 PAIR_HOLD3 → DISCHARGE/IDLE；HOLD5/TRIPLE/RELEASE 动作未实现；
- LP 唤醒键计入第一次短按（`Button_OnWake`）；
- 手势进行中/充电中不自动 LP；
- 完整 L1 状态转换覆盖表必须等 M5/状态机实现后补。

### 15.3 LP/充电

> **状态**：`[部分实现/冲突]`。已有台架记录覆盖 30 s IWDG tick、PB5 EXTI、BQ sniff/monitor；没有 1 s WUT、没有 DEAD/SHIP、没有 ≤1.5 s 充电唤醒记录。

- 断开调试器测 IDLE LP 电流；当前周期 ~30 s，不是 1 s；
- HSI16 BQ I2C 连接/CRC 成功率需按当前 HSI16 轻量路径复测；
- 充电器/电阻负载/空载矩阵验证 LOAD_PRESENT/COMP2；
- ~30 s 轻量 boot 不写 Flash（30 min 标记除外）、不产生 ALERT 风暴；
- 5 V 只在完整唤醒时打开；
- **充电插入延迟 ≤1.5 s 未满足**：实际最坏约一个 IWDG 周期（~30 s）。

### 15.4 充放电/保护

> **状态**：`[部分验证/多项未实现]`。

- 30 A/5 A 持续、双管栅极/温升/体二极管：未验证；
- SCD/OCD 配置与 30 A 不误触发：未做正式 30 A 记录；
- 软件 OCP：已恢复 5.5 A/1 s、33 A/500 ms（至少 2 帧）；仍需台架验证；
- 2.8 V 硬欠压/3.0 V 恢复、BQ UV-only CHG、3.1 V/SOC10 低电充电、>200/<120 mA DSG 窗口、充满白灯：**未上板验证**；原 5%/0% 不再实现；
- 均衡 die 温 65 °C/外部 NTC：均衡永久关闭，无 die 温路径；
- 30 A 分流耗散/R2/R26/Q1-Q3 温升：未验证。

### 15.5 SOC/SOH

> **状态**：`[未验证/化学体系冲突]`。

- 0.2C/0.5C/1C 充放电记录 V/I/T：无记录；
- 离线对比 SOC、5%/10%/0%、OCV 静置重同步、温度修正、SOH：无 LFP 标定；
- CC 零点/增益标定：未完成；
- 掉电 SOC 恢复：toolkit NVM 有，但 StateStore SOC 恢复无消费者；换电重置无调用者。

### 15.6 换电/死存储

> **状态**：`[未实现，无法验收]`。没有长按 5 s 动作、expected_swap、三连击清除、DEAD/SHIP。

- 长按 5 s 提示音、0.5 s 释放窗口；
- expected_swap 后上电必报错；
- 三次短按（含唤醒短按）清除；
- 热插拔/整板断电/同电压不同电池/非法包压/单节开路；
- 500 次 StateStore 写入/掉电中断；
- SHIP 电流、PB1 时序、充电恢复。

### 15.7 保护/故障

> **状态**：`[部分实现]`。现有 Protection/FAULT/ADC 恢复可做回归；WUT 已不存在，改为 IWDG 复位式 LP。

- BQ OV/UV/OCD/SCD、锁存清除、FET 安全 mask、软件 OCP；
- I2C 超时/CRC、HSE 失败、IWDG 复位、PVD、5 V 掉电、按键卡死；
- 拔电池/插充电器/组合按键；
- 确认两 FET 不会非预期同时导通。

---

## 16. 风险与待办

> **状态（2026-11-11）**：`[部分更新]`。原风险仍保留，下面是按当前实现更新后的版本；新增 C-25/VC5 降级相关风险。

1. **LFP 标定是硬阻塞**：当前只有 NMC 占位表；M3 前 SOC 不能作为 5%/0% 唯一依据。切 LFP 前必须替换 OCV/R0/R1/τ、OV/UV（D-CHEM1）。
2. **~30 s 充电检测不是“立即”**：当前每个 IWDG 周期（~30 s）才嗅探一次；原计划 1–1.5 s 目标未满足。若产品要求 ≤1.5 s，需要恢复 RTC WUT 1 s 或改板拆分 PA1/HSE_DIS 与 CHGSC_DIS。
3. **LOAD_PRESENT 需台架确认**：只有 `LOAD_PRESENT=1` 才做 COMP2；若充电器不置该位，当前不会唤醒。D-LP1 已决定接受 ~30 s 唤醒，不恢复 WUT 1 s。
4. **REGSRC 必须接 12 V**：否则 REGOUT、栅极驱动、SHIP 恢复不可信；外部上拉已确认，REGSRC 仍按实物确认。
5. **BQ SHIP 不切 3.3 V**：DEAD 是 SHIP + MCU STOP1，不是整板零功耗；需实测 LDO/分压。
6. **30 A/5 A 热与短路**：1.5 mΩ 30 A≈45 mV、1.35 W；OCD/SCD 必须高于 30 A；分流/FET/PCB 温升需实测。
7. **无电池 ID**：电压启发式无法识别电压几乎相同的不同电池；100% 识别需增加 ID 触点。
8. **Flash 磨损**：轻量 boot 不写 Flash；StateStore/黑匣子只在 30 min/事件写；核算页寿命。
9. **IWDG 已实现**：NORM `/64`≈8.2 s 死机保险；轻量 boot `/256`≈32.8 s 启动保护；LP 进入前写 calibrated timeout 并停止喂狗，让 IWDG 复位形成 LP tick。原“待评估”已关闭。
10. **5 V 唤醒延迟**：当前 `APP_5V_STARTUP_DELAY_MS=20`，原 2 ms 风险已关闭；仍建议按负载实测软启动。
11. **均衡热量/电压读数**：均衡永久关闭；原 100 Ω/30 mA/温升/SOC 补偿要求保留为未来恢复条件。
12. **SOC<0 语义**：toolkit 输出钳位 0..100；DEAD 用 SOC=0 + 最低节≤2.5 V + 10 s 定义；当前 DEAD 未实现。
13. **HSI16 BQ I2C**：当前每 ~30 s 轻量 boot 和 30 min 监测都依赖；M0 必须实测；失败先修时序，不把储存监测切 HSE。
14. **双管全开/30 A**：必须 DSG=1/CHG=1；VGS/温升/体二极管不导通/老化影响需记录。
15. **BQ die 温与外部 NTC 冲突**：`TEMP_SEL=0` 才能读 die 温；当前无 die 温 API、无均衡；包温度 4.0–54.0 °C 门控已生效（`PROT_TEMP_ENABLE=1`），TS1 模型仍需现场复核。
16. **BQ SCD 恢复**：SCD 后必须主机清 SYS_STAT、双管关、FAULT 锁存，不自动重合闸；软件 OCP 只作后备。
17. **BQ 无充电 OCP**：充电 5 A 上限只能靠软件 OCP + 充电器自身限流；软件 OCP 现为 5.5 A/1 s，已留余量（D-PROT1）。
18. **CAN 未实现**：L1 不实现 CAN 业务；NORM 80 MHz 已满足后续需求，但不得因此把轻量 LP/30 min 监测切到 HSE。
19. **PACK_SC 分压/COMP2 只用于方向判定**：不能作为电压测量或主保护。
20. **SWD 与 LP 电流**：`POWER_LP_KEEP_SWD` 默认 **0**（不是原计划的 1）；调试期设 1 会显著抬高 STOP1 电流，量产保持 0。
21. **VC5/BAT 降级是临时缝补**：真实值可信优先、异常估算；估算不能进保护；fake-switch 只同值重写 `SYS_CTRL2`，不修复模拟通道（C-25）。
22. **NMC/LFP 混用风险**：当前 4.25/2.80 V 与 NMC 表不能用于 LFP；误当 LFP 参数可能造成过充/过放。
23. **StateStore 生命周期 flags 是死定义**：swap/cutoff/dead/charge_full 没有写入/读取，启动不恢复；不能声称掉电锁存可用。
24. **黑匣子 wake 截断**：`power_wake_mask & 0x07` 丢失 CHARGER/MONITOR；事件覆盖不完整（C-17）。
25. **软件 OCP 已恢复余量**：5.5 A/1 s、33 A/500 ms（至少 2 帧）；仍需台架确认动作时间和正常满载不误锁（D-PROT1 已决）。
26. **LP 架构差异**：复位式 IWDG LP 与计划 RTC WUT 1 s 不是同一架构；修改前不能按原计划估算唤醒延迟/电流。

---

## 17. 交付物

> **状态（2026-11-11）**：`[部分交付]`。

| 交付物 | 状态 |
|---|---|
| 本文档：L1 开发方案核对版 | 已生成（本文件）；原文件保留 |
| 更新后的 `DOCS/POWER_MODE.md`、`DOCS/SYSTEM_STATES.md`、`CLAUDE.md` | 已有当前状态说明；`DOCS/PINMAP.md` 主要硬件映射，未因本核对改动 |
| L1 源码、`app.mk`、链接脚本、`compile_commands.json` | 部分：当前树可构建；不是完整 L1 |
| LFP 标定表/CSV、SOC 对比报告、BQ CC 零点/增益记录 | **缺失**；当前 NMC 占位 |
| 状态转换表、按键时序记录、LP 电流记录 | 部分：状态子集/手势检测/复位 LP 有；无完整 L1 表、无 LP 电流记录 |
| 30 A/5 A/SCD/die 温测试记录 | **缺失** |
| 换电/死存储测试记录、故障注入记录 | **缺失** |
| 最终验收清单与遗留风险清单 | 本文 G3/G4/§16 是当前清单；不是最终验收 |

---

## 18. 完成定义

> **结论（2026-11-11）**：**L1 未完成**。下面是逐项状态，不能把本文件当作“完成”证明。

L1 完成需同时满足：

| # | 原完成条件 | 当前状态/剩余工作 |
|---|---|---|
| 1 | 空闲 5 s 自动 LP；充电器插入 1 s 级唤醒并进入 CHARGE | 5 s LP 有；唤醒约 30 s；D-LP1 已决定接受当前 ~30 s 方案 |
| 2 | 放电手势激活、双管全开；退出/5% 切断/10% 恢复 | 手势切换/双管有；ARM、5%/10% 未实现 |
| 3 | 充电充满只鸣叫不断电；充电器移除回 IDLE/LP | 充满未实现；移除需连续 5.2 s 低电平才回 IDLE；LP 5 s |
| 4 | SOC/SOH 标定后可用于显示/辅助；低压有电压兜底 | NMC R0/R1 已实测、OCV/tau 仍有占位、`CAL_VALID=1`；硬欠压 2.8 V 已实现，3.1 V/SOC10 低电充电已接入；原 5%/0% 未实现 |
| 5 | 均衡单节轮换，die 温 65 °C 停止 | 均衡永久关闭；无 die 温 |
| 6 | 30 min 监测/存档在 HSI16 下不切 HSE | 已实现；不更新 SOC/SOH；无正式 LP 电流记录 |
| 7 | 换电/换电模式/三连击清除完整 | 未实现 |
| 8 | 死存储 SHIP + STOP1 + 充电恢复 | 未实现；`BQ_ShipMode` 未接线 |
| 9 | 30 A/5 A/SCD/双管全开/温升通过 | 配置有；软件 OCP 已恢复 5.5 A/33 A 余量；无正式测试记录 |
| 10 | 所有故障路径回双管全关 | 现有路径部分实现；完整 L1 状态机未实现 |

---

## 附录 A：关键参数速查（计划值 / 当前实现 / 状态）

> **警告**：当前“实现值”是 2026-11-11 工作树；NMC 占位参数不能当作 LFP 认证值。

| 参数 | 原计划值 | 当前实现值 | 状态/备注 |
|---|---|---|---|
| 电池 | 3S LiFePO4，1500 mAh | NMC 1500 mAh 占位表 | 冲突；D-CHEM1 |
| 放电最大 | 30 A | 30 A 目标 | 未验证温升 |
| 充电最大 | 5 A | 5 A 目标 | 未验证温升 |
| 分流 | 1.5 mΩ | 1.5 mΩ | 已实现 |
| BQ OV | 3.70 V/cell，1 s | **4.25 V/cell，1 s** | NMC 占位（C-09） |
| BQ UV | 2.50 V/cell，4 s | **2.80 V/cell，4 s** | NMC 占位（C-09） |
| BQ OCD | 56 mV/320 ms≈37 A | `0x57`≈37.3 A/320 ms | 已实现 |
| BQ SCD | 67 mV/100–200 µs≈44.7 A | `0x89`≈44.7 A/100 µs | 已实现 |
| 软件充电 OCP | 5.5 A/1 s | **5.5 A/1 s，至少 2 个新发布帧** | 已恢复（D-PROT1） |
| 软件放电 OCP | 33 A/500 ms | **33 A/500 ms，至少 2 个新发布帧** | 已恢复（D-PROT1） |
| 充电温度 | 0–45 °C | **包温度 4.0–54.0 °C，充放电都收紧** | `PROT_TEMP_ENABLE=1`（D-PROT1） |
| 放电温度 | −20–60 °C | **包温度 4.0–54.0 °C，充放电都收紧** | 2 °C 回差/2 s（D-PROT1） |
| 均衡 | max≥3.35 V，diff≥30 mV | 永久关闭 | 冲突（C-10） |
| 均衡 die 温 | 65 °C，5 °C 回差 | 无 die 温 API | 未实现 |
| 放电 5% | SOC≤5% + 最低节≤2.8 V | 无 | 未实现（C-14） |
| 10% 恢复 | 充电 SOC>10% | 无 | 未实现（C-14） |
| 0% 死存储 | SOC=0 + 最低节≤2.5 V，10 s | 无；BQ_ShipMode 未调用 | 未实现（C-05） |
| LP 嗅探 | RTC WUT 1 s | **IWDG 复位 ~30 s** | 冲突（C-01/C-02） |
| 储存监测 | 30 min HSI16 | 已实现；不更新 SOC/SOH | 部分（C-07） |
| 空闲 LP | 5 s | 5 s | 已实现 |
| RTC/WUT | LSE 32.768 kHz；WUT 1 s 自由运行 | LSE 死，默认 LSI；WUT 关闭 | 冲突（C-01/C-03） |
| 代码 Flash | 96 KB，`_image_end<0x08018000` | 96 KB，`0x08015700` | 已实现 |
| BMS NVM | 页 48–49，132/136 B | 同 | 已实现 |
| StateStore | 页 50–51，48 B | 同（v2） | 已实现；生命周期 flags 未用 |
| 黑匣子 | 页 52–63，24 B×1020 | 同 | 已实现；事件/wake 不完整 |
| SWD in LP | `POWER_LP_KEEP_SWD=1` | **0** | 冲突（C-21） |
| VC5/BAT 控制 | 原计划未定义 | 真实可信优先，异常估算 | 计划外新增（C-25） |
| fake-switch | 原计划未定义 | 同值写 `SYS_CTRL2`，2.5 s 重试 | 计划外新增（C-25） |

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
| expected_swap | 已确认即将换电，下次上电报错（当前仅定义） |
| cutoff_latch | 5% 放电锁存（当前仅定义） |
| dead_storage | 0% 死存储（当前仅定义） |
| reset LP | 复位式 IWDG LP：写 BKP 会话 token，复位后轻量 boot |
| IWDG | 独立看门狗；当前同时作为 NORM 死机保护和 LP 周期复位源 |
| fake-switch | 在 VC5/BAT 异常时同值重写 `SYS_CTRL2` + 同向提示音 |
| NMC/LFP | 当前 BMS 参数是 NMC 占位；目标电芯是 LFP |
| C-xx | G3 中的计划 vs 实现冲突编号 |
| E1–E4 | 证据等级：代码/构建/台架/未验证 |

---

## 附录 C：证据索引（当前树，2026-11-11）

> 本索引用于快速定位“当前实现”依据；行号会随代码变动，优先按符号搜索。原计划的行号不再作为当前行为依据。

| 主题 | 关键文件/符号 |
|---|---|
| NORM 时钟 | `App/app/power_mode.c` `PowerMode_ClkSwitchToHsePll()`；`Core/Src/main.c` `SystemClock_Config()` |
| LP 分类/轻量路径 | `App/app/lp_cycle.c` `LpCycle_ClassifyBoot()`、`LpCycle_Run()`、`LpCycle_EnterStop()`；`LP_BASE_CYCLE_MS=30000` |
| BKP 会话 | `App/app/lp_session.c/.h`；`App/app/lp_cycle.c` `LpCycle_RequestStart()` |
| IWDG | `App/app/watchdog.c/.h`；`PowerMode_LpDbgStopApply()` |
| LP BQ sniff/monitor | `App/bsp/bq_ic.c` `BQ_LpBootPrepare()`、`BQ_LpMonitorRun()`、`BQ_LpSniffRead()`；`bq_ic.h` LP 声明 |
| 充电检测 | `App/app/lp_cycle.c` `LpCycle_BqSniffOnce()`、`LpCycle_CheckCharger()`；`App/app/main_app.c` COMP2 消抖 |
| RTC | `App/app/rtc_time.c/.h`；`App/app/power_mode.c` WUT 关闭；`board.h` `LP_RTC_USE_LSE` |
| 工作状态/策略 | `App/app/state_machine.c/.h`；`App/app/charge_policy.c/.h` |
| 按键/手势 | `App/bsp/button.c/.h`；`App/detect/gesture.c/.h`；`main_app.c` 仅派发 `GESTURE_PAIR_HOLD3` |
| 保护/OCP/温度 | `App/app/protection.c/.h`；`bq_ic.c` `BQ_OV_TRIP_MV/UV_TRIP_MV`、`PROTECT1/2/3` |
| VC5/BAT 降级/fake-switch | `App/bsp/bq_ic.c` `Bq_SelectControlVoltages()`、`BQ_ApplyFetMaskIfUnchanged()`；`App/app/main_app.c` `App_FakeSwitchTask()` |
| SOC/SOH/NVM | `App/detect/soc_est.c/.h`；`App/detect/bms/bms_config.h`；`App/detect/bms_nvm_port.c` |
| StateStore | `App/app/state_store.c/.h`；`main_app.c` `App_CommitArchiveSnapshot()` |
| 黑匣子 | `App/app/black_box.c/.h`；`BlackBox_WriteWake()` 的 `& 0x07`；事件枚举 |
| LED/蜂鸣器 | `App/app/led_status.c/.h`；`App/bsp/buzzer.c/.h` |
| 构建/分区 | `app.mk`；`STM32L432xx_FLASH.ld`；`board.h` `APP_FLASH_END`；当前 `build/BMC_CMZY_080.map` `_image_end=0x08015700` |
| 当前文档 | `CLAUDE.md` 2026-11-11 段；`DOCS/POWER_MODE.md`/`SYSTEM_STATES.md` 顶部更新说明 |

---

## 变更记录

| 日期 | 变更 |
|---|---|
| 2026-11-11 | 从 `DOCS/L1_DEVELOPMENT_PLAN.md`（SHA-256 `9a1d9e10…8ed7`）复制并核对当前工作树；新增 G1–G4 与附录 C；已实现段按实际改写；未实现项保留并加状态；原始计划未修改 |

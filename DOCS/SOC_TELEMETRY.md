# USART1 SOC/SOH 自动遥测

## 目的

NORM 状态下每秒通过 USART1 发送一条机器可读的 D 帧，PC 用
`li-ion-soc-soh-toolkit/脚本/soc_record.py` 直接记录为 CSV。用于 SOC/SOH
标定、充放电曲线和环境测试记录；不是保护通道。

## 硬件与串口

- USART1：PB6 TX / PB7 RX，U18 4-pin 接口，115200 8N1，无流控。
- 固件只发送，不实现 USART1 RX 命令通道；PB7 暂不使用。
- 5 V 由 PB0 使能；自动 LP 后 USART1 关闭，遥测停止，唤醒后恢复。

## 帧格式

```text
D,V_mV,I_mA,T_0p1C,SOC_0p01,SOH_0p01,Q_mAh,R0_mohm\r\n
```

示例：

```text
D,3845,-1234,253,8654,9666,3238,62
```

| 字段 | 单位 | 来源/说明 |
|---|---|---|
| `V_mV` | mV | 保护/SOC 参考节：正常为三节最低值；VC5/BAT 不可信时仅为 VC2 |
| `I_mA` | mA | **放电为正、充电为负** |
| `T_0p1C` | 0.1 °C | TS1 温度，`temp_valid` 才发 |
| `SOC_0p01` | 0.01 % | SocEst 当前 SOC |
| `SOH_0p01` | 0.01 % | SocEst 当前 SOH |
| `Q_mAh` | mAh | SOH 学到的容量；未学到时为 `0` |
| `R0_mohm` | mΩ | SOH 学到的 10–90% 平均 R0；未学到时为 `0` |

发送周期 1 Hz。BQ 原始 `current_ma` 是充电为正，D 帧乘
`BQ_CC_DISCHARGE_SIGN` 后变为放电为正；`[DIAG] I=` 仍是原始符号，二者相反。

## 发帧闸门

以下任一条件失败时，该秒不发 D 行（PC 端出现时间缺口是预期行为，不补 0）：

- BQ 快照 `valid && fresh` 且 `BQ_IsFresh()` 成立；
- `SocEst` 快照 `frame_valid`；
- `temp_valid`；
- `V_mV` 在 1500–5000 mV；
- `I_mA` 在 -30000–30000 mA；
- `T_0p1C` 在 -400–1250。

发送前由 `App_Telemetry_Task()` 检查；失败原因和计数保存在
`s_telemetry`（`skip_bq_count`、`skip_soc_count`、`skip_temp_count`、
`skip_range_count`、`last_skip_reason`），可在 Ozone 查看。

## 重要限制

- `SOC_EST_CAL_VALID=1`：OCV/tau 仍有占位成分，SOC/SOH 可参与低电判断和显示，
  但不能替代 2.8 V 电压硬保护。当前没有按 SOC 的 5%/0% 硬截止；硬欠压是独立的
  **电压-only** 保护（参考节 <2.8 V 只关 DSG，≥3.0 V/2 帧恢复；正常为三节最低值，VC5/BAT 异常时只取 VC2）。
- **VC2-only 降级**：BQ 帧 `log_cell3_valid==0 || log_pack_valid==0` 时，D 帧
  `V_mV`、SOC/SOH 和低压判断都改为只使用实测 VC2；正常帧仍取三节最低值。
  这是针对 VC5/BAT 异常时 VC1 虚假偏低的保护策略，不是新的 SOC 标定。
- **低电充电状态**：参考节<3.1 V 或 SOC<10% 连续 2 帧进入，默认 CHG-only，
  不需要先检测到充电器；充电电流 >200 mA/2 帧且参考节≥2.8 V 才开 DSG，
  <120 mA 或电流无效立即关；3.1 V+SOC≥10%/2 帧退出。低电状态是 RAM-only，
  复位按实时电压/SOC 重建；无充电器可自动 LP，按键唤醒后红灯 5 Hz。
  `SOC_EST_CAL_VALID=1`，但 2.8 V 电压硬保护仍是独立兜底。
- **充满提示**：参考节≥4.15 V 或标定 SOC≥98%，再加 −20..75 mA/30 s 时只置
  白灯和事件，不关 CHG；BQ OV 4.25 V 仍是最终保护。SOC/SOH 的 D 帧不会因
  该提示停止。
- LED 的 SOC 渐变是显示，不是保护；SOC 无效时 LED 保持最后有效颜色（或
  回退固定色），D 帧仍按自己的有效性门限发送/暂停。
- `Q_mAh=0` / `R0_mohm=0` 表示尚未学到或当前无效，不是真实的 0。
- R0 学习窗口为 15–35 °C；温度无效时 D 行会暂停。
- `soc_record.py` 终端显示的“丢弃”计数包含所有 `[DIAG]`/`[CAL]`/启动日志，
  不代表 D 帧丢失；以 `n_ok` 和 CSV 行数为准。

## PC 记录

需要 Python 3 和 pyserial：

```powershell
pip install pyserial
python "C:\Users\Shuchen Tao\Documents\BMC_CMZY_080\li-ion-soc-soh-toolkit\脚本\soc_record.py" `
  --port COM7 --baud 115200 --out soc_run1.csv
```

`--port` 可省略，脚本会枚举并交互选择 COM；`--baud` 建议显式写 115200。
CSV 列为：

```text
time_iso, elapsed_s, V_mV, I_mA, T_0p1C, SOC_0p01, SOH_0p01, Q_mAh, R0_mohm
```

记录脚本不修改固件；`soc_bench.py` 可继续读取生成的 CSV 做离线分析，
但它的 `record` 子命令只接受 4 字段 D 帧，与本固件的 8 字段帧不同。

## 日志环与诊断

- `App/comm/app_log.c` 的环形缓冲为 **2 KB**；10 s 一次的 `[DIAG]` 突发
  最坏约 1.1 KB，2 KB 环给 1 Hz D 行留出余量。
- Ozone 可看 `s_log.drop_bytes`、`s_log.dma_drop`、`s_log.dma_stuck_count`
  以及 `s_telemetry.emit_count`、`s_telemetry.last_*`。
- D 行与 `[SOC]`/`[DIAG]`/`[CAL]` 共用 USART1；解析器只取 `D,` 前缀。
- 自动 LP/降级模式/启动期间不保证遥测；这是设计行为。

---

## 推荐标定流程（25 °C，NMC 1500 mAh）

> **安全前提**：仅台架进行；使用限流电源/电子负载和保险丝，不无人值守。
> 当前 `SOC_EST_CAL_VALID=1`，BQ OV/UV、2.8 V 电压硬保护和低电充电逻辑仍独立生效；
> 标定期间不得为了“跑完整曲线”绕过这些保护、温度门控或软件 OCP。

### 0. 先决条件

- 电芯/包温度稳定在 **25 ± 2 °C**，TS1 有效且 D 帧温度字段正常。
- 记录固件版本、`bms_config.h` 当前表、实际标称容量和测试日期。
- 电流方向先确认：D 帧放电为正、充电为负；`[DIAG] I=` 是相反符号。
- 先确认 D 帧 1 Hz、V/I/T 合理、10 s 诊断突发期间无丢行。
- 标定使用**实际容量**（例如满充后 0.2C 放出的 mAh），不要假设它一定等于 1500 mAh。

### 1. 基础校验

1. 静置，记录 BQ CC 零点；D 帧 `I` 应在 ±几十 mA 内。
2. 加一个已知负载（如 300 mA、1 A），与外部电流表/分流器比较；
   BQ CC 换算固定，若偏差大先检查分流器、接线和 `BQ_CC_DISCHARGE_SIGN`。
3. 用已知温度确认 TS1；温度无效时 D 帧不会发送。
4. 确认 `SOH_R` 的出厂基线来自当前 `SOC_R0_25C_MOHM`，不是旧 NVM 学习值。

### 2. OCV-SOC 标定（优先）

目标：替换 `SOC_OCV_25C_MV` 的 11 点（0/10/.../100%）。

1. 25 °C 下恒流/恒压充满到 **4.20 V**，截止电流约 **C/20（75 mA）**；
   静置至少 **15 min**，记录 100% OCV。
2. 以 **0.2C（300 mA）** 放电，每放出约 **150 mAh（10%）** 停一次；
   每档静置至少 **15 min**（当前慢 tau 占位最大约 125 s，约 7τ）。
3. 每档静置末尾取 **1–5 min 平均电压**作为该 SOC 的 OCV；不要取刚断电的瞬时值。
4. 重复到 **10%** 或 BQ UV 保护前；0% 若无电芯级测试夹具，按 10% 端点平延/
   外推并标注“未实测”，不要为到 0% 绕过 BQ UV 保护。
5. 用安时积分和实际容量把每档换算成 SOC，线性插值到 10% 网格，写入
   `SOC_OCV_25C_MV`；5/45 °C 仍可复用 25 °C 占位并注明。
6. 复测：用新的 OCV 表在静置点反查 SOC，报告最大/平均偏差；NMC 平台区
   10–20 mV 的误差都可能显著放大 SOC 误差，不能只看曲线“像不像”。

### 3. tau 处理

- 本次已有的 **83–138 ms** 是快极化 tau，不能直接写进单位为秒的
  `SOC_TAU_25C_S`，也低于 500 ms EKF 步长的有效辨识范围。
- 若要标定算法库的慢 tau：对上面 15 min 静置的回弹段拟合
  `ln(OCV - V(t))`，得到 36–125 s 量级的慢 tau 后再更新 `SOC_TAU_25C_S`。
- 更新 tau 后重新核对 `SOC_IDLE_MS` 和 `SOH_MIN_REST_MS`：
  通常取 **5–8 倍 τ_max** 和 **4–5 倍 τ_max**。
- 若无法确认慢 tau，保持占位值，并在标定报告中注明；不要用快 tau 冒充。

### 4. R0/R1 复标（可选）

当前 R0/R1 已按 25 °C、10 组放电数据接入；只有换电芯、改采样链路或发现
D 帧电压与外部表偏差明显时才重做。

- 每个 10% SOC 点做一次足够大的电流阶跃（例如 1–3 A，持续数秒），
  用**至少 10 Hz**的外部采集/示波器记录电压和电流；
- `R0 = ΔV_instant / ΔI`，R1/τ 从断电后回弹拟合；
- 1 Hz D 帧只适合 OCV/SOC 记录，不适合 1 s 以内的快 R0 脉冲；
- 复标后更新 `SOC_R0_25C_MOHM` / `SOC_R1_25C_MOHM`，并重做 SOH 基线（见第 6 节）。

### 5. 容量、SOH-Q 与 SOH-R 验证

1. 满充后 0.2C 放到 2.80 V，记录实际容量；与 `SOC_CAPACITY_NOMINAL_MAH`
   和 `SOH_GetLearnedCapacityMAh()` 对比。
2. SOH-Q 需要至少 **15% ΔSOC** 且 **≥200 mAh ΔQ** 的静置 OCV 锚点；
   建议做 80%→20% 一类的分段循环，每段静置 10–15 min。
3. SOH-R 学习需要 >500 mA、≥5 s 的放电阶跃，且温度在 15–35 °C；
   检查 `SOH_GetR0AvgMOhm()` 是否在 R0 表附近。
4. 可用 toolkit 的 `soh_learn.py` / `soh_mcu_sim.py` 对同一 CSV 做离线对照；
   以趋势和基线为准，不用单次容量学习值直接改保护阈值。

### 6. 写回与清理旧 SOH 状态

1. 修改 `App/detect/bms/bms_config.h` 的 OCV/R0/R1（以及确认后的慢 tau）。
2. 更换 R0 基线后，旧的 SOH 学习基线/NVM 参数不能继续沿用：
   - 台架可在 Ozone 调用 `SocEst_ResetForNewPack()`，或擦除 BMS NVM 页 48–49；
   - 让 `SOH_Init()` 以新 R0 表重新快照基线，并等待 `BMS_NVM_Task` 落盘；
   - 确认 `SOH_GetResistancePercent()` 在新基线下约为 100%。
3. 当前 `SOC_EST_CAL_VALID=1` 已用于低电判断/显示；若继续做精确标定，仍需验证 OCV、tau、R0/R1、CC 方向/零点后再宣称完成；
   仅完成 R0/R1 或仅完成 OCV 时保持 0。
4. 重新 `make clean && make -j2 all`、烧录，记录新 `_image_end` 和标定文件哈希。

### 7. 验收标准

- D 帧 1 Hz 连续；10 s 诊断突发期间无丢行。
- 0.2C 放电全程 SOC 单调下降；与安时积分参考的偏差在项目目标内并记录。
- OCV 表在 10/20/.../90% 静置点的残差有报告；平台区误差单独列出。
- SOH_R 在新基线下接近 100%，经过老化/循环后趋势合理。
- 任何阶段都不出现因标定操作绕过的 BQ OV/UV、温度、软件 OCP 或 Protection 锁存。
- 保存 CSV、测试温度、实际容量、固件版本和修改后的表，作为可追溯标定记录。

### 8. 工具注意事项

- 采集用本文的 `soc_record.py`（8 字段 D）；`soc_bench.py record` 只接受
  4 字段旧协议，不能直接接本固件。
- `soc_bench.py stair --csv soc_run.csv --capacity <实际mAh> --init-soc 100`
  可对 CSV 做多档 OCV/R 分析；它内置默认容量 3350 mAh，必须显式覆盖。
- `soc_bench.py` 的默认 OCV 表也是旧 NMC 数据；起始 SOC 最好显式给
  `--init-soc 100`，或把当前已知表通过 `--ocv-table` 传入，不要用错误的
  默认表反查起始 SOC。
- 1 Hz D 帧不能代替高速脉冲采集；R0/快 tau 必须用外部高速记录。

---

## 2026-11-11 初测充放电曲线（粗略，旧 SOH 影响）

> **状态：粗略数据，不作为最终标定或验收依据。** 受时间限制，这两次测试的
> 温度、倍率和 SOC 起点都不满足精细 OCV 标定条件；后续会按上面的流程重做。
> 原始文件保留在仓库根目录：
> `soc_runtestCHG.csv`、`soc_runtestDSG.csv`。

### 数据概况

| 文件 | 时长 | 主要电流 | 温度 | 电压范围 | D 帧 SOC 起→止 |
|---|---:|---:|---:|---:|---:|
| `soc_runtestCHG.csv` | ~1634 s | 约 -2.9 A（充电） | 21.6–27.5 °C | 2785→4201 mV | 36.09%→91.78% |
| `soc_runtestDSG.csv` | ~1420 s | 约 +2.7 A（放电） | 19.0–21.6 °C | 4175→3506 mV | 100%→43.15% |

两次都约 1 Hz，D 帧字段完整；`SOH_0p01=10000`、`Q_mAh=0`、`R0_mohm=59`
在整个文件中基本不变。

### 观察到的旧 SOH/模型影响

- 充电开始后 D 帧 SOC 曾从 36% 附近**先跌到约 14%**（t≈19 s），随后才随充电
  上升；放电开头 SOC 在 100% 附近停滞后又有回跳。这是 EKF/OCV 重同步、初始
  SOC 和旧 SOH 参数共同作用的结果，不是真实电池行为。
- `Q_mAh=0` 只是 `SOH_IsCapacityValid()==0` 的“未学习”哨兵，**不能证明
  运行时 `s_cap_mah` 没有使用旧的 NVM 学习容量**；旧的 R0/容量仍可能影响
  Ah 积分和 EKF。
- `SOH_0p01=10000` 是综合显示值，不代表容量基线、R0 基线或 NVM 学习状态
  是干净的。
- 因此这两条曲线的 **D 帧 SOC 不能直接作为 OCV-SOC 或容量标定输入**。

### 用电流反算（独立于旧 SOH 的横轴）

D 帧电流已经是“放电为正、充电为负”。按实际时间间隔积分：

```text
Q_net(t) = ∫ I_mA dt / 3600        [mAh，充电为负，放电为正]

SOC_cc_01(t) = clamp(
    SOC0_01 - 10000 × Q_net(t) / C_ref,
    0, 10000)
```

其中 `C_ref` 必须来自**外部可信容量**或明确的参考容量，不能来自旧 SOH 输出。

本次数据的总量和端点等效容量：

| 文件 | 积分净电量 | SOC 变化 | 端点等效容量 |
|---|---:|---:|---:|
| CHG | -976.6 mAh | +55.69% | ≈1754 mAh |
| DSG | +970.4 mAh | -56.85% | ≈1707 mAh |

两者相差约 47 mAh（≈2.8%），且端点等效值受 EKF/OCV 修正、静置段和电流
积分误差影响，**不能据此认定真实容量**。当前运行时容量被限制在
`SOC_CAP_LO_PCT..SOC_CAP_HI_PCT`（默认 50–110% × 1500 = 750–1650 mAh），
所以 1.7 Ah 的端点值只能说明旧模型/采样存在偏差，不能直接写成新容量。

若用固定参考容量重算端点（初值仍取 D 帧首帧 SOC，仅用于对比）：

| `C_ref` | CHG 终点 SOC_cc | DSG 终点 SOC_cc |
|---:|---:|---:|
| 1500 mAh（标称） | 超过 100%，需钳位 | 约 35.3% |
| 1650 mAh（旧容量上限） | 约 95.3% | 约 41.2% |
| 1730 mAh（数据端点拟合） | 约 92.5% | 约 43.9% |
| D 帧原值 | 91.78% | 43.15% |

这张表只说明“旧 SOH/容量把 SOC 斜率拉到了大约 1.7 Ah 的量级”，**不是**
新的标定结果。数据端点拟合值高于 `SOC_CAP_HI_PCT` 上限，进一步说明这次
粗测不能用于容量标定。

### 建议的后续精细测试

1. 25 ± 2 °C，0.2C（300 mA）充放电，而不是本次约 3 A/2.7 A。
2. 每 10% SOC 静置 15 min，记录 OCV；不要用本次高温升/高倍率数据。
3. 使用已标定的 OCV/R0/R1/tau 和外部实测容量作为 `C_ref`。
4. 先在台架上清零旧 SOH 学习状态/基线，再记录 D 帧和 `[CAL]`。
5. 同时保存原始 CSV、实际容量、温度、起止 SOC 和固件版本；用
   `V vs Q_net` 或 `V vs SOC_cc` 作为独立于旧 SOH 的曲线横轴。
6. 精细曲线完成前，本文只把这两份数据标记为“粗略趋势”，不得据此修改
   OCV/R0/R1/tau 或 `SOC_EST_CAL_VALID`。

# BMC_CMZY_080 固件发布 — 软件 SW0.0.11 / 硬件 HW0.80

## 发布信息

- 标签：`v0.0.11+hw0.80`
- 分支：`Fuck_ADC_2`（未合入 `main`）
- 代码基线：`dd6fded`（相对 `main` 5 个提交）
- 发布日期：2026-11-14
- 工具链：`arm-none-eabi-gcc` 12.2.0、GNU Make 4.4.1
- 编译配置：`DEBUG=1`、`OPT=-Og`、`-g -gdwarf-2`；`POWER_LP_KEEP_SWD=0`、`LP_RTC_USE_LSE=0`、`LP_SMBA_WAKE_TEST=0`
- 镜像尺寸：text 87640 B、data 168 B、bss 8200 B；`_image_end=0x08015700`
- Flash 限制：96 KB 应用区，`_image_end<=0x08018000` 断言通过
- 启动日志：`[BOOT] app init, HW=0.80 SW=0.0.11, reset=0x..`

## 自 main 以来的主要变更

- 使用正常 VC1/VC2 旁路异常 ADC 通道，增加 BQ FET 影子回读校准。
- 完善基于电压和 SOC 的过充/过放保护、放电负载与平衡充稳定性处理。
- 充电电压异常保护旁路，改进 BQADC 复位与异常恢复。
- BQ OV/UV 在 VC5/BAT 不可信时退避，改用 VC2 软件 OV/UV。
- 本次发布：嵌入 HW/SW 版本启动打印，补齐发布文档。

## 关键功能

### 保护

- BQ7692003PWR 硬件 OV/UV/SCD/OCD；MCU 软件 OCP：充电 5.5 A/1 s、放电 33 A/500 ms。
- 单节 2.8 V 硬欠压（3.0 V 恢复）；VC2-only 降级时只使用实测 VC2。
- BQ OV/UV 退避：首个 VC2-only 完整批次临时放宽 BQ TRIP，改用 VC2 软件 OV 4.25 V / 恢复 4.15 V（只关 CHG）与既有 2.8 V UV；连续 3 个正常发布批次后恢复 BQ TRIP。
- 温度门控 4.0–54.0 °C，2 °C 回差；无效温度按 fail-closed 处理。
- 亏电充电状态：参考节 <3.1 V 或有效 SOC <10% 连续 2 帧进入 CHG-only；DSG 仅在 >200 mA/2 帧且参考节 ≥2.8 V 时开放，<120 mA 立即关闭。
- 充满提示：参考节 ≥4.15 V 或有效 SOC ≥98%，且电流在 -20…75 mA 持续 30 s；只提示，不关 CHG。
- 假开关：VC5/BAT 异常时同值重写 `SYS_CTRL2` 并提示；不改变 FET 状态，也不能覆盖 BQ 硬件 OV/UV 锁存。

### SOC/SOH、存储与交互

- SOC/SOH 估算与学习；NMC 1500 mAh 参数，R0/R1 为 25 °C 实测，OCV/tau 仍含占位成分。
- Flash 分区：BMS NVM 页 48–49、StateStore 页 50–51、黑匣子页 52–63。
- RGB SOC 渐变指示、LED/蜂鸣模式、按键/手势。
- 低功耗：NORM 250 Hz；IWDG 复位式周期 LP + STOP1；约 30 min 监测/黑匣子记录；PB5/充电器/ALERT 唤醒。

## 烧录

- 使用 SWD：J-Link/Ozone 或 STM32CubeProgrammer。
- `.hex` 可直接烧写；`.bin` 起始地址必须为 `0x08000000`。
- 不建议全片擦除：页 48–63 含标定/NVM/StateStore/黑匣子数据；仅升级应用时按 `.hex` 地址范围烧写。
- 上电后 USART1（115200 8N1）应看到 `HW=0.80 SW=0.0.11` 的启动日志。

## 验证状态与已知限制

- 本版本通过 clean build，无编译警告，链接 96 KB 断言通过。
- 仓库没有自动化测试目标；验证方式为编译 + 烧录 + 上板观察。
- 台架验证不完整：低压、早 SOC、充满、RGB 行为仍待完成验收；不声称已完成全量硬件验证。
- 当前板 LSE 不稳定/未起振，发布默认 `LP_RTC_USE_LSE=0`（LSI，RTC 为相对时间，存在漂移）。
- BQ OV/UV 退避是 VC5/BAT 异常时的软件降级策略，不修复硬件；BQ 硬件比较器仍可能动作，假开关不能覆盖硬件锁存。
- BQ cell3/VC5 输入通道的现场状态需按实际板确认；若 BQ 自身 cell3 仍高于其 OV 门槛，CHG 会被 BQ 硬件关闭。
- SOC 的 OCV/tau 仍有占位值；TS1 上拉参数需按实际板测量确认。
- `.elf`/`.map` 含调试符号和本地构建路径；发布基于 `Fuck_ADC_2`，不是 `main`。
- 产物校验以 `SHA256SUMS.txt` 和 GitHub Release 正文中的 ZIP SHA256 为准。

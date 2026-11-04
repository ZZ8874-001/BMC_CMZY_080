---
name: stm32-app-architecture
description: Use when adding, migrating, or restructuring BMC_CMZY_080 firmware modules; enforces the Armor_Cmzy26_081 App/ layout (App/app, App/bsp, App/comm, App/detect, board.h, app.mk), USER CODE integration, App_Init/App_Loop/250 Hz tick, named static state structs, ISR ownership, and CubeMX regeneration checks. 用于本工程新增或迁移固件模块、按参考工程分层结构落地代码时。
---

# STM32 App 分层结构（BMC_CMZY_080）

本技能把参考工程 `C:\Users\Shuchen Tao\Documents\Armor_Cmzy26_081` 的组织方式固化成本工程的开发规范。**本工程后续新增的应用代码必须放在 `App/`，采用相同的四层结构与命名习惯。**

## 何时使用

- 在本工程新增/迁移驱动、通信、状态机、灯效、自检或持久化模块；
- 从参考工程移植某个模块（只复制"形状"，不要照搬硬件常量）；
- CubeMX 重新生成后恢复应用层接入；
- 用户要求"按 Armor_Cmzy26_081 的结构/分层来写"。

先读参考工程（只读，不要修改）的 `App/README.md`（规范权威）和 `CLAUDE.md`（架构与重生成踩坑）。本机路径不存在时先询问用户，不要凭空重建。

## 目标目录与分层

```
App/
├─ board.h          # 唯一的外设句柄 extern 入口 + 板级常量/极性宏
├─ bsp/             # 硬件驱动：BQ IC、RGB 灯、蜂鸣器、Flash、模拟通道……
├─ comm/            # 传输/协议：app_can、can_node、board_comm、app_log、host_cmd
├─ detect/          # 信号/判定算法；本工程按需建立，没有算法就不要建目录
└─ app/             # 编排：main_app、state_machine、led_status、faults、self_test、reset_cause
app.mk              # App/ 的源文件与 include 路径，独立于 CubeMX 生成文件
```

依赖方向（只允许向下）：

- `app/` → `detect/`、`comm/`、`bsp/`
- `detect/`、`comm/` → `bsp/`
- `bsp/` → HAL + `board.h`
- **`detect/` 不得依赖 `comm/`**（唯一无例外的规则）

允许的例外（每个都要在头文件注释里说明原因）：

- `faults.h`、`calibration.h` 这类跨层共享的接口头可被多层 include；
- `comm/board_comm.c`、`comm/host_cmd.c` 可 include `app/` 头做状态/灯效/上报编排；
- `bsp/` 可 include `comm/app_log.h` 打诊断日志或分发 UART TX 完成。

## 不可破坏的规则

1. **手写代码只放在 `App/`**。`Core/` 只允许在 `/* USER CODE BEGIN/END */` 块内写接入代码，其余内容每次 CubeMX 生成都会被覆盖。
2. **`App/board.h` 是唯一 `extern` 外设句柄的地方**；禁止在模块里自行 `extern hcan1` / `hi2c1`。
3. 新增 `.c` 必须进 `app.mk` 的 `APP_SOURCES`；新增层/路径进 `C_INCLUDES`。**文件名全局唯一**，因为 Makefile 用 `notdir` 把对象压到 `build/`。
4. `Makefile` 是 CubeMX 生成物。每次重生成后，必须在 `C_INCLUDES` 定义之后、`# compile gcc flags` 之前重新加一行 `-include app.mk`；否则应用源文件不会参与构建。
5. 初始化顺序：`App_Init()` 按依赖顺序初始化模块，**最后**才启动应用时基中断；`App_Loop()` 在主循环非阻塞轮询；`App_OnTick250Hz()` 由 TIM2 ISR 调用（本工程 250 Hz）。
6. ISR 内只允许：置标志、计数、启动 DMA、拷贝小帧。**禁止**在 ISR 里发 CAN、解析协议、写 Flash、`printf`、`HAL_Delay`；发送与应答统一回主循环。
7. 共享环形/队列遵循 R17：**写指针只由 ISR 改，读指针只由主循环改**，槽状态用原子标志。Flash 擦写只允许主循环/初始化上下文。
8. 不使用动态内存。运行时可观察量放在**模块私有、字段完整命名的静态结构体**里（`s_bq`、`s_power`、`s_app`、`s_log`、`s_bb`、`s_fault`……；ISR 更新的字段加 `volatile`），调试器直接展开全名（如 `s_bq.state`），不用裸标量，也不要把结构体数组暴露给调试器。
9. 自写代码注释用中文。函数/全局符号用模块前缀（`BQ_`、`Ws2812_`、`Buzzer_`、`App_Can_`、`App_Log_`、`BoardComm_`、`HostCmd_`、`StateMachine_`、`LedStatus_`、`Fault_`、`SelfTest_`、`ResetCause_`、`App_`）；类型后缀 `_t`；私有符号 `static`；宏全大写，位域用 `(1u << n)` 并注释含义。
10. include guard 统一 `APP_<文件名大写>_H`。include 顺序：本模块头 → `board.h` / `main.h` → 标准库 → 下层模块头。不要 include 其他模块的 `.c`。

## 接入 main.c（只动 USER CODE 块）

参考工程的三处接入，本工程照做：

| USER CODE 块 | 内容 |
| --- | --- |
| `Includes` | `#include "app/main_app.h"` |
| `2` | `App_Init();`（放在所有 `MX_*_Init()` 之后） |
| `3`（`while(1)` 内） | `App_Loop();` |

本工程的应用时基是 **TIM2 250 Hz**。TIM2 已在 `.ioc` 中（占位 PSC=0 / ARR=0xFFFFFFFF），但按用户要求**时基参数由固件手动覆写寄存器**：`App_TickTimerInit()` 写 PSC=319 / ARR=999（80 MHz → 250 Hz）、清 UIF、NVIC 优先级 2；`App_Init()` 末尾 `App_TickStart()`（`HAL_TIM_Base_Start_IT` + NVIC 使能）。CubeMX 只生成 `MX_TIM2_Init()`，**Start 必须显式写**。`HAL_TIM_PeriodElapsedCallback` 中按 `htim->Instance == TIM2` 调 `App_OnTick250Hz()`。LP 进入前 `App_TickStop()`，唤醒后重新启动。

## 新模块落地步骤

1. 在正确的层创建 `App/<layer>/<module>.h/.c`，套用 `references/module-templates.md`。
2. 若引用新外设/DMA 句柄，在 `App/board.h` 加 `extern`；引脚继续使用 `main.h` 里的语义宏（`BQ_ALT_EXTI8_Pin` 等），不写死引脚号。
3. 在根目录维护 `app.mk`（首次创建时）：

   ```make
   APP_SOURCES = \
   App/bsp/xxx.c \
   App/comm/xxx.c \
   App/app/xxx.c

   # C_SOURCES 是递归变量：必须立即展开后再追加，`+= $(filter-out $(C_SOURCES), ...)` 会自引用报错。
   APP_NEW_SOURCES := $(filter-out $(C_SOURCES), $(APP_SOURCES))
   C_SOURCES := $(C_SOURCES) $(APP_NEW_SOURCES)
   C_INCLUDES += -IApp -IApp/bsp -IApp/comm -IApp/app
   ```

   `detect/` 未使用时可不加对应 `-I`；应用额外需要、但 IOC 未启用的 HAL 源（如 `stm32l4xx_hal_rtc.c`）同样先用 `filter-out` 去重再加入。
4. 在生成的 `Makefile` 中加 `-include app.mk`（位置见规则 4）；重生成后同时把 `STM32L432xx_FLASH.ld` 的 FLASH 改回 **96K**（`_image_end <= 0x08018000`），不要改成 128K/256K。
5. 在 `App_Init()` 按依赖顺序调用 `Xxx_Init()`；周期任务放 `App_OnTick250Hz()`（用 `s_app.uptime_s`/分频计数或 250 Hz tick 计数分频），耗时/阻塞工作放 `App_Loop()`。
6. 若模块占用 HAL weak 回调，先查 `references/reference-map.md` 的"中断归属"表。**同一回调全库只能有一个 owner**，按 `Instance` 分发并在注释里登记。DMA 通道分配见 CLAUDE.md 的通道表（只用 DMA1：I2C1_RX 用生成的 `hdma_i2c1_rx`/DMA1_Ch7；USART2_TX 用 USART2 中断，不占 DMA）。
7. 手工同步 `compile_commands.json`（按 `app.mk` 补条目）。
8. 验证：`make -j2 all`，再核对 `arm-none-eabi-size build/BMC_CMZY_080.elf` 与 Flash 上限（`arm-none-eabi-nm -n build/BMC_CMZY_080.elf | grep _image_end` 必须 ≤ `0x08018000`；链接脚本 96K，页 48–51 归 NVM/StateStore，黑匣子从 `0x0801A000` 起）。本工程无测试工程，验证 = 编译 + 上板行为。

## BQ 电压异常排查规则（强制）

用户已确认本板 PCB 接线与引脚电压正常。**BQ 某节电压异常（尤其是 cell3/VC5 偶发 6 V、复位相关、reject 持续增加）时，不得以“怀疑硬件/接线”结束排查。** 必须先完成下列软件初始化取证，没有这些证据不得结案：

1. **ADC 受控 0→1 重启**：每次 CONFIG（冷启动、LP 唤醒、总线恢复、运行期恢复）必须先写 `SYS_CTRL2=0`、`CELLBAL1=0`、`SYS_CTRL1=0`，回读确认 ADC/TEMP/CC/FET/均衡全部关闭，保持 ≥300 ms，再在 ADC off 状态写 CC_CFG/增益/保护，最后写 `SYS_CTRL2=CC_EN`、`SYS_CTRL1=0x18` 并逐项回读。MCU 复位不会复位 BQ；若 ADC_EN 本来已是 1，缺少 0→1 边沿就可能保留异常通道。
2. **SYS_CTRL1/2 回读**：配置结束必须确认 `ADC_EN|TEMP_SEL=1`、`CC_EN=1`、FET 位符合预期；回读失败按配置失败重试，不能继续发布电压。
3. **保护写入时序**：保护阈值必须在 ADC off 阶段写入并回读，不能先开 ADC 再写保护。
4. **XREADY**：settle 后读 `SYS_STAT`，XREADY 置位时按“清一次 + 等待 10 ms + 再确认”的有界流程处理；持久 XREADY 报故障。
5. **连续有效帧确认**：首帧（以及 reject 后的首帧）只作为候选，连续 2 帧 cells 差 ≤250 mV、pack 差 ≤300 mV 才发布；发布值不能掩盖 raw 异常。
6. **reject 恢复**：连续 4 帧拒绝触发一次受控 ADC 重启；连续 12 帧或 3 次重启仍失败置 `FAULT_BIT_BQ_ADC_STUCK`，不能只让 `reject_count` 静默增长。
7. **自检动态复查**：BQ 首帧短暂异常只进入 12 s 恢复宽限，不立即锁死三声报错；完成后 MONITOR 持续复查，恢复后自动清除 BQ 失败位并解除 LP 阻止。
8. **pre/post 快照**：保留 `pre_ctrl1/pre_ctrl2/pre_sys_stat/pre_cellbal1`、`adc_off_ctrl1/2`、`post_ctrl1/2`、`settle_sys_stat`、`xready_*`、`adc_restart_*`、`last_reject_*`，台架复位循环时用它们证明软件初始化是否产生真实 ADC 重启。

硬件结论只能在这些软件路径全部验证、且 pre/post 快照与 raw VC3/VC4 仍异常后才能提出，并需给出可复现的引脚/波形证据。

## 参考工程与本工程的对应关系

- 参考工程根目录：`C:\Users\Shuchen Tao\Documents\Armor_Cmzy26_081`（只读）。
- 模块清单、ISR 归属、可移植/不可移植清单、BMC 外设适配映射：`references/reference-map.md`。
- 头/源模板、app.mk 模板、扩展配方：`references/module-templates.md`。
- 需要更细的命名/归属/扩展配方时，以参考工程 `App/README.md` 为准；重生成相关的真实事故见其 `CLAUDE.md`。

## 完成检查表

- [ ] 文件位于正确层，依赖方向无反向/跨层违例（尤其 `detect/` 未依赖 `comm/`）
- [ ] 模块前缀、`_t` 类型、include guard、中文注释符合规范
- [ ] 新 `.c` 已进 `app.mk`，文件名全局唯一
- [ ] `-include app.mk` 在 Makefile 的 `C_INCLUDES` 之后、`OBJECTS` 之前
- [ ] `main.c` 三个 USER CODE 块仍在，Core 其余内容未手改
- [ ] 新句柄只在 `App/board.h` 中 `extern`
- [ ] ISR 只置标志/计数/启动 DMA；协议、CAN 发送、Flash 写都在主循环
- [ ] 250 Hz TIM2 tick 在全部模块初始化完成后才启动；LP 进入前停止、唤醒后重建
- [ ] 调试可见量用字段完整命名的 `static struct`（ISR 写字段加 `volatile`）
- [ ] 涉及 BQ 电压/ADC 的改动已按“BQ 电压异常排查规则”检查 ADC 0→1 重启、回读、XREADY、双帧确认、reject 恢复与自检复查，不以硬件怀疑结案
- [ ] `make -j2 all` 通过，镜像大小核对（`_image_end` ≤ `0x08018000`；链接脚本 96K）
- [ ] 相关协议文档、参数默认值、上位机/主机工具已同步（如适用）

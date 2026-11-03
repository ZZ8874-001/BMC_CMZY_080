# 参考工程结构映射（Armor_Cmzy26_081 → BMC_CMZY_080）

参考工程：`C:\Users\Shuchen Tao\Documents\Armor_Cmzy26_081`（本机只读样本，不要修改）。
阅读顺序：`App/README.md`（目录/命名/中断/构建约定）→ `CLAUDE.md`（架构分层、启动调度、重生成踩坑）→ 目标模块的 `.h`。

## 1. 分层与模块清单

### App/board.h

- 引用 CubeMX 生成句柄的唯一入口：`hadc1`、`hcan1`、`hdac1`、`hspi1`、`htim1/htim2/htim16`、`huart1/huart2`、`hdma_spi1_tx/rx`、`hdma_usart2_tx`。
- 板级常量：`LED_COUNT`、`ADS_FRAME_BYTES`、`ADS_RING_DEPTH`、`APP_TICK_HZ`、通道极性宏。
- 规则：任何模块需要句柄，都在这里加 `extern`，其他文件不自行声明。

### App/bsp/（硬件抽象，面向具体器件/外设）

| 模块 | 角色 | 可复用的结构 |
| --- | --- | --- |
| `ads131m04.c/.h` | ADS131M04 24-bit ADC：SPI1 模式 1、软件 CS、DRDY EXTI、TX/RX DMA、32 槽环形缓冲、诊断 | SPI + EXTI + DMA 驱动的完整样板；`Init/IsInitOk/ReadFrame/GetDiag/Irq*` 形态 |
| `ws2812_uart.c/.h` | USART2 WS2812 灯带驱动，TX 反相、GRB、12 B/LED（本工程 L0 用 **TX 中断，无 DMA**） | UART 编码驱动样板；回调分发模式 |
| `temp_mon.c/.h` | ADC1 + TIM1 触发的温度健康监测 | 周期采样 + 健康判据模块样板 |
| `ref_store.c/.h` | 击打参考样本的 Flash 单页存储（页 63） | "单一数据块 + magic/version/CRC + 纯读上电"样板 |
| `flash_store.c/.h` | 通用 Flash 擦/写原语，主循环专用 | 可直接移植的持久化底层 |

### App/detect/（算法/判定，本工程按需）

- `hit_detect.c/.h`：触发 + 窗口锁定 + 相关度确认 + 事件队列。
- `calibration.c/.h`：P01–P27 参数表、默认值、双槽 Flash 持久化、在线调参入口。

规则：**`detect/` 不得 include `comm/`**。若本工程没有信号判定算法，就不建该层；一旦建立，必须保持这个依赖边界。

### App/comm/（传输/协议）

| 模块 | 角色 | 可复用的结构 |
| --- | --- | --- |
| `app_can.c/.h` | CAN 平台抽象：逻辑端口、帧结构、RX 回调、诊断、Bus-Off 恢复 | HAL CAN 适配层样板；ISR 入队/置标志，主循环恢复 |
| `can_node.c/.h` | UID 排序 → 运行时 NodeID 枚举 | 只在需要同样节点枚举机制时移植 |
| `board_comm.c/.h` | 业务帧定义/解析/上报/ACK、心跳 | 固定帧协议样板；不要照搬 ID 与业务语义 |
| `app_log.c/.h` | USART1 非阻塞文本日志 + 二进制块 DMA 发送 + 100 ms 看门狗 | 日志/调试通道样板 |
| `host_cmd.c/.h` | USART1 RX DMA（零接收中断）+ 命令解析 + ACK | 上位机命令通道样板 |

### App/app/（编排层）

| 模块 | 角色 |
| --- | --- |
| `main_app.c/.h` | `App_Init` / `App_Loop` / `App_OnTick1ms`、模块初始化顺序、弱回调桥接、诊断快照 |
| `state_machine.c/.h` | 应用状态机（BOOT/NORMAL/HIT/FAULT/...），1 kHz 驱动 |
| `led_status.c/.h` | 状态 → 灯效映射，20 Hz 刷新 |
| `faults.c/.h` | 统一 16-bit 故障注册表（live + sticky），状态机/灯/上报的唯一来源 |
| `self_test.c/.h` | 非阻塞上电自检 |
| `reset_cause.c/.h` | 复位原因锁存（RCC->CSR + RTC BKP0R） |

### 顶层构建文件

- `app.mk`：`APP_SOURCES` + `C_INCLUDES += -IApp ...`；由生成的 `Makefile` 用 `-include app.mk` 引入。
- CubeMX 生成的 `Makefile` 中，引入点位于 `C_INCLUDES` 定义之后、编译规则/`OBJECTS` 之前（参考工程在第 136–139 行）。

## 2. 启动与调度（参考工程实现）

`App_Init()` 的顺序（依赖优先，最后一个动作永远是启动 tick）：

```
ResetCause_Init → Cal_Load → App_Log_Init → Fault_Init → App_Can_Init
→ 传感器/BSP Init（失败置 sticky 故障）→ 灯/LED Init → 状态机 Init
→ BoardComm_Init → HostCmd_Init → SelfTest_Init/Request
→ HAL_TIM_Base_Start_IT(&htim2)
```

`App_Loop()` 的原则：非阻塞、固定顺序、只做一层薄编排。

```
SelfTest_Poll → 消费硬件帧 → 事件出队 → 状态机/上报 → 诊断输出
→ HostCmd_Loop → BoardComm_Loop → 恢复/看门狗 → 日志小批量刷出
```

`App_OnTick1ms()`（TIM2 中断）：

- 每 1 ms：状态机、灯效相位；
- 每 50 ms：健康评估/慢任务；
- 每 1000 ms：诊断速率快照（DRDY/帧/错误/丢帧）。

关键约定：

- HAL 时基仍用 SysTick（`HAL_GetTick` 用于超时），应用调度用独立的 TIMx tick。
- 所有发送类动作（CAN 帧、UART 二进制块）在 `App_Loop`，ISR 只置标志/入队/启动 DMA。
- 每个主循环最多处理有限工作量（参考工程：每圈最多一帧 CAN、2 字符日志刷出），保证采样/实时性。
- **本工程实际**：TIM2 **250 Hz**（PSC=319/ARR=999 由 `App_TickTimerInit()` 覆写，不依赖 IOC），回调 `App_OnTick250Hz()`；LP 前 `App_TickStop()`，唤醒重建后 `App_TickStart()`。初始化顺序见 `App/app/main_app.c` 的 `App_Init()`。

## 3. 中断（weak 回调）归属

参考工程的做法：**一个 HAL weak 回调全库只有一个 owner**，在 owner 内按 `Instance` 分发，并在 `App/README.md` 的归属表登记。

| weak 回调 | owner | 规则 |
| --- | --- | --- |
| `HAL_TIM_PeriodElapsedCallback` | `app/main_app.c` | 仅 TIM2 → `App_OnTick1ms()` |
| `HAL_GPIO_EXTI_Callback` | 对应 BSP 驱动 | 按 pin 分发，只置标志/启动 DMA |
| `HAL_SPI_TxRxCpltCallback` | `bsp/ads131m04.c` | 按 SPI 实例分发 |
| `HAL_UART_TxCpltCallback` | `bsp/ws2812_uart.c`（唯一 owner） | 按 UART 实例分发 WS2812 / 日志 |
| `HAL_UART_RxCpltCallback` / `ErrorCallback` | 不使用（USART1 RX 走 DMA，零接收中断） | 不要重新占用 |
| `HAL_CAN_RxFifo0MsgPendingCallback` | `comm/app_can.c` | FIFO0 排空 → 回调/入队，不做业务解析 |
| `HAL_CAN_ErrorCallback` | `comm/app_can.c` | 记录 LEC/Bus-Off 标志，主循环恢复 |
| `HAL_PWR_PVD_Callback` | `app/faults.c` | 置 sticky 欠压故障 |

本工程（BMC_CMZY_080）L0 的实际分配：

| 中断源 | owner | 说明 |
| --- | --- | --- |
| TIM2 更新 | `app/main_app.c` | 250 Hz 应用 tick（PSC/ARR 由固件覆写）；SysTick 仍留给 HAL。LP 前停、唤醒后重启 |
| USART1 TX + DMA1_Ch4 | `comm/app_log.c` | 日志 DMA；`HAL_GPIO_EXTI_Callback` 之外唯一的 UART TX 完成分发点之一 |
| USART2 TX（IT，无 DMA） | `bsp/ws2812_uart.c` | RGB 灯中断发送（prio1）；DMA1_Ch7 留给 I2C1_RX |
| I2C1 EV/ER + DMA1_Ch6 + DMA1_Ch7 | `bsp/bq_ic.c` | BQ 事务；`HAL_I2C_Master*Cplt/ErrorCallback` 只置标志/推进阶段 |
| EXTI9_5 + `HAL_GPIO_EXTI_Callback` | `app/power_mode.c` | PB5 下降沿中断置唤醒标志；PA8 为事件模式（无中断、无挂起位，只唤醒 STOP1） |
| RTC_WKUP | `app/power_mode.c` | WUT 20 s；回调关 WUTIE 并置 `flag_rtc` |
| PVD | `app/faults.c` | sticky 欠压故障 |
| CAN1 RX0 / SCE | 未启用 | L0 不做 CAN；落地通信时在 `.ioc` 使能 NVIC 并新建 `comm/app_can.c` |

落地前先在 `.ioc` 的 NVIC 页确认中断已使能，并在 `Core/Src/stm32l4xx_it.c` 确认 HAL IRQ handler 已生成。不要手改 `Core/` 的非 USER CODE 区域。

## 4. 命名与头文件

- 函数/全局前缀 = 模块名：`ADS131M04_`、`Ws2812_`、`TempMon_`、`RefStore_`、`Fs_`、`HitDetect_`、`Cal_`、`App_Can_`、`App_Log_`、`CanNode_`、`BoardComm_`、`HostCmd_`、`StateMachine_`、`LedStatus_`、`Fault_`、`SelfTest_`、`ResetCause_`、`App_`。
- 类型后缀 `_t`；模块私有符号 `static`；宏全大写，位域 `(1u << n)`。
- include guard：`APP_<文件名大写>_H`（参考工程唯一例外 `APP_BSP_REF_STORE_H`，新代码不要仿照）。
- include 顺序：本模块头 → `board.h`/`main.h` → 标准库 → 下层模块头。
- 自写注释用中文；生成代码的英文注释保持原样。
- 调试可见量放在模块私有、字段完整命名的静态结构体（`s_bq`、`s_power`、`s_app`、`s_log`、`s_bb`、`s_fault`……；ISR 更新字段加 `volatile`），便于调试器逐字段展开；不再用裸标量 `s_foo`。

## 5. BMC_CMZY_080 适配映射

参考工程是装甲板，本工程是电池/电源控制板。**只移植结构与模式，不照搬硬件常量、协议定义和算法。**

| 参考模块 | 本工程用法 | 注意事项 |
| --- | --- | --- |
| `app/main_app.c/.h` | 直接采用三段式结构（Init/Loop/OnTick1ms） | 初始化列表替换为本板 BSP/COMM 模块 |
| `app/state_machine`、`led_status`、`faults`、`self_test`、`reset_cause` | 采用同样的编排/单一故障源/非阻塞自检模式 | 状态、故障位、灯效按本板需求重定义 |
| `comm/app_log` | 已移植：USART1（PB6/PB7）非阻塞日志 | 115200 由固件覆写 BRR=694（不依赖 IOC 值）；TX DMA1_Ch4，句柄 `hdma_log_usart1_tx` 由 App 定义 |
| `comm/app_can` | CAN1（PA11/PA12）适配层 | 当前约 833 kbit/s；参考工程 500 kbit/s，不要照抄 Prescaler/TQ |
| `comm/can_node`、`board_comm` | 仅在需要多节点枚举/固定帧业务协议时移植 | CAN ID、DLC、字节序必须按本工程协议文档重新定义 |
| `comm/host_cmd` | 可选：USART1 下行命令通道 | 需要先定义帧格式/命令表，双端（固件+上位机）同步 |
| `bsp/ws2812_uart` | 已移植：USART2 PA2 `RGB_USART2_TX` → U20（WS2812 类） | 运行时 deinit/reinit：USART2SEL=PCLK1（80 MHz）、BaudRate=3809523、UE=0 写 BRR=21（3.8095 MBd）、TXINV、推挽、清 HDSEL、1 灯 12 B；冷启动 NORM 和 LP 唤醒均重配。**USART2 中断发送（prio1，无 DMA）**，DMA1_Ch7 留给 I2C1_RX。硬件风险：5 V 灯 VIH≈3.5 V > 3.3 V 输出，需实测 |
| `bsp/ads131m04` | 不直接适用（本工程未配置 SPI ADC） | 作为 SPI + EXTI + DMA 驱动模板参考 |
| `bsp/temp_mon` | 可作为周期健康监测模板 | 数据源换成 BQ IC（I2C1）或新增的模拟通道 |
| `bsp/flash_store` | 已移植：黑匣子日志底层 | 页 52–63（`0x0801A000–0x0801FFFF`，24 KB），1020 条 × 24 B、CRC16；`_etext` 必须低于 `0x0801A000`。链接脚本已改 128K，CubeMX 重生成会恢复 256K，必须改回 |
| `bsp/ref_store` | 不适用（击打参考专用） | 新持久化模块要重新分配页，禁止占用参考工程地址 |
| `detect/hit_detect`、`calibration` | 不适用（装甲击打算法） | 若本板有包/按键判定算法，新建 `detect/` 模块并沿用"detect 不依赖 comm"规则 |
| `App/board.h` | 直接采用 | extern 本工程 L0 句柄：`hi2c1`、`hcomp2`、`hrtc`、`htim1`、`htim2`、`huart1`、`huart2`、`hdma_i2c1_tx`、`hdma_i2c1_rx`（I2C1_RX），以及 App 定义的 `hdma_log_usart1_tx`；`hcan1`/`hdac1` 在 L0 不使用，未 extern |

### 明确不要照搬的内容

- ADS131M04 的寄存器/时序、击打相关度算法、P01–P27 参数、13 灯与队色、装甲板 CAN 枚举 `0x120–0x123` / 业务槽位 `0x130+...`、HP/死亡语义、Referee 协议。
- USART 波特率、TX 反相、GPIO 有效电平、LED 数量、Flash 页地址、链接脚本内存大小、所有引脚映射。
- 参考工程的历史踩坑（USART IRQ 丢失、页 63/62/61 划分）只作为检查项，不要复制其数值。

## 6. 模块移植配方（形状优先）

1. 读目标模块的 `.h`（API/类型/常量）和 `.c` 的 `Init/Loop/Tick/Isr` 结构。
2. 复制骨架到本工程正确层，先改模块前缀和 include guard。
3. 把 `board.h` 中的句柄/常量映射到本板；把外设实例、波特率、DMA、引脚宏替换成 `.ioc` 实际值。
4. 在 `App_Init`/`App_Loop`/`App_OnTick1ms` 接入，按依赖顺序放置。
5. 登记 ISR owner；在 `.ioc` 使能对应 NVIC。
6. 加入 `app.mk`，构建并核对镜像大小；上板验证后再补文档/参数表。

## 7. 参考工程文档索引

- `App/README.md` — 目录、分层、命名、头文件、ISR 归属、app.mk、参数表、扩展配方。
- `CLAUDE.md` — 架构、启动/调度、数据流、CAN、Flash 布局、CubeMX 重生成清单、调试变量约定。
- `Docs/CAN_PROTOCOL.md`、`Docs/USART1波形数据帧格式.md` — 协议文档写法样例（本工程需要自己的协议文档）。
- `Docs/开机标定与状态手册.md`、`Docs/装甲模块固件设计方案.md` — 操作手册/设计文档的组织样例。

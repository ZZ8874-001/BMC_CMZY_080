# BMC_CMZY_080 系统状态、电源/时钟/外设与状态切换

> **2026-11-10 降级说明**：真实 VC5/包压不再参与控制/保护/SOC；cell3=(cell1+cell2+1)/2、pack=三节和，raw VC5/BAT 仅诊断。`bq_anomaly.c/.h`、partial/2 A OCP、9 帧/2 min/5 min 锁存、`SM_STATE_BQ_ANOMALY`、`PROTECTION_PARTIAL`、LP `-2` 均已删除；均衡永久关闭。下文涉及这些状态/路径的段落视为历史，以当前源码为准。
>
> **范围**：当前工作树的复位式 LP、M2 保护监督；以实际集成后的源码与宏为准，不将旧构建或上板记录视为本次修改的验证。
>
> **事实来源优先级**：
>
> 1. 当前 `App/`、`Core/Src`、`Core/Inc` 的 C/H 源码（本文档的唯一行为依据）；
> 2. 必要的 STM32L4 HAL/CMSIS 行为（只用于解释 HAL 调用）；
> 3. `BMC_CMZY_080.ioc`（只表示 CubeMX 的配置意图，不代表运行时时钟/外设真实状态）；
> 4. `CLAUDE.md`、`DOCS/POWER_MODE.md`（已按当前源码更新；与本文冲突时仍以源码为准）。
>
> 当前 LP 不是 RTC/LPTIM 中断唤醒，而是 **IWDG 超时复位式 LP**：进入 STOP1 后由 IWDG 计时复位，下一次 boot 的轻量路径完成“sniff”。**RTC 时钟源由宏选择：默认 `LP_RTC_USE_LSE=0` 走 LSI（当前硬件 LSE 不起振），`LP_RTC_USE_LSE=1` 走 LSE；LSE 失败回退 LSI 但禁用 LP。** RTC 只作绝对时间参考；LPTIM1 已被显式关闭；RTC WUT 已被显式关闭。PB5 生产唤醒是 EXTI5；`LP_SMBA_WAKE_TEST=1` 仅为台架备用路径。

## 0. 阅读说明

### 0.1 状态分层

本文档把系统状态分成四层，避免把“电源状态”和“应用逻辑状态”混为一谈：

```text
异常/复位层：POR、NRST、BOR、IWDG、软件复位、HardFault/NMI
        ↓
启动分类层：LP_BOOT_NORMAL / START / RESUME / WAKE / FAULT
        ↓
电源/CPU 层：S0 复位、S1 HSI16 启动、S2 LP 轻量 boot、S3 LP STOP1、
             S4 完整 boot、S5 NORM、S6 DEGRADED、S7 异常复位
        ↓
逻辑应用层：SM BOOT/NORM/BQ_ANOMALY/FAULT、self-test、charge policy、BQ 子状态
```

- 第一节按**电源/启动状态**组织 H2；SM/self-test/charge policy/BQ 子状态放在 S5 NORM 的 `1.5.4 逻辑子状态`。
- 第二节按**状态切换**组织 H2；每个切换都给出时序和切换前后全资源对照。
- 附录给出跨状态总表、复位分类真值表、关键寄存器索引、旧文档过时项。

### 0.2 图例


| 标记               | 含义                                                           |
| -------------------- | ---------------------------------------------------------------- |
| `运行` / `已启动`  | 外设时钟使能，外设已启动并实际工作。                           |
| `已初始化，未启动` | HAL/CubeMX 初始化已执行，但未调用 Start/Enable，或无实际业务。 |
| `已停止`           | 外设已启动过，当前被停止（计数/输出关闭）。                    |
| `已反初始化`       | 调用 DeInit，外设/GPIO 时钟或通道被释放。                      |
| `保持`             | STOP1 中由 SMEN/睡眠时钟保持，退出后可访问。                   |
| `未配置`           | 当前状态没有配置该资源（不是“配置为关闭”）。                 |
| `未使用`           | 芯片存在但项目从未启用该 IP。                                  |

> “未配置”与“已初始化，未启动”不同：前者没有初始化/时钟/引脚配置；后者已经执行过 `MX_*_Init`，只是没有 Start。

### 0.3 统一资源清单

本文档所有状态/切换表都覆盖以下资源：

**时钟/时序**

- `SYSCLK`、`HCLK`、`PCLK1`、`PCLK2`、AHB/APB 分频；
- `MSI`、`HSI16`、`HSE`、`PLL`、`LSE`、`LSI`；
- `VOS`（电压调节范围）、`FLASH_ACR` 等待周期；
- `SysTick`；`RTC`（`LP_RTC_USE_LSE` 选 LSI/LSE、异步/同步预分频、无 WUT）；`IWDG`（LSI）；
- `LPTIM1`（LSI 内核，未启动）；`TIM1`/`TIM2`；`I2C1`、`USART1`、`USART2`、`CAN1`、`DAC1`、`COMP2`、`PVD` 的内核时钟；
- `RCC_CFGR.STOPWUCK`、`PWR_CR1.LPMS`、`SCB_SCR.SLEEPDEEP`。

**电源/复位**

- `PWR_CR1` 的 `VOS`、`LPMS`、`LPR`、`DBP`；`PWR_CR2` 的 PVD；`PWR_CR3/CR4`（未配置）；
- `PWR->SR1` 唤醒标志；`RCC->CSR` 复位标志；备份域 `RTC->BKPxR`；
- 5 V 电源 `PB0/SUP_5V_EN`；`PA1/HSE_DIS`；`PB1/BQ_BOOT`；
- `IWDG` 运行/冻结；`DBGMCU_CR.DBG_STOP`、`DBGMCU_APB1FZR1.DBG_IWDG_STOP`；
- BQ 的 `SYS_CTRL1`（ADC_EN/TEMP_SEL）、`SYS_CTRL2`（CC_EN/CHG/DSG）、`SYS_STAT`、保护寄存器。

**外设**

- GPIOA/GPIOB/GPIOC 及其引脚：PA1、PA2、PA3、PA4、PA7、PA8、PA9、PA10、PA11、PA12、PB0、PB1、PB3、PB4、PB5、PB6、PB7、PC14、PC15；
- EXTI line 5（PB5）、line 8（PA8）、line 16（PVD）、line 20（RTC WUT）、line 32（LPTIM1）；
- NVIC、DMA1 及通道 Ch4/Ch6/Ch7；
- I2C1、USART1、USART2、TIM1、TIM2、COMP2、DAC1、CAN1、LPTIM1、RTC、IWDG、PVD、SysTick、DBGMCU、内部 Flash/备份域；
- BQ76920（外部 AFE，经 I2C1）：ADC/TEMP、CC、FET、保护、状态机、LP sniff/monitor；
- MCU 内未使用 IP：ADC1/ADC2、SPI1/2、USB、WWDG、I2C2/I2C3、LPUART1、USART3/UART4/UART5、SAI、OPAMP、VREFBUF、COMP1、TIM6/7/15/16/17、LPTIM2、RNG、AES、CRC、PVM 等——所有状态均为 `未使用`。

---

# 第一节 系统状态

## 1.1 S0：硬件复位 / 上电

### 1.1.1 时钟配置


| 项目               | 状态                                      | 说明                                                                                                                                                                  |
| -------------------- | ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SYSCLK`           | 复位默认 MSI 4 MHz                        | 未执行 `SystemClock_Config()` 前由 MSI 供钟；`HAL_Init()` 用该时钟建立初始 1 ms tick。                                                                                |
| `HCLK/PCLK1/PCLK2` | 4 MHz，/1                                 | 复位默认分频。                                                                                                                                                        |
| `MSI`              | 开（复位默认）                            | 4 MHz；后续 `SystemClock_Config()` 不会关闭 MSI。                                                                                                                     |
| `HSI16`            | 未配置/关                                 | 复位默认关闭。                                                                                                                                                        |
| `HSE`              | 未配置/关                                 | 复位默认关闭；PA0 为 OSC_IN，PA1 由外部 4k7 上拉为高。                                                                                                                |
| `PLL`              | 未配置/关                                 | 复位默认关闭。                                                                                                                                                        |
| `LSE`              | POR：未配置/关；非 POR 复位：可能仍在运行 | 备份域在 POR 时清除；NRST/IWDG/软件复位不会清除备份域，RTC（默认 LSI）/BKP 可能保留。当前默认 LSI 模式不会启动 LSE。                                                  |
| `LSI`              | POR：未配置；非 POR：可能仍在运行         | IWDG 使用 LSI；默认 RTC 也使用 LSI。                                                                                                                                  |
| `RTC`              | POR：未配置；非 POR：可能保持             | RTC 时钟源/预分频/日历由备份域保持；源由 `LP_RTC_USE_LSE` 和 `RtcTime_MspClockSource()` 决定。                                                                        |
| `IWDG`             | 未由应用配置                              | `MX_IWDG_Init()` 的 USER CODE 在 `HAL_Init()`/`MX_RTC_Init()` 之后调用 `Watchdog_Init()`，把 IWDG 配成 /256、RLR 4095（约 32.8 s）boot guard；`App_Init()` 再改 /64。 |
| `SysTick`          | 未启动                                    | `HAL_Init()` 之后才建立初始 tick。                                                                                                                                    |
| `FLASH_ACR`        | 复位默认 0 WS                             | 未设置等待周期。                                                                                                                                                      |
| `VOS`              | 复位默认 Range 2                          | 未写 `PWR_CR1.VOS`。                                                                                                                                                  |
| `STOPWUCK`         | 复位默认 0（MSI）                         | 未配置。                                                                                                                                                              |
| `LPMS`             | 复位默认                                  | 未配置。                                                                                                                                                              |

### 1.1.2 电源配置


| 项目               | 状态                        | 说明                                                                                        |
| -------------------- | ----------------------------- | --------------------------------------------------------------------------------------------- |
| `PWR_CR1.VOS`      | 复位默认（Range 2）         | 未由应用写。                                                                                |
| `PWR_CR1.LPMS`     | 复位默认                    | 未进入任何低功耗模式。                                                                      |
| `PWR_CR1.LPR`      | 复位默认 0                  | 未配置低功耗运行。                                                                          |
| `PWR_CR1.DBP`      | 0（写保护）                 | 备份域写保护开启；应用稍后 `HAL_PWR_EnableBkUpAccess()`。                                   |
| `PWR_CR2`（PVD）   | 未配置                      | PVD 关闭；`Fault_Init()` 尚未运行。                                                         |
| `PWR_CR3/CR4`      | 未配置                      | 无唤醒引脚/内部唤醒线配置。                                                                 |
| 5 V（PB0）         | 硬件高阻/关                 | `MX_GPIO_Init()` 前 PB0 高阻；外部 5 V 使能无效。                                           |
| `PA1/HSE_DIS`      | 硬件高阻，外部 4k7 上拉到高 | 复位期间 PA1 高阻，R16 4k7 拉高，OSC1/PACK_SC 前端关闭；`MX_GPIO_Init()` 会短暂拉低再拉高。 |
| `PB1/BQ_BOOT`      | 高阻/低                     | 未配置；BQ 可能处于 SHIP/NORMAL。                                                           |
| BQ ADC/TEMP/CC/FET | 未配置/未知                 | BQ 尚未由 MCU 初始化；其自身 POR 默认进入 SHIP，或由备份电源保持。                          |
| 备份域             | POR 清除；非 POR 保持       | BKP0..BKP23、RTC 日历、LSE 状态取决于复位类型。                                             |
| IWDG               | 未配置/未启动               | 应用即将用 `Watchdog_Init()` 启动 /256。                                                    |

### 1.1.3 外设配置


| 外设/资源                 | 状态                              | 说明                                         |
| --------------------------- | ----------------------------------- | ---------------------------------------------- |
| GPIOA/B/C                 | 未配置                            | 复位后模拟/输入；`MX_GPIO_Init()` 尚未执行。 |
| EXTI line 5/8/16/20/32    | 未配置                            | 无 EXTI 中断/事件。                          |
| NVIC                      | 未配置                            | 仅复位默认；无应用 IRQ。                     |
| DMA1                      | 未配置                            | 复位关闭。                                   |
| I2C1                      | 未配置                            | 无时钟/引脚/时序。                           |
| USART1/USART2             | 未配置                            | 无时钟/引脚/波特率。                         |
| TIM1/TIM2                 | 未配置                            | 无时钟/计数。                                |
| COMP2                     | 未配置                            | 无输入/输出。                                |
| DAC1                      | 未配置                            | 无时钟/通道。                                |
| CAN1                      | 未配置                            | 无时钟/引脚。                                |
| LPTIM1                    | 未配置                            | 无时钟/计数。                                |
| RTC/WUT                   | 未配置（POR）；可能保持（非 POR） | WUT 不会由硬件自动布防；应用尚未运行。       |
| IWDG                      | 未配置                            | 应用即将启动 boot guard。                    |
| PVD                       | 未配置                            | 未使能。                                     |
| SysTick                   | 未启动                            | `HAL_Init()` 之前无 tick。                   |
| DBGMCU                    | 未配置                            | `DBG_STOP`/`DBG_IWDG_STOP` 尚未写。          |
| Flash                     | 未配置                            | 无应用写/擦除。                              |
| BQ76920                   | 未初始化                          | 外部 AFE 状态未知；`BQ_Init()` 尚未运行。    |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用                            | 项目未启用。                                 |

> S0 的“未配置”表示应用尚未接管；POR 与非 POR 复位的备份域状态不同，后文的启动分类会利用这一点。

## 1.2 S1：HSI16 启动 / 复位守卫

S1 是从 `main()` 进入 `App_Boot()` 的启动阶段；执行 `Watchdog_Init()`、`HAL_Init()`、`SystemClock_Config()` 和全部 `MX_*_Init()`，但还没有执行 `App_Init()` 的业务初始化。

### 1.2.1 时钟配置


| 项目                     | 状态                                         | 说明                                                                                                                                                 |
| -------------------------- | ---------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `SYSCLK`                 | HSI16 = 16 MHz                               | `SystemClock_Config()`：`RCC_SYSCLKSOURCE_HSI`；AHB/APB1/APB2 均为 DIV1。                                                                            |
| `HCLK/PCLK1/PCLK2`       | 16 MHz                                       | /1 分频。                                                                                                                                            |
| `VOS`                    | Scale2                                       | `HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2)`。                                                                                    |
| `FLASH_ACR`              | 2 WS                                         | `FLASH_LATENCY_2`。                                                                                                                                  |
| `MSI`                    | 开（4 MHz）                                  | `SystemClock_Config()` 未选择 MSI，HAL 不会关闭它；整个运行期保持开启。                                                                              |
| `HSI16`                  | 开，SYSCLK                                   | `RCC_OSCILLATORTYPE_HSI`。                                                                                                                           |
| `HSE`                    | 关                                           | 仅 `PowerMode_ClkSwitchToHsePll()` 在完整 boot 时开启。                                                                                              |
| `PLL`                    | 关                                           | 未配置/未启动。                                                                                                                                      |
| `LSE`                    | 默认关（LSI 模式）；LSE 模式才由 helper 启动 | `LP_RTC_USE_LSE=0` 不启动 LSE；`=1` 时 `RtcTime_MspClockSource()` 用 `LP_LSE_DRIVE_CAPABILITY`（默认 MEDIUMHIGH）有界启动，失败清 LSEON 并回退 LSI。 |
| `LSI`                    | 开                                           | RTC 默认时钟源；IWDG 也使用。                                                                                                                        |
| `LSE CSS`                | 关                                           | `.ioc` 与 `main.c` 都保持 LSECSSON=0；旧构建残留会被防御性清除。                                                                                     |
| `RTC`                    | 默认 LSI，1 Hz                               | 异步 127 / 同步 255；WUT 未布防；`RtcTime_Init()` 严格校验宏对应源。                                                                                 |
| `LPTIM1`                 | LSI，已初始化，未启动                        | `MX_LPTIM1_Init()`；不参与 LP；`PowerMode_Init()`/轻量 boot 关闭并屏蔽。                                                                             |
| `IWDG`                   | /256，约 32.8 s                              | `MX_IWDG_Init()` 先用 HAL /4 启动，随后 `USER CODE` 的 `Watchdog_Init()` 改为 /256。                                                                 |
| `SysTick`                | 1 ms @16 MHz，优先级 15                      | `HAL_Init()` 先建 4 MHz tick，`HAL_RCC_ClockConfig()` 后重建 16 MHz tick。                                                                           |
| `TIM1/TIM2` 内核时钟     | 16 MHz                                       | APB1/APB2 16 MHz；TIM2 未启动；TIM1 未启动。                                                                                                         |
| `I2C1` 内核时钟          | SYSCLK = 16 MHz                              | MSP 选择 `RCC_I2C1CLKSOURCE_SYSCLK`；`MX_I2C1_Init()` 写入 `0x1424A4FF`，在 16 MHz 下实际约 19 kHz（NORM 80 MHz 下才是约 95 kHz）。                  |
| `USART1/USART2` 内核时钟 | PCLK2/PCLK1 = 16 MHz                         | 115200 8N1；USART2 半双工 115200。                                                                                                                   |
| `CAN1` 内核时钟          | PCLK1 = 16 MHz                               | 已初始化，但从未启动；`.ioc` 的分频在 16 MHz 下约 166.7 kbit/s。                                                                                     |
| `DAC1/COMP2` 内核时钟    | APB1/APB2 16 MHz                             | DAC1 初始化但未启动；COMP2 初始化但未启动。                                                                                                          |
| `PVD`                    | 未配置                                       | `Fault_Init()` 尚未运行。                                                                                                                            |
| `STOPWUCK`               | 0（MSI）                                     | 未配置；S1 不进入 STOP。                                                                                                                             |

### 1.2.2 电源配置


| 项目               | 状态                                                              | 说明                                                                                                                                          |
| -------------------- | ------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `PWR_CR1.VOS`      | Scale2                                                            | 见上。                                                                                                                                        |
| `PWR_CR1.DBP`      | 1（已使能备份访问）                                               | `SystemClock_Config()` 调 `HAL_PWR_EnableBkUpAccess()`；之后未清除。                                                                          |
| `PWR_CR2`（PVD）   | 未配置                                                            | 未使能；PVD 只在 `Fault_Init()` 的完整 boot 中开启。                                                                                          |
| `PWR_CR3/CR4`      | 未配置                                                            | 无内部唤醒线/唤醒引脚。                                                                                                                       |
| 5 V（PB0）         | 低/关                                                             | `MX_GPIO_Init()` 将 PB0 置低；`App_Boot()` 再次置低。                                                                                         |
| `PA1/HSE_DIS`      | `MX_GPIO_Init()` 先低后高；`App_Boot()` 再拉高                    | 生成代码先写低（可能短暂使能 OSC1/PACK_SC），USER CODE 再写高；`App_Boot()` 明确写高。完整 boot 的 `PowerMode_Init()` 才会拉低。              |
| `PB1/BQ_BOOT`      | 低                                                                | `MX_GPIO_Init()` 输出低；BQ 尚未由 `BQ_Init()` 探测。                                                                                         |
| `PB3/PB4`          | 低                                                                | LED 全部灭。                                                                                                                                  |
| BQ ADC/TEMP/CC/FET | 未配置                                                            | `BQ_Init()` 尚未运行；BQ 保持其自身状态。                                                                                                     |
| 备份域             | 保持（非 POR）                                                    | RTC/BKP 可保留；`ResetCause_Init()` 正在读取/消费 BKP0。                                                                                      |
| IWDG               | /256，boot guard                                                  | 约 32.8 s；在 `MX_IWDG_Init()` 中、`App_Boot()` 的 RTC 读取/BQ sniff/session 判定之前启动。                                                   |
| `DBGMCU`           | `DBG_IWDG_STOP` 已置位；`DBG_STOP` 尚未由 `PowerMode_Init()` 决定 | `Watchdog_Program()` 设置 IWDG 调试冻结；`DBG_STOP` 由完整 boot 的 `PowerMode_DbgStopEnable()`/轻量路径的 `PowerMode_LpDbgStopApply()` 决定。 |
| SysTick            | 1 ms                                                              | 16 MHz 下；HAL_Delay 可用。                                                                                                                   |

### 1.2.3 外设配置


| 外设/资源                 | 状态                   | 说明                                                                                                                                                                                                    |
| --------------------------- | ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| GPIOA                     | 已配置                 | PA1 输出（先低后高）；PA2 AF7（USART2，OD/pull-up）；PA3/PA4 模拟（COMP2）；PA7 AF1（TIM1_CH1N）；PA8 `GPIO_MODE_EVT_RISING`（EXTI8 event，无 IMR/NVIC）；PA9/PA10 AF4（I2C1）；PA11/PA12 AF9（CAN1）。 |
| GPIOB                     | 已配置                 | PB0/PB1/PB3/PB4 输出低；PB5 输入上拉（NORM 仅轮询）；PB6/PB7 AF7（USART1）。                                                                                                                            |
| GPIOC                     | 时钟已使能，未配置引脚 | LSE 的 PC14/PC15 由 RCC 控制。                                                                                                                                                                          |
| EXTI line 5（PB5）        | 未配置                 | `MX_GPIO_Init()` 只做 `GPIO_Input`；EXTI5 只在 LP 轻量路径的 `Button_WakeInit()` 中配置。                                                                                                               |
| EXTI line 8（PA8）        | event rising           | `GPIO_MODE_EVT_RISING`；EMR1 bit8；无 IMR/NVIC；不能唤醒 WFI，只作事件线/电平轮询。                                                                                                                     |
| EXTI line 16（PVD）       | 未配置                 | PVD 尚未启用。                                                                                                                                                                                          |
| EXTI line 20（RTC WUT）   | 未配置                 | RTC WUT 未布防；`PowerMode_Init()` 稍后会显式关闭/清 pending。                                                                                                                                          |
| EXTI line 32（LPTIM1）    | 未配置/未启动          | LPTIM1 未启动；无 NVIC/EXTI 使能。                                                                                                                                                                      |
| NVIC                      | 部分配置               | DMA1 Ch6/Ch7 优先级 0、TIM2 优先级 0（由 MX MSP）；USART/I2C/TIM1/COMP/EXTI 应用 IRQ 尚未配置。                                                                                                         |
| DMA1                      | 时钟使能；Ch6/7 已配置 | I2C1 TX/RX 的 DMA1 Ch6/Ch7 已由 I2C MSP 建立；Ch4 尚未建立；无 DMA 业务。                                                                                                                               |
| I2C1                      | 已初始化，未启动业务   | 16 MHz 下 BQ 无访问；时序 `0x1424A4FF` 在 16 MHz 下约 19 kHz。                                                                                                                                          |
| USART1                    | 已初始化，未启动业务   | 115200 @16 MHz；无 DMA/IRQ；仅占位。                                                                                                                                                                    |
| USART2                    | 已初始化，未启动业务   | 半双工 115200 @16 MHz；PA2 AF7 OD；无 IRQ。                                                                                                                                                             |
| TIM1                      | 已初始化，未启动       | PA7 AF1；MOE 关闭；无 PWM。                                                                                                                                                                             |
| TIM2                      | 已初始化，未启动       | PSC/ARR 为占位值；NVIC 优先级 0 已使能；`App_Init()` 稍后改为 250 Hz。                                                                                                                                  |
| COMP2                     | 已初始化，未启动       | PA3/PA4 模拟；`HAL_COMP_Start` 尚未调用。                                                                                                                                                               |
| DAC1                      | 已初始化，未启动       | 内部通道 1；无输出；从未 `HAL_DAC_Start`。                                                                                                                                                              |
| CAN1                      | 已初始化，未启动       | 无过滤器、无 `HAL_CAN_Start`、无 CAN IRQ；16 MHz 下时序约 166.7 kbit/s。                                                                                                                                |
| LPTIM1                    | 已初始化，未启动       | LSI 内核时钟；无 `HAL_LPTIM_Start`；后续 `PowerMode_Init()` 会清 ENABLE/EXTI/NVIC。                                                                                                                     |
| RTC/WUT                   | RTC 运行；WUT 未配置   | 默认 LSI 1 Hz（LSE 模式为 LSE 1 Hz）；`RTC_WKUP` NVIC 尚未关闭（`PowerMode_Init()` 在完整 boot 中关闭）；LP 轻量路径也会 `LpCycle_DisableLegacyWut()`。                                                 |
| IWDG                      | 运行                   | /256；`Watchdog_Feed()` 尚未开始主循环。                                                                                                                                                                |
| PVD                       | 未配置                 | 无 PVD IRQ。                                                                                                                                                                                            |
| SysTick                   | 运行                   | 1 ms @16 MHz，优先级 15。                                                                                                                                                                               |
| DBGMCU                    | 部分配置               | `DBG_IWDG_STOP` 已置位；`DBG_STOP` 待 `PowerMode_Init()`/`PowerMode_LpDbgStopApply()`。                                                                                                                 |
| Flash                     | 已初始化接口，未写     | `HAL_Init()` 初始化 Flash 接口；无应用写入。                                                                                                                                                            |
| BQ76920                   | 未初始化               | `BQ_Init()` 尚未运行；PB1 未脉冲。                                                                                                                                                                      |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用                 | 项目未启用。                                                                                                                                                                                            |

> S1 结束时，`App_Boot()` 执行 `ResetCause_Init()`、`RtcTime_Init()`、`LpCycle_ClassifyBoot()`。分类结果决定进入 S2（LP 轻量 boot）还是 S4（完整 boot）；S1 本身的时钟/电源/外设状态保持不变。

## 1.3 S2：LP 轻量 boot（START / RESUME）

`LpCycle_ClassifyBoot()` 返回 `LP_BOOT_START` 或 `LP_BOOT_RESUME` 时，`App_Boot()` 直接调用 `LpCycle_Run()`，**不执行 `App_Init()`**。该路径的最终目的是做一次 BQ sniff/充电检查，然后进入 STOP1。

### 1.3.1 时钟配置


| 项目               | 状态                                                  | 说明                                                                                               |
| -------------------- | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `SYSCLK`           | HSI16 = 16 MHz                                        | 仍是 S1 的 `SystemClock_Config()` 结果；轻量路径不切 HSE/PLL。                                     |
| `HCLK/PCLK1/PCLK2` | 16 MHz                                                | /1。                                                                                               |
| `VOS`              | Scale2                                                | 未切 Scale1。                                                                                      |
| `FLASH_ACR`        | 2 WS                                                  | 保持。                                                                                             |
| `MSI`              | 开（4 MHz）                                           | 未关闭。                                                                                           |
| `HSI16`            | 开                                                    | 轻量路径与 BQ sniff 使用。                                                                         |
| `HSE`              | 关                                                    | 轻量路径不启动；复位已清除。                                                                       |
| `PLL`              | 关                                                    | 轻量路径不启动。                                                                                   |
| `LSE`              | 默认关（LSI 模式）                                    | RTC 默认 LSI；LSE 模式由宏启用；`RtcTime_Read()` 使用。                                            |
| `LSI`              | 开                                                    | IWDG。                                                                                             |
| `RTC`              | 默认 LSI，1 Hz                                        | 读日历/亚秒；WUT 被 `LpCycle_DisableLegacyWut()` 关闭。                                            |
| `LPTIM1`           | LSI 内核时钟，外设关闭                                | `LpCycle_StopUnusedPeripherals()` 清 `ENABLE`、关 NVIC、清 EXTI32；时钟门仍可能使能。              |
| `IWDG`             | /256（boot guard）→ `Watchdog_SetTimeoutMs()` 目标值 | 轻量 boot 先保持 /256；`LpCycle_EnterStop()` 重新编程为目标值。                                    |
| `SysTick`          | 1 ms @16 MHz                                          | 用于 `HAL_Delay`、超时；进 STOP 前挂起。                                                           |
| `I2C1` 内核时钟    | HSI16 = 16 MHz                                        | `BQ_LpSniffInit()` 使用 `BQ_I2C_TIMINGR_HSI16_100K`（约 100 kHz）；sniff 后 `BQ_LpSniffDeInit()`。 |
| `USART1/USART2`    | 反初始化                                              | `HAL_UART_DeInit()` 后无内核时钟业务。                                                             |
| `TIM1`             | 反初始化/停止                                         | `HAL_TIM_PWM_DeInit(&htim1)`；但该路径不完整，PA7 可能保持 AF1（见附录 E）。                       |
| `TIM2`             | 反初始化/停止                                         | `HAL_TIM_Base_DeInit(&htim2)`；无 250 Hz。                                                         |
| `CAN1`             | 反初始化                                              | `HAL_CAN_DeInit`。                                                                                 |
| `DAC1`             | 反初始化                                              | `HAL_DAC_DeInit`。                                                                                 |
| `COMP2`            | 停止                                                  | `HAL_COMP_Stop`；充电检查时短暂 `HAL_COMP_Start`。                                                 |
| `PVD`              | 未配置                                                | 轻量 boot 不执行 `Fault_Init()`，所以 PVD 未使能。                                                 |
| `STOPWUCK`         | 进入 STOP 前设置为 HSI16                              | `SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK)`。                                                          |

### 1.3.2 电源配置


| 项目               | 状态                                                                                                                 | 说明                                                                                                                                                                                       |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `PWR_CR1.VOS`      | Scale2                                                                                                               | 保持。                                                                                                                                                                                     |
| `PWR_CR1.LPMS`     | 尚未设置（进 STOP 前才写 STOP1）                                                                                     | 轻量 boot 期间为普通 Run。                                                                                                                                                                 |
| `PWR_CR1.DBP`      | 1                                                                                                                    | 备份访问保持。                                                                                                                                                                             |
| `PWR_CR2`（PVD）   | 未配置                                                                                                               | 轻量 boot 不使能 PVD；`Fault_PvdDisable()` 也未被调用。                                                                                                                                    |
| `PWR_CR3/CR4`      | 未配置                                                                                                               | 无唤醒引脚/内部唤醒线。                                                                                                                                                                    |
| 5 V（PB0）         | 低/关                                                                                                                | `LpCycle_Run()` 明确写低。                                                                                                                                                                 |
| `PA1/HSE_DIS`      | 高（默认）；`LpCycle_CheckCharger()` 时短暂低                                                                        | 高=OSC1/PACK_SC 前端关闭；充电检查临时低约 30 ms，随后恢复高。                                                                                                                             |
| `PB1/BQ_BOOT`      | 低                                                                                                                   | 轻量 boot 不启动 BQ 配置；`BQ_LpBootPrepare()` 只读/写控制寄存器。                                                                                                                         |
| `PB3/PB4`          | 低                                                                                                                   | LED 灭；PB4 每轮 10 ms 心跳。                                                                                                                                                              |
| BQ ADC/TEMP/CC/FET | 全部关闭                                                                                                             | `BQ_LpBootPrepare()` 若发现 ADC/TEMP/CC/FET 任一开启，会写 `SYS_CTRL1=0x00`、`SYS_CTRL2=0x00`，等待 300 ms 后再读 SYS_STAT；清 `CC_READY/XREADY/OVRD_ALERT`，保留 OV/UV/SCD/OCD 保护锁存。 |
| 备份域             | 保持                                                                                                                 | LP session BKP1..BKP10、RTC 日历、BKP0 复位原因。                                                                                                                                          |
| IWDG               | /256 → 目标值                                                                                                       | `LpCycle_EnterStop()` 计算目标（默认 30 s，减去 boot overhead；最小 1 s），并记录 PR/RLR。                                                                                                 |
| `DBGMCU`           | `DBG_STOP` 由 `PowerMode_LpDbgStopApply()` 按 `POWER_LP_KEEP_SWD` 决定；`DBG_IWDG_STOP` 由 `Watchdog_Program()` 置位 | 默认构建 `POWER_LP_KEEP_SWD=0` 清除 `DBG_STOP`；调试器 halt 时 IWDG 会被冻结。                                                                                                             |

### 1.3.3 外设配置


| 外设/资源                 | 状态                                            | 说明                                                                                                                                                           |
| --------------------------- | ------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| GPIOA                     | 已配置                                          | PA1=高；PA2 已由 UART DeInit 释放；PA3/PA4 模拟（COMP2 停止）；PA7 可能保持 AF1；PA8 event rising；PA9/PA10 由 I2C DeInit 释放；PA11/PA12 由 CAN DeInit 释放。 |
| GPIOB                     | 已配置                                          | PB0/PB1/PB3/PB4 低；PB5 `GPIO_MODE_IT_FALLING` + 内部上拉（`Button_WakeInit()`）；PB6/PB7 由 UART1 DeInit 释放。                                               |
| GPIOC                     | 时钟作用域内保持；未配置引脚                    | PC14/PC15 为 LSE。                                                                                                                                             |
| EXTI line 5（PB5）        | 已配置                                          | `Button_WakeInit()`：SYSCFG EXTICR2 → PB5，FTSR1 bit5，IMR1 bit5，清 PR1 bit5，NVIC EXTI9_5 优先级 0。                                                        |
| EXTI line 8（PA8）        | event rising                                    | 无 IMR/NVIC；只作 PA8 电平/事件线；不能唤醒 WFI。                                                                                                              |
| EXTI line 16（PVD）       | 未配置                                          | PVD 未使能。                                                                                                                                                   |
| EXTI line 20（RTC WUT）   | 关闭/清 pending                                 | `LpCycle_DisableLegacyWut()` 清 WUTF/EXTI20/NVIC；无 RTC 唤醒。                                                                                                |
| EXTI line 32（LPTIM1）    | 关闭/屏蔽                                       | `LpCycle_StopUnusedPeripherals()` 清 IMR2/EMR2、关 NVIC/清 pending。                                                                                           |
| NVIC                      | 部分配置                                        | EXTI9_5 优先级 0；DMA1 Ch6/7 仍可能使能（Ch6/7 被 I2C DeInit 释放）；I2C1_ER 仅在 SMBA 测试时优先级 1；无 TIM2/USART/LPTIM IRQ。                               |
| DMA1                      | 时钟保持；Ch6/7 反初始化，NVIC 可能仍使能       | `BQ_LpSniffInit` 用阻塞 I2C，不用 DMA；Ch4 未建立。                                                                                                            |
| I2C1                      | sniff 期间 100 kHz 阻塞；sniff 后反初始化       | `BQ_LpSniffInit()`：I2C1 DeInit/Init，`BQ_I2C_TIMINGR_HSI16_100K`，模拟/数字滤波；`BQ_LpSniffDeInit()`：DeInit。                                               |
| USART1                    | 已反初始化                                      | `HAL_UART_DeInit`；无日志。                                                                                                                                    |
| USART2                    | 已反初始化                                      | `HAL_UART_DeInit`；无 RGB。                                                                                                                                    |
| TIM1                      | 已反初始化/停止                                 | `HAL_TIM_PWM_DeInit`；PA7 可能残留 AF1；无蜂鸣器。                                                                                                             |
| TIM2                      | 已反初始化/停止                                 | 无 250 Hz。                                                                                                                                                    |
| COMP2                     | 停止；充电检查时短暂启动                        | `HAL_COMP_Stop`；`LpCycle_CheckCharger()` 中 `HAL_COMP_Start`，30 ms 后 `HAL_COMP_Stop`。                                                                      |
| DAC1                      | 已反初始化                                      | 无输出。                                                                                                                                                       |
| CAN1                      | 已反初始化                                      | 无 CAN。                                                                                                                                                       |
| LPTIM1                    | 关闭/屏蔽                                       | `CR.ENABLE=0`；NVIC/EXTI32 屏蔽；不参与唤醒。                                                                                                                  |
| RTC/WUT                   | RTC 运行，WUT 关闭                              | `RtcTime_Read()` 读取；WUT 被 `LpCycle_DisableLegacyWut()` 关闭。                                                                                              |
| IWDG                      | 运行                                            | /256 → 目标；`LpCycle_EnterStop()` 成功后不喂，直到 IWDG 复位或 EXTI 提前唤醒。                                                                               |
| PVD                       | 未配置                                          | 轻量 boot 无 PVD。                                                                                                                                             |
| SysTick                   | 1 ms                                            | 进 STOP 前 `HAL_SuspendTick()` + 清 PendST。                                                                                                                   |
| DBGMCU                    | `DBG_IWDG_STOP` 置位；`DBG_STOP` 由编译开关决定 | 默认 `POWER_LP_KEEP_SWD=0` 清除 `DBG_STOP`。                                                                                                                   |
| Flash                     | 30 min 监测时写                                 | `LpCycle_DoMark()` → `BlackBox_Init()` + `BlackBox_WritePeriodicSnapshot()`；失败 → full wake。                                                              |
| BQ76920                   | 阻塞 sniff/监测，无 BQ 状态机                   | `BQ_LpBootPrepare()` 强制 ADC/TEMP/CC/FET off；`BQ_LpMonitorRun()` 在 30 min 点临时开 ADC/CC 采样。                                                            |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用                                          | 项目未启用。                                                                                                                                                   |

> `LP_BOOT_START` 与 `LP_BOOT_RESUME` 的硬件状态相同；RESUME 额外执行 `LpCycle_Calibrate()`（用 RTC 间隔校准 LSI），并在 `anomaly_count >= 2` 时强制 full wake。

## 1.4 S3：LP STOP1

S3 是 `LpCycle_EnterStop()` 执行 `WFI` 后的 STOP1 窗口。CPU/总线停止，只有 IWDG、RTC（默认 LSI）、必要 SMEN 时钟和可选 SMBA/I2C1 保持。

### 1.4.1 时钟配置


| 项目                      | 状态                                      | 说明                                                                                                                       |
| --------------------------- | ------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `SYSCLK/HCLK/PCLK1/PCLK2` | 停止                                      | STOP1 中 VCORE 域时钟停止。                                                                                                |
| `STOPWUCK`                | 1 = HSI16                                 | `SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK)`；退出 STOP1 时选 HSI16，而不是 MSI。                                               |
| `HSI16`                   | STOP 中停；退出时开                       | 作为 STOP 唤醒时钟；代码退出后 `SystemCoreClock=16 MHz` + `HAL_InitTick()`。                                               |
| `MSI`                     | STOP 中停                                 | 不参与唤醒；复位默认 4 MHz 在 Run 才开。                                                                                   |
| `HSE`                     | 关                                        | 轻量路径从未启动。                                                                                                         |
| `PLL`                     | 关                                        | 轻量路径从未启动。                                                                                                         |
| `LSE`                     | 默认关（LSI 模式）                        | RTC 默认 LSI；LSE 模式由宏启用；`RTCAPB` SMEN 保持。                                                                       |
| `LSI`                     | 开                                        | IWDG。                                                                                                                     |
| `RTC`                     | 默认 LSI（LSE 模式为 LSE）运行；WUT 关闭  | 无 RTC WUT 唤醒；RTCAPB SMEN 保持。                                                                                        |
| `LPTIM1`                  | LSI 内核时钟，外设关闭/EXTI32 屏蔽        | 不参与唤醒。                                                                                                               |
| `IWDG`                    | 运行，动态目标                            | `Watchdog_SetTimeoutMs()` 设置；STOP 中继续计数，超时产生复位。                                                            |
| `SysTick`                 | 挂起                                      | `HAL_SuspendTick()`；`SCB->ICSR = PENDSTCLR`；退出后 `HAL_InitTick()`。                                                    |
| `I2C1` 内核时钟           | HSI16，仅在 SMBA 测试时                   | `Button_SmbaEnable()` 将 `CCIPR.I2C1SEL=HSI16`；`LpCycle_EnterStop()` 仅在 `s_smba_test_active` 时保持 `I2C1EN/I2C1SMEN`。 |
| `USART/TIM/CAN/DAC`       | 停止/反初始化                             | 无内核时钟业务。                                                                                                           |
| `PVD`                     | 未配置                                    | STOP 中无 PVD。                                                                                                            |
| `DBGMCU`                  | `DBG_STOP` 默认清除；`DBG_IWDG_STOP` 置位 | `POWER_LP_KEEP_SWD=0` 时 STOP 关闭 SWD；IWDG 在调试 halt 时冻结。                                                          |

### 1.4.2 电源配置


| 项目                          | 状态                                            | 说明                                                                                    |
| ------------------------------- | ------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `PWR_CR1.LPMS`                | `STOP1`                                         | `MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP1)`。                              |
| `SCB_SCR.SLEEPDEEP`           | 1                                               | `SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk)`；WFI 进入 Stop。                             |
| `PWR_CR1.VOS`                 | Scale2                                          | 保持轻量 boot 的 Scale2。                                                               |
| `PWR_CR1.LPR`                 | 0（未配置）                                     | 未使用低功耗运行模式。                                                                  |
| `PWR_CR1.DBP`                 | 1                                               | 备份访问保持。                                                                          |
| `PWR_CR2`（PVD）              | 未配置                                          | 无 PVD。                                                                                |
| `PWR_CR3/CR4`                 | 未配置                                          | 无内部唤醒线/唤醒引脚。                                                                 |
| `APB1ENR1/SMENR1`             | `PWREN`/`RTCAPBEN`、`PWRSMEN`/`RTCAPBSMEN` 保持 | `LpCycle_EnterStop()` 显式置位；保证 STOP 后第一次 PWR/RTC 访问不挂死。                 |
| `GPIOA/GPIOB/SYSCFG` 睡眠时钟 | 保持                                            | `__HAL_RCC_GPIOA_CLK_SLEEP_ENABLE()`、`GPIOB`、`SYSCFG`；用于唤醒后读 PA8/PB5 与 EXTI。 |
| `I2C1` APB/SMEN               | 仅 SMBA 测试时保持                              | `RCC_APB1ENR1.I2C1EN`、`RCC_APB1SMENR1.I2C1SMEN`；生产 EXTI5 路径不保持。               |
| 5 V（PB0）                    | 低/关                                           | 所有 5 V 负载断电。                                                                     |
| `PA1/HSE_DIS`                 | 高                                              | OSC1/PACK_SC 前端关闭；充电检查已结束。                                                 |
| `PB1/BQ_BOOT`                 | 低                                              | BQ 不启动。                                                                             |
| PB3/PB4                       | 低（PB4 心跳已结束）                            | LED 灭。                                                                                |
| BQ ADC/TEMP/CC/FET            | 全部关闭                                        | `BQ_LpBootPrepare()` 已强制 off；保护锁存保留；无电压/电流/温度采样。                   |
| 备份域                        | 保持                                            | RTC/BKP/session 保持；IWDG 复位不会清除。                                               |
| IWDG                          | 运行，超时复位                                  | 周期唤醒的**唯一**来源；动态目标约 1–30 s。                                            |

### 1.4.3 外设配置


| 外设/资源                 | 状态                                            | 说明                                                                                                              |
| --------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| GPIOA                     | 保持                                            | PA1 高；PA2/PA9/PA10/PA11/PA12 由 DeInit 释放；PA3/PA4 模拟；PA7 可能 AF1；PA8 event rising；GPIOA 睡眠时钟保持。 |
| GPIOB                     | 保持                                            | PB0/PB1/PB3/PB4 低；PB5 EXTI5 IT falling + pull-up；PB6/PB7 释放；GPIOB 睡眠时钟保持。                            |
| GPIOC                     | 未保持                                          | 无 GPIOC 睡眠时钟；PC14/PC15 仍由 RCC LSE 控制。                                                                  |
| EXTI line 5（PB5）        | 保持/使能                                       | IMR1 bit5、FTSR1 bit5、NVIC EXTI9_5；下降沿可唤醒。                                                               |
| EXTI line 8（PA8）        | event rising                                    | 无 IMR/NVIC；不能唤醒 WFI；唤醒后由软件读电平。                                                                   |
| EXTI line 16（PVD）       | 未配置                                          | 无 PVD。                                                                                                          |
| EXTI line 20（RTC WUT）   | 关闭                                            | WUTE/WUTIE 关闭；WUTF/EXTI20 清；无 RTC 唤醒。                                                                    |
| EXTI line 32（LPTIM1）    | 屏蔽                                            | IMR2/EMR2 清；NVIC 关；无 LPTIM 唤醒。                                                                            |
| NVIC                      | 部分使能                                        | EXTI9_5 优先级 0；DMA1 Ch6/7 NVIC 可能仍使能但通道已释放；I2C1_ER 仅在 SMBA 测试时优先级 1；SysTick 挂起。        |
| DMA1                      | 时钟保持，通道关闭                              | Ch6/7 已由 I2C DeInit；Ch4 未建立；无 DMA 传输。                                                                  |
| I2C1                      | 反初始化；SMBA 测试时启用 HSI16 + WUPEN/ALERTEN | 生产路径无 I2C；SMBA 测试时 I2C1_ER 可唤醒。                                                                      |
| USART1                    | 已反初始化                                      | 无日志/唤醒。                                                                                                     |
| USART2                    | 已反初始化                                      | 无 RGB/唤醒。                                                                                                     |
| TIM1                      | 已反初始化/停止                                 | 无蜂鸣器；PA7 可能残留 AF1。                                                                                      |
| TIM2                      | 已反初始化/停止                                 | 无 250 Hz。                                                                                                       |
| COMP2                     | 停止                                            | 无比较器；充电检查只在进 STOP 前。                                                                                |
| DAC1                      | 已反初始化                                      | 无输出。                                                                                                          |
| CAN1                      | 已反初始化                                      | 无 CAN。                                                                                                          |
| LPTIM1                    | 关闭/屏蔽                                       | 不参与唤醒。                                                                                                      |
| RTC/WUT                   | RTC 运行，WUT 关闭                              | 仅绝对时间；`RTCAPB` 保持。                                                                                       |
| IWDG                      | 运行                                            | 超时复位；不喂。                                                                                                  |
| PVD                       | 未配置                                          | 无 PVD。                                                                                                          |
| SysTick                   | 挂起                                            | 退出后恢复。                                                                                                      |
| DBGMCU                    | `DBG_STOP` 默认清除；`DBG_IWDG_STOP` 置位       | SWD 在 STOP 关闭；IWDG 调试冻结。                                                                                 |
| Flash                     | 无访问                                          | 只在 30 min 监测前/后由轻量 boot 访问。                                                                           |
| BQ76920                   | 全部关闭                                        | ADC/TEMP/CC/FET off；保护锁存保留；无轮询。                                                                       |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用                                          | 项目未启用。                                                                                                      |

> **S3 的唤醒源只有三类**：IWDG 超时复位（周期 tick）、PB5 EXTI5 下降沿（按键）、SMBA 测试期间的 I2C1 ALERT/错误中断。**PA8 BQ ALERT 不是 STOP1 唤醒源**；它只在轻量 boot 的进 STOP 前和提前唤醒后被读电平，且电平只作触发——随后读 SYS_STAT，清除 `CC_READY/XREADY/OVRD_ALERT`，只有 OV/UV/SCD/OCD/XREADY 才作为 `LP_WAKE_ALERT`。RTC WUT 和 LPTIM1 均被显式关闭/屏蔽。

## 1.5 S4：完整 boot（NORMAL / WAKE / FAULT）

S4 是 `LpCycle_ClassifyBoot()` 返回 `LP_BOOT_NORMAL`、`LP_BOOT_WAKE` 或 `LP_BOOT_FAULT` 后执行完整 `App_Init()` 的阶段；最终进入 S5 NORM（若 HSE/PLL 失败则进入 S6 DEGRADED）。

### 1.5.1 时钟配置


| 项目               | 状态                                          | 说明                                                                                                             |
| -------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `SYSCLK`           | HSI16 启动 → HSE bypass 12 MHz → PLL 80 MHz | `PowerMode_ClkSwitchToHsePll()`：PA1 低、30 ms 稳定、HSEBYP、PLLM=3/PLLN=40/PLLR=2；VCO 160 MHz，SYSCLK 80 MHz。 |
| `HCLK/PCLK1/PCLK2` | 80 MHz，/1                                    | `HAL_RCC_ClockConfig`。                                                                                          |
| `VOS`              | Scale1                                        | `HAL_PWREx_ControlVoltageScaling(SCALE1)`。                                                                      |
| `FLASH_ACR`        | 4 WS                                          | `FLASH_LATENCY_4`。                                                                                              |
| `MSI`              | 开（4 MHz）                                   | 未关闭；与 HSI16 一样保持运行。                                                                                  |
| `HSI16`            | 开                                            | 未关闭；NORM 中仍可作 I2C1 SMBA/其它用途。                                                                       |
| `HSE`              | 开，12 MHz bypass                             | OSC1 → R14 22 Ω → PA0；PA1 低使能 OSC1/PACK_SC。                                                              |
| `PLL`              | 开，160 MHz VCO，80 MHz R                     | PLLP/Q 未使用；PLLR 作为 SYSCLK。                                                                                |
| `LSE`              | 默认关（LSI 模式）                            | RTC 默认 LSI；LSE 模式才开。                                                                                     |
| `LSI`              | 开                                            | IWDG。                                                                                                           |
| `RTC`              | 默认 LSI，1 Hz；WUT 关闭                      | `PowerMode_Init()` 清 WUT/WUTF/EXTI20/NVIC。                                                                     |
| `LPTIM1`           | LSI 内核时钟，外设关闭                        | `PowerMode_Init()` 清 ENABLE/EXTI32/NVIC。                                                                       |
| `IWDG`             | 先 /256，`App_Init()` 开头改为 /64 ≈8.2 s    | `Watchdog_InitNorm()`。                                                                                          |
| `SysTick`          | 1 ms @80 MHz                                  | `HAL_RCC_ClockConfig` 重建；优先级 15。                                                                          |
| `TIM1/TIM2` 内核   | 80 MHz                                        | TIM1 按需蜂鸣器；TIM2 250 Hz。                                                                                   |
| `I2C1` 内核        | SYSCLK 80 MHz                                 | `Bq_HwInit()` 设 95 kHz。                                                                                        |
| `USART1/2` 内核    | PCLK2/PCLK1 80 MHz                            | 115200 / 3.8095 MBd。                                                                                            |
| `CAN1` 内核        | PCLK1 80 MHz                                  | 已初始化，从未启动；若启动，同样分频约 833 kbit/s（`.ioc` 16 MHz 数字不再适用）。                                |
| `DAC1/COMP2` 内核  | APB 80 MHz                                    | DAC 未启动；COMP2 启动。                                                                                         |
| `PVD`              | 使能，约 2.8 V                                | `Fault_Init()`；`PWR_PVDLEVEL_5`，EXTI16 rising，NVIC 优先级 4。                                                 |

### 1.5.2 电源配置


| 项目               | 状态                                      | 说明                                                                                        |
| -------------------- | ------------------------------------------- | --------------------------------------------------------------------------------------------- |
| `PWR_CR1.VOS`      | Scale1                                    | NORM 80 MHz。                                                                               |
| `PWR_CR1.LPMS`     | 未配置（Run）                             | 不进入 Stop；`LPMS` 保持之前值，但不影响 Run。                                              |
| `PWR_CR1.DBP`      | 1                                         | 备份访问。                                                                                  |
| `PWR_CR2`（PVD）   | 使能，约 2.8 V                            | PVD 上升沿中断；`FAULT_BIT_POWER` sticky。                                                  |
| `PWR_CR3/CR4`      | 未配置                                    | 无唤醒引脚/内部线。                                                                         |
| 5 V（PB0）         | 高/开                                     | `App_Init()` 置高并 `HAL_Delay(20)`；5 V 负载（CAN/RGB/蜂鸣器/UART）上电。                  |
| `PA1/HSE_DIS`      | 低                                        | OSC1/PACK_SC 前端使能；HSE 可用。                                                           |
| `PB1/BQ_BOOT`      | 低（BQ 初始化时可能 2 ms 高脉冲）         | `BQ_Init()` 探测；SHIP 时 PB1 高 ≥2 ms 再低。                                              |
| `PB3/PB4`          | 由 LED 驱动控制                           | BOOT/NORM/FAULT 灯效；默认低。                                                              |
| BQ ADC/TEMP/CC/FET | 由 BQ 配置/策略控制                       | `BQ_Init` 配置 ADC/TEMP/CC；FET 由 `ChargePolicy` 请求，初始 `ChargePolicy_Init()` 请求 0。 |
| 备份域             | 保持                                      | RTC/BKP/session 保留。                                                                      |
| IWDG               | /64，约 8.2 s                             | 主循环/初始化点喂狗。                                                                       |
| `DBGMCU`           | `DBG_STOP` 默认清除；`DBG_IWDG_STOP` 置位 | `POWER_LP_KEEP_SWD=0` 默认。                                                                |

### 1.5.3 外设配置


| 外设/资源                 | 状态                                      | 说明                                                                                                                                    |
| --------------------------- | ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| GPIOA                     | 已配置                                    | PA1 低；PA2 AF7 推挽（WS2812）；PA3/PA4 模拟（COMP2）；PA7 AF1（TIM1）；PA8 event rising；PA9/PA10 AF4（I2C1）；PA11/PA12 AF9（CAN1）。 |
| GPIOB                     | 已配置                                    | PB0 高（5 V）；PB1 低/脉冲；PB3/PB4 LED；PB5 输入上拉（NORM 轮询）；PB6/PB7 AF7（USART1）。                                             |
| GPIOC                     | 时钟使能，无应用引脚                      | PC14/PC15 LSE。                                                                                                                         |
| EXTI line 5（PB5）        | 未配置                                    | NORM 只轮询 PB5；EXTI5 只在 LP 轻量路径配置。                                                                                           |
| EXTI line 8（PA8）        | event rising                              | EMR1 bit8；无 IMR/NVIC；不唤醒。                                                                                                        |
| EXTI line 16（PVD）       | 已配置                                    | PVD rising，NVIC 优先级 4。                                                                                                             |
| EXTI line 20（RTC WUT）   | 关闭                                      | WUT 未布防；NVIC 关闭。                                                                                                                 |
| EXTI line 32（LPTIM1）    | 屏蔽                                      | LPTIM1 不启动。                                                                                                                         |
| NVIC                      | 完整应用配置                              | TIM2 2、USART2 1、I2C1/DMA1 Ch6/7 3、USART1/DMA1 Ch4/PVD 4、SysTick 15；EXTI9_5 仅在 LP 轻量路径。                                      |
| DMA1                      | Ch4/6/7 建立                              | Ch4 USART1 TX；Ch6/7 I2C1 TX/RX；Ch4/6/7 中断按各驱动优先级。                                                                           |
| I2C1                      | 运行，95 kHz + DMA                        | BQ 配置/轮询；`Bq_HwInit()` 设 `BQ_I2C_TIMINGR_95K`，DMA1 Ch6/7，IRQ 优先级 3。                                                         |
| USART1                    | 运行，115200，DMA1 Ch4                    | `App_Log_Init()`；BRR=694 @80 MHz；DMA/IRQ 优先级 4。                                                                                   |
| USART2                    | 运行，3.8095 MBd，IT                      | `Ws2812_HwInit()`：PCLK1、BRR=21、TXINV、清 HDSEL、PA2 推挽；USART2 IRQ 优先级 1；无 DMA。                                              |
| TIM1                      | 已初始化；按需 PWM                        | PA7 AF1；蜂鸣器按 tone 设 ARR/CCR，启 PWMN + MOE；无中断。                                                                              |
| TIM2                      | 运行，250 Hz                              | PSC=319、ARR=999；更新中断优先级 2；`App_TickStart()`。                                                                                 |
| COMP2                     | 运行                                      | `HAL_COMP_Start`；TIM2 ISR 采样，32 样本去抖；PA3/PA4 模拟。                                                                            |
| DAC1                      | 已初始化，未启动                          | 无输出；从未 `HAL_DAC_Start`。                                                                                                          |
| CAN1                      | 已初始化，未启动                          | 无过滤器/无 `HAL_CAN_Start`/无 CAN IRQ。                                                                                                |
| LPTIM1                    | 关闭/屏蔽                                 | 不启动；EXTI32/NVIC 关闭。                                                                                                              |
| RTC/WUT                   | RTC 运行；WUT 关闭                        | 1 Hz；无 RTC 唤醒。                                                                                                                     |
| IWDG                      | /64，约 8.2 s                             | 运行；主循环喂狗。                                                                                                                      |
| PVD                       | 运行                                      | PVD 2.8 V；PVD_PVM IRQ 优先级 4；`FAULT_BIT_POWER`。                                                                                    |
| SysTick                   | 1 ms @80 MHz                              | 运行。                                                                                                                                  |
| DBGMCU                    | `DBG_STOP` 默认清除；`DBG_IWDG_STOP` 置位 | 默认构建。                                                                                                                              |
| Flash                     | 可写                                      | BlackBox/StateStore/BMS NVM；NORM 30 min 记录；LP 30 min 轻量路径。                                                                     |
| BQ76920                   | 运行/配置                                 | ADC_EN/TEMP_SEL=0x18；CC_EN=0x40；FET 由策略；保护 0x89/0x57/0x40；OV/UV 按 ADC gain/offset 计算；状态机 RUN。                          |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用                                    | 项目未启用。                                                                                                                            |

**三种完整 boot 子类**：


| 子类             | 分类条件                                                          | `App_Init()` 差异                                                                                                                                 | 结束状态                    |
| ------------------ | ------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------- |
| `LP_BOOT_NORMAL` | PIN/BOR、无有效 session、非 START/WAKE 的复位                     | 正常完整 `App_Init()`；仍会调用 `LpCycle_ApplyWake()`，但 session 无 `FULL_WAKE` 时直接返回                                                       | NORM 或 DEGRADED            |
| `LP_BOOT_WAKE`   | SOFTWARE + session `PAUSED` + `LP_FLAG_FULL_WAKE`                 | 完整 `App_Init()` 后 `LpCycle_ApplyWake()`：映射 wake cause、`Button_OnWake()`、`ChargePolicy_OnLpWake()`、黑匣子 wake 记录；FAULT 时禁用 auto-LP | NORM 或 DEGRADED            |
| `LP_BOOT_FAULT`  | HardFault/NMI、IWDG 但 session/interval 不合法、START 时 RTC 失败 | 完整 `App_Init()` 后 `PowerMode_ForceDisableAutoLp()` + `LpSession_Invalidate()`                                                                  | NORM/DEGRADED，auto-LP 关闭 |

## 1.6 S5：NORM 运行态

S5 是 `App_Init()` 完成、`POWER_STATE_NORM`、TIM2 250 Hz 已启动后的稳定运行态。逻辑状态可在 `BOOT/NORM/BQ_ANOMALY/FAULT` 间变化，电压质量与 self-test/charge policy/BQ 子状态独立；S5 的时钟/电源/外设基线不因**仅电压异常**而变化。

### 1.6.1 时钟配置


| 项目               | 状态                         | 说明                              |
| -------------------- | ------------------------------ | ----------------------------------- |
| `SYSCLK`           | 80 MHz HSE/PLL               | 同 S4 最终状态。                  |
| `HCLK/PCLK1/PCLK2` | 80 MHz，/1                   | 同 S4。                           |
| `VOS`              | Scale1                       | 同 S4。                           |
| `FLASH_ACR`        | 4 WS                         | 同 S4。                           |
| `MSI`              | 开（4 MHz）                  | 未关闭；非 SYSCLK。               |
| `HSI16`            | 开                           | 未关闭；I2C1 SMBA/其它可用。      |
| `HSE`              | 12 MHz bypass                | OSC1 由 PA1 低使能。              |
| `PLL`              | 160 MHz VCO，80 MHz R        | 同 S4。                           |
| `LSE`              | 默认关（LSI 模式）           | RTC 默认 LSI。                    |
| `LSI`              | 开                           | IWDG。                            |
| `RTC`              | 默认 LSI，1 Hz；WUT 关闭     | 只作绝对时间。                    |
| `LPTIM1`           | LSI 内核，外设关闭           | 不启动。                          |
| `IWDG`             | /64，约 8.2 s                | 主循环喂狗。                      |
| `SysTick`          | 1 ms @80 MHz                 | 运行。                            |
| `TIM1`             | 80 MHz，按需                 | 蜂鸣器 tone。                     |
| `TIM2`             | 80 MHz，250 Hz               | PSC 319 / ARR 999。               |
| `I2C1`             | SYSCLK 80 MHz，95 kHz        | BQ。                              |
| `USART1`           | PCLK2 80 MHz，115200         | 日志。                            |
| `USART2`           | PCLK1 80 MHz，3.8095 MBd     | WS2812。                          |
| `CAN1`             | PCLK1 80 MHz，已初始化未启动 | 若启动约 833 kbit/s；当前无业务。 |
| `DAC1/COMP2`       | APB 80 MHz                   | DAC 未启动；COMP2 运行。          |
| `PVD`              | 约 2.8 V                     | 运行。                            |
| `STOPWUCK/LPMS`    | 未配置（Run）                | NORM 不使用。                     |

### 1.6.2 电源配置


| 项目               | 状态                                      | 说明                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| -------------------- | ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `PWR_CR1.VOS`      | Scale1                                    | 80 MHz。                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| `PWR_CR1.DBP`      | 1                                         | 备份访问。                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `PWR_CR2`（PVD）   | 2.8 V，IRQ                                | `FAULT_BIT_POWER` sticky。                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 5 V（PB0）         | 高/开                                     | CAN/RGB/蜂鸣器/UART 供电。                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `PA1/HSE_DIS`      | 低                                        | OSC1/PACK_SC 使能。                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `PB1/BQ_BOOT`      | 低                                        | BQ 正常运行；不得再高。                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| PB3/PB4            | LED 驱动                                  | BOOT/NORM/部分电压异常/FAULT 灯效；严格部分异常额外闪 PB3/HOT，BOOT 蓝 RGB 与 HOT 可并行，只有真实关键 FAULT 切红 RGB。                                                                                                                                                                                                                                                                                                                                                     |
| BQ ADC/TEMP/CC/FET | RUN 中开启；FET 按策略/监督层             | 严格 `PARTIAL_VC5_PACK` 时 ADC/CC 持续开启：只有本次已过自检/原保护、曾有真实好帧且距好帧 <20 s、同批 VC1/VC2 与新 CC/COMP2/通信/保护证据持续可信，才允许有条件 DISCHARGE；两次不同 CC 转换 ≥2 A 锁存 OCP。CC/保护证据失效立即关输出，20 s 无新 FULL 关输出并报警。输出中 10 s 仅额外复判，严禁 ADC 重启；输出全关且仍是严格 PARTIAL 才每 ≥10 s 受控重启（ADC/CC/FET off 800 ms，再 CONFIG/300 ms settle/双帧确认）。估算 CELL3/BAT 只显示/标记存档，不代表真实电芯保护。 |
| 备份域             | 保持                                      | RTC/BKP/session。                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| IWDG               | /64，喂狗                                 | 主循环 + 初始化点。                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `DBGMCU`           | `DBG_STOP` 默认清除；`DBG_IWDG_STOP` 置位 | 默认构建。                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

### 1.6.3 外设配置

同 S4 的 NORM 外设基线；补充运行状态：


| 外设/资源                 | 状态               | 说明                                                                                                                                                                                                                    |
| --------------------------- | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| GPIOA/B/C                 | 运行               | PA1 低、PB0 高、PA2 AF7、PA7 AF1、PA8 event、PA9/10 AF4、PA11/12 AF9、PB5 input、PB6/7 AF7。                                                                                                                            |
| EXTI 5                    | 未配置             | NORM 只轮询 PB5；不使能 EXTI5。                                                                                                                                                                                         |
| EXTI 8                    | event rising       | 无 IMR/NVIC；不唤醒。                                                                                                                                                                                                   |
| EXTI 16                   | PVD                | IRQ 优先级 4。                                                                                                                                                                                                          |
| EXTI 20/32                | 关闭               | RTC WUT/LPTIM1 不参与。                                                                                                                                                                                                 |
| NVIC/DMA1                 | 运行               | TIM2 2、USART2 1、I2C1/DMA1 Ch6/7 3、USART1/DMA1 Ch4/PVD 4、SysTick 15。                                                                                                                                                |
| I2C1                      | 运行               | 95 kHz + DMA Ch6/7；BQ 500 ms 轮询。                                                                                                                                                                                    |
| USART1                    | 运行               | 115200 + DMA Ch4；`[BOOT]/[DIAG]/[BQ]/[TEST]/[POL]/[SOC]` 日志。                                                                                                                                                        |
| USART2                    | 运行               | 3.8095 MBd；WS2812 1 LED。                                                                                                                                                                                              |
| TIM1                      | 按需               | 蜂鸣器；无请求时 MOE off。                                                                                                                                                                                              |
| TIM2                      | 运行               | 250 Hz tick；COMP2 采样/计数器/标志。                                                                                                                                                                                   |
| COMP2                     | 运行               | 32 样本去抖；充电/放电判定。                                                                                                                                                                                            |
| DAC1                      | 已初始化，未启动   | 无输出。                                                                                                                                                                                                                |
| CAN1                      | 已初始化，未启动   | 无业务。                                                                                                                                                                                                                |
| LPTIM1                    | 关闭/屏蔽          | 不启动。                                                                                                                                                                                                                |
| RTC/WUT                   | RTC 运行；WUT 关闭 | 无 RTC 唤醒。                                                                                                                                                                                                           |
| IWDG                      | 运行               | /64。                                                                                                                                                                                                                   |
| PVD                       | 运行               | 2.8 V。                                                                                                                                                                                                                 |
| SysTick                   | 运行               | 1 ms。                                                                                                                                                                                                                  |
| DBGMCU                    | 默认               | SWD 在 Run 可用；STOP 关闭（默认）。                                                                                                                                                                                    |
| Flash                     | 可写               | BlackBox/StateStore/BMS NVM；30 min 记录。                                                                                                                                                                              |
| BQ76920                   | RUN                | ADC/TEMP/CC on；FET 按策略；保护 0x89/0x57/0x40（SCD 100 µs，OCD 仅放电）；`BQ_IsFresh` 2 s 窗口；M2 Protection 监督层评估 BQ 锁存、5 A/30 A 软件 OCP、NTC 温度和允许掩码；CC 符号由 `BQ_CC_DISCHARGE_SIGN` 单点定义。 |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用             | 项目未启用。                                                                                                                                                                                                            |

### 1.6.4 逻辑子状态

这些子状态**不改变 S5 的时钟/电源/外设基线**，只改变 BQ FET/控制字、LED、蜂鸣器、日志、自检门控/auto-LP 等。

#### SM 状态


| 状态                  | 条件/动作                                                                             | 影响                                                                                       |
| ----------------------- | --------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `SM_STATE_BOOT`       | `StateMachine_Init()`；无新真实 FULL；自检恢复期                                      | BOOT 蓝色 RGB/PB4；冷启动自检通过后仍蓝闪至少 2 s，直到真实 FULL；部分电压异常可叠加 HOT。 |
| `SM_STATE_NORM`       | 真实 BQ 帧 fresh；无关键 fault                                                        | NORM；flow 由比较器/策略决定 CHARGING/DISCHARGING/STANDBY。                                |
| `SM_STATE_BQ_ANOMALY` | 新批严格判为 PARTIAL_VC5_PACK，且无更高优先级 fault                                   | PB3/HOT 闪烁、RGB 不因部分电压异常切红；策略/保护只按经验证的有条件窗口决定是否放电。      |
| `SM_STATE_FAULT`      | 真实 critical fault（BQ_COMM/XREADY/OV/UV/SCD/OCD/ADC_STUCK/PROTECTION/POWER/CLK 等） | FAULT PB3 5 Hz + RGB 红；策略 FAULT、双 FET 关；优先于 BQ_ANOMALY。                        |

#### Protection 监督层（M2）


| 状态/条件             | 动作                                                   | 影响                                                                                                                                                                     |
| ----------------------- | -------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `PROTECTION_INIT`     | 上电/数据未就绪                                        | `allow=0`，等首帧和配置完成。                                                                                                                                            |
| `PROTECTION_NORMAL`   | 无 BQ 锁存、无软件 OCP、数据有效                       | `allow=0x03`；`PROT_TEMP_ENABLE=1` 时按温度只收紧 CHG 或 DSG。                                                                                                           |
| `PROTECTION_PARTIAL`  | 严格 `BQ_BATCH_PARTIAL_VC5_PACK` 且无硬件/通信真实故障 | 默认 `allow=0`；只有本次曾通过真实 FULL/自检、最近 FULL <20 s、新 CC 与 COMP2/控制/保护证据持续可靠时，可限时放电；部分期 2 A/两次新 CC 转换 OCP。其它 M2 禁止条件优先。 |
| `PROTECTION_RECOVERY` | boot/LP 首次配置前已锁存 SCD/OCD                       | `allow=0`；BQ RUN/idle、FET 全关、`                                                                                                                                      |
| `PROTECTION_LATCHED`  | BQ 运行期锁存、清除失败、软件 OCP                      | `allow=0`、`FAULT_BIT_PROTECTION`；软件 OCP 只接受 `Protection_RequestLatchClear()`。                                                                                    |
| 软件充电 OCP          | CC 充电方向 ≥5 A，2 个新发布帧                        | 锁存双 FET 关；事件 0x1C；清除额外要求充电器移除。                                                                                                                       |
| 软件放电 OCP          | CC 放电方向 ≥30 A，2 个新发布帧                       | 锁存双 FET 关；事件 0x1D；BQ 37.3 A OCD/44.7 A SCD 是硬件后备。                                                                                                          |
| 温度 inhibit          | NTC 充/放电越界，2 s 去抖、2 °C 回差；无效传感器 2 s  | 只禁对应方向（无效则双禁）；不锁存、不置 PROTECTION；`PROT_TEMP_ENABLE=1` 才生效；`FAULT_BIT_TEMP` 只报传感器无效。                                                      |

#### BQ 数据质量、归档与报警

每次完整 RUN 批次先按同批 CRC、范围、状态/控制/保护及新转换证据分类：`BQ_BATCH_FULL_CANDIDATE` 只是合理候选，`BQ_BATCH_FULL_REAL_OK` 才是原双帧确认后真实发布；`BQ_BATCH_PARTIAL_VC5_PACK` 必须是 VC1/VC2 在两批独立读数中可信、仅 VC5/BAT 明确异常且 VC3/VC4、CC、状态、保护、通信均正常；其它均为 `BQ_BATCH_INVALID`。`estimated_cell3_mv`/`estimated_pack_mv` 仅展示/质量标志存档，不能改 `bq_snapshot_t.valid/fresh`、清硬件锁存或作保护输入。独立 `bq_cc_sample_t` 仍是同一 BQ 的单一 CC ADC，不是冗余通道。

NORM 每个 RTC 30 min 标记缓存连续 9 个真实 FULL 发布帧；到点最多延后 60 s，达标候选不被后续坏帧覆盖。前 1–3 个异常标记写降级质量记录，缺失电压字段写 0，SOC 只保留估算/无效标志，不单独报警；第四个连续异常，或窗口结束时 VC1/VC2 仍无可信证据，进入 HOT/PB3 + 最多 2 min 蜂鸣。2 min 内 9 个新 FULL 可取消；否则双 FET 经 BQ 回读确认关断后持久锁存禁止输出/自动 LP。报警起算满 5 min 仅置 STOP2/SHIP 未来 trigger，不执行该模式。StateStore 归档标记和报警锁存跨 IWDG/PIN/掉电保留；Flash 写失败不得假设已持久成功，解锁只能由独立安全条件下的人工入口触发。

正常 LP 30 min 标记只需 `BQ_LpMonitorRun()` 两批独立确认，写一条黑匣子记录，不计入 NORM 九帧、不做轻量 SOC/StateStore 电池快照；异常监测保留 mark 并以无 `LP_WAKE_FAULT` 的完整唤醒交给 NORM 60 s 归档，真正通信/保护/黑匣子故障仍按 FAULT full wake。


| 状态            | 条件/动作                                                        | 影响                                                                        |
| ----------------- | ------------------------------------------------------------------ | ----------------------------------------------------------------------------- |
| `IDLE`          | `SelfTest_Init()`                                                | 瞬时；随后 Request。                                                        |
| `WAIT_BQ`       | `SelfTest_Request()`；BQ fresh 或 3 s 后检查                     | 自检未完成，auto-LP block bit2。                                            |
| `WAIT_BQ_GRACE` | 仅 BQ link/data 失败；grace 只在 `BQ_IsRecovering()==0` 累计     | 12 s grace / 30 s hard cap；失败/通过都进 MONITOR。                         |
| `MONITOR`       | 完成后 1 s 复查；5 s unhealthy → FAIL；3 s healthy → RECOVERED | PASS/FAIL 事件、蜂鸣器、`[TEST]` 日志；影响 policy NormalReady 和 auto-LP。 |

#### Charge policy 状态


| 状态        | 条件                                           | FET 请求       | LP block              |
| ------------- | ------------------------------------------------ | ---------------- | ----------------------- |
| `IDLE`      | 非 NormalReady；无充电器；无用户放电；CAL 结束 | 0x00           | 0（CAL pending 时 1） |
| `CHARGE`    | NormalReady + 充电器稳定                       | 0x03           | 1                     |
| `DISCHARGE` | NormalReady + `PAIR_HOLD3` 用户放电            | 0x03           | 1                     |
| `CAL`       | debugger `cal_mode` + CalReady                 | `cal_fet_mask` | 1                     |
| `FAULT`     | SM FAULT                                       | 0x00           | 1                     |

#### BQ 子状态


| 状态              | 条件                                    | 影响                                                                                |
| ------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------- |
| `RESET`           | `BQ_Init()` 开始                        | 未 fresh。                                                                          |
| `BOOT_PULSE`      | 探测 NACK                               | PB1 高 ≥2 ms 后低。                                                                |
| `BOOT_WAIT`       | 探测 ACK 或脉冲后                       | 20 ms 后配置。                                                                      |
| `CONFIG`          | 配置脚本                                | CC_CFG/gain/保护/ADC_EN。                                                           |
| `SETTLE`          | 配置完成                                | 300 ms ADC settle；`BQ_IsRecovering()`。                                            |
| `RUN`             | settle 验证通过                         | 500 ms 轮询；FET 同步；fresh 发布。                                                 |
| `BACKOFF`         | 通信/配置失败                           | 20 ms 重试或 2 s 后重配。                                                           |
| `ADC_RESTART_OFF` | 受控 ADC 重启（仅输出关的严格部分异常） | ADC/CC/FET off 800 ms，重新 CONFIG；ADC settle 300 ms，窗口内须重新双帧确认。       |
| `PARTIAL_RECHECK` | 输出开时严格部分异常                    | 保持 ADC/CC/FET，不触发 off→on；500 ms 全量读取照常，额外 10 s 只作复判/限频日志。 |

> LP 轻量路径**不运行**上述 BQ 状态机；它使用阻塞 `BQ_LpSniffInit/BQ_LpBootPrepare`/`BQ_LpMonitorRun`，并强制 BQ off。

## 1.7 S6：DEGRADED

`PowerMode_ClkSwitchToHsePll()` 三次尝试失败后进入 DEGRADED；`App_Init()` 在 `PowerMode_Init()` 后提前返回，不启动 BQ/日志/WS2812/COMP/TIM2/自检/状态机。

### 1.7.1 时钟配置


| 项目               | 状态                           | 说明                                                                                              |
| -------------------- | -------------------------------- | --------------------------------------------------------------------------------------------------- |
| `SYSCLK`           | 强制 HSI16 16 MHz              | `PowerMode_ClkForceHsi16()`：HSI on/ready、SYSCLK HSI、`SystemCoreClock=16 MHz`、`HAL_InitTick`。 |
| `HCLK/PCLK1/PCLK2` | 16 MHz                         | /1（前提是时钟配置已回到 HSI；失败路径可能留下分频）。                                            |
| `VOS`              | Scale2                         | `PowerMode_ClkForceHsi16()` 设置。                                                                |
| `FLASH_ACR`        | 可能仍为 4 WS                  | 失败路径**不恢复** Flash latency；16 MHz 下 4 WS 合法但偏慢。                                     |
| `MSI`              | 开（4 MHz）                    | 未关闭。                                                                                          |
| `HSI16`            | 开，SYSCLK                     | 强制。                                                                                            |
| `HSE`              | 可能仍开（失败路径未显式关闭） | 失败发生位置决定；不能保证已关。                                                                  |
| `PLL`              | 可能仍开（失败路径未显式关闭） | 同上。                                                                                            |
| `LSE`              | 默认关                         | RTC 默认 LSI。                                                                                    |
| `LSI`              | 开                             | IWDG。                                                                                            |
| `RTC`              | 默认 LSI，1 Hz；WUT 关闭       | 无唤醒。                                                                                          |
| `LPTIM1`           | LSI 内核，外设关闭             | 未启动。                                                                                          |
| `IWDG`             | /64，约 8.2 s                  | `App_Init()` 开头已设置；喂狗。                                                                   |
| `SysTick`          | 1 ms @16 MHz                   | `PowerMode_ClkForceHsi16()` 重建。                                                                |
| `PVD`              | 2.8 V                          | `Fault_Init()` 在 `PowerMode_Init()` 前已启用。                                                   |
| `STOPWUCK/LPMS`    | 未配置                         | 不进入 Stop。                                                                                     |

### 1.7.2 电源配置


| 项目               | 状态                                      | 说明                                                                                        |
| -------------------- | ------------------------------------------- | --------------------------------------------------------------------------------------------- |
| `PWR_CR1.VOS`      | Scale2                                    | 强制回 HSI 后设置。                                                                         |
| `PWR_CR1.DBP`      | 1                                         | 备份访问。                                                                                  |
| `PWR_CR2`（PVD）   | 2.8 V，IRQ                                | 已由 `Fault_Init()` 开启。                                                                  |
| 5 V（PB0）         | 低/关                                     | `App_Init()` 的 DEGRADED 分支明确写低。                                                     |
| `PA1/HSE_DIS`      | 可能仍低                                  | `PowerMode_ClkSwitchToHsePll()` 入口已拉低；失败路径未抬回高，OSC1/PACK_SC 前端可能仍使能。 |
| `PB1/BQ_BOOT`      | 低/未初始化                               | 无 BQ 业务。                                                                                |
| PB3/PB4            | PB3 250 ms 闪烁；PB4 低                   | 降级指示。                                                                                  |
| BQ ADC/TEMP/CC/FET | 未配置                                    | `BQ_Init()` 未执行。                                                                        |
| 备份域             | 保持                                      | RTC/BKP/session。                                                                           |
| IWDG               | /64，喂狗                                 | `App_Loop` 每次迭代喂狗。                                                                   |
| `DBGMCU`           | `DBG_STOP` 默认清除；`DBG_IWDG_STOP` 置位 | 默认构建。                                                                                  |

### 1.7.3 外设配置


| 外设/资源                 | 状态                 | 说明                                                                                        |
| --------------------------- | ---------------------- | --------------------------------------------------------------------------------------------- |
| GPIOA/B/C                 | 仅 MX 初始化残留     | PA1 低；PB0 低；PB3 闪烁；PB4 低；PA2/PA7/PA8/PA9/PA10/PA11/PA12/PB5/PB6/PB7 保持 MX 配置。 |
| EXTI 5                    | 未配置               | 未进入 LP，PB5 无 EXTI。                                                                    |
| EXTI 8                    | event rising         | 无 IMR/NVIC。                                                                               |
| EXTI 16                   | PVD                  | 2.8 V IRQ。                                                                                 |
| EXTI 20/32                | 关闭                 | 无 RTC WUT/LPTIM。                                                                          |
| NVIC/DMA1                 | MX 残留              | DMA1 Ch6/7 NVIC 可能使能；TIM2 NVIC 可能使能但未启动；无业务 IRQ。                          |
| I2C1                      | 仅 MX 初始化         | 16 MHz 时序；无 BQ。                                                                        |
| USART1                    | 仅 MX 初始化         | 无日志。                                                                                    |
| USART2                    | 仅 MX 初始化         | 无 RGB。                                                                                    |
| TIM1                      | 仅 MX 初始化         | 无蜂鸣器。                                                                                  |
| TIM2                      | 仅 MX 初始化，未启动 | 无 250 Hz。                                                                                 |
| COMP2                     | 仅 MX 初始化，未启动 | 无比较器。                                                                                  |
| DAC1                      | 仅 MX 初始化，未启动 | 无输出。                                                                                    |
| CAN1                      | 仅 MX 初始化，未启动 | 无 CAN。                                                                                    |
| LPTIM1                    | 仅 MX 初始化，未启动 | 无唤醒。                                                                                    |
| RTC/WUT                   | RTC 运行；WUT 关闭   | 无唤醒。                                                                                    |
| IWDG                      | /64，喂狗            | 运行。                                                                                      |
| PVD                       | 运行                 | 2.8 V。                                                                                     |
| SysTick                   | 1 ms @16 MHz         | 运行。                                                                                      |
| DBGMCU                    | 默认                 | SWD 可用；STOP 未使用。                                                                     |
| Flash                     | 无应用写             | 无黑匣子。                                                                                  |
| BQ76920                   | 未配置               | 无 BQ 业务。                                                                                |
| MCU ADC/SPI/USB/WWDG/其它 | 未使用               | 项目未启用。                                                                                |

> DEGRADED 没有自动恢复路径；只能复位。`App_Loop()` 顶层先喂 IWDG，然后进入 `App_DegradedLoop()` 每 250 ms 翻转 PB3、PB4 低。

## 1.8 S7：异常复位 / 终止

### 1.8.1 时钟配置


| 项目                      | 状态          | 说明                                                                                |
| --------------------------- | --------------- | ------------------------------------------------------------------------------------- |
| `SYSCLK`                  | 故障现场时钟  | 异常可能发生在任意状态；处理器不执行应用时钟配置。                                  |
| `HSE/PLL/HSI/MSI/LSE/LSI` | 维持故障现场  | 不在异常处理中切换。                                                                |
| `SysTick`                 | 维持/可能停止 | HardFault 处理不依赖 HAL tick。                                                     |
| `IWDG`                    | 继续运行      | `Error_Handler()` 死循环时最终复位；HardFault/NMI 处理会主动 `NVIC_SystemReset()`。 |
| `PWR/LPMS/STOPWUCK`       | 维持现场      | 不进入 Stop；只写 BKP。                                                             |

### 1.8.2 电源配置


| 项目          | 状态           | 说明                                                                            |
| --------------- | ---------------- | --------------------------------------------------------------------------------- |
| 5 V/PA1/BQ    | 维持故障现场   | 不在异常处理中改变。                                                            |
| 备份域        | 可写           | `ResetCause_RecordAndReset()` 使能备份访问，写 BKP0；HardFault 还写 BKP20–23。 |
| `PWR_CR1.DBP` | 由异常处理置位 | `ResetCause_EnableBackupAccess()`。                                             |
| IWDG          | 运行           | 异常处理主动复位；`Error_Handler()` 等待 IWDG。                                 |
| PVD           | 维持现场       | 不处理。                                                                        |

### 1.8.3 外设配置


| 外设/资源    | 状态                           | 说明                                                                              |
| -------------- | -------------------------------- | ----------------------------------------------------------------------------------- |
| 全部应用外设 | 维持故障现场/未配置            | 异常处理不初始化/反初始化外设；`Error_Handler()` 只关 IRQ 并死循环。              |
| Flash        | 不写                           | 异常处理不写 Flash；只写备份寄存器。                                              |
| BQ           | 维持现场                       | 不访问。                                                                          |
| NVIC         | `Error_Handler()` 关闭全局 IRQ | `__disable_irq()`；HardFault/NMI 记录后复位。                                     |
| 复位后       | S0/S1 重新开始                 | 下次 boot 由 `LpCycle_ClassifyBoot()` 优先识别 HardFault/NMI → `LP_BOOT_FAULT`。 |

---

# 第二节 状态切换操作

> 每个切换都给出：`x.1 切换时序` 与 `x.2 切换前后外设工作状态对照`。
> 对照表使用统一资源行；未使用/未配置显式写 `未配置`。

## 2.1 复位分类与启动分派

### 2.1.1 切换时序

1. 任意复位（POR/NRST/BOR/IWDG/软件/HardFault/NMI）→ S0。
2. `main()`：`HAL_Init()` → `SystemClock_Config()`（HSI16/Scale2/2WS，不等 LSE）→ MX 外设初始化（含 `MX_RTC_Init()` 按宏选源、`MX_IWDG_Init()` 的 USER CODE 调 `Watchdog_Init()` 设 /256 boot guard）→ `App_Boot()`。
3. `App_Boot()`：
   - `ResetCause_Init()`：读 `RCC->CSR` + `RTC->BKP0R`，合并复位原因；清 `RCC_CSR.RMVF`。
   - PA1 高、PB0 低。
   - `RtcTime_Init()`：严格检查宏对应的 RTC 源（默认 `RTCSEL=LSI && LSIRDY`；LSE 模式 `RTCSEL=LSE && LSERDY`）和 RTC ready；必要时写固定日历。
   - `LpCycle_ClassifyBoot()`：读取 BKP session（magic/CRC）、RTC epoch；按复位原因+session 状态分类。
4. 分类结果：
   - `LP_BOOT_START` / `LP_BOOT_RESUME` → `LpCycle_Run()`（S2）；
   - `LP_BOOT_WAKE` / `LP_BOOT_NORMAL` / `LP_BOOT_FAULT` → `App_Init()`（S4）；
   - `LP_BOOT_FAULT` 在 `App_Init()` 后禁用 auto-LP 并使 session 失效。

**复位/分类真值表**


| 复位证据              | session 条件                    | 分类             | 动作                                  |
| ----------------------- | --------------------------------- | ------------------ | --------------------------------------- |
| HardFault/NMI（BKP0） | 任意                            | `LP_BOOT_FAULT`  | session 失效；完整 boot；auto-LP 关闭 |
| PIN/BOR               | 任意                            | `LP_BOOT_NORMAL` | session 失效；完整 boot               |
| SOFTWARE              | `START` + RTC 可读              | `LP_BOOT_START`  | 轻量 LP                               |
| SOFTWARE              | `START` + RTC 失败              | `LP_BOOT_FAULT`  | session 失效；完整 boot；auto-LP 关闭 |
| SOFTWARE              | `PAUSED` + `FULL_WAKE`          | `LP_BOOT_WAKE`   | 完整 boot + ApplyWake                 |
| WATCHDOG              | `STOPPED` + RTC 可读 + 间隔合法 | `LP_BOOT_RESUME` | 轻量 LP（LSI 校准）                   |
| WATCHDOG              | `STOPPED` 但间隔不合法/无 RTC   | `LP_BOOT_FAULT`  | session 失效；完整 boot；auto-LP 关闭 |
| 其它/无 session       | —                              | `LP_BOOT_NORMAL` | 完整 boot                             |

### 2.1.2 切换前后外设工作状态对照


| 资源                   | 切换前（S0）                           | 切换后（S1 分类完成）                                                    |
| ------------------------ | ---------------------------------------- | -------------------------------------------------------------------------- |
| 时钟/SysTick           | MSI 4 MHz；SysTick 未启动；VOS/WS 复位 | HSI16 16 MHz；Scale2；2 WS；RTC 1 Hz（默认 LSI）；SysTick 1 ms；MSI 仍开 |
| GPIOA                  | 未配置                                 | PA1 高；PA2 AF7；PA3/4 模拟；PA7 AF1；PA8 event；PA9/10 AF4；PA11/12 AF9 |
| GPIOB                  | 未配置                                 | PB0/1/3/4 低；PB5 input pull-up；PB6/7 AF7                               |
| GPIOC                  | 未配置                                 | 时钟开、无应用引脚；LSE PC14/15                                          |
| EXTI                   | 未配置                                 | PA8 event；PB5 无 EXTI；PVD/RTC/LPTIM 未配置                             |
| DMA/NVIC               | 未配置                                 | DMA1 时钟/Ch6/7；TIM2 NVIC 0；其它应用 IRQ 未配置                        |
| I2C1                   | 未配置                                 | 已初始化；16 MHz 下约 19 kHz；无业务                                     |
| USART1/2               | 未配置                                 | 已初始化 115200 @16 MHz；无 DMA/IRQ 业务                                 |
| TIM1/TIM2              | 未配置                                 | TIM1 停止；TIM2 占位、未启动                                             |
| COMP2/DAC1/CAN1/LPTIM1 | 未配置                                 | 均初始化但未启动                                                         |
| RTC/WUT                | POR 未配置；非 POR 可能保持            | RTC 1 Hz（默认 LSI）；WUT 未布防                                         |
| IWDG                   | 未配置                                 | /256 boot guard                                                          |
| PVD                    | 未配置                                 | 未配置（完整 boot 才在 `Fault_Init` 开启）                               |
| Flash/DBGMCU           | 未配置                                 | Flash 2 WS；`DBG_IWDG_STOP` 置位                                         |
| BQ                     | 未初始化                               | 未初始化                                                                 |

## 2.2 HSI16 启动 → 完整启动（LP_BOOT_NORMAL）

### 2.2.1 切换时序

1. `LpCycle_ClassifyBoot()` 返回 `LP_BOOT_NORMAL`（PIN/BOR/无匹配 token/其它）。
2. `App_Init()`：`Watchdog_InitNorm()`（/64）→ 清 `s_app` → `Fault_Init()`（PVD 2.8 V）→ `PowerMode_Init()`：
   - PWR/RTCAPB 时钟 + SMEN；备份访问；RTC 源检查；WUT 关闭/清 pending；RTC_WKUP NVIC 关；LPTIM1 关闭/EXTI32 屏蔽。
   - `PowerMode_ClkSwitchToHsePll()`：PA1 低、30 ms、HSE bypass 12 MHz、PLL 80 MHz、Scale1、4 WS，3 次重试。
3. 成功 → NORM；`BlackBox_Init`/`StateStore_Init` → 5 V 高 + 20 ms → `App_Log_Init` → `BQ_Init` → `SocEst_Init` → `Ws2812_Init` → LED/Buzzer/Button/Gesture → SelfTest/SM/Policy/Cal → `HAL_COMP_Start` → TIM2 250 Hz → `LpCycle_ApplyWake()`（无 FULL_WAKE 时无动作）。
4. 失败 → DEGRADED（见 2.6）。

### 2.2.2 切换前后外设工作状态对照


| 资源                   | 切换前（S1）                     | 切换后（NORM）                                                    |
| ------------------------ | ---------------------------------- | ------------------------------------------------------------------- |
| 时钟/SysTick           | HSI16 16 MHz；Scale2；2 WS       | PLL 80 MHz；Scale1；4 WS；SysTick 1 ms @80 MHz；MSI/HSI 仍开      |
| GPIOA                  | MX 配置；PA1 高                  | PA1 低；PA2 AF7 推挽；PA7 AF1；PA8 event；PA9/10 AF4；PA11/12 AF9 |
| GPIOB                  | MX 配置；PB0/1/3/4 低；PB5 input | PB0 高（5 V）；PB1 低；PB3/PB4 LED；PB5 input；PB6/7 AF7          |
| GPIOC                  | 时钟开                           | 同左；LSE                                                         |
| EXTI                   | PA8 event；PB5 无                | PVD 使能；PA8 event；PB5 仍无 EXTI；RTC/LPTIM 关                  |
| DMA/NVIC               | DMA1 Ch6/7；TIM2 NVIC            | Ch4/6/7；TIM2 2、USART2 1、I2C/DMA 3、USART1/DMA4/PVD 4           |
| I2C1                   | 已初始化，无业务                 | 95 kHz + DMA；BQ 运行                                             |
| USART1/2               | 已初始化，无业务                 | 115200 + DMA；3.8095 MBd + IT                                     |
| TIM1/TIM2              | 停止/未启动                      | TIM1 按需；TIM2 250 Hz                                            |
| COMP2/DAC1/CAN1/LPTIM1 | 均未启动                         | COMP2 运行；DAC/CAN 仍初始化未启动；LPTIM1 关闭                   |
| RTC/WUT                | RTC 1 Hz；WUT 未配置             | RTC 1 Hz；WUT 关闭；无唤醒                                        |
| IWDG                   | /256                             | /64 ≈8.2 s，喂狗                                                 |
| PVD                    | 未配置                           | 2.8 V IRQ                                                         |
| Flash/DBGMCU           | 2 WS；DBG_IWDG_STOP              | 4 WS；DBG_STOP 默认清；DBG_IWDG_STOP                              |
| BQ                     | 未初始化                         | RUN/配置；ADC/TEMP/CC；FET 按策略                                 |

## 2.3 HSI16 启动 → 完整启动（LP_BOOT_WAKE）

### 2.3.1 切换时序

1. 软件复位 + session `PAUSED` + `LP_FLAG_FULL_WAKE` → `LP_BOOT_WAKE`。
2. 完整 `App_Init()` 同 2.2；结束时 `LpCycle_ApplyWake()`：
   - `LpSession_Load()`；要求 `FULL_WAKE`。
   - 映射 `LP_WAKE_BUTTON/CHARGER/ALERT/MONITOR` → `POWER_WAKE_*`；无映射（如纯 `LP_WAKE_FAULT`/仅 SMBA）→ `POWER_WAKE_RTC`。
   - `PowerMode_NoteLpFullWake()` 计数。
   - `Button_OnWake()`：按下则开始计时；已释放则合成 DOWN/UP 短按。
   - `ChargePolicy_OnLpWake()`：清 user discharge/charger/CAL，请求 FET 0，IDLE。
   - `LP_FLAG_FAULT`/`LP_WAKE_FAULT` → `PowerMode_ForceDisableAutoLp()`。
   - 非 MONITOR → `BlackBox_WriteWake()`。
   - 清 `FULL_WAKE`，session `PAUSED`，保存。
3. 结束于 NORM（或 DEGRADED）。

### 2.3.2 切换前后外设工作状态对照

与 2.2 表相同（完整 boot 后 NORM），差异：


| 资源               | 切换前（S1）                 | 切换后（NORM + ApplyWake）                                   |
| -------------------- | ------------------------------ | -------------------------------------------------------------- |
| 时钟/电源/外设基线 | HSI16 16 MHz                 | 80 MHz NORM；同 2.2                                          |
| 5 V/PA1/BQ         | 5 V off；PA1 高；BQ 未初始化 | 5 V on；PA1 低；BQ 运行；FET 由策略（ApplyWake 后 IDLE=0）   |
| 按钮/LED/蜂鸣器    | 未初始化                     | Button 生效；LED 由 SM；ApplyWake 可能合成短按；无额外蜂鸣器 |
| 自动 LP            | 未配置                       | 若 FAULT 标志/原因则 `lp_disable_auto=1`；否则允许           |
| 黑匣子             | 无                           | 写 wake 记录（MONITOR 除外）                                 |
| 其它外设           | S1 表                        | 同 2.2 NORM 表（I2C/USART/TIM/COMP/PVD 等）                  |

## 2.4 HSI16 启动 → 完整启动（LP_BOOT_FAULT）

### 2.4.1 切换时序

1. `ClassifyBoot` 返回 `LP_BOOT_FAULT`（HardFault/NMI、IWDG 无有效 STOPPED/interval、START 时 RTC 失败）。
2. 完整 `App_Init()` 同 2.2；结束时：
   - `PowerMode_ForceDisableAutoLp()`；
   - `LpSession_Invalidate()`；
   - `PowerMode_Init()` 若复位原因包含 WATCHDOG 也置 `lp_disable_auto=1`。
3. 结束于 NORM（或 DEGRADED）；auto-LP 本 boot 不再启动。

### 2.4.2 切换前后外设工作状态对照

与 2.2 表相同；差异：


| 资源               | 切换前（S1）         | 切换后（NORM，auto-LP off）                     |
| -------------------- | ---------------------- | ------------------------------------------------- |
| 时钟/电源/外设基线 | HSI16 16 MHz         | 80 MHz NORM；同 2.2                             |
| 5 V/BQ/日志/LED    | 5 V off；BQ 未初始化 | 5 V on；BQ 运行；日志/LED/自检正常              |
| 自动 LP            | —                   | `PowerMode_RequestLp()` 直接返回；不会自动进 LP |
| LP session         | 无效/不匹配          | 被 `LpSession_Invalidate()` 清除                |
| 其它外设           | S1                   | 同 2.2 NORM                                     |

## 2.5 完整启动 → NORM

### 2.5.1 切换时序

1. `App_Init()` 完成全部初始化（顺序见 S4/2.2）。
2. `POWER_STATE_NORM`；`SelfTest_Request()` 进入 `WAIT_BQ`；SM 初始 `BOOT`；policy `IDLE`。
3. `App_TickStart()` 最后启动 TIM2 250 Hz。
4. `LpCycle_ApplyWake()`（若有 wake）。
5. 进入 `App_Loop()`：feed → PowerMode_Loop → 按键/手势 → BQ → self-test → diag → SM → policy → auto-LP → blackbox → buzzer/LED/log。
6. 首次 BQ fresh 后 SM `BOOT → NORM`；self-test 完成后 policy 可进入 CHARGE/DISCHARGE。

### 2.5.2 切换前后外设工作状态对照


| 资源                   | 切换前（S4 结束）                     | 切换后（NORM 运行）               |
| ------------------------ | --------------------------------------- | ----------------------------------- |
| 时钟/SysTick           | 80 MHz；Scale1；4 WS；SysTick 1 ms    | 同左                              |
| GPIOA/B/C              | 已配置；5 V 高；PA1 低                | 同左；LED 由状态机                |
| EXTI/NVIC              | PVD/PA8；应用 IRQ 已配置              | 同左；TIM2/USART/I2C/PVD IRQ 运行 |
| DMA1                   | Ch4/6/7 初始化                        | 运行（日志/RGB/I2C）              |
| I2C1                   | 初始化                                | 95 kHz + DMA；BQ 运行             |
| USART1/2               | 初始化                                | 日志/RGB 运行                     |
| TIM1/TIM2              | TIM1 停止；TIM2 已启动                | TIM1 按需；TIM2 250 Hz            |
| COMP2/DAC1/CAN1/LPTIM1 | COMP2 启动；DAC/CAN 未启动；LPTIM1 关 | 同左                              |
| RTC/WUT                | RTC 1 Hz；WUT 关                      | 同左                              |
| IWDG                   | /64                                   | 同左；主循环喂狗                  |
| PVD                    | 2.8 V                                 | 同左                              |
| Flash/DBGMCU           | 4 WS；默认                            | 同左；30 min 记录                 |
| BQ                     | RUN/配置                              | RUN；500 ms 轮询；FET 按策略      |

## 2.6 完整启动 → DEGRADED

### 2.6.1 切换时序

1. `PowerMode_Init()` 调用 `PowerMode_ClkSwitchToHsePll()`。
2. 每次尝试：PA1 低 → 30 ms → HSE bypass/PLL 配置 → `HAL_RCC_ClockConfig`（Scale1/80 MHz/4 WS）。失败 → `PowerMode_ClkForceHsi16()`（HSI/Scale2/16 MHz/`HAL_InitTick`）。
3. 3 次失败 → `s_power.degraded=1`、`state=DEGRADED`、sticky `FAULT_BIT_CLK`，返回。
4. `App_Init()`：`BlackBox_Init`/`StateStore_Init` 后检查 DEGRADED → 5 V 低 → return；不执行 App_Log/BQ/SOC/WS2812/LED/Buzzer/Button/SelfTest/SM/Policy/Cal/COMP/TIM2。
5. `App_Loop()`：feed → `App_DegradedLoop()`：PB3 每 250 ms 翻转，PB4 低。

### 2.6.2 切换前后外设工作状态对照


| 资源                   | 切换前（S1/完整 boot 开头）                      | 切换后（DEGRADED）                                                                    |
| ------------------------ | -------------------------------------------------- | --------------------------------------------------------------------------------------- |
| 时钟/SysTick           | HSI16 16 MHz；Scale2；2 WS                       | HSI16 16 MHz；Scale2；Flash latency 可能 4 WS；SysTick 1 ms @16 MHz；HSE/PLL 可能残留 |
| GPIOA                  | PA1 高；PA2/PA7/PA8/PA9/10/11/12 MX 配置         | PA1 可能低（失败路径未抬）；其余 MX 残留                                              |
| GPIOB                  | PB0/1/3/4 低；PB5 input；PB6/7 AF7               | PB0 低；PB3 250 ms 闪；PB4 低；其余 MX 残留                                           |
| GPIOC                  | 时钟开                                           | 同左                                                                                  |
| EXTI/NVIC              | PA8 event；PVD（Fault_Init 已开）；DMA/TIM2 NVIC | PA8 event；PVD；TIM2 NVIC 可能开但未启动；无业务                                      |
| DMA1                   | Ch6/7                                            | 残留；无业务                                                                          |
| I2C1                   | 初始化                                           | 残留；无 BQ                                                                           |
| USART1/2               | 初始化                                           | 残留；无日志/RGB                                                                      |
| TIM1/TIM2              | TIM1 停止；TIM2 未启动                           | 残留；无蜂鸣器/250 Hz                                                                 |
| COMP2/DAC1/CAN1/LPTIM1 | 初始化未启动                                     | 残留；无业务                                                                          |
| RTC/WUT                | RTC 1 Hz；WUT 关                                 | 同左                                                                                  |
| IWDG                   | /64（App_Init 开头）                             | /64，喂狗                                                                             |
| PVD                    | 2.8 V                                            | 2.8 V                                                                                 |
| Flash/DBGMCU           | 2 WS/4 WS 不定；DBG                              | 无应用写；DBG_IWDG_STOP                                                               |
| BQ                     | 未初始化                                         | 未初始化                                                                              |

## 2.7 NORM → LP 轻量启动（LP START）

### 2.7.1 切换时序

1. `App_Loop` 计算 `s_app.auto_lp_block`；全部为 0 且空闲 ≥ `APP_IDLE_LP_MS=5000 ms` → `PowerMode_RequestLp()`；或 debugger 写 `s_power.request_lp=1`。
2. 下一次 `PowerMode_Loop()`：清 `request_lp`，调用 `LpCycle_RequestStart()`。
3. `LpCycle_RequestStart()`：
   - 等 BQ idle + I2C 不 busy，最多 100 × 1 ms，每 ms feed；超时返回 0，保持 NORM。
   - `RtcTime_Read()`；失败返回 0。
   - `LpSession_Load()`；无效/IDLE/FAULT → 清 session、`state=START`、`next_mark_epoch=now+1800 s`、LSI 32 kHz、PR=6/RLR=4095、overhead 300 ms；否则复用并置 `state=START`、清 FULL_WAKE/FAULT、wake_cause、anomaly_count、推进 next_mark。
   - `LpCycle_DisableLegacyWut()`；`seq++`；`LpSession_Save()`；feed；`NVIC_SystemReset()`。
4. 软件复位 → S1；`ClassifyBoot` 看到 SOFTWARE + START + RTC OK → `LP_BOOT_START` → S2。

### 2.7.2 切换前后外设工作状态对照


| 资源                   | 切换前（NORM）                                       | 切换后（软件复位后 S2 LP 轻量 boot）                    |
| ------------------------ | ------------------------------------------------------ | --------------------------------------------------------- |
| 时钟/SysTick           | 80 MHz；Scale1；4 WS；SysTick 1 ms                   | HSI16 16 MHz；Scale2；2 WS；SysTick 1 ms；HSE/PLL 关    |
| GPIOA                  | PA1 低；PA2 AF7；PA7 AF1；PA8 event；PA9/10/11/12 AF | PA1 高；PA2/PA9/10/11/12 释放；PA7 可能 AF1；PA8 event  |
| GPIOB                  | PB0 高；PB1 低；PB3/4 LED；PB5 input；PB6/7 AF7      | PB0/1/3/4 低；PB5 EXTI5；PB6/7 释放                     |
| GPIOC                  | 时钟开                                               | 同左                                                    |
| EXTI/NVIC              | PVD/PA8；应用 IRQ                                    | EXTI5；PA8 event；PVD/RTC/LPTIM 关；应用 IRQ 由复位清除 |
| DMA1                   | Ch4/6/7 运行                                         | 复位；轻量 boot 仅 Ch6/7 残留/释放                      |
| I2C1                   | 95 kHz + DMA                                         | sniff 100 kHz 阻塞；之后 DeInit                         |
| USART1/2               | 日志/RGB                                             | DeInit                                                  |
| TIM1/TIM2              | TIM1 按需；TIM2 250 Hz                               | DeInit/停止                                             |
| COMP2/DAC1/CAN1/LPTIM1 | COMP2 运行；DAC/CAN 未启动；LPTIM1 关                | COMP2 停止；DAC/CAN DeInit；LPTIM1 关闭                 |
| RTC/WUT                | RTC 1 Hz；WUT 关                                     | RTC 1 Hz；WUT 关；session START                         |
| IWDG                   | /64，喂狗                                            | /256 boot guard → 目标值                               |
| PVD                    | 2.8 V                                                | 未配置（轻量 boot 不跑 Fault_Init）                     |
| Flash/DBGMCU           | 4 WS；默认                                           | 2 WS；DBG_STOP 默认清；DBG_IWDG_STOP                    |
| BQ                     | RUN；ADC/CC on；FET 按策略                           | 未初始化；`BQ_LpBootPrepare` 强制 ADC/TEMP/CC/FET off   |

## 2.8 LP 轻量启动 → LP STOP1

### 2.8.1 切换时序

1. `LpCycle_Run(boot)`：
   - 校验 session/RTC；RESUME 时 `LpCycle_Calibrate()`，anomaly≥2 → fault。
   - `state=PREP`；清 FULL_WAKE/FAULT；`PowerMode_LpDbgStopApply()`。
   - 5 V off、PB3/PB4 low、PA1 high、`HAL_COMP_Stop`。
   - `LpCycle_StopUnusedPeripherals()`：`HAL_UART_DeInit(huart1/2)`、`HAL_CAN_DeInit`、`HAL_DAC_DeInit`、`HAL_TIM_PWM_DeInit(htim1)`、`HAL_TIM_Base_DeInit(htim2)`；LPTIM1 `ENABLE=0`、NVIC 关、EXTI32 清。
   - `Button_WakeInit()`：PB5 IT falling/pull-up、SYSCFG、FTSR/IMR、清 PR5、NVIC EXTI9_5 0。
   - `LpCycle_DisableLegacyWut()`；feed；预检按键。
   - 可选 SMBA 测试：`pb5_mode=1`、`s_smba_test_active=1`（首次）；否则沿用。
   - `LpCycle_BqSniffOnce()`：`BQ_LpSniffInit`（HSI16 100 kHz）→ `BQ_LpBootPrepare`（强制 ADC/TEMP/CC/FET off、300 ms 等）→ `BQ_LpSniffDeInit`；若 `LOAD_PRESENT && CHG_ON=0` → `LpCycle_CheckCharger()`（PA1 低 30 ms + COMP 30 ms）。
   - BQ prepare 后按 post-clear SYS_STAT 的真实保护位检查 ALERT；PB4 10 ms 心跳；`RtcTime_Read` + `UpdateBootOverhead`；30 min 到点 → `LpCycle_DoMark()`（BQ monitor + BlackBox）。
   - 计算 `target_ms`：`min(remaining_to_mark, 30000) - boot_overhead`，最小 1000 ms。
   - SMBA active → `Button_SmbaEnable()`（PB5 AF4、I2C1 HSI16 SMBA）。
2. `LpCycle_EnterStop(target_ms)`：
   - `Watchdog_SetTimeoutMs(target_ms, lsi_hz)`；失败 → full wake。
   - `RtcTime_Read`；失败 → full wake。
   - 最终按键检查；PA8 高时读 SYS_STAT 确认，只有真实保护位才唤醒；保存 `last_pr/last_rlr/last_arm_epoch/ms`、`state=STOPPED`。
   - 使能 PWR/RTCAPB APB+SMEN；SMBA 时 I2C1；GPIOA/B/SYSCFG sleep。
   - `__enable_irq()`、feed、`HAL_SuspendTick()`、清 PendST。
   - `SET_BIT(RCC->CFGR, STOPWUCK)`；`PWR_CR1.LPMS=STOP1`；`SLEEPDEEP=1`；`__DSB`；`__WFI`。

### 2.8.2 切换前后外设工作状态对照


| 资源                   | 切换前（S2 LP 轻量 boot）                              | 切换后（S3 STOP1）                                                 |
| ------------------------ | -------------------------------------------------------- | -------------------------------------------------------------------- |
| 时钟/SysTick           | HSI16 16 MHz；Scale2；2 WS；SysTick 1 ms               | CPU/总线停；STOPWUCK=HSI16；RTC 默认 LSI/LSE 宏 运行；SysTick 挂起 |
| GPIOA                  | PA1 高；PA2/PA9/10/11/12 释放；PA7 可能 AF1；PA8 event | 保持；PA1 高；PA8 event；GPIOA 睡眠时钟保持                        |
| GPIOB                  | PB0/1/3/4 低；PB5 EXTI5；PB6/7 释放                    | 保持；PB5 EXTI5；GPIOB 睡眠时钟保持                                |
| GPIOC                  | 时钟开                                                 | 未保持 GPIOC 睡眠时钟；LSE 仍由 RCC 控制                           |
| EXTI/NVIC              | EXTI5；PA8 event；PVD/RTC/LPTIM 关                     | EXTI5 使能；PA8 event；EXTI20/32 关闭；NVIC EXTI9_5                |
| DMA1                   | Ch6/7 释放，NVIC 可能使能                              | 时钟保持，通道关闭；无 DMA                                         |
| I2C1                   | sniff 后 DeInit；SMBA 时可能启用                       | 生产 DeInit；SMBA 时 HSI16 + WUPEN/ALERTEN + I2C1_ER               |
| USART1/2               | DeInit                                                 | 保持 DeInit；无日志/RGB                                            |
| TIM1/TIM2              | DeInit/停止                                            | 保持停止；TIM1 时钟可能残留、PA7 AF1                               |
| COMP2/DAC1/CAN1/LPTIM1 | COMP2 停止；DAC/CAN DeInit；LPTIM1 关                  | 同左；COMP2/DAC/CAN 无；LPTIM1 屏蔽                                |
| RTC/WUT                | RTC 1 Hz；WUT 关                                       | RTC 1 Hz；WUT 关；RTCAPB 保持                                      |
| IWDG                   | /256 → 目标值                                         | 运行，目标值；不喂                                                 |
| PVD                    | 未配置                                                 | 未配置                                                             |
| Flash/DBGMCU           | 2 WS；DBG_STOP 默认清                                  | 无访问；DBG_STOP 默认清；DBG_IWDG_STOP                             |
| BQ                     | 未初始化；ADC/TEMP/CC/FET off                          | 全部 off；无轮询；保护锁存保留                                     |

## 2.9 LP STOP1 → LP 轻量启动（有效 IWDG 周期复位）

### 2.9.1 切换时序

1. STOP1 中 IWDG 到 RLR → MCU 系统复位；无代码在 WFI 后继续。
2. S0/S1：`Watchdog_Init()` /256；`HAL_Init`；`SystemClock_Config` HSI16；MX 初始化；`App_Boot`。
3. `ResetCause_Init` 读 `IWDGRSTF` → `RESET_CAUSE_WATCHDOG`；BKP session `STOPPED`；RTC 可读。
4. `LpCycle_ClassifyBoot`：
   - `expected = (last_rlr+1)*(4<<last_pr)*1000/lsi_hz`；
   - `elapsed = RTC now - last_arm`；
   - expected <5000 ms → elapsed 0..10000 ms 接受；否则 elapsed 75%..125% 接受；
   - 合法 → `LP_BOOT_RESUME`；否则 `LP_BOOT_FAULT`。
5. `LpCycle_Run(RESUME)`：
   - `LpCycle_Calibrate()`：`span = elapsed - boot_overhead`，sample = (RLR+1)*div*1000/span，接受 26–34 kHz，EMA 7/8；anomaly≥2 → fault。
   - 其余同 2.8：PREP → 清理 → sniff → target → STOP1。

### 2.9.2 切换前后外设工作状态对照


| 资源                   | 切换前（S3 STOP1）                            | 切换后（S2 LP 轻量 RESUME）                                                     |
| ------------------------ | ----------------------------------------------- | --------------------------------------------------------------------------------- |
| 时钟/SysTick           | CPU/总线停；RTC 默认 LSI/LSE 宏；SysTick 挂起 | HSI16 16 MHz；Scale2；2 WS；SysTick 1 ms；RTC 默认 LSI/LSE 宏                   |
| GPIOA/B/C              | 保持 STOP 状态；PB5 EXTI5；PA8 event          | PA1 高；PB0/1/3/4 低；PB5 EXTI5；PA8 event；PA2/PA9/10/11/12 释放；PA7 可能 AF1 |
| EXTI/NVIC              | EXTI5；PA8 event；EXTI20/32 关                | 同左；应用 IRQ 仍复位默认；EXTI5                                                |
| DMA1                   | 时钟保持，通道关                              | DMA1 时钟开；Ch6/7 释放/可能残留；无业务                                        |
| I2C1                   | 生产 DeInit；SMBA 可能启用                    | sniff 100 kHz 阻塞；之后 DeInit                                                 |
| USART1/2               | DeInit                                        | DeInit（无日志/RGB）                                                            |
| TIM1/TIM2              | 停止                                          | DeInit/停止；TIM1/PA7 可能残留                                                  |
| COMP2/DAC1/CAN1/LPTIM1 | COMP2 停；DAC/CAN DeInit；LPTIM1 关           | COMP2 停；DAC/CAN DeInit；LPTIM1 关                                             |
| RTC/WUT                | RTC 1 Hz；WUT 关                              | RTC 1 Hz；WUT 关；session STOPPED→PREP                                         |
| IWDG                   | 运行，超时复位                                | /256 boot guard → 目标值                                                       |
| PVD                    | 未配置                                        | 未配置                                                                          |
| Flash/DBGMCU           | 无访问；DBG_STOP 默认清                       | 2 WS；30 min 时写；DBG_STOP 默认清                                              |
| BQ                     | 全关                                          | 未初始化；sniff 强制 off                                                        |

## 2.10 LP STOP1 → 完整启动（PB5 / SMBA 提前唤醒）

### 2.10.1 切换时序

1. STOP1 中 PB5 下降沿 → EXTI5 IRQ 优先级 0 → `HAL_GPIO_EXTI_IRQHandler(PB5)` → `HAL_GPIO_EXTI_Callback` 置 `s_button.exti_flag`；WFI 返回。
   - SMBA 测试时，I2C1 ALERT/ER 也可唤醒；`I2C1_ER_IRQHandler` 检查 `I2C_ISR_ALERT`，清 ALERTCF，`Button_NotifySmbaFromIsr()`。
2. WFI 返回后：
   - 清 `SLEEPDEEP`；`SystemCoreClock=16 MHz`；`HAL_InitTick()`；feed。
   - 若 SMBA active：`Button_SmbaTakeAlert()` / `LpCycle_IsButtonAsserted()` 分支；结果 `smba_result=1/3/2`；`Button_SmbaDisable()`。
   - 否则：`LpCycle_IsButtonAsserted()` → `LP_WAKE_BUTTON`；`LpCycle_IsAlertAsserted()` → `LP_WAKE_ALERT`；否则 `LP_WAKE_FAULT`。
3. `LpCycle_ForceFullWake(cause)`：`state=PAUSED`、`wake_cause=cause`、`FULL_WAKE`（FAULT 时加 `FAULT`）、保存、feed、`NVIC_SystemReset()`。
4. 下一次 boot：SOFTWARE + PAUSED + FULL_WAKE → `LP_BOOT_WAKE` → S4 完整 App_Init + `LpCycle_ApplyWake`。

> **PA8 不是 STOP1 唤醒源**：PA8 是 event-rising，无 IMR/NVIC；只能在 PB5/SMBA/未知唤醒返回后读电平，或在轻量 boot 进 STOP 前读电平。若 PA8 在 STOP 中变高，最早在下一个 IWDG 周期复位后的轻量 boot 才被检测到。

### 2.10.2 切换前后外设工作状态对照


| 资源                   | 切换前（S3 STOP1）                                | 切换后（S4 完整 boot → NORM）                       |
| ------------------------ | --------------------------------------------------- | ------------------------------------------------------ |
| 时钟/SysTick           | CPU/总线停；RTC 默认 LSI/LSE 宏                   | HSI16 → PLL 80 MHz；Scale1；4 WS；SysTick 1 ms      |
| GPIOA                  | PA1 高；PA2/PA9/10/11/12 释放；PA7 AF1；PA8 event | PA1 低；PA2 AF7；PA7 AF1；PA8 event；PA9/10/11/12 AF |
| GPIOB                  | PB0/1/3/4 低；PB5 EXTI5                           | PB0 高；PB1 低；PB3/4 LED；PB5 input；PB6/7 AF7      |
| GPIOC                  | 未保持                                            | 时钟开；LSE                                          |
| EXTI/NVIC              | EXTI5；PA8 event；EXTI20/32 关                    | PVD；PA8 event；PB5 无 EXTI；应用 IRQ 配置           |
| DMA1                   | 通道关                                            | Ch4/6/7 运行                                         |
| I2C1                   | 生产 DeInit；SMBA 可能                            | 95 kHz + DMA；BQ                                     |
| USART1/2               | DeInit                                            | 115200 DMA；3.8095 MBd IT                            |
| TIM1/TIM2              | 停止                                              | TIM1 按需；TIM2 250 Hz                               |
| COMP2/DAC1/CAN1/LPTIM1 | 停/DeInit/关                                      | COMP2 运行；DAC/CAN 未启动；LPTIM1 关                |
| RTC/WUT                | RTC 1 Hz；WUT 关                                  | RTC 1 Hz；WUT 关                                     |
| IWDG                   | 运行，复位                                        | /64 ≈8.2 s，喂狗                                    |
| PVD                    | 未配置                                            | 2.8 V IRQ                                            |
| Flash/DBGMCU           | 无访问；DBG                                       | 4 WS；DBG_IWDG_STOP；DBG_STOP 默认清                 |
| BQ                     | 全关                                              | RUN；ADC/TEMP/CC；FET 按策略                         |

## 2.11 LP 轻量启动 → 完整启动（未进入 STOP 的中止/故障）

### 2.11.1 切换时序

在 `LpCycle_Run()` 的任何阶段都可能调用 `LpCycle_ForceFullWake()`；它不等待 STOP，而是直接写 session `PAUSED + FULL_WAKE` 后软件复位：


| 触发点                              | 原因                               | 结果                    |
| ------------------------------------- | ------------------------------------ | ------------------------- |
| `LpCycle_Run` 入口 session/RTC 无效 | `LP_WAKE_FAULT`                    | 完整 boot；auto-LP 关   |
| RESUME 校准 anomaly≥2              | `LP_WAKE_FAULT`                    | 完整 boot；auto-LP 关   |
| 预检 PB5 按下                       | `LP_WAKE_BUTTON`                   | 完整 boot；button wake  |
| SMBA 测试未知/失败                  | `LP_WAKE_FAULT` 或 `BUTTON`/`SMBA` | 完整 boot               |
| BQ sniff 连续失败                   | `LP_WAKE_FAULT`                    | 完整 boot；auto-LP 关   |
| BQ sniff 判充电                     | `LP_WAKE_CHARGER`                  | 完整 boot；charger wake |
| PA8 高                              | `LP_WAKE_ALERT`                    | 完整 boot；ALERT wake   |
| 30 min 监测/blackbox 失败           | `LP_WAKE_MONITOR`/`FAULT`          | 完整 boot；auto-LP 关   |
| `Watchdog_SetTimeoutMs` 失败        | `LP_WAKE_FAULT`                    | 完整 boot               |
| `RtcTime_Read` 失败                 | `LP_WAKE_FAULT`                    | 完整 boot               |
| 最终 PB5/PA8 检查                   | `BUTTON`/`ALERT`                   | 完整 boot               |

时序：

1. `LpCycle_ForceFullWake(cause)`：`s_lp.state=PAUSED`；`wake_cause=cause`；`flags |= FULL_WAKE`（FAULT 时加 `FAULT`）；`LpSession_Save()`；`Watchdog_Feed()`；`NVIC_SystemReset()`。
2. 软件复位 → S1 → `ClassifyBoot` 看到 SOFTWARE + PAUSED + FULL_WAKE → `LP_BOOT_WAKE`。
3. 完整 `App_Init()`；`LpCycle_ApplyWake()` 按 cause 映射；`LP_FLAG_FAULT`/`LP_WAKE_FAULT` → auto-LP 关。
4. 结束于 NORM（可能 DEGRADED）。

### 2.11.2 切换前后外设工作状态对照


| 资源                   | 切换前（S2 LP light）                                  | 切换后（S4 完整 boot → NORM）                       |
| ------------------------ | -------------------------------------------------------- | ------------------------------------------------------ |
| 时钟/SysTick           | HSI16 16 MHz；Scale2；2 WS；SysTick 1 ms               | HSI16 → PLL 80 MHz；Scale1；4 WS；SysTick 1 ms      |
| GPIOA                  | PA1 高；PA2/PA9/10/11/12 释放；PA7 可能 AF1；PA8 event | PA1 低；PA2 AF7；PA7 AF1；PA8 event；PA9/10/11/12 AF |
| GPIOB                  | PB0/1/3/4 低；PB5 EXTI5                                | PB0 高；PB1 低；PB3/4 LED；PB5 input；PB6/7 AF7      |
| GPIOC                  | 时钟开                                                 | 同左；LSE                                            |
| EXTI/NVIC              | EXTI5；PA8 event                                       | PVD；PA8 event；PB5 无 EXTI；应用 IRQ                |
| DMA1                   | Ch6/7 释放，NVIC 可能                                  | Ch4/6/7 运行                                         |
| I2C1                   | sniff/DeInit                                           | 95 kHz + DMA；BQ                                     |
| USART1/2               | DeInit                                                 | 日志/RGB                                             |
| TIM1/TIM2              | 停止                                                   | TIM1 按需；TIM2 250 Hz                               |
| COMP2/DAC1/CAN1/LPTIM1 | 停/DeInit/关                                           | COMP2 运行；DAC/CAN 未启动；LPTIM1 关                |
| RTC/WUT                | RTC 1 Hz；WUT 关                                       | RTC 1 Hz；WUT 关                                     |
| IWDG                   | /256→目标                                             | /64，喂狗                                            |
| PVD                    | 未配置                                                 | 2.8 V                                                |
| Flash/DBGMCU           | 2 WS；DBG                                              | 4 WS；DBG_IWDG_STOP                                  |
| BQ                     | 未初始化；off                                          | RUN；ADC/CC/FET 按策略                               |

## 2.12 IWDG 超时复位但不满足 LP 有效 tick → 完整启动（LP_BOOT_FAULT）

### 2.12.1 切换时序

1. NORM/轻量路径/STOP 中 IWDG 超时 → 系统复位。
2. S1：`ResetCause_Init` 置 `RESET_CAUSE_WATCHDOG`；session 可能为 `STOPPED`、`PREP`、`IDLE` 或无有效 magic/CRC。
3. `ClassifyBoot`：
   - session 无效 → `LP_BOOT_FAULT`；
   - session `STOPPED` 但 RTC 失败/interval 不在范围 → `LP_BOOT_FAULT`；
   - session 其它状态 → `LP_BOOT_FAULT`。
4. 完整 `App_Init()`；`PowerMode_ForceDisableAutoLp()`（main_app 的 fault 分支）+ `PowerMode_Init` 的 WATCHDOG 检查；session invalidate。
5. 结束于 NORM/ DEGRADED；auto-LP 关。

### 2.12.2 切换前后外设工作状态对照


| 资源                   | 切换前（故障/NORM/STOP） | 切换后（完整 boot，auto-LP off）            |
| ------------------------ | -------------------------- | --------------------------------------------- |
| 时钟/SysTick           | 任意源状态               | HSI16 → 80 MHz；Scale1；4 WS；SysTick 1 ms |
| GPIOA/B/C              | 现场                     | NORM 配置；PA1 低、PB0 高、LED              |
| EXTI/NVIC              | 现场                     | PVD/PA8；应用 IRQ；PB5 无 EXTI              |
| DMA1                   | 现场                     | Ch4/6/7                                     |
| I2C1                   | 现场                     | 95 kHz + DMA                                |
| USART1/2               | 现场                     | 日志/RGB                                    |
| TIM1/TIM2              | 现场                     | TIM1 按需；TIM2 250 Hz                      |
| COMP2/DAC1/CAN1/LPTIM1 | 现场                     | COMP2 运行；DAC/CAN 未启动；LPTIM1 关       |
| RTC/WUT                | 现场                     | RTC 1 Hz；WUT 关                            |
| IWDG                   | 复位前                   | /64，喂狗                                   |
| PVD                    | 现场                     | 2.8 V                                       |
| Flash/DBGMCU           | 现场                     | 4 WS；DBG_IWDG_STOP                         |
| BQ                     | 现场                     | RUN/ADC/CC；FET 策略                        |

## 2.13 任意状态 → 异常复位（HardFault/NMI/Error_Handler）→ 完整启动（LP_BOOT_FAULT）

### 2.13.1 切换时序

1. HardFault/MemManage/BusFault/UsageFault → `ResetCause_RecordHardFaultAndReset()`；NMI → `ResetCause_RecordNmiAndReset()`。
2. `ResetCause_EnableBackupAccess()`（PWR/RTCAPB/DBP）→ BKP0 = magic|cause；HardFault 还写 BKP20–23 = CFSR/HFSR/MMFAR/BFAR；`NVIC_SystemReset()`。
3. `Error_Handler()`：`__disable_irq()` + 无限循环；IWDG 最终复位（若调试器未冻结）。
4. 下次 boot：`ResetCause_Init` 读 BKP0 + RCC_CSR；`ClassifyBoot` 优先识别 HARDFAULT/NMI → invalidate session → `LP_BOOT_FAULT`。
5. 完整 `App_Init()`；auto-LP 关；session invalid。

### 2.13.2 切换前后外设工作状态对照


| 资源                   | 切换前（异常现场）   | 切换后（完整 boot，LP_BOOT_FAULT）          |
| ------------------------ | ---------------------- | --------------------------------------------- |
| 时钟/SysTick           | 故障现场时钟         | HSI16 → 80 MHz；Scale1；4 WS；SysTick 1 ms |
| GPIOA/B/C              | 故障现场             | NORM 配置；PA1 低、PB0 高、LED              |
| EXTI/NVIC              | 故障现场（可能部分） | PVD/PA8；应用 IRQ；PB5 无 EXTI              |
| DMA1                   | 故障现场             | Ch4/6/7                                     |
| I2C1                   | 故障现场             | 95 kHz + DMA                                |
| USART1/2               | 故障现场             | 日志/RGB                                    |
| TIM1/TIM2              | 故障现场             | TIM1 按需；TIM2 250 Hz                      |
| COMP2/DAC1/CAN1/LPTIM1 | 故障现场             | COMP2 运行；DAC/CAN 未启动；LPTIM1 关       |
| RTC/WUT                | 故障现场；BKP 写入   | RTC 1 Hz；WUT 关；session invalid           |
| IWDG                   | 继续运行/复位        | /64                                         |
| PVD                    | 故障现场             | 2.8 V                                       |
| Flash/DBGMCU           | 故障现场；BKP 写     | 4 WS；DBG_IWDG_STOP                         |
| BQ                     | 故障现场             | RUN/ADC/CC；FET 策略                        |

## 2.14 NORM 内部逻辑状态切换

### 2.14.1 切换时序

这些切换不改变 S5 的时钟/电源/外设基线，只改变逻辑状态/控制输出：

**SM**


| 从        | 触发                       | 到    | 影响                           |
| ----------- | ---------------------------- | ------- | -------------------------------- |
| BOOT      | BQ fresh                   | NORM  | LED NORM；flow 按充电/放电     |
| BOOT/NORM | critical fault             | FAULT | LED FAULT；policy FAULT；FET 0 |
| FAULT     | critical clear + fresh     | NORM  | LED NORM；policy 重新评估      |
| FAULT     | critical clear + not fresh | BOOT  | LED BOOT；policy IDLE          |

**Self-test**


| 从            | 触发                               | 到            | 影响                      |
| --------------- | ------------------------------------ | --------------- | --------------------------- |
| IDLE          | Request                            | WAIT_BQ       | auto-LP block             |
| WAIT_BQ       | fresh/3 s，健康/静态失败/ADC latch | MONITOR       | PASS/FAIL 事件            |
| WAIT_BQ       | 仅 BQ 动态失败                     | WAIT_BQ_GRACE | 12 s grace/30 s hard      |
| WAIT_BQ_GRACE | healthy/12 s/30 s/static fail      | MONITOR       | PASS/FAIL                 |
| MONITOR       | 5 s unhealthy/3 s healthy          | MONITOR       | FAIL/RECOVERED 一次性事件 |

**Charge policy**


| 从        | 触发                           | 到         | FET        |
| ----------- | -------------------------------- | ------------ | ------------ |
| IDLE      | NormalReady + charger          | CHARGE     | 0x03       |
| IDLE      | NormalReady + user discharge   | DISCHARGE  | 0x03       |
| CHARGE    | charger absent 1 s / not ready | IDLE       | 0x00       |
| DISCHARGE | PAIR_HOLD3 toggle / not ready  | IDLE       | 0x00       |
| IDLE      | cal_mode + CalReady            | CAL        | debug mask |
| CAL       | USER/FAULT/NOTREADY/LP         | IDLE/FAULT | 0x00       |
| any       | SM FAULT                       | FAULT      | 0x00       |

**BQ**


| 从        | 触发                  | 到                      |
| ----------- | ----------------------- | ------------------------- |
| RESET     | probe ACK/NACK        | BOOT_WAIT/BOOT_PULSE    |
| BOOT_WAIT | 20 ms                 | CONFIG                  |
| CONFIG    | script/verify OK      | SETTLE                  |
| CONFIG    | fail                  | BACKOFF                 |
| SETTLE    | 300 ms + verify OK    | RUN                     |
| SETTLE    | fail                  | BACKOFF                 |
| RUN       | transfer fail/restart | BACKOFF/ADC_RESTART_OFF |
| BACKOFF   | 20 ms/2 s             | prior state/CONFIG      |

### 2.14.2 切换前后外设工作状态对照


| 资源                   | 切换前                                | 切换后                                      |
| ------------------------ | --------------------------------------- | --------------------------------------------- |
| 时钟/SysTick           | NORM 80 MHz                           | 不变                                        |
| GPIOA/B/C              | NORM 配置                             | 不变；LED/PA7 蜂鸣器按逻辑变化              |
| EXTI/NVIC              | PVD/PA8；应用 IRQ                     | 不变                                        |
| DMA1                   | Ch4/6/7                               | 不变；BQ 事务时 Ch6/7                       |
| I2C1                   | 95 kHz + DMA                          | 不变；BQ 事务                               |
| USART1/2               | 日志/RGB                              | 不变；日志/颜色变化                         |
| TIM1/TIM2              | TIM1 按需；TIM2 250 Hz                | 不变；蜂鸣器按逻辑                          |
| COMP2/DAC1/CAN1/LPTIM1 | COMP2 运行；DAC/CAN 未启动；LPTIM1 关 | 不变                                        |
| RTC/WUT                | RTC 1 Hz；WUT 关                      | 不变                                        |
| IWDG                   | /64                                   | 不变                                        |
| PVD                    | 2.8 V                                 | 不变                                        |
| Flash/DBGMCU           | 4 WS                                  | 不变；BQ 故障可写黑匣子                     |
| BQ                     | RUN/配置                              | FET/控制字/状态机变化；ADC/TEMP/CC 保持运行 |

---

# 附录

## A. 状态 × 时钟总表


| 状态          | SYSCLK/总线   | VOS/WS                 | HSE/PLL                      | HSI/MSI                 | RTC（默认 LSI）/WUT         | LSI/IWDG      | SysTick | TIM1/TIM2                     | USART/I2C/LPTIM                                        | STOPWUCK/LPMS         |
| --------------- | --------------- | ------------------------ | ------------------------------ | ------------------------- | ----------------------------- | --------------- | --------- | ------------------------------- | -------------------------------------------------------- | ----------------------- |
| S0 复位       | MSI 4 MHz     | 复位/0                 | 关                           | HSI 关/MSI 开           | POR 关；非 POR 可能保持     | 未配置        | 未启动  | 未配置                        | 未配置                                                 | 复位                  |
| S1 HSI16 启动 | HSI16 16 MHz  | Scale2/2 WS            | 关                           | HSI 开/MSI 开           | RTC 默认 LSI 1 Hz/WUT 关    | /256 ≈32.8 s | 1 ms    | 16 MHz，未启动                | 16 MHz，无业务                                         | 未配置                |
| S2 LP light   | HSI16 16 MHz  | Scale2/2 WS            | 关                           | HSI 开/MSI 开           | RTC 默认 LSI 1 Hz/WUT 关    | /256→目标    | 1 ms    | 16 MHz，停止                  | I2C sniff 100 kHz；USART 关；LPTIM 关                  | 进 STOP 前设 HSI16    |
| S3 LP STOP1   | 停            | Scale2                 | 关                           | HSI 停（退出开）/MSI 停 | RTC 默认 LSI 1 Hz/WUT 关    | 动态          | 挂起    | 停止                          | I2C 停/SMBA 可选；USART 停；LPTIM 关                   | STOPWUCK=1/LPMS=STOP1 |
| S4 完整 boot  | HSI16→80 MHz | Scale2→Scale1/2→4 WS | HSE bypass 12 MHz/PLL 80 MHz | HSI 开/MSI 开           | RTC 默认 LSI 1 Hz/WUT 关    | /256→/64     | 1 ms    | 80 MHz；TIM1 按需/TIM2 250 Hz | 80 MHz 配置                                            | 未配置                |
| S5 NORM       | 80 MHz        | Scale1/4 WS            | HSE/PLL 80 MHz               | HSI/MSI 开              | RTC（默认 LSI） 1 Hz/WUT 关 | /64 ≈8.2 s   | 1 ms    | TIM1 按需；TIM2 250 Hz        | I2C 95 kHz；USART1 115200；USART2 3.8095 MBd；LPTIM 关 | 未配置                |
| S6 DEGRADED   | HSI16 16 MHz  | Scale2/2 或 4 WS       | 可能残留                     | HSI/MSI 开              | RTC（默认 LSI） 1 Hz/WUT 关 | /64 喂        | 1 ms    | 16 MHz，停止                  | 16 MHz，无业务                                         | 未配置                |
| S7 异常复位   | 故障现场      | 现场                   | 现场                         | 现场                    | 现场/BKP                    | 继续          | 现场    | 现场                          | 现场                                                   | 现场                  |

## B. 状态 × 外设总表

### B1 接口/定时器


| 状态 | I2C1                       | USART1                   | USART2                   | CAN1             | DMA1/NVIC             | TIM1             | TIM2             |
| ------ | ---------------------------- | -------------------------- | -------------------------- | ------------------ | ----------------------- | ------------------ | ------------------ |
| S0   | 未配置                     | 未配置                   | 未配置                   | 未配置           | 未配置                | 未配置           | 未配置           |
| S1   | 已初始化，16 MHz ~19 kHz   | 已初始化，115200 @16 MHz | 已初始化，115200 @16 MHz | 已初始化，未启动 | Ch6/7；TIM2 NVIC      | 已初始化，未启动 | 已初始化，未启动 |
| S2   | sniff 100 kHz，之后 DeInit | DeInit                   | DeInit                   | DeInit           | Ch6/7 释放；NVIC 可能 | DeInit/停止      | DeInit/停止      |
| S3   | 生产 DeInit；SMBA 时 HSI16 | 停                       | 停                       | 停               | 通道关                | 停               | 停               |
| S4   | 95 kHz + DMA               | 115200 + DMA             | 3.8095 MBd + IT          | 已初始化，未启动 | Ch4/6/7               | 按需             | 250 Hz           |
| S5   | 95 kHz + DMA               | 115200 + DMA             | 3.8095 MBd + IT          | 未启动           | Ch4/6/7               | 按需             | 250 Hz           |
| S6   | MX 残留                    | MX 残留                  | MX 残留                  | MX 残留          | 残留                  | MX 残留          | 未启动           |
| S7   | 现场                       | 现场                     | 现场                     | 现场             | 现场                  | 现场             | 现场             |

### B2 模拟/系统


| 状态 | COMP2              | DAC1             | LPTIM1           | RTC/WUT                | IWDG       | PVD    | SysTick/DBG           |
| ------ | -------------------- | ------------------ | ------------------ | ------------------------ | ------------ | -------- | ----------------------- |
| S0   | 未配置             | 未配置           | 未配置           | POR 未配置/非 POR 保持 | 未配置     | 未配置 | 未配置                |
| S1   | 已初始化，未启动   | 已初始化，未启动 | 已初始化，未启动 | RTC 1 Hz/WUT 未布防    | /256       | 未配置 | 1 ms；DBG_IWDG_STOP   |
| S2   | 停；充电检查瞬时启 | DeInit           | 关闭/屏蔽        | RTC 1 Hz/WUT 关        | /256→目标 | 未配置 | 1 ms；DBG_STOP 默认清 |
| S3   | 停                 | DeInit           | 关闭/屏蔽        | RTC 1 Hz/WUT 关        | 动态       | 未配置 | 挂起；DBG_STOP 默认清 |
| S4   | 运行               | 已初始化，未启动 | 关闭             | RTC 1 Hz/WUT 关        | /64        | 2.8 V  | 1 ms；DBG_IWDG_STOP   |
| S5   | 运行               | 未启动           | 关闭             | RTC 1 Hz/WUT 关        | /64        | 2.8 V  | 1 ms                  |
| S6   | MX 残留            | MX 残留          | 未启动           | RTC 1 Hz/WUT 关        | /64 喂     | 2.8 V  | 1 ms                  |
| S7   | 现场               | 现场             | 现场             | 现场                   | 继续       | 现场   | 现场                  |

### B3 板级/外部


| 状态 | PA1                | PB0/5V  | PB1     | PB3/PB4        | PA7      | PB5           | PA8          | BQ/FET              | 备份/Flash         |
| ------ | -------------------- | --------- | --------- | ---------------- | ---------- | --------------- | -------------- | --------------------- | -------------------- |
| S0   | 高阻（外部上拉高） | 高阻/关 | 高阻/低 | 高阻           | 高阻     | 高阻          | 高阻         | 未配置/未知         | POR 清/非 POR 保持 |
| S1   | 先低后高           | 低/关   | 低      | 低             | AF1      | input pull-up | event rising | 未初始化            | 保持               |
| S2   | 高（充电窗口低）   | 低/关   | 低      | 低（PB4 心跳） | 可能 AF1 | EXTI5         | event rising | ADC/TEMP/CC/FET off | 30 min 写          |
| S3   | 高                 | 低/关   | 低      | 低             | 可能 AF1 | EXTI5         | event rising | 全关                | 保持               |
| S4   | 低                 | 高/开   | 低/脉冲 | LED            | AF1      | input pull-up | event rising | RUN/配置            | 可写               |
| S5   | 低                 | 高/开   | 低      | LED            | AF1      | input pull-up | event rising | RUN/策略            | 30 min             |
| S6   | 可能低             | 低/关   | 低      | PB3 闪/PB4 低  | AF1      | input pull-up | event rising | 未配置              | 无写               |
| S7   | 现场               | 现场    | 现场    | 现场           | 现场     | 现场          | 现场         | 现场                | BKP 写             |

## C. 复位/启动分类真值表


| 优先级 | 复位原因/条件                                  | session         | `LP_BOOT_*`                  | session 动作  | 下一路径                               |
| -------- | ------------------------------------------------ | ----------------- | ------------------------------ | --------------- | ---------------------------------------- |
| 1      | HardFault/NMI                                  | 任意            | FAULT                        | invalidate    | 完整 boot，auto-LP off                 |
| 2      | 时基未就绪（`PowerMode_LpTimebaseReady()==0`） | 任意            | WATCHDOG→FAULT，否则 NORMAL | invalidate    | 完整 boot；LSE 模式回退 LSI 时 LP 禁用 |
| 3      | SOFTWARE + START                               | RTC OK          | START                        | 保持 START    | LP light                               |
| 4      | SOFTWARE + START                               | RTC fail        | FAULT                        | invalidate    | 完整 boot，auto-LP off                 |
| 5      | SOFTWARE + PAUSED + FULL_WAKE                  | 含 FULL_WAKE    | WAKE                         | 保持          | 完整 boot + ApplyWake                  |
| 6      | WATCHDOG + STOPPED + RTC OK + 间隔合法         | 有效            | RESUME                       | STOPPED→PREP | LP light                               |
| 7      | WATCHDOG + STOPPED + 间隔不合法/RTC fail       | 有效但无效 tick | FAULT                        | invalidate    | 完整 boot，auto-LP off                 |
| 8      | PIN/BOR（无上面的合法 SW/WDG tick）            | 任意            | NORMAL                       | invalidate    | 完整 boot                              |
| 9      | WATCHDOG 其它/无 session                       | 任意            | FAULT                        | invalidate    | 完整 boot，auto-LP off                 |
| 10     | 其它/无                                        | —              | NORMAL                       | 保持/无效     | 完整 boot                              |

> 该板在软件复位/IWDG 复位时可能同时报告 PIN。合法的 SOFTWARE START/WAKE 或 WATCHDOG+STOPPED 间隔必须先于 PIN/BOR 判断，否则会把合法 LP tick 当冷启动并清会话（历史 bug：NORM→reset→NORM 每 5 s 循环、LP 首个 IWDG 周期后自动退出）。

## D. 关键寄存器/写入索引


| 阶段    | 寄存器/外设                | 值/位                                              | 条件/时序                                    |
| --------- | ---------------------------- | ---------------------------------------------------- | ---------------------------------------------- |
| 启动    | `IWDG->KR`                 | `0xCCCC`、`0x5555`、`0xAAAA`                       | `Watchdog_Init()` /256；`Watchdog_Program()` |
| 启动    | `IWDG->PR/RLR/WINR`        | PR=6/RLR=4095；NORM PR=4                           | boot guard/NORM/动态 LP                      |
| HSI16   | `RCC->CR`/`CFGR`/`PLLCFGR` | HSI on；SW=HSI；PLL off                            | `SystemClock_Config()`                       |
| HSI16   | `PWR->CR1.VOS`             | Scale2                                             | `HAL_PWREx_ControlVoltageScaling`            |
| HSI16   | `FLASH->ACR`               | 2 WS                                               | `FLASH_LATENCY_2`                            |
| HSI16   | `RCC->BDCR`                | RTCSEL=LSI/LSE 按宏；LSEON 仅 LSE 模式；LSECSSON=0 | `RtcTime_MspClockSource()`                   |
| NORM    | `RCC->CR/CFGR/PLLCFGR`     | HSEBYP；PLLM=3/N=40/R=2；SW=PLL                    | `PowerMode_ClkSwitchToHsePll()`              |
| NORM    | `PWR->CR1.VOS`             | Scale1                                             | 同上                                         |
| NORM    | `FLASH->ACR`               | 4 WS                                               | 同上                                         |
| NORM    | `RCC->APB1ENR1/SMENR1`     | PWREN/RTCAPBEN；PWRSMEN/RTCAPBSMEN                 | `PowerMode_Init()`                           |
| STOP    | `RCC->CFGR`                | `STOPWUCK=1`                                       | `LpCycle_EnterStop()`                        |
| STOP    | `PWR->CR1`                 | `LPMS=STOP1`                                       | 同上                                         |
| STOP    | `SCB->SCR`                 | `SLEEPDEEP=1`                                      | 同上                                         |
| STOP    | `RCC->APB1ENR1/SMENR1`     | PWR/RTCAPB；SMBA 时 I2C1                           | 同上                                         |
| STOP    | `GPIOA/B/SYSCFG` sleep     | `__HAL_RCC_*_CLK_SLEEP_ENABLE`                     | 同上                                         |
| STOP    | `EXTI->IMR1/FTSR1/PR1`     | PB5 line5                                          | `Button_WakeInit()`                          |
| SMBA    | `RCC->CCIPR`               | `I2C1SEL=HSI16`                                    | `Button_SmbaEnable()`                        |
| SMBA    | `I2C1->CR1`                | SMBHEN/ALERTEN/ERRIE/WUPEN                         | 同上                                         |
| BQ NORM | `SYS_CTRL1`                | 0x18                                               | `Bq_StartConfig()`                           |
| BQ NORM | `SYS_CTRL2`                | 0x40 + FET mask                                    | 配置/运行                                    |
| BQ LP   | `SYS_CTRL1`                | 0x00                                               | `BQ_LpBootPrepare()`                         |
| BQ LP   | `SYS_CTRL2`                | 0x00                                               | 同上                                         |
| BQ      | `SYS_STAT`                 | 0xA0                                               | 清 CC_READY/XREADY                           |
| BQ      | `PROTECT1/2/3`             | 0x89/0x57/0x40                                     | `Bq_ApplyProtectionConfig()`                 |
| USART1  | `BRR`                      | 694                                                | `App_Log_HwInit()` @80 MHz                   |
| USART2  | `BRR`                      | 21                                                 | `Ws2812_HwInit()` @80 MHz                    |
| TIM2    | `PSC/ARR`                  | 319/999                                            | `App_TickTimerInit()`                        |
| RTC     | `PRER/CR/WUTR`             | 127/255；WUTE off                                  | MX RTC init/LP WUT disable                   |
| 复位    | `RTC->BKP0R`               | magic `/` cause                                    | `ResetCause_RecordAndReset()`                |
| 复位    | `RTC->BKP20R..23R`         | CFSR/HFSR/MMFAR/BFAR                               | HardFault                                    |
| session | `RTC->BKP1..10`            | magic/CRC/data                                     | `LpSession_Save()`                           |
| session | `RTC->BKP1..10`            | 读/校验                                            | `LpSession_Load()`                           |

## E. 当前实现的边界与已知限制

> 以下按当前工作树源码整理；本文描述本次目标的质量/归档边界，但不把编译、烧录或上板行为写成已验证事实。

1. **LSE 硬件**：当前板 LSE 不起振，默认 `LP_RTC_USE_LSE=0`，RTC 走 LSI，LP 正常。切到 `=1` 时必须先修好晶振；LSE 失败会回退 LSI 保 boot，但 `PowerMode_LpTimebaseReady()` 失败、LP 禁用并置 LSE/TIMEBASE 故障。RTCSEL 改变会触发一次 `BDRST`，清 BKP/日历一次。
2. **LSI 绝对时间**：RTC 与 IWDG 同源，30 min 点按 RTC epoch 计，IWDG timeout 用 RTC 差分校准；相对周期自洽，但 LSI 的绝对精度/温漂未标定，不能当长时间计量。
3. **PA8**：EXTI 事件上升沿，无 IMR/NVIC，不能唤醒 STOP1；只在每次轻量 boot 和进 STOP 前按电平检查，ALERT 响应最坏延迟一个 IWDG 周期。
4. **PB5/SMBA**：生产唤醒是 EXTI5；`LP_SMBA_WAKE_TEST=1` 只做一次 SMBA host-alert 实测并保留 EXTI5 回退，台架结果存在 EXTI/SMBA 同时触发的歧义，不随生产发布。
5. **PVD**：NORM 由 `Fault_Init()` 开 2.8 V；轻量 LP boot 不跑 `Fault_Init`，PVD 未配置。
6. **MSI/STOPWUCK**：`SystemClock_Config()` 未关闭 MSI；`LpCycle_EnterStop()` 在 WFI 前设 `STOPWUCK=1`（HSI16），WFI 退出后硬编码 `SystemCoreClock=16 MHz` 并重建 tick，未读回 SWS 校验。
7. **DEGRADED**：`PowerMode_ClkForceHsi16()` 不恢复 Flash latency（可能仍 4 WS），失败路径也不保证关闭 HSE/PLL、不把 PA1 抬回高；降级只保故障灯，不复位循环。
8. **TIM1 DeInit**：轻量路径 `HAL_TIM_PWM_DeInit(&htim1)` 未到达 `HAL_TIM_Base_MspDeInit`，TIM1 时钟/PA7 AF1 可能残留；最终会复位，实际影响仅 LP 漏电。
9. **未调用 API**：`BQ_ReInitAfterLp`、`App_Log_DeInitForLp`、`Ws2812_DeInitForLp`、`Buzzer_DeInitForLp`、`App_TickStop`、`Fault_PvdDisable` 在当前轻量 LP 路径中没有调用；轻量路径使用原始 HAL DeInit。
10. **BQ LP 兼容 API**：当前 LP 不使用 `BQ_PrepareForLpMask`/`s_power.lp_fet_mask`；`BQ_LpBootPrepare()`/`BQ_LpMonitorRun()` 直接管理 ADC/TEMP/CC/FET。旧 API 保留只为兼容。
11. **session/状态**：`LP_SESSION_FAULT`、`RESET_CAUSE_OTHER` 仍可能没有赋值路径；`FAULT_BIT_TIMEBASE` 现在由 `PowerMode_Init()` 和 LSE 模式下 `PowerMode_LseHealthTask()` 写入/清除。
12. **CAN/DAC**：MX 初始化但从未 `HAL_CAN_Start`/`HAL_DAC_Start`；`.ioc` 的 16 MHz CAN 时序仅代表 CubeMX 配置，不是运行时。
13. **I2C1 时序**：`0x1424A4FF` 在 16 MHz 下约 19 kHz，在 80 MHz 下约 95 kHz；S1 不应标成 95 kHz。
14. **旧文档符号**：`PowerMode_LogBootRtcState`、`lp_stage`、`lp_fet_mask`、`wake_rtc_count`、`Timebase_*`、`LP_SMBA_TEST` 等均为历史名称，当前源码不存在或不再适用。
15. **历史 CSV**：`dbglog/*.csv`、`Ozone_Watched_Data_1_*.csv` 只记录当时的 watch 集；当前最小诊断是 `s_power.boot_reset_cause/wake_cause/wake_count`、`s_lp.*`、`s_app.auto_lp_block`。

## F. 关键时序常数


| 项目                      | 值                           | 说明                                                                        |
| --------------------------- | ------------------------------ | ----------------------------------------------------------------------------- |
| `APP_IDLE_LP_MS`          | 5000 ms                      | NORM 空闲自动 LP                                                            |
| `LP_BASE_CYCLE_MS`        | 30000 ms                     | IWDG 复位式 LP 基础周期                                                     |
| `LP_MIN_SLEEP_MS`         | 1000 ms                      | 最小 STOP 时间                                                              |
| `LP_HEARTBEAT_MS`         | 10 ms                        | PB4 每次轻量 boot 心跳                                                      |
| `LP_MARK_TOLERANCE_MS`    | 1000 ms                      | 30 min 点容差                                                               |
| `LP_BOOT_OVERHEAD_MAX_MS` | 5000 ms                      | boot overhead 上限                                                          |
| IWDG boot guard           | /256，RLR 4095 ≈32.8 s      | `Watchdog_Init()`                                                           |
| IWDG NORM                 | /64，RLR 4095 ≈8.2 s        | `Watchdog_InitNorm()`                                                       |
| IWDG LP                   | 动态 1–30 s                 | `Watchdog_SetTimeoutMs()`                                                   |
| BQ transfer timeout       | 100 ms                       | 总线恢复                                                                    |
| BQ ADC conversion         | 250 ms                       | 采样最小间隔                                                                |
| BQ ADC settle             | 300 ms                       | `BQ_ADC_SETTLE_MS`                                                          |
| BQ resample               | 500 ms                       | `BQ_ADC_RESAMPLE_MS`；OOR 后读数间隔不低于 ADC 250 ms 转换                  |
| BQ restart off            | 800 ms                       | `BQ_ADC_RESTART_OFF_HOLD_MS`；仅输出关闭的严格 PARTIAL 重试                 |
| BQ restart attempts       | 2                            | `BQ_ADC_RESTART_MAX_ATTEMPTS`；严格 PARTIAL 冷却重试不作为通信/真实故障豁免 |
| BQ OOR streak             | 8                            | `BQ_OOR_STREAK_RESTART`                                                     |
| BQ good-frame clear       | 4                            | `BQ_ADC_GOOD_FRAMES_CLEAR`                                                  |
| BQ verdict                | 1500 ms                      | `BQ_ADC_VERDICT_MS`；重启后等待有效发布                                     |
| BQ partial retry          | 10000 ms                     | `BQ_PARTIAL_RETRY_MS`；输出关闭时最短重试间隔                               |
| BQ LP off hold            | 300 ms                       | ADC/TEMP/CC/FET 关闭后等待                                                  |
| Self-test WAIT_BQ         | 3 s                          | 首帧/分类                                                                   |
| Self-test grace           | 12 s                         | 非恢复累计                                                                  |
| Self-test hard cap        | 30 s                         | 绝对上限                                                                    |
| Self-test monitor         | 1 s / 5 s fail / 3 s recover |                                                                             |
| Charger settle            | 30 ms ×2                    | PA1/COMP                                                                    |
| 5 V startup               | 20 ms                        | `APP_5V_STARTUP_DELAY_MS`                                                   |
| HSE settle                | 30 ms ×3                    | `PowerMode_ClkSwitchToHsePll()`                                             |
| LP request drain          | ≤100 ms                     | BQ idle/I2C busy                                                            |
| RTC recovery/LSE          | 100 ms stable                | `PowerMode_LseHealthTask()`                                                 |
| Button sample/debounce    | 4 ms / 20 ms                 | 轮询                                                                        |
| Comparator debounce       | 32 × 4 ms = 128 ms          | TIM2 ISR                                                                    |
| TIM2 tick                 | 250 Hz / 4 ms                | PSC 319/ARR 999                                                             |
| BQ read                   | 500 ms                       | `APP_BQ_READ_PERIOD_MS`                                                     |
| BQ stale                  | 2000 ms                      | `APP_BQ_STALE_MS`                                                           |
| Diag log                  | 1 s flag / 10 s print        | `APP_DIAG_LOG_PERIOD_MS`                                                    |
| Black-box NORM            | 30 min                       | `APP_BB_PERIOD_MS`                                                          |
| LP 30 min mark            | 1800 s                       | RTC epoch                                                                   |

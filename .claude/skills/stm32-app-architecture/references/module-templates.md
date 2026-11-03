# 模块模板与扩展配方

参考工程的实际代码位于 `C:\Users\Shuchen Tao\Documents\Armor_Cmzy26_081\App\`（只读）。以下是按其风格整理的可套用模板；先替换模块名/层级，再接入本板句柄与引脚。

## 1. 头文件模板（BSP 模块示例）

```c
/**
 ******************************************************************************
 * @file    bq_ic.h
 * @brief   BQ 电池管理 IC 驱动（I2C1）。
 *
 * 层：bsp。依赖：HAL + board.h。对外只暴露本模块 API，不暴露 HAL 句柄。
 ******************************************************************************
 */
#ifndef APP_BQ_IC_H
#define APP_BQ_IC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 初始化：只做无阻塞配置；I2C1 外设本身由 CubeMX 的 MX_I2C1_Init 配置。 */
void BqIc_Init(void);

/* 主循环调用：非阻塞推进（状态机/事务解析），禁止 HAL_Delay/长阻塞。 */
void BqIc_Loop(void);

/* 250 Hz tick 内由 App_OnTick250Hz 分频调用（如 20 Hz 健康评估）。 */
void BqIc_Tick(void);

/* 最近一次通信/自检是否正常（供故障注册表与状态机读取）。 */
bool BqIc_IsOk(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BQ_IC_H */
```

规范要点：

- 文件名与模块前缀一致；include guard 为 `APP_<文件名大写>_H`。
- 头文件不 include 其他模块的 `.c`；只 include 本层/下层公开头。
- 需要 ISR 入口时显式暴露 `Xxx_Irq*()`，并在注释里写明由哪个 weak 回调分发。
- `Init` 一般返回 `void`；若需要初始化结果，增加 `Xxx_IsInitOk()`（参考 `ADS131M04_IsInitOk`）。

## 2. 源文件模板

```c
/**
 ******************************************************************************
 * @file    bq_ic.c
 * @brief   BQ 电池管理 IC 驱动（I2C1）。
 ******************************************************************************
 */
#include "bsp/bq_ic.h"     /* 本模块头优先 */

#include "board.h"         /* 句柄/板级常量 */

#include <stddef.h>
#include <string.h>

/* 调试可见量：模块私有静态结构体，字段完整命名；ISR 更新的字段加 volatile。 */
static struct
{
    volatile uint8_t  fresh;          /* 最近一次数据是否有效 */
    volatile uint32_t ok_count;
    volatile uint32_t err_count;
    bq_state_t        state;
    uint32_t          timeout_count;
} s_bq;

void BqIc_Init(void)
{
    s_bq.fresh       = 0u;
    s_bq.ok_count    = 0u;
    s_bq.err_count   = 0u;
    s_bq.state       = BQ_ST_IDLE;
    s_bq.timeout_count = 0u;
    /* 只做无阻塞配置；需要等待器件启动的流程放 BqIc_Loop 的状态机里。 */
}

void BqIc_Loop(void)
{
    /* 非阻塞 I2C 事务/状态推进；错误只计数并上报故障注册表。 */
}

void BqIc_Tick(void)
{
    /* 低频健康评估；由 App_OnTick1ms 的模运算分频调用。 */
}

bool BqIc_IsOk(void)
{
    return s_bq.fresh != 0u;
}
```

## 3. 应用编排模板（`App/app/main_app.c`）

```c
#include "app/main_app.h"

#include "board.h"
#include "bsp/xxx.h"
#include "comm/app_can.h"
#include "comm/app_log.h"
#include "comm/board_comm.h"
#include "comm/host_cmd.h"
#include "app/faults.h"
#include "app/led_status.h"
#include "app/self_test.h"
#include "app/state_machine.h"
#include "app/reset_cause.h"

/* 调试可见量：模块私有静态结构体，字段完整命名；ISR 写字段加 volatile。 */
static struct
{
    volatile uint32_t tick_250hz_count;
    volatile uint32_t uptime_s;
    volatile uint8_t  flag_bq_read_request;
    volatile uint8_t  flag_diag_request;
    uint32_t          prescale_500ms;
    uint32_t          prescale_1s;
} s_app;

void App_Init(void)
{
    ResetCause_Init();
    Fault_Init();              /* 先于故障源初始化 */
    PowerMode_Init();          /* HSI16 → HSE/PLL 80 MHz，失败则降级 */
    App_Log_Init();

    Xxx_Init();                /* BSP：按依赖顺序逐个初始化 */
    LedStatus_Init();
    StateMachine_Init();
    SelfTest_Init();

    /* 全部模块就绪后才启动 250 Hz 应用时基。
     * TIM2 的参数由固件覆写寄存器，Start 必须显式写。 */
    App_TickTimerInit();       /* PSC=319 / ARR=999，NVIC 优先级 2 */
    App_TickStart();
}

void App_Loop(void)
{
    SelfTest_Poll();

    /* 硬件帧消费 → 事件出队 → 状态机 → 上报（按实际模块替换） */
    Xxx_Loop();
    HostCmd_Loop();
    BoardComm_Loop();

    App_Log_FlushSmall(2u);    /* 非阻塞日志小批量刷出 */
}

void App_OnTick250Hz(void)
{
    s_app.tick_250hz_count++;

    LedStatus_Tick();

    if (++s_app.prescale_500ms >= 125u)        /* 250 Hz / 125 = 2 Hz */
    {
        s_app.prescale_500ms = 0u;
        s_app.flag_bq_read_request = 1u;
    }
    if (++s_app.prescale_1s >= 250u)           /* 1 Hz 诊断/运行时基 */
    {
        s_app.prescale_1s = 0u;
        s_app.uptime_s++;
        s_app.flag_diag_request = 1u;
    }
}

/* it.c 已生成 TIM2_IRQHandler；本回调是唯一 owner。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        App_OnTick250Hz();
    }
}
```

## 4. `app.mk` 模板

```make
# ---- App/ 应用层构建片段 ----
# CubeMX 重生成会覆写 Makefile：需在 C_INCLUDES 之后、OBJECTS 之前重新加
#   -include app.mk
# Makefile 用 notdir 压平对象，APP_SOURCES 内文件名必须全局唯一。

APP_SOURCES = \
App/bsp/bq_ic.c \
App/bsp/ws2812_uart.c \
App/comm/app_can.c \
App/comm/app_log.c \
App/comm/board_comm.c \
App/app/main_app.c \
App/app/state_machine.c \
App/app/led_status.c \
App/app/faults.c \
App/app/self_test.c \
App/app/reset_cause.c

# C_SOURCES 是递归变量：先立即展开追加，再赋回，避免 += 造成自引用。
# 应用需要的 HAL 源（IOC 未启用时，如 stm32l4xx_hal_rtc.c）一并放入 APP_SOURCES 去重。
APP_NEW_SOURCES := $(filter-out $(C_SOURCES), $(APP_SOURCES))
C_SOURCES := $(C_SOURCES) $(APP_NEW_SOURCES)
C_INCLUDES += -IApp -IApp/bsp -IApp/comm -IApp/app
# 使用 detect/ 时再加：
# C_INCLUDES += -IApp/detect
```

Makefile 插入位置：

```make
C_INCLUDES = \
...
-IDrivers/CMSIS/Include

# ---- 应用层 ----
-include app.mk

# compile gcc flags
CFLAGS += ...
```

要求：

- 必须在 `C_SOURCES` / `C_INCLUDES` 定义之后、`OBJECTS` 计算/编译规则之前。
- 每次 CubeMX 重生成后重新添加；检查应用源文件是否仍参与编译。
- 新增源文件后手工同步 `compile_commands.json`（参考工程没有生成脚本）。

## 5. `main.c` 的 USER CODE 接入

```c
/* USER CODE BEGIN Includes */
#include "app/main_app.h"
/* USER CODE END Includes */

/* ... main() 中所有 MX_*_Init() 之后 ... */
  /* USER CODE BEGIN 2 */
  App_Init();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Loop();
  }
  /* USER CODE END 3 */
```

禁止在这些块之外添加/修改手写逻辑。CubeMX 保留 USER CODE 块内容，但会覆盖其余生成内容。

## 6. 250 Hz 应用时基（TIM2）

- 本工程 TIM2 在 `.ioc` 中只是占位（PSC=0、ARR=0xFFFFFFFF、OC CH1）；**时基参数不在 IOC 配置**，由 `App_TickTimerInit()` 写寄存器覆写：PSC=319、ARR=999（80 MHz → 250 Hz），清 UIF，NVIC 优先级 2。
- `App_Init()` **最后**调用 `App_TickStart()`（`HAL_TIM_Base_Start_IT(&htim2)` + NVIC 使能）；LP 进入前 `App_TickStop()`，唤醒重建后重新启动。
- `HAL_TIM_PeriodElapsedCallback` 归 `app/main_app.c` 唯一所有，按 `htim->Instance == TIM2` 分发到 `App_OnTick250Hz()`。
- HAL 的 `HAL_GetTick()` 仍来自 SysTick；应用周期节拍不要掺杂进 SysTick。LP 中用 `HAL_SuspendTick()`，唤醒后 `HAL_InitTick(TICK_INT_PRIORITY)` 修复。

## 7. ISR 分发与并发模板

同一 HAL weak 回调只保留一个 owner，按实例分发：

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Ws2812_IrqTxDone();       /* RGB 灯 DMA 完成 */
    }
    else if (huart->Instance == USART1)
    {
        App_Log_IrqTxDone();      /* 日志/二进制块 DMA 完成 */
    }
}
```

ISR 内允许的动作：

```c
/* OK：置标志、计数、启动 DMA、把小帧拷进队列 */
s_pending = 1u;
s_diag.rx_irq_count++;

/* 禁止：发 CAN、解析协议、写 Flash、printf、HAL_Delay、动态内存 */
```

环形/队列并发注释模板：

```c
/* 并发规则（R17）：
 *   写指针/生产者：仅 ISR；
 *   读指针/消费者：仅主循环；
 *   槽状态：volatile 原子标志；ISR 不改读指针，主循环不改写指针。
 */
```

## 8. 扩展配方

| 场景 | 步骤 |
| --- | --- |
| 新增 BSP 设备 | ① `App/bsp/xxx.h/.c`，前缀 `Xxx_`；② 句柄进 `App/board.h`；③ `app.mk` 加源文件；④ `App_Init` 按依赖顺序调 `Xxx_Init()`；⑤ 若有中断，登记 owner 并在 `.ioc` 使能 NVIC |
| 新增 CAN 报文/命令 | ① `board_comm.h` 定义固定 ID/DLC/字段；② `board_comm.c` 构造/解析；③ 主循环每圈最多发一帧；④ 同步协议文档；⑤ 与对端联调 |
| 新增上位机命令 | ① `host_cmd` 命令号 + 载荷 + 处理函数 + ACK 语法；② 主机工具构造器；③ 协议文档；④ Flash 写命令保留"忙则拒绝" |
| 新增状态/灯效 | ① 状态/灯效枚举加值；② `state_machine.c` 转移；③ `led_status.c` 生成灯效；④ 同步状态/灯效文档 |
| 新增可调参数 | ① 参数结构体加字段；② 默认值函数唯一维护；③ 同步设计文档；④ 若在线可调：命令范围表 + 主机工具 + 文档双端同步 |
| 新增 Flash 持久化 | ① 先划分保留页，避开已有存储；② 复用 `flash_store` 原语；③ 上电纯读、只在写时擦；④ CRC 覆盖全字段；⑤ `_Static_assert` 钉死地址/尺寸；⑥ 核对镜像末址；⑦ 擦写只在主循环 |
| 新增算法模块 | 放 `App/detect/`，模块前缀 + 输入/输出显式 API；**禁止 include `comm/`**；事件/结果只出队，日志与上报由 `app/` 层取走 |
| 迁移参考模块 | 读头文件与 `Init/Loop/Tick/Isr` 结构 → 复制骨架 → 改前缀/guard → 映射本板句柄/外设/常量 → 接入 → 构建与上板验证 |

## 9. 模块级完成检查表

- [ ] 文件在正确层；依赖方向合法；`detect/` 未依赖 `comm/`
- [ ] 模块前缀 + `_t` 类型 + `APP_*_H` guard
- [ ] 头文件只暴露公开 API；私有状态 `static`
- [ ] 新句柄仅在 `App/board.h` 中 `extern`
- [ ] 已在 `app.mk` 登记且文件名唯一
- [ ] `App_Init/App_Loop/App_OnTick250Hz` 已接入，tick 启动在最后
- [ ] ISR owner 唯一且已登记；ISR 内无协议/Flash/阻塞
- [ ] 共享数据遵守 R17；Flash 只在主循环
- [ ] 调试可见量放在字段完整命名的 `static struct` 中（ISR 写字段加 `volatile`）
- [ ] `make -j2 all` 通过；镜像大小/链接上限核对（`_etext` < `0x0801A000`）
- [ ] 协议/参数/状态文档已同步

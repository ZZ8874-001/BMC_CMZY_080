# =============================================================================
# app.mk — App/ 分层应用源码接入
#
# 接入方式（每次 CubeMX 重新生成 Makefile 后必须重做）：
#   在 Makefile 的 C_INCLUDES 之后、 "# compile gcc flags" 之前插入：
#       -include app.mk
#
# 说明：
#   - 生成的 C_SOURCES 是静态列表，这里以 filter-out 去重后追加，避免 HAL 源
#     （应用同样需要但 IOC 可能未启用）被重复加入链接；
#   - 源文件名必须全库唯一（Makefile 用 notdir 扁平化到 build/）；
#   - HAL_RTC_MODULE_ENABLED 已由 Core/Inc/stm32l4xx_hal_conf.h 打开，勿重复 -D。
# =============================================================================

# LP/STOP1 是否默认保持 SWD 调试连接（DBGMCU_CR_DBG_STOP）：
#   0 = STOP1 关 SWD（默认，最低功耗，PB5 唤醒不受调试器影响）
#   1 = 保持（LP 中可连接/保持调试器，LP 电流升高）
# 运行期也可在 NORM 下写 s_power.lp_keep_swd=1 覆盖初值。
# 用法：make POWER_LP_KEEP_SWD=1；改变后必须 make clean。
POWER_LP_KEEP_SWD ?= 0
C_DEFS += -DPOWER_LP_KEEP_SWD=$(POWER_LP_KEEP_SWD)

APP_SOURCES =  \
App/app/main_app.c \
App/app/power_mode.c \
App/app/black_box.c \
App/app/state_store.c \
App/app/state_machine.c \
App/app/charge_policy.c \
App/app/cal_stream.c \
App/app/led_status.c \
App/app/faults.c \
App/app/self_test.c \
App/app/reset_cause.c \
App/detect/gesture.c \
App/detect/soc_est.c \
App/detect/bms_port_stm32.c \
App/detect/bms_nvm_port.c \
App/detect/bms/bms_port.c \
App/detect/bms/bms_cal.c \
App/detect/bms/soc.c \
App/detect/bms/soh.c \
App/detect/bms/bms_nvm.c \
App/bsp/bq_ic.c \
App/bsp/button.c \
App/bsp/flash_store.c \
App/bsp/ws2812_uart.c \
App/bsp/buzzer.c \
App/comm/app_log.c

# RTC 已在 IOC 中，生成的 Makefile 通常已包含；这里保留兜底，已包含时由 filter-out 去重。
APP_HAL_SOURCES =  \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c

APP_INCLUDES =  \
-IApp \
-IApp/bsp \
-IApp/comm \
-IApp/app \
-IApp/detect \
-IApp/detect/bms

APP_ALL_SOURCES = $(APP_SOURCES) $(APP_HAL_SOURCES)

# C_SOURCES 是递归变量，必须先立即展开再追加，否则 += 会造成自引用。
APP_NEW_SOURCES := $(filter-out $(C_SOURCES), $(APP_ALL_SOURCES))
C_SOURCES := $(C_SOURCES) $(APP_NEW_SOURCES)
C_INCLUDES += $(APP_INCLUDES)

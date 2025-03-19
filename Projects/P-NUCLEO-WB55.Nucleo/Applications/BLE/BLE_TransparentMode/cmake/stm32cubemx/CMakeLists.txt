cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_STM32WBXX_NUCLEO 
	USE_HAL_DRIVER 
	STM32WB55xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../STM32_WPAN/App
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Inc
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Inc/Legacy
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Utilities/lpm/tiny_lpm
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/utilities
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble/core
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble/core/auto
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble/core/template
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble/svc/Inc
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble/svc/Src
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/lhci
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/vcp
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/CMSIS/Device/ST/STM32WBxx/Include
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Utilities/sequencer
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/CMSIS/Include
    ../../../../../../../Drivers/BSP/P-NUCLEO-WB55.Nucleo
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/app_entry.c
    ../../Core/Src/app_debug.c
    ../../Core/Src/hw_timerserver.c
    ../../Core/Src/hw_uart.c
    ../../Core/Src/stm32_lpm_if.c
    ../../Core/Src/stm32wbxx_it.c
    ../../Core/Src/stm32wbxx_hal_msp.c
    ../../STM32_WPAN/App/tm.c
    ../../STM32_WPAN/Target/hw_ipcc.c
    ../../../../../../../Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_hsem.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_rcc.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_gpio.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_cortex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_exti.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_ipcc.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc_ex.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim_ex.c
    ../../Core/Src/system_stm32wbxx.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/utilities/otp.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/utilities/stm_list.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/utilities/stm_queue.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/ble/core/template/osal.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/lhci/lhci.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Utilities/lpm/tiny_lpm/stm32_lpm.c
    C:/Users/Olly/STM32Cube/Repository/STM32Cube_FW_WB_V1.20.0/Utilities/sequencer/stm32_seq.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32wb55xx_cm4.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()



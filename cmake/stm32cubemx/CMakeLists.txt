cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_FULL_LL_DRIVER 
	HSE_VALUE=8000000 
	HSE_STARTUP_TIMEOUT=100 
	LSE_STARTUP_TIMEOUT=5000 
	LSE_VALUE=32768 
	HSI_VALUE=8000000 
	LSI_VALUE=40000 
	VDD_VALUE=3300 
	PREFETCH_ENABLE=1 
	STM32F103xB
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32F1xx_HAL_Driver/Inc
    ../../Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/gpio.c
    ../../Core/Src/dma.c
    ../../Core/Src/iwdg.c
    ../../Core/Src/spi.c
    ../../Core/Src/stm32f1xx_it.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_spi.c
    ../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c
    ../../Core/Src/system_stm32f1xx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f103xb.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()



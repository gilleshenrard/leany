#############################################################################################################################
# file:  CMakeLists.txt
# brief: Template "CMakeLists.txt" for building of executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements.
#        For debug build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#          make -C Debug VERBOSE=1
#        For release build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
#          make -C Release VERBOSE=1
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M0				      "-mcpu=cortex-m0")
set (MCPU_CORTEX_M0PLUS				  "-mcpu=cortex-m0plus")
set (MCPU_CORTEX_M3				      "-mcpu=cortex-m3")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MCPU_CORTEX_M7				      "-mcpu=cortex-m7")
set (MCPU_CORTEX_M33				  "-mcpu=cortex-m33")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "stm32-leveler")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8TX_FLASH.ld)
set (MCPU                     ${MCPU_CORTEX_M3})
set (MFPU                     "")
set (MFLOAT_ABI               ${MFLOAT_ABI_SOFTWARE})
set (RUNTIME_LIBRARY          ${RUNTIME_LIBRARY_REDUCED_C})
set (RUNTIME_LIBRARY_SYSCALLS ${RUNTIME_LIBRARY_SYSCALLS_MINIMAL})

set(CMAKE_C_STANDARD                23)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)

set (PROJECT_SOURCES
	Core/Startup/startup_stm32f103c8tx.s
	Core/Src/main.c
	Core/Src/stm32f1xx_hal_msp.c
	Core/Src/stm32f1xx_it.c
	Core/Src/syscalls.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f1xx.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c

	Core/Src/errors/errorstack.c
	Core/Src/hardware/accelerometer/ADXL345.c
	Core/Src/hardware/screen/numbersVerdana16.c
	Core/Src/hardware/screen/SSD1306.c
)

set (PROJECT_DEFINES
	USE_HAL_DRIVER
	STM32F103xB
)

set (PROJECT_INCLUDES
	Drivers/STM32F1xx_HAL_Driver/Inc
	Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
	Drivers/CMSIS/Include
	Drivers/CMSIS/Device/ST/STM32F1xx/Include
	Core/Inc

	Core/Inc/errors
	Core/Inc/hardware/accelerometer
	Core/Inc/hardware/screen
)

set (CPU_OPTIONS
	${MCPU}
	${MFPU}
	${MFLOAT_ABI}
	${RUNTIME_LIBRARY}
	${RUNTIME_LIBRARY_SYSCALLS}
	-fdata-sections
	-ffunction-sections
	-Wl,--gc-sections
	-mthumb
)

set(CPP_FLAGS
	-fno-rtti
	-fno-exceptions
	-fno-threadsafe-statics
)

############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################

################## PROJECT SETUP ######################################
project(${PROJECT_NAME})
enable_language(ASM C)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_compile_definitions (${PROJECT_NAME} PRIVATE ${PROJECT_DEFINES})
target_include_directories (${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDES})

target_compile_options(${PROJECT_NAME} PRIVATE
	${CPU_OPTIONS}
	-Og
	$<$<CONFIG:Debug>:-g3 -ggdb>
)

target_link_options(${PROJECT_NAME} PRIVATE
	-T${LINKER_SCRIPT}
	${CPU_OPTIONS}
	${RUNTIME_LIBRARY_SYSCALLS}
	-Wl,-Map=${PROJECT_NAME}.map,--cref
	-Wl,--gc-sections
	-Wl,--print-memory-usage
	-static
	-Wl,--start-group
	-lc
	-lm
	-Wl,--end-group
)

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME}
	POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX} ${PROJECT_NAME}.hex
)

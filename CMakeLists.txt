#############################################################################################################################
# file:  CMakeLists.txt
# date:  04/01/2023
# brief: Root CMakeLists file
#
# Prerequisites:
#        - GNU Arm Embedded Toolchain (gcc-arm-none-eabi) is installed
#        - CMake is installed
#
# note:  This repository includes a CMakePresets.json file to ease up the CMake use.
#        This file uses the toolchain file cmake/gcc-arm-none-eabi.cmake
#
# usage: cmake --preset Debug
#        cmake --build build/Debug
#
#        cmake --preset Release
#        cmake --build build/Release
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

#define the C standard used
set(CMAKE_C_STANDARD                23)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)

#define the definitions used when compiling (-D)
set (PROJECT_DEFINES
	USE_HAL_DRIVER
	STM32F103xB
	$<$<CONFIG:Debug>:DEBUG>
)

#define the included directories list
set (PROJECT_INCLUDES
	${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
	${CMAKE_SOURCE_DIR}/Core/Inc

	${CMAKE_SOURCE_DIR}/Core/Inc/errors
	${CMAKE_SOURCE_DIR}/Core/Inc/hardware/accelerometer
	${CMAKE_SOURCE_DIR}/Core/Inc/hardware/screen
)

#define the CPU-specific arguments used when compiling
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

#define C++ arguments (unused)
set(CPP_FLAGS
	-fno-rtti
	-fno-exceptions
	-fno-threadsafe-statics
)

#declare the compilation options
set(CUSTOM_COMPILE_OPTIONS
	${CPU_OPTIONS}
	$<$<CONFIG:Release>:-O1>			#add flags to compile when Release
	$<$<CONFIG:Debug>:	-Og				#add flags to compile when Debug
						-g3
						-ggdb
						-fstack-usage
						-fcyclomatic-complexity
	>
)

#declare the linker options
set(CUSTOM_LINK_OPTIONS
	-T${LINKER_SCRIPT}
	${CPU_OPTIONS}
	${RUNTIME_LIBRARY_SYSCALLS}
	-Wl,-Map=${PROJECT_NAME}.map
	-Wl,--gc-sections
	-Wl,--print-memory-usage
	-static
	-Wl,--start-group
	-lc
	-lm
	-Wl,--end-group
)

################## PROJECT SETUP ######################################
#declare the project name, languages used and executable name
project(${PROJECT_NAME})
enable_language(ASM C)
add_executable(${PROJECT_NAME}
	Core/Startup/startup_stm32f103c8tx.s
	Core/Src/main.c
	Core/Src/stm32f1xx_hal_msp.c
	Core/Src/stm32f1xx_it.c
	Core/Src/syscalls.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f1xx.c
)

#add directories in which find other CMakeLists.txt files
add_subdirectory(Drivers)
add_subdirectory(Core)

#use all previous declarartions
target_compile_definitions (${PROJECT_NAME} PRIVATE ${PROJECT_DEFINES})
target_include_directories (${PROJECT_NAME} PRIVATE Core/Inc ${PROJECT_INCLUDES})
target_compile_options(${PROJECT_NAME} PRIVATE ${CUSTOM_COMPILE_OPTIONS})
target_link_options(${PROJECT_NAME} PRIVATE ${CUSTOM_LINK_OPTIONS})
target_link_libraries(${PROJECT_NAME} PRIVATE
						CubeMXgenerated
						errorStack
						adxl345
						ssd1306
)

#declare Assembly compilation arguments
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

#set the executables extension
set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")

#add a post-build command to create a *.hex file
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}
	POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX} ${PROJECT_NAME}.hex
)

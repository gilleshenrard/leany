#############################################################################################################################
# file:  CMakeLists.txt
# date:  15/02/2024
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

#declare the project and languages used
set (PROJECT_NAME "stm32-inclinometer")
project(${PROJECT_NAME})
enable_language(ASM C)

#define the C standard used
set(CMAKE_C_STANDARD                23)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)

#define the definitions used when compiling (-D)
set (PROJECT_DEFINES
	USE_FULL_LL_DRIVER
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
)

#define the CPU-specific arguments used when compiling
set (CPU_OPTIONS
	-mcpu=cortex-m3
	-mfloat-abi=soft
	--specs=nano.specs
	--specs=nosys.specs
	-fdata-sections
	-ffunction-sections
	-Wl,--gc-sections
	-mthumb
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
	-T${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld
	${CPU_OPTIONS}
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
#add directories in which find other CMakeLists.txt files
add_subdirectory(Drivers)
add_subdirectory(Core)

#declare final executable and link to libraries
#	(compilation and linking attributes inherited from drivers library)
add_executable(${PROJECT_NAME}
	Core/Startup/startup_stm32f103c8tx.s
	Core/Src/main.c
	Core/Src/stm32f1xx_it.c
	Core/Src/syscalls.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f1xx.c
)
target_link_libraries(${PROJECT_NAME} PRIVATE
						CubeMXgenerated
						adxl345
						ssd1306
						buttons
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

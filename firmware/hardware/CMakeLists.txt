# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

add_subdirectory(imu)

#make sure STM32CubeMX library knows about sysutils/ (to use it in interrupts, ...)
target_include_directories(stm32cubemx INTERFACE sysutils/)

#create the sysUtils library, taking care of the error stack management
#	STM32CubeMX includes and definitions are manually added instead of linking the library
#	to avoid re-compiling STMCube libraries and optimising compilation
add_library(sysUtils
	sysutils/errorstack.c
	sysutils/hardware_events.c)
target_include_directories(sysUtils PUBLIC sysutils/)
target_compile_definitions(sysUtils PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(sysUtils SYSTEM PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(sysUtils PUBLIC ${WARNING_FLAGS})

#populate softVersion.c with the latest git commit hash
add_custom_command(TARGET sysUtils
	PRE_BUILD
	COMMAND ${CMAKE_COMMAND}
	-P gitversion.cmake
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/sysutils/
	COMMENT "Populating the latest git commit hash"
)

#
# Declare the HAL library
#
add_library(hal hal/halspi.c)
target_include_directories(hal PUBLIC hal)
target_link_libraries(hal PRIVATE sysUtils)

#
# Declare the UART serial library
#
add_library(serial
	serial/serial.c
	serial/scpi_parser.c
	serial/scpi_commands.c)
target_include_directories(serial PUBLIC serial)
target_link_libraries(serial PRIVATE sysUtils)

#
# declare the ST7735 library, taking care of the display
#
add_library(st7735s
	display/st7735_registers.inc
	display/st7735s.c
	display/st7735_initialisation.c)
target_include_directories(st7735s PUBLIC display)
target_include_directories(st7735s PUBLIC imu/)
target_link_libraries(st7735s PRIVATE sysUtils)
target_link_libraries(st7735s PRIVATE hal)

#create the buttons library, taking care of the control buttons
add_library(gpio
	gpio/gpiotask.c
	gpio/buttons.c
	gpio/led.c)
target_include_directories(gpio PUBLIC gpio)
target_link_libraries(gpio PRIVATE sysUtils)

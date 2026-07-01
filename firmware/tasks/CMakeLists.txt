# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#
# Declare the UART serial library
#
add_library(serial
	serialtask/task_serial.c
	serialtask/scpi_parser.c
	serialtask/scpi_commands.c)
target_compile_options(serial PRIVATE ${WARNING_FLAGS})
target_include_directories(serial PUBLIC serialtask)
target_link_libraries(serial PRIVATE utils)

add_library(tasks STATIC
	task_ui.c
	task_dispatcher.c
	task_imu.c
	task_gpio.c
	task_battery.c)
target_compile_definitions(tasks PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(tasks SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(tasks PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(tasks PRIVATE
	st7735s
	IMU
	gpio
	utils
	serial
	BATTERY
	hal
	UI)
target_compile_options(tasks PRIVATE ${WARNING_FLAGS})

# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/hardware)
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(${CMAKE_SOURCE_DIR}/ui)

#
# Declare the UART serial library
#
add_library(serial
	serialtask/task_serial.c
	serialtask/scpi_parser.c
	serialtask/scpi_commands.c)
target_include_directories(serial PUBLIC serialtask)
target_link_libraries(serial PRIVATE utils)

add_library(tasks STATIC
	task_ui.c
	task_dispatcher.c
	task_imu.c
	task_gpio.c)
target_include_directories(tasks PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_definitions(tasks PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(tasks SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(tasks PRIVATE
	st7735s
	IMU
	gpio
	utils
	serial
	hal
	UI)
target_compile_options(tasks PRIVATE ${WARNING_FLAGS})

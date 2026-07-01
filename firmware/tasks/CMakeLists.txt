# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

add_library(tasks STATIC
	task_ui.c
	task_dispatcher.c
	task_imu.c
	task_gpio.c
	task_battery.c
	task_serial.c
)
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
	BATTERY
	hal
	UI
	scpi
	sensorfusion
)
target_compile_options(tasks PRIVATE ${WARNING_FLAGS})

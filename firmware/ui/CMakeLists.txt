# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/tasks)

add_library(UI STATIC
	assets/bitmap.c
	assets/icons.c
	assets/fonts.c
	assets/label.c
	screens/screen_main.c
	screens/screen_system.c
	screens/screen_error.c
	display.c)
target_include_directories(UI PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	assets
	screens
)
target_compile_definitions(UI PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(UI SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(UI PRIVATE
	st7735s
	gpio
	hal
	tasks
	utils)
target_compile_options(UI PRIVATE ${WARNING_FLAGS})

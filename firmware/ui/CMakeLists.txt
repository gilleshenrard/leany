# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

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
target_link_libraries(UI PRIVATE
	st7735s
	BATTERY
	hal
	utils
	tasks
	sensorfusion
	stm32cubemx
)
target_compile_options(UI PRIVATE ${WARNING_FLAGS})

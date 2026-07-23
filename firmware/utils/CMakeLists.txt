# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

add_subdirectory(custom_stdlib)

#create the utils library, taking care of the error stack management
#	STM32CubeMX includes and definitions are manually added instead of linking the library
#	to avoid re-compiling STMCube libraries and optimising compilation
add_library(utils
	errorstack.c
	hardware_events.c
	leany_std.c
	systick.c)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utils PRIVATE stm32cubemx)
target_compile_options(utils PRIVATE ${WARNING_FLAGS})

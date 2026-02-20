# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#make sure STM32CubeMX library knows about the current directory (to use it in interrupts, ...)
target_include_directories(stm32cubemx INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#create the utils library, taking care of the error stack management
#	STM32CubeMX includes and definitions are manually added instead of linking the library
#	to avoid re-compiling STMCube libraries and optimising compilation
add_library(utils
	errorstack.c
	hardware_events.c
	softversion.c
	leany_std.c)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(utils PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(utils SYSTEM PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(utils PUBLIC ${WARNING_FLAGS})

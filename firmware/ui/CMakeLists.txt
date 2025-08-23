# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/hardware)
include_directories(${CMAKE_SOURCE_DIR}/dispatcher)

add_library(UI STATIC
	icons.c
	ui.c)
target_include_directories(UI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(UI PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(UI SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(UI PRIVATE
	st7735s
	dispatcher)
target_compile_options(UI PRIVATE ${WARNING_FLAGS})

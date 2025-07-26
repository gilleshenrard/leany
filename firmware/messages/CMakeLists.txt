# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/Hardware)

add_library(messages STATIC
	dispatcher.c)
target_include_directories(UI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(UI PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(UI SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(UI PRIVATE ${WARNING_FLAGS})

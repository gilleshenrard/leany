# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/Hardware)
include_directories(${CMAKE_SOURCE_DIR}/UI)

add_library(messages STATIC
	dispatcher.c)
target_include_directories(messages PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(messages PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(messages SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(messages PRIVATE ${WARNING_FLAGS})
target_link_libraries(messages PRIVATE
	st7735s
	bmi270
	buttons
	UI
)

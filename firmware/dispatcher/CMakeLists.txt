# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/Hardware)
include_directories(${CMAKE_SOURCE_DIR}/UI)

add_library(dispatcher STATIC
	dispatcher.c)
target_include_directories(dispatcher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(dispatcher PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(dispatcher SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(dispatcher PRIVATE ${WARNING_FLAGS})
target_link_libraries(dispatcher PRIVATE
	st7735s
	bmi270
	buttons
	UI
)

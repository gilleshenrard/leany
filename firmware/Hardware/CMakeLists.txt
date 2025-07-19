# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

include_directories(${CMAKE_SOURCE_DIR}/cmake/bosch-sensortec)

#make sure STM32CubeMX library knows about sysutils/ (to use it in interrupts, ...)
target_include_directories(stm32cubemx INTERFACE sysutils/)

#create the sysUtils library, taking care of the error stack management
#	STM32CubeMX includes and definitions are manually added instead of linking the library
#	to avoid re-compiling STMCube libraries and optimising compilation
add_library(sysUtils
	sysutils/errorstack.c)
target_include_directories(sysUtils PUBLIC sysutils/)
target_compile_definitions(sysUtils PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(sysUtils SYSTEM PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(sysUtils PUBLIC ${WARNING_FLAGS})

#populate softVersion.c with the latest git commit hash
add_custom_command(TARGET sysUtils
	PRE_BUILD
	COMMAND ${CMAKE_COMMAND}
	-P gitVersion.cmake
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/sysutils/
	COMMENT "Populating the latest git commit hash"
)

add_library(hal HAL/halspi.c)
target_include_directories(hal PUBLIC HAL)
target_link_libraries(hal PRIVATE sysUtils)

#create the ssd1306 library, taking care of the display
add_library(st7735s
	display/ST7735S.c
	display/ST7735_initialisation.c)
target_include_directories(st7735s PUBLIC display)
target_include_directories(st7735s PUBLIC sensor/)
target_link_libraries(st7735s PRIVATE sysUtils)
target_link_libraries(st7735s PRIVATE hal)

#create the BMI270 library, taking care of the MEMS sensor
add_library(bmi270
	sensor/memsBMI270.c
	sensor/sensorfusion.c)
target_include_directories(bmi270 PUBLIC sensor/)
target_link_libraries(bmi270 PRIVATE sysUtils)
target_link_libraries(bmi270 PRIVATE hal)
target_link_libraries(bmi270 PRIVATE st7735s)
target_link_libraries(bmi270 PRIVATE bmi270_api)

#create the buttons library, taking care of the control buttons
add_library(buttons
	buttons/buttons.c)
target_include_directories(buttons PUBLIC buttons)
target_link_libraries(buttons PRIVATE sysUtils)

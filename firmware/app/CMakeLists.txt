# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#
# Declare the SCPI serial commands library
#
add_library(scpi
	scpi_commands/scpi_commands.c
	scpi_commands/scpi_parser.c)
target_include_directories(scpi PUBLIC scpi_commands)
target_compile_definitions(scpi PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(scpi SYSTEM PRIVATE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(scpi PRIVATE ${WARNING_FLAGS})
target_link_libraries(scpi PRIVATE
	tasks
	utils
)

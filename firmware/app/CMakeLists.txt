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
target_compile_options(scpi PRIVATE ${WARNING_FLAGS})
target_link_libraries(scpi PRIVATE
	tasks
	utils
)

add_library(sensorfusion
	sensor_fusion/sensorfusion.c
)
target_include_directories(sensorfusion PUBLIC sensor_fusion)
target_compile_options(sensorfusion PRIVATE ${WARNING_FLAGS})

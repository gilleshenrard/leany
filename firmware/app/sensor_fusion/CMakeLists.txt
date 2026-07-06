# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

add_library(sensorfusion
	mahony.c
)
target_include_directories(sensorfusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(sensorfusion PRIVATE ${WARNING_FLAGS})
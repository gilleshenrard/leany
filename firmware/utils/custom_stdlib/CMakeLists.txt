# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#create the custom standard library, reimplementing standard C functions
add_library(custom_stdlib
	custom_string.c
	custom_stringparser.c)
target_include_directories(custom_stdlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(custom_stdlib PRIVATE ${WARNING_FLAGS})

# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
# SPDX-License-Identifier: MIT

set(CMAKE_SYSTEM_PROCESSOR          GNU)
set(CMAKE_C_COMPILER_ID GNU)

# Some default GCC settings
set(CMAKE_C_COMPILER    gcc   CACHE FILEPATH "")
set(CMAKE_AR            gcc-ar     CACHE FILEPATH "")
set(CMAKE_RANLIB        gcc-ranlib CACHE FILEPATH "")
set(CMAKE_OBJCOPY       objcopy CACHE FILEPATH "")
set(CMAKE_SIZE          size    CACHE FILEPATH "")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g3 -ggdb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")

include(${CMAKE_CURRENT_LIST_DIR}/gcc-warnings.cmake)

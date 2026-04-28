# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          GNU)
set(CMAKE_C_COMPILER_ID GNU)

# Some default GCC settings
set(CMAKE_C_COMPILER   gcc   CACHE FILEPATH "")
set(CMAKE_CXX_COMPILER g++   CACHE FILEPATH "")
set(CMAKE_ASM_COMPILER gcc   CACHE FILEPATH "")

# --- LTO-safe archive tools ---
set(CMAKE_AR     gcc-ar     CACHE FILEPATH "")
set(CMAKE_RANLIB gcc-ranlib CACHE FILEPATH "")

set(CMAKE_OBJCOPY objcopy CACHE FILEPATH "")
set(CMAKE_SIZE    size    CACHE FILEPATH "")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g3 -ggdb")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

#declare GCC-specific warning flags
set(WARNING_FLAGS
	-Wall
	-Wextra
	-Werror
	-pedantic
	-pedantic-errors
	-Waggressive-loop-optimizations
	-Wbad-function-cast
	-Wbuiltin-macro-redefined
	-Wdate-time
	-Wdisabled-optimization
	-Wdiscarded-array-qualifiers
	-Wdiscarded-qualifiers
	-Wdiv-by-zero
	-Wduplicated-branches
	-Wduplicated-cond
	-Wfloat-equal
	-Wignored-attributes
	-Winline
	-Winvalid-memory-model
	-Winvalid-pch
	-Wjump-misses-init
	-Wlogical-op
	-Wlogical-not-parentheses
	-Wmissing-declarations
	-Wmissing-include-dirs
	-Wnested-externs
	-Wnormalized=nfc
	-Wnull-dereference
	-Wredundant-decls
	-Wtrampolines
	-Wunsuffixed-float-constants
	-Wswitch-default
	-Wswitch-unreachable
	-Wswitch-enum
	-Wconversion
	-Wshadow
	-Wformat=2
	-Wformat-truncation
	-Wformat-signedness
	-Wundef
	-fno-common
	-Wdouble-promotion	# only on 32-bits microcontrollers
	-fstack-usage
	-fanalyzer
	-ffast-math
)

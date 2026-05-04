# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Setup compiler settings
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc   CACHE FILEPATH "")
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++   CACHE FILEPATH "")
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc   CACHE FILEPATH "")

# --- LTO-safe archive tools ---
set(CMAKE_AR     ${TOOLCHAIN_PREFIX}gcc-ar     CACHE FILEPATH "")
set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}gcc-ranlib CACHE FILEPATH "")

set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy CACHE FILEPATH "")
set(CMAKE_SIZE    ${TOOLCHAIN_PREFIX}size    CACHE FILEPATH "")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(ARM_FLAGS "-march=armv7-m -mcpu=cortex-m3 -mthumb")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g3 -ggdb")
else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g1")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ARM_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

#declare GCC-specific warning flags
set(WARNING_FLAGS
# --- Core error policy ---
-Wall
-Wextra
-Werror
-pedantic
-pedantic-errors
-fmax-errors=0

# --- Flow and logic ---
-Waggressive-loop-optimizations
-Wduplicated-branches
-Wduplicated-cond
-Wjump-misses-init
-Wlogical-op
-Wlogical-not-parentheses
-Wswitch-default
-Wswitch-enum
-Wswitch-unreachable

# --- Arithmetic and conversion ---
-Wfloat-equal
-Wdouble-promotion                # 32-bit MCU only: implicit float->double promotion
-Wconversion
#-Warith-conversion                # narrowing in arithmetic expressions, not caught by -Wconversion
-Wunsuffixed-float-constants

# --- Pointers and casts ---
-Wbad-function-cast
-Wcast-qual                       # cast removes const/volatile qualifier
-Wcast-align=strict               # pointer cast increases alignment requirement
-Wpointer-arith                   # arithmetic on void* or function pointers
-Wnull-dereference
-Wdiscarded-array-qualifiers
-Wdiscarded-qualifiers

# --- Stack and memory (embedded) ---
-Walloca                          # prohibit alloca (NASA rule 10 equivalent)
-Wvla                             # prohibit variable-length arrays (NASA rule 10)
-fstack-usage                     # emit .su files for stack analysis by STM32CubeMX

# --- Declarations and prototypes ---
-Wmissing-declarations
-Wmissing-prototypes
-Wstrict-prototypes
-Wredundant-decls
-Wnested-externs

# --- Preprocessor and macros ---
-Wbuiltin-macro-redefined
-Wdate-time
-Wundef

# --- Format strings ---
-Wformat=2
-Wformat-truncation=2
-Wformat-signedness

# --- Attributes and inlining ---
-Wignored-attributes
-Winline
-Wtrampolines

# --- Miscellaneous ---
-Wdisabled-optimization
-Winvalid-memory-model
-Winvalid-pch
-Wmissing-include-dirs
-Wnormalized=nfc
-Wshadow
-Wdiv-by-zero
-Wbidi-chars=any

# --- Analysis ---
-fanalyzer

# --- Optimisation ---
-ffast-math						# disables strict IEEE 754 — verify against sensorfusion if behaviour differs

# --- Linker ---
-fno-common
)

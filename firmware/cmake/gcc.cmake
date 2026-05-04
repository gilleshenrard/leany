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
-Warith-conversion                # narrowing in arithmetic expressions, not caught by -Wconversion
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

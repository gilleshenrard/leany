set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Setup compiler settings
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)
set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}ld)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(ARM_FLAGS "-march=armv7-m -mcpu=cortex-m3 -mthumb")

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
	# $<$<CONFIG:Debug>:-fanalyzer>
	$<$<CONFIG:Debug>:-fstack-usage>
)
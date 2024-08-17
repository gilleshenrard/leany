##############################################################################################
# brief: User-defined modules CMakeLists file
# date:  17/08/2024
##############################################################################################
#declare warning flags
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
	$<$<CONFIG:Debug>:-fanalyzer>
	$<$<CONFIG:Debug>:-fstack-usage>
)

#make sure STM32CubeMX library knows about sysutils/ (to use it in interrupts, ...)
target_include_directories(stm32cubemx INTERFACE sysutils/)

#create the sysUtils library, taking care of the error and application tick management
#	STM32CubeMX includes and definitions are manually added instead of linking the library
#	to avoid re-compiling STMCube libraries and optimising compilation
add_library(sysUtils
	sysutils/errorstack.c
	sysutils/systick.c)
target_include_directories(sysUtils PUBLIC sysutils/)
target_compile_definitions(sysUtils PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(sysUtils SYSTEM INTERFACE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(sysUtils PUBLIC ${WARNING_FLAGS})

#create the LSM6DSO library, taking care of the MEMS sensor
add_library(lsm6dso
	sensor/LSM6DSO.c)
target_include_directories(lsm6dso PUBLIC sensor/)
target_link_libraries(lsm6dso PRIVATE sysUtils)

#create the ssd1306 library, taking care of the display
add_library(ssd1306
	display/SSD1306.c
	display/numbersVerdana16.c
	display/icons.c)
target_include_directories(ssd1306 PUBLIC display)
target_link_libraries(ssd1306 PRIVATE sysUtils)

#create the buttons library, taking care of the control buttons
add_library(buttons
	buttons/buttons.c)
target_include_directories(buttons PUBLIC buttons)
target_link_libraries(buttons PRIVATE sysUtils)

##############################################################################################
# file:  CMakeLists.txt
# date:  30/05/2024
# brief: C modules CMakeLists file
##############################################################################################
#declare warning flags
set(WARNING_FLAGS
	-Wall
	-Wextra
	-Werror
	-pedantic
	-pedantic-errors
	-Wmissing-include-dirs
	-Wswitch-default
	-Wswitch-enum
	-Wconversion
	$<$<CONFIG:Debug>:-fanalyzer>
)

#make sure STM32CubeMX library knows about Inc/errors/
target_include_directories(stm32cubemx INTERFACE Inc/errors)

#create the errorStack library, taking care of the error management
#	STM32CubeMX includes and definitions are manually added instead of linking the library
#	to avoid re-compiling STMCube libraries and optimising compilation
add_library(errorStack
	Src/errors/errorstack.c)
target_include_directories(errorStack PUBLIC Inc/errors)
target_compile_definitions(errorStack PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_COMPILE_DEFINITIONS>)
target_include_directories(errorStack SYSTEM INTERFACE $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(errorStack PUBLIC ${WARNING_FLAGS})

#create the LSM6DSO library, taking care of the MEMS sensor
add_library(lsm6dso
	Src/hardware/sensor/LSM6DSO.c
	Src/hardware/sensor/LSM6DSO_config_script.c)
target_include_directories(lsm6dso PUBLIC Inc/hardware/sensor)
target_link_libraries(lsm6dso PRIVATE errorStack)

#create the ssd1306 library, taking care of the display
add_library(ssd1306
	Src/hardware/display/SSD1306.c
	Src/hardware/display/numbersVerdana16.c
	Src/hardware/display/icons.c)
target_include_directories(ssd1306 PUBLIC Inc/hardware/display)
target_link_libraries(ssd1306 PRIVATE errorStack)

#create the buttons library, taking care of the control buttons
add_library(buttons
	Src/hardware/buttons/buttons.c)
target_include_directories(buttons PUBLIC Inc/hardware/buttons)
target_link_libraries(buttons PRIVATE errorStack)

#############################################################################################################################
# file:  CMakeLists.txt
# date:  25/04/2024
# brief: C modules CMakeLists file
#############################################################################################################################
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

#create the errorStack library, taking care of the error management
add_library(errorStack Src/errors/errorstack.c)
target_compile_definitions (errorStack PUBLIC ${PROJECT_DEFINES})
target_include_directories(errorStack PUBLIC ${PROJECT_INCLUDES})
target_compile_options(errorStack PUBLIC ${CUSTOM_COMPILE_OPTIONS} ${WARNING_FLAGS})
target_link_options(errorStack PUBLIC ${CUSTOM_LINK_OPTIONS})

#create the LSM6DSO library, taking care of the MEMS sensor
add_library(lsm6dso Src/hardware/sensor/LSM6DSO.c Src/hardware/sensor/LSM6DSO_config_script.c)
target_include_directories(lsm6dso AFTER PUBLIC Inc/hardware/sensor)
target_link_libraries(lsm6dso PRIVATE errorStack)

#create the ssd1306 library, taking care of the display
add_library(ssd1306 Src/hardware/display/SSD1306.c Src/hardware/display/numbersVerdana16.c Src/hardware/display/icons.c)
target_include_directories(ssd1306 AFTER PUBLIC Inc/hardware/display)
target_link_libraries(ssd1306 PRIVATE errorStack)

#create the buttons library, taking care of the control buttons
add_library(buttons Src/hardware/buttons/buttons.c)
target_include_directories(buttons AFTER PUBLIC Inc/hardware/buttons)
target_link_libraries(buttons PRIVATE errorStack)

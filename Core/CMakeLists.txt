#############################################################################################################################
# file:  CMakeLists.txt
# date:  15/02/2024
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

#create the adxl345 library, taking care of the accelerometer
add_library(adxl345 Src/hardware/accelerometer/ADXL345.c)
target_include_directories(adxl345 AFTER PUBLIC Inc/hardware/accelerometer)
target_link_libraries(adxl345 PRIVATE errorStack)

#create the ssd1306 library, taking care of the screen
add_library(ssd1306 Src/hardware/screen/SSD1306.c Src/hardware/screen/numbersVerdana16.c Src/hardware/screen/icons16.c)
target_include_directories(ssd1306 AFTER PUBLIC Inc/hardware/screen)
target_link_libraries(ssd1306 PRIVATE errorStack)

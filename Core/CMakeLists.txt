#############################################################################################################################
# file:  CMakeLists.txt
# date:  04/01/2023
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
target_compile_definitions (errorStack PRIVATE ${PROJECT_DEFINES})
target_include_directories(errorStack PRIVATE Inc/errors)
target_compile_options(errorStack PRIVATE ${CUSTOM_COMPILE_OPTIONS} ${WARNING_FLAGS})
target_link_options(errorStack PRIVATE ${CUSTOM_LINK_OPTIONS})

#create the adxl345 library, taking care of the accelerometer
add_library(adxl345 Src/hardware/accelerometer/ADXL345.c)
target_compile_definitions (adxl345 PRIVATE ${PROJECT_DEFINES})
target_include_directories(adxl345 PRIVATE ${PROJECT_INCLUDES})
target_compile_options(adxl345 PRIVATE ${CUSTOM_COMPILE_OPTIONS} ${WARNING_FLAGS})
target_link_options(adxl345 PRIVATE ${CUSTOM_LINK_OPTIONS})
target_link_libraries(adxl345 PRIVATE errorStack)

#create the ssd1306 library, taking care of the screen
add_library(ssd1306 Src/hardware/screen/SSD1306.c Src/hardware/screen/numbersVerdana16.c)
target_compile_definitions (ssd1306 PRIVATE ${PROJECT_DEFINES})
target_include_directories(ssd1306 PRIVATE ${PROJECT_INCLUDES})
target_compile_options(ssd1306 PRIVATE ${CUSTOM_COMPILE_OPTIONS} ${WARNING_FLAGS})
target_link_options(ssd1306 PRIVATE ${CUSTOM_LINK_OPTIONS})
target_link_libraries(ssd1306 PRIVATE errorStack)
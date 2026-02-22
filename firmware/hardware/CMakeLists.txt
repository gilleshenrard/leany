# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

add_subdirectory(imu)

#
# Declare the HAL library
#
add_library(hal
	hal/hal_spi.c
	hal/hal_adc.c
	hal/hal_i2c.c
)
target_include_directories(hal PUBLIC hal)
target_link_libraries(hal PRIVATE utils)

#
# declare the ST7735 library, taking care of the display
#
add_library(st7735s
	display/st7735_registers.inc
	display/st7735s.c
	display/st7735_initialisation.c)
target_include_directories(st7735s PUBLIC display)
target_include_directories(st7735s PUBLIC imu/)
target_link_libraries(st7735s PRIVATE utils)
target_link_libraries(st7735s PRIVATE hal)

#create the buttons library, taking care of the control buttons
add_library(gpio
	gpio/buttons.c
	gpio/led.c)
target_include_directories(gpio PUBLIC gpio)
target_link_libraries(gpio PRIVATE
	utils
	serial
)

#
# Declare the HAL library
#
add_library(BATTERY
	battery/bq25619.c
)
target_include_directories(BATTERY PUBLIC battery)
target_link_libraries(BATTERY PRIVATE utils)
target_link_libraries(BATTERY PRIVATE hal)

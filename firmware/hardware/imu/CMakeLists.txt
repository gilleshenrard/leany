# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#make sure STM32CubeMX library knows about sysutils/ (to use it in interrupts, ...)
target_include_directories(stm32cubemx INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#
#declare the IMU library
#
if(NOT IMU_MODEL)
	set(IMU_MODEL "LSM6")
endif()

#depending on the IMU model chosen, set source files and external libraries to use
if(IMU_MODEL STREQUAL "BMI270")
	include(${CMAKE_SOURCE_DIR}/cmake/bosch-sensortec/CMakeLists.txt)
	set(IMUSRC bmi270/mems_bmi270.c)
	set(IMULIB bmi270_api)
elseif(IMU_MODEL STREQUAL "LSM6")
	set(IMUSRC lsm6/lsm6.c)
endif()

#declare the library
add_library(IMU
	imu.c
	${IMUSRC}
	sensorfusion.c)
target_include_directories(IMU PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IMU PRIVATE sysUtils)
target_link_libraries(IMU PRIVATE hal)
target_link_libraries(IMU PRIVATE st7735s)
target_link_libraries(IMU PRIVATE serial)
target_link_libraries(IMU PUBLIC ${IMULIB})

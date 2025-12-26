# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#declare CMake variables and preprocessor definitions for each IMU model
set(IMU_LSM6 1)
set(IMU_BMI270 2)
add_compile_definitions(
	IMU_LSM6=${IMU_LSM6}
	IMU_BMI270=${IMU_BMI270}
)

#set the IMU_MODEL preprocessor definition depending on the model defined in presets
if((NOT DEFINED IMU_MODEL) OR (IMU_MODEL STREQUAL "LSM6"))
    set(IMU_MODEL ${IMU_LSM6})
    add_compile_definitions(IMU_MODEL=IMU_LSM6)
else()
    set(IMU_MODEL ${IMU_BMI270})
    add_compile_definitions(IMU_MODEL=IMU_BMI270)
endif()

#depending on the IMU model chosen, set source files and external libraries to use
if(IMU_MODEL EQUAL IMU_BMI270)
	include(${CMAKE_SOURCE_DIR}/cmake/bosch-sensortec/CMakeLists.txt)
	set(IMUSRC bmi270/mems_bmi270.c)
	set(IMULIB bmi270_api)
elseif(IMU_MODEL EQUAL IMU_LSM6)
	set(IMUSRC lsm6/lsm6.c)
endif()

#declare the library
add_library(IMU
	${IMUSRC}
	sensorfusion.c)
target_include_directories(IMU PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IMU PRIVATE
	utils
	hal
	${IMULIB}
)

# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.22)

# Core project settings
project(UnitTests VERSION 0.1.0)

include(gcc.cmake)

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          GNU)

# Setup compiler settings
set(CMAKE_C_COMPILER_ID GNU)

add_library(Unity
    ${CMAKE_SOURCE_DIR}/../Middlewares/Third_Party/Unity/src/unity.c
)
target_include_directories(Unity PUBLIC
    ${CMAKE_SOURCE_DIR}/../Middlewares/Third_Party/Unity/src
)

# tests/CMakeLists.txt
add_executable(${CMAKE_PROJECT_NAME}
    test_sensorfusion.c
    ${CMAKE_SOURCE_DIR}/../hardware/imu/sensorfusion.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/../hardware/imu/
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Unity)
target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC ${WARNING_FLAGS})

enable_testing()
add_test(NAME sensorfusion_tests COMMAND ${CMAKE_PROJECT_NAME})
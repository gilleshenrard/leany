# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.22)

# Core project settings
project(Leany VERSION 0.1.0)

# Compiler settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Populate the CMake template files
execute_process(
    COMMAND ${CMAKE_COMMAND}
        -DPROJECT_VERSION=${PROJECT_VERSION}
        -DPROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DPROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}
        -P fill_templates.cmake
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/cmake/templates/
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx SYSTEM)
add_subdirectory(hardware)
add_subdirectory(ui)
add_subdirectory(utils)
add_subdirectory(tasks)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    IMU
    gpio
    serial
    hal
    tasks
)

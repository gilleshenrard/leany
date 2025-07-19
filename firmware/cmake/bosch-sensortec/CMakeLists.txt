# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

set(API_DIR ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/BMI270_SensorAPI)

add_library(bmi270_api
    ${API_DIR}/bmi2.c
    ${API_DIR}/bmi270.c)
target_include_directories(bmi270_api PUBLIC
    ${API_DIR}
)
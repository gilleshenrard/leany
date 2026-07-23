# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT


######################################################################################
# Unity unit-tests library
######################################################################################

add_library(Unity
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/Unity/src/unity.c
)
target_include_directories(Unity PUBLIC
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/Unity/src
)

######################################################################################
# tests/test_sensorfusion
######################################################################################

add_executable(test_sensorfusion
    test_sensorfusion.c
)

target_link_libraries(test_sensorfusion PRIVATE
    sensorfusion
    Unity
    m
)
target_compile_options(test_sensorfusion PRIVATE ${WARNING_FLAGS})

enable_testing()
add_test(NAME sensorfusion_tests COMMAND test_sensorfusion)


######################################################################################
# tests/test_stdlib
######################################################################################

add_executable(test_stdlib
    test_stdlib.c
)

target_link_libraries(test_stdlib PRIVATE
    custom_stdlib
    Unity
    m
)
target_compile_options(test_stdlib PRIVATE ${WARNING_FLAGS})

enable_testing()
add_test(NAME stdlib_tests COMMAND test_stdlib)


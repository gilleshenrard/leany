# tests/CMakeLists.txt
add_executable(test_sensorfusion
    test_sensorfusion.c
    ${CMAKE_SOURCE_DIR}/hardware/imu/sensorfusion.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/Unity/src/unity.c
)

target_include_directories(test_sensorfusion PRIVATE
    ${CMAKE_SOURCE_DIR}/hardware/imu/
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/Unity/src
)

#target_compile_options(test_sensorfusion PUBLIC ${WARNING_FLAGS})
target_compile_options(test_sensorfusion PRIVATE -Wall -Wextra -Werror -lm)

add_test(NAME sensorfusion_tests COMMAND test_sensorfusion)
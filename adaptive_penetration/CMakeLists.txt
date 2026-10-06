cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(adaptive_penetration LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    include(CTest)
endif()

add_library(adaptive_penetration_core
    src/axial_force_estimator.cpp
    src/adaptive_penetration_controller.cpp
)
add_library(AdaptivePenetration::core ALIAS adaptive_penetration_core)
target_include_directories(adaptive_penetration_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(adaptive_penetration_one_step examples/one_step_example.cpp)
target_link_libraries(adaptive_penetration_one_step PRIVATE AdaptivePenetration::core)

add_executable(adaptive_penetration_cycle_example examples/controller_cycle_example.cpp)
target_link_libraries(adaptive_penetration_cycle_example PRIVATE AdaptivePenetration::core)

if(BUILD_TESTING)
    add_executable(adaptive_penetration_unit_tests tests/test_adaptive_penetration.cpp)
    target_link_libraries(adaptive_penetration_unit_tests PRIVATE AdaptivePenetration::core)

    add_test(NAME adaptive_penetration_unit_tests COMMAND adaptive_penetration_unit_tests)
    add_test(NAME adaptive_penetration_cycle_example COMMAND adaptive_penetration_cycle_example)

    find_program(PYTHON3_EXECUTABLE python3)
    if(PYTHON3_EXECUTABLE)
        add_test(
            NAME adaptive_penetration_calibration_tests
            COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/calibration/test_calibration_tool.py
        )
    endif()
endif()

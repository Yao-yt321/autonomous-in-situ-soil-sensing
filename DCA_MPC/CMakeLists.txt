cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(DCA_MPC LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    include(CTest)
endif()

add_library(dca_mpc_core
    src/vehicle_model.cpp
    src/reference_path.cpp
    src/dca_mpc_scheduler.cpp
    src/dense_qp.cpp
    src/ltv_mpc.cpp
)
add_library(DCA_MPC::core ALIAS dca_mpc_core)
target_include_directories(dca_mpc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(dca_mpc_one_step examples/one_step_example.cpp)
target_link_libraries(dca_mpc_one_step PRIVATE DCA_MPC::core)

add_executable(dca_mpc_closed_loop examples/closed_loop_sim.cpp)
target_link_libraries(dca_mpc_closed_loop PRIVATE DCA_MPC::core)

add_executable(dca_mpc_benchmark examples/benchmark.cpp)
target_link_libraries(dca_mpc_benchmark PRIVATE DCA_MPC::core)

add_executable(dca_mpc_scenario_sweep examples/scenario_sweep.cpp)
target_link_libraries(dca_mpc_scenario_sweep PRIVATE DCA_MPC::core)

if(BUILD_TESTING)
    add_executable(dca_mpc_unit_tests tests/test_dca_mpc.cpp)
    target_link_libraries(dca_mpc_unit_tests PRIVATE DCA_MPC::core)

    add_test(NAME dca_mpc_unit_tests COMMAND dca_mpc_unit_tests)
    add_test(NAME dca_mpc_scenario_sweep COMMAND dca_mpc_scenario_sweep)
    add_test(
        NAME dca_mpc_closed_loop_smoke
        COMMAND dca_mpc_closed_loop ${CMAKE_CURRENT_BINARY_DIR}/closed_loop_trace.csv
    )
endif()

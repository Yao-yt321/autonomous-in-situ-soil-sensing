cmake_minimum_required(VERSION 3.10)
project(autonomous_in_situ_soil_sensing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_subdirectory(DCA_MPC)
add_subdirectory(adaptive_penetration)

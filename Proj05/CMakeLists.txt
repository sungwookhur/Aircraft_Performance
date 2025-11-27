cmake_minimum_required(VERSION 3.10)
project(SixDofAircraft VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(aircraft_sim
    src/main.cpp
    src/dynamics/SixDofEOM.cpp
    src/model/Aircraft.cpp
    src/model/Wing.cpp
    src/model/Tail.cpp
    src/model/Fuselage.cpp
    src/sim/Simulation.cpp
)

target_include_directories(aircraft_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
cmake_minimum_required(VERSION 3.10)
project(proj01 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
file(GLOB SOURCES "${SRC_DIR}/*.cpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(proj01 ${SOURCES})
target_include_directories(proj01 PRIVATE ${SRC_DIR})

message(STATUS "Source dir: ${SRC_DIR}")
message(STATUS "Found sources: ${SOURCES}")

cmake_minimum_required(VERSION 3.10)
project(proj01 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(app_lib
    src/class.cpp
)
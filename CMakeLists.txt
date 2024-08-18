cmake_minimum_required(VERSION 3.24)
project(disassemble-8086 
    VERSION 0.1 
    LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

file(GLOB SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/*.h"
    "${PROJECT_SOURCE_DIR}/*.cpp")

add_executable(disassemble-8086 ${SOURCE_FILES})

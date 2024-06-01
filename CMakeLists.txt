cmake_minimum_required(VERSION 3.24)
project(disassemble-8086)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(disassemble-8086 main.cpp peeking_iterator.h instruction.h types.h parse.h stringify.h)

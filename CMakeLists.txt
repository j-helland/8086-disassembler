cmake_minimum_required(VERSION 3.24)
project(disassemble-8086)

set(CMAKE_CXX_STANDARD 20)

add_executable(disassemble-8086 main.cpp peeking_iterator.h instruction.h types.h parse.h stringify.h)

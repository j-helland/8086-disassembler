cmake_minimum_required(VERSION 3.24)
project(disassemble-8086)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(disassemble-8086 
    main.cpp 
    types.h 
    peeking_iterator.h 
    instruction.h instruction.cpp
    parse.h parse.cpp 
    stringify.h stringify.cpp)

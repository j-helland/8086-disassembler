cmake_minimum_required(VERSION 3.24)
project(perfaware)

set(CMAKE_CXX_STANDARD 20)

add_executable(perfaware main.cpp peeking_iterator.h)

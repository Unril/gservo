cmake_minimum_required(VERSION 3.9)
project(gservo)

set(CMAKE_CXX_STANDARD 11)

add_executable(gservotest library.h catch.hpp main.cpp tests.cpp)

cmake_minimum_required(VERSION 3.15...3.28)
project(ST3215)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)

option(ST3215_BUILD_TESTS "Build tests" OFF)

add_subdirectory(src)

if(ST3215_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

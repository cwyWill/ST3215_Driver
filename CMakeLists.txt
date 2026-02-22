cmake_minimum_required(VERSION 3.15...3.28)
project(ST3215 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src)
option(ST3215_BUILD_TESTS "Build tests" OFF)

if(ST3215_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

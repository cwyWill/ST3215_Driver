cmake_minimum_required(VERSION 3.15...3.28)
project(MyProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

add_subdirectory(ST3215_Driver)
# add_subdirectory(src)
add_subdirectory(test)

cmake_minimum_required(VERSION 3.15...3.28)
project(ST3215_Driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)


find_package(Boost REQUIRED COMPONENTS system)
# target_link_libraries(ST3215_Driver Boost::system)

add_library(ST3215_Driver
    SCS.cpp
    SCSerial.cpp
    SMS_STS.cpp
)

target_link_libraries(ST3215_Driver
    PUBLIC Boost::system
)

target_include_directories(ST3215_Driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

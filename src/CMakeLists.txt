cmake_minimum_required(VERSION 3.15...3.28)
project(ST3215)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# target_compile_options(ST3215_Driver PRIVATE -Wall -Wextra -Wpedantic)

find_package(Boost REQUIRED COMPONENTS system)
# target_link_libraries(ST3215_Driver Boost::system)

add_library(st3215_driver
    SCS.cpp
    SCSerial.cpp
    SMS_STS.cpp
    Actuator_Handler.h
)

target_compile_options(st3215_driver PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(st3215_driver
    PUBLIC Boost::system
)

target_include_directories(st3215_driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    # PUBLIC
    #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    #     $<INSTALL_INTERFACE:include>
)

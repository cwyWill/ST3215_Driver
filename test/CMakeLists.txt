cmake_minimum_required(VERSION 3.15...3.28)

add_executable(test_driver
    test_driver.cpp
)

target_link_libraries(test_driver
    PRIVATE ST3215
)

find_package(Boost REQUIRED COMPONENTS system)


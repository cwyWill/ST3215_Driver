cmake_minimum_required(VERSION 3.15...3.28)

add_executable(test_driver
    test_driver.cpp
)

target_link_libraries(test_driver
    PRIVATE st3215_driver
)

add_executable(test_driver_yaml
    test_driver_yaml.cpp
)

target_link_libraries(test_driver_yaml
    PRIVATE st3215_driver
)
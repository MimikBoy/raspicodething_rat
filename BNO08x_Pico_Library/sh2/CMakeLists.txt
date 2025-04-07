add_library(sh2 STATIC
        src/sh2.c
        src/shtp.c
        src/sh2_util.c
        src/sh2_SensorValue.c
)
target_include_directories(sh2 PUBLIC
        include
)
cmake_minimum_required(VERSION 3.13)

# Set the project name and language
project(BNO08x_Test_Project C CXX ASM)

# Pull in the Pico SDK (make sure pico_sdk_import.cmake is correctly referenced)
# You might have your Pico SDK path set up or use the provided pico_sdk_import.cmake.
include(pico_sdk_import.cmake)

# Initialize the Pico SDK
pico_sdk_init()

# Add the BNO08x_Pico_Library as a subdirectory
add_subdirectory(BNO08x_Pico_Library)

# Add your test executable
add_executable(test_sensor
    main.cpp
)

# Link against the Pico standard library, hardware I2C, and the BNO08x library
target_link_libraries(test_sensor
    pico_stdlib
    hardware_i2c
    BNO08x_Pico_Library
)

# Create map/bin/uf2 files
pico_add_extra_outputs(test_sensor)
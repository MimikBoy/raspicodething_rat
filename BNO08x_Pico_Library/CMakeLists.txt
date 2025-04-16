add_library(BNO08x_Pico_Library STATIC
        src/bno08x.cpp
        src/utils.cpp
        src/bno08x2.cpp
        src/utils2.cpp
)
add_subdirectory(
	sh2
)
target_link_libraries(BNO08x_Pico_Library
        pico_stdlib 
        hardware_i2c
        sh2
)
target_include_directories(BNO08x_Pico_Library PUBLIC
        include
)

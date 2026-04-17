cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(HW5_mpu6050_oled C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(HW5_mpu6050_oled
    HW5.c
    ssd1306.c
)

target_link_libraries(HW5_mpu6050_oled
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(HW5_mpu6050_oled 1)
pico_enable_stdio_uart(HW5_mpu6050_oled 0)

pico_add_extra_outputs(HW5_mpu6050_oled)


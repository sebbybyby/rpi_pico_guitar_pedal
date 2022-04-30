 cmake_minimum_required(VERSION 3.12)
 
 set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
 include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
 
 pico_sdk_init()
 
 project(digital_pedal C CXX ASM)
 set(CMAKE_C_STANDARD 11)
 set(CMAKE_CXX_STANDARD 17)
 
 add_executable(main main.cpp)
 
 target_link_libraries(main pico_stdlib)
 target_link_libraries(main pico_stdlib hardware_adc)
 target_link_libraries(main pico_stdlib hardware_pwm)
 target_link_libraries(main pico_stdlib hardware_i2c)

 add_subdirectory(pico-ssd1306)
 target_link_libraries(main pico_ssd1306)

 pico_enable_stdio_usb(main 1)
 pico_enable_stdio_uart(main 0)
 
 pico_add_extra_outputs(main)

cmake_minimum_required(VERSION 3.9)
project(gservo)

set(CMAKE_CXX_STANDARD 11)

include_directories(
        C:/Users/unril/Documents/Arduino/libraries/ardyno/src
        C:/Users/unril/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.6.20/cores/arduino
        C:/Users/unril/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.6.20/variants/mega
        C:/Users/unril/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.6.20/libraries/SoftwareSerial/src
        C:/Users/unril/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.6.20/libraries/EEPROM/src
)

add_definitions(-DARDUINO)

add_executable(gservotest gservo.h tests/catch.hpp tests/main.cpp tests/tests.cpp parser.h)
target_compile_options(gservotest PRIVATE --std=c++11 -Wall -Wextra -Wunreachable-code -O0 -fuse-ld=gold -Wl,--disable-new-dtags -pipe -DCATCH_CONFIG_FAST_COMPILE)

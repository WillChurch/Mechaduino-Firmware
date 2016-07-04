#
# Using arduino-cmake, with intellij plugin.

cmake_minimum_required(VERSION 2.8.4)


register_hardware_platform(/home/marco/.arduino15/packages/arduino/hardware/samd/1.6.6)


print_board_list()
set(${CMAKE_PROJECT_NAME}_BOARD arduino_zero_native)
#set(${CMAKE_PROJECT_NAME}_SKETCH Mechaduino.ino)
generate_arduino_firmware(
        ${CMAKE_PROJECT_NAME}
        SRCS Mechaduino.ino
             Mechaduino_functions.cpp
             Mechaduino_functions.h
)

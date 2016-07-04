#
# Using arduino-cmake, with intellij plugin.

cmake_minimum_required(VERSION 2.8.4)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME Mechaduino)
project(${PROJECT_NAME})

add_subdirectory("Mechaduino")
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(diff_swerve VERSION 1.0)

add_library(minipid STATIC libs/MiniPID.cpp libs/MiniPID.h)
include_directories(minipid PUBLIC libs)
find_library(PIGPIO pigpio)
find_library(RT rt)
find_library(EVDEV evdev)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(output ${SRC_FILES})
target_link_libraries(output PUBLIC minipid ${PIGPIO} ${RT} ${EVDEV})

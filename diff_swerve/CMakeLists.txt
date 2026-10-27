cmake_minimum_required(VERSION 3.10)
project(diff_swerve VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(minipid STATIC libs/MiniPID.cpp)
target_include_directories(minipid PUBLIC libs)

if(APPLE)
    message(STATUS "Mock hardware definitions for macOS")
    add_compile_definitions(MACOS_BUILD)
else()
    message(STATUS "Building on Linux/Raspberry Pi: Linking hardware libraries")
    find_library(PIGPIO pigpio REQUIRED)
    find_library(RT rt REQUIRED)
    find_library(EVDEV evdev REQUIRED)
endif()

file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(output ${SRC_FILES})

target_include_directories(output PRIVATE src)

if(APPLE)
    target_include_directories(output PRIVATE mock)
    target_link_libraries(output PRIVATE minipid)
else()
    target_link_libraries(output PRIVATE minipid ${PIGPIO} ${RT} ${EVDEV})
endif()

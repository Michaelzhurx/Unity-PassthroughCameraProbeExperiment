cmake_minimum_required(VERSION 3.22)
project(apriltag_unity LANGUAGES C)

set(APRILTAG_SOURCE_DIR "" CACHE PATH "Path to the AprilRobotics/apriltag source tree.")

if(NOT APRILTAG_SOURCE_DIR)
    message(FATAL_ERROR "APRILTAG_SOURCE_DIR must point to the AprilRobotics/apriltag source tree.")
endif()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build AprilTag as a static library for the Unity wrapper." FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build AprilTag examples." FORCE)
set(ASAN OFF CACHE BOOL "Do not use ASAN for Android plugin builds." FORCE)

add_subdirectory("${APRILTAG_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/apriltag")

add_library(apriltag_unity SHARED apriltag_unity.c)

target_include_directories(apriltag_unity PRIVATE
    "${APRILTAG_SOURCE_DIR}"
    "${APRILTAG_SOURCE_DIR}/common")

target_link_libraries(apriltag_unity PRIVATE apriltag m)

target_compile_features(apriltag_unity PRIVATE c_std_99)

if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(apriltag_unity PRIVATE
        -Wall
        -Wextra
        -Wno-gnu-zero-variadic-macro-arguments)
endif()

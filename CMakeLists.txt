
# This file is a part of MRNIU/VLP-16 (https://github.com/MRNIU/VLP-16).
#
# CMakeLists.txt for MRNIU/VLP-16.

# Set minimum cmake version
cmake_minimum_required(VERSION 3.12)

project(VLP16 C)

# Set C gnu11
set(CMAKE_C_STANDARD 11)

# Set common flags
set(CMAKE_C_FLAGS_DEBUG "-Wall -Wextra")

set(LIBNAME vlp16)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(${VLP16_SOURCE_DIR} STREQUAL ${VLP16_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there.")
endif()

set(VLP16_SOURCE_CODE_DIR ${VLP16_SOURCE_DIR}/src)

add_subdirectory(${VLP16_SOURCE_CODE_DIR})
add_subdirectory(${VLP16_SOURCE_DIR}/test)

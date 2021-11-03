
# This file is a part of MRNIU/VLP-16 (https://github.com/MRNIU/VLP-16).
#
# CMakeLists.txt for MRNIU/VLP-16.

include_directories(${VLP16_SOURCE_DIR}/include)

set(src
    ${VLP16_SOURCE_CODE_DIR}/vlp16.cpp)

add_library(${LIBNAME} SHARED ${src})

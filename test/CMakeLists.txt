
# This file is a part of MRNIU/VLP-16 (https://github.com/MRNIU/VLP-16).
#
# CMakeLists.txt for MRNIU/VLP-16.

include_directories(${VLP16_SOURCE_DIR}/include)

aux_source_directory(${VLP16_SOURCE_DIR}/test src)

set(test_src
    test.c
    fifo.c
    time.c)

link_libraries(${LIBNAME})

add_executable(test ${test_src})

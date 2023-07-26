# ADAPTED FROM https://github.com/kctess5/range_libc/issues/1
cmake_minimum_required(VERSION 3.3.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

SET(BASEPATH "${CMAKE_SOURCE_DIR}")
add_definitions ( -DBASEPATH="${BASEPATH}" )

set(RANGELIB_DIR ${BASEPATH}/include/utils/range_libc)
#set(RANGELIB_INCLUDE_DIRS ${RANGELIB_DIR}/includes ${RANGELIB_DIR} ${RANGELIB_DIR}/vendor)
set(RANGELIB_INCLUDE_DIRS ${RANGELIB_DIR}/includes})
add_subdirectory(${RANGELIB_DIR}/vendor/gflags)
include_directories(${RANGELIB_INCLUDE_DIRS})

file(GLOB SRCS main.cpp ${RANGELIB_DIR}/vendor/lodepng/lodepng.cpp)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "-march=native -ffast-math -fno-math-errno  ${CMAKE_CXX_FLAGS}")

#include_directories("./")

set(CMAKE_BUILD_TYPE Release)

add_executable(particle_filter_rangelibc ${SRCS})

target_link_libraries(particle_filter_rangelibc gflags pthread)

add_dependencies(particle_filter_rangelibc
  ${catkin_EXPORTED_TARGETS}
)

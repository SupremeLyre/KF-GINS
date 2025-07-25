cmake_minimum_required(VERSION 3.10)
project(KF-GINS)

set(CMAKE_CXX_STANDARD 20)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions( -D_USE_MATH_DEFINES=True)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_CXX_FLAGS "-Wno-unused-result")
endif()

file(GLOB_RECURSE SOURCE
        src/kf_gins.cpp
        src/common/time.cc
        src/fileio/fileloader.cc
        src/fileio/filesaver.cc
        src/fileio/adisfileloader.cc
        src/fileio/pppfileloader.cc
        src/fileio/posfileloader.cc
        src/kf-gins/gi_engine.cpp
        src/kf-gins/insmech.cpp)

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(${PROJECT_NAME} ${SOURCE})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp-0.7.0
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

find_package(glog REQUIRED)
target_link_libraries(${PROJECT_NAME} glog::glog)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1)
target_link_libraries(${PROJECT_NAME} 
        absl::strings
        absl::str_format
        absl::time)

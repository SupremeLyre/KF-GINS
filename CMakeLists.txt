cmake_minimum_required(VERSION 3.21)
project(KF-GINS)

set(CMAKE_CXX_STANDARD 20)

set(KF_GINS_RUNTIME_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)
set(KF_GINS_LIBRARY_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${KF_GINS_RUNTIME_OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${KF_GINS_LIBRARY_OUTPUT_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${KF_GINS_LIBRARY_OUTPUT_DIR})

function(set_runtime_output_directory_recursively directory runtime_output_directory)
    get_property(targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(target_name IN LISTS targets)
        get_target_property(target_type ${target_name} TYPE)
        if(NOT target_type STREQUAL "INTERFACE_LIBRARY")
            set_target_properties(${target_name} PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY "${runtime_output_directory}"
                RUNTIME_OUTPUT_DIRECTORY_DEBUG "${runtime_output_directory}"
                RUNTIME_OUTPUT_DIRECTORY_RELEASE "${runtime_output_directory}"
                RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${runtime_output_directory}"
                RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${runtime_output_directory}")
        endif()
    endforeach()

    get_property(subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
    foreach(subdirectory IN LISTS subdirectories)
        set_runtime_output_directory_recursively("${subdirectory}" "${runtime_output_directory}")
    endforeach()
endfunction()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions(-D_USE_MATH_DEFINES=True)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_CXX_FLAGS "-Wno-unused-result")
endif()

file(GLOB_RECURSE SOURCE
    src/common/time.cc
    src/common/crc32.cc
    src/fileio/fileloader.cc
    src/fileio/filesaver.cc
    src/fileio/adisfileloader.cc
    src/fileio/pppfileloader.cc
    src/fileio/posfileloader.cc
    src/kf-gins/gi_engine_psi.cpp
    src/kf-gins/gi_engine.cpp
    src/kf-gins/insmech.cpp
    src/kf-gins/insmech_ecef.cpp)

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(${PROJECT_NAME} ${SOURCE} src/kf_gins.cpp)


# Eigen3
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(ThirdParty/eigen-3.4.0)

# yaml-cpp-0.7.0
# find_package(yaml-cpp REQUIRED)
add_subdirectory(ThirdParty/yaml-cpp-0.7.0)
set_runtime_output_directory_recursively("${PROJECT_SOURCE_DIR}/ThirdParty/yaml-cpp-0.7.0" "${KF_GINS_RUNTIME_OUTPUT_DIR}")
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${KF_GINS_RUNTIME_OUTPUT_DIR})
set_runtime_output_directory_recursively("${PROJECT_SOURCE_DIR}/ThirdParty/abseil-cpp-20220623.1" "${KF_GINS_RUNTIME_OUTPUT_DIR}")
target_link_libraries(${PROJECT_NAME}
    absl::strings
    absl::str_format
    absl::time)
add_executable(kf_gins_rt ${SOURCE} src/kf_gins_rt.cpp)
target_link_libraries(kf_gins_rt yaml-cpp absl::strings absl::str_format absl::time)

if(WIN32)
    if(MINGW)
        get_filename_component(MINGW_COMPILER_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
        foreach(runtime_dll IN ITEMS
            libgcc_s_seh-1.dll
            libgcc_s_sjlj-1.dll
            libgcc_s_dw2-1.dll
            libstdc++-6.dll
            libwinpthread-1.dll)
            find_file(runtime_dll_path NAMES ${runtime_dll} HINTS "${MINGW_COMPILER_BIN_DIR}" NO_DEFAULT_PATH)
            if(runtime_dll_path)
                list(APPEND MINGW_RUNTIME_DLLS "${runtime_dll_path}")
            endif()
            unset(runtime_dll_path CACHE)
        endforeach()
        if(MINGW_RUNTIME_DLLS)
            file(COPY ${MINGW_RUNTIME_DLLS} DESTINATION "${KF_GINS_RUNTIME_OUTPUT_DIR}")
        endif()
    endif()
endif()

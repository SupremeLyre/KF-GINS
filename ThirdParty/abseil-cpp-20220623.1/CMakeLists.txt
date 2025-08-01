#
# Copyright 2017 The Abseil Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Most widely used distributions have cmake 3.5 or greater available as of March
# 2019.  A notable exception is RHEL-7 (CentOS7).  You can install a current
# version of CMake by first installing Extra Packages for Enterprise Linux
# (https://fedoraproject.org/wiki/EPEL#Extra_Packages_for_Enterprise_Linux_.28EPEL.29)
# and then issuing `yum install cmake3` on the command line.
cmake_minimum_required(VERSION 3.5)

# Compiler id for Apple Clang is now AppleClang.
if (POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif (POLICY CMP0025)

# if command can use IN_LIST
if (POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif (POLICY CMP0057)

# Project version variables are the empty string if version is unspecified
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

# option() honor variables
if (POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif (POLICY CMP0077)

# Allow the user to specify the MSVC runtime
if (POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW)
endif (POLICY CMP0091)

project(absl LANGUAGES CXX VERSION 20220623)
include(CTest)

# Output directory is correct by default for most build setups. However, when
# building Abseil as a DLL, it is important to have the DLL in the same
# directory as the executable using it. Thus, we put all executables in a single
# /bin directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(BUILD_SHARED_LIBS ON)
# when absl is included as subproject (i.e. using add_subdirectory(abseil-cpp))
# in the source tree of a project that uses it, install rules are disabled.
if(NOT CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  option(ABSL_ENABLE_INSTALL "Enable install rule" OFF)
else()
  option(ABSL_ENABLE_INSTALL "Enable install rule" ON)
endif()

option(ABSL_PROPAGATE_CXX_STD
  "Use CMake C++ standard meta features (e.g. cxx_std_11) that propagate to targets that link to Abseil"
  OFF)  # TODO: Default to ON for CMake 3.8 and greater.
if((${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.8) AND (NOT ABSL_PROPAGATE_CXX_STD))
  message(WARNING "A future Abseil release will default ABSL_PROPAGATE_CXX_STD to ON for CMake 3.8 and up. We recommend enabling this option to ensure your project still builds correctly.")
endif()

list(APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_LIST_DIR}/CMake
  ${CMAKE_CURRENT_LIST_DIR}/absl/copts
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
include(AbseilDll)
include(AbseilHelpers)


##
## Using absl targets
##
## all public absl targets are
## exported with the absl:: prefix
##
## e.g absl::base absl::synchronization absl::strings ....
##
## DO NOT rely on the internal targets outside of the prefix


# include current path
list(APPEND ABSL_COMMON_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(ABSL_USING_CLANG ON)
else()
  set(ABSL_USING_CLANG OFF)
endif()

# find dependencies
## pthread
find_package(Threads REQUIRED)

include(CMakeDependentOption)

option(ABSL_BUILD_TESTING
  "If ON, Abseil will build all of Abseil's own tests." OFF)

option(ABSL_USE_EXTERNAL_GOOGLETEST
  "If ON, Abseil will assume that the targets for GoogleTest are already provided by the including project. This makes sense when Abseil is used with add_subdirectory." OFF)

cmake_dependent_option(ABSL_FIND_GOOGLETEST
  "If ON, Abseil will use find_package(GTest) rather than assuming that GoogleTest is already provided by the including project."
  ON
  "ABSL_USE_EXTERNAL_GOOGLETEST"
  OFF)


option(ABSL_USE_GOOGLETEST_HEAD
  "If ON, abseil will download HEAD from GoogleTest at config time." OFF)

set(ABSL_GOOGLETEST_DOWNLOAD_URL "" CACHE STRING "If set, download GoogleTest from this URL")

set(ABSL_LOCAL_GOOGLETEST_DIR "/usr/src/googletest" CACHE PATH
  "If ABSL_USE_GOOGLETEST_HEAD is OFF and ABSL_GOOGLETEST_URL is not set, specifies the directory of a local GoogleTest checkout."
  )

if(BUILD_TESTING AND ABSL_BUILD_TESTING)
  ## check targets
  if (ABSL_USE_EXTERNAL_GOOGLETEST)
    if (ABSL_FIND_GOOGLETEST)
      find_package(GTest REQUIRED)
    elseif(NOT TARGET GTest::gtest)
      if(TARGET gtest)
        # When Google Test is included directly rather than through find_package, the aliases are missing.
        add_library(GTest::gtest ALIAS gtest)
        add_library(GTest::gtest_main ALIAS gtest_main)
        add_library(GTest::gmock ALIAS gmock)
        add_library(GTest::gmock_main ALIAS gmock_main)
      else()
        message(FATAL_ERROR "ABSL_USE_EXTERNAL_GOOGLETEST is ON and ABSL_FIND_GOOGLETEST is OFF, which means that the top-level project must build the Google Test project. However, the target gtest was not found.")
      endif()
    endif()
  else()
    set(absl_gtest_build_dir ${CMAKE_BINARY_DIR}/googletest-build)
    if(ABSL_USE_GOOGLETEST_HEAD AND ABSL_GOOGLETEST_DOWNLOAD_URL)
      message(FATAL_ERROR "Do not set both ABSL_USE_GOOGLETEST_HEAD and ABSL_GOOGLETEST_DOWNLOAD_URL")
    endif()
    if(ABSL_USE_GOOGLETEST_HEAD)
      set(absl_gtest_download_url "https://github.com/google/googletest/archive/main.zip")
    elseif(ABSL_GOOGLETEST_DOWNLOAD_URL)
      set(absl_gtest_download_url ${ABSL_GOOGLETEST_DOWNLOAD_URL})
    endif()
    if(absl_gtest_download_url)
      set(absl_gtest_src_dir ${CMAKE_BINARY_DIR}/googletest-src)
    else()
      set(absl_gtest_src_dir ${ABSL_LOCAL_GOOGLETEST_DIR})
    endif()
    include(CMake/Googletest/DownloadGTest.cmake)
  endif()

  check_target(GTest::gtest)
  check_target(GTest::gtest_main)
  check_target(GTest::gmock)
  check_target(GTest::gmock_main)
endif()

add_subdirectory(absl)

if(ABSL_ENABLE_INSTALL)
  

  # install as a subdirectory only
  install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE absl::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  )

  configure_package_config_file(
    CMake/abslConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  )
  install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  )

  # Abseil only has a version in LTS releases.  This mechanism is accomplished
  # Abseil's internal Copybara (https://github.com/google/copybara) workflows and
  # isn't visible in the CMake buildsystem itself.
  if(absl_VERSION)
    write_basic_package_version_file(
      "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
      COMPATIBILITY ExactVersion
    )

    install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
      DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    )
  endif()  # absl_VERSION

  install(DIRECTORY absl
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
      PATTERN "*.inc"
      PATTERN "*.h"
      PATTERN "copts" EXCLUDE
      PATTERN "testdata" EXCLUDE
    )
endif()  # ABSL_ENABLE_INSTALL

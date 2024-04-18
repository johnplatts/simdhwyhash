# Copyright 2024 John Platts
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.10)

# Set PIE flags for POSITION_INDEPENDENT_CODE targets, added in 3.14.
if(POLICY CMP0083)
  cmake_policy(SET CMP0083 NEW)
endif()

# Workaround for 3.19 raising error 'IMPORTED_LOCATION not set for imported
# target "GTest::gtest_main"'.
if(POLICY CMP0111)
  cmake_policy(SET CMP0111 OLD)
endif()

# Starting with GCC-13, we want to make sure to remove gnu extension (ie.
# explicit -std=c++17 instead of implicit `gnu++17`)
# Without this cmake property, CMAKE_CXX_EXTENSIONS=OFF was not properly
# considered
if(POLICY CMP0128)
  cmake_policy(SET CMP0128 NEW)
endif()

project(simdhwyhash LANGUAGES C CXX VERSION 1.0.0)

# Directly define the ABI version from the cmake project() version values:
set(LIBRARY_VERSION "${simdhwyhash_VERSION}")
set(LIBRARY_SOVERSION ${simdhwyhash_VERSION_MAJOR})

if(CMAKE_CXX_STANDARD)
  if(CMAKE_CXX_STANDARD LESS 17)
    message(FATAL_ERROR "CMAKE_CXX_STANDARD is less than 17, simdhwyhash requires C++17 or later")
  endif()
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)

# Enabled PIE binaries by default if supported.
include(CheckPIESupported OPTIONAL RESULT_VARIABLE CHECK_PIE_SUPPORTED)
if(CHECK_PIE_SUPPORTED)
  check_pie_supported(LANGUAGES CXX)
  if(CMAKE_CXX_LINK_PIE_SUPPORTED)
    set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
  endif()
endif()

include(GNUInstallDirs)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(HWY_CMAKE_ARM7 OFF CACHE BOOL "Set copts for Armv7 with NEON (requires vfpv4)?")

set(HWY_CMAKE_SSE2 OFF CACHE BOOL "Set SSE2 as baseline for 32-bit x86?")

set(HWY_CMAKE_RVV ON CACHE BOOL "Set copts for RISCV with RVV?")

# Unconditionally adding -Werror risks breaking the build when new warnings
# arise due to compiler/platform changes. Enable this in CI/tests.
set(SIMDHWYHASH_WARNINGS_ARE_ERRORS OFF CACHE BOOL "Add -Werror flag?")

set(SIMDHWYHASH_ENABLE_INSTALL ON CACHE BOOL "Install simdhwyhash library")

set(SIMDHWYHASH_ENABLE_TESTS ON CACHE BOOL "Enable simdhwyhash tests")

include(CheckCXXSourceCompiles)

check_cxx_source_compiles(
   "int main() {
      #if !defined(__riscv)
      static_assert(false, \"__riscv is not defined\");
      #endif
      return 0;
    }"
  SIMDHWYHASH_RISCV
)

set(SIMDHWYHASH_INCLUDES
  ${PROJECT_SOURCE_DIR}/include/simdhwyhash.h
)

set(SIMDHWYHASH_SOURCES
  ${PROJECT_SOURCE_DIR}/src/simdhwyhash.cc
)

# By default prefer SHARED build
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(SIMDHWYHASH_FORCE_STATIC_LIBS "Ignore BUILD_SHARED_LIBS" OFF)
# only expose shared/static options to advanced users:
mark_as_advanced(BUILD_SHARED_LIBS)
mark_as_advanced(SIMDHWYHASH_FORCE_STATIC_LIBS)
# Define visibility settings globally:
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

# Copy-cat "add_library" logic + add override.
set(SIMDHWYHASH_LIBRARY_TYPE "SHARED")
if (NOT BUILD_SHARED_LIBS OR SIMDHWYHASH_FORCE_STATIC_LIBS)
  set(SIMDHWYHASH_LIBRARY_TYPE "STATIC")
endif()

# This preprocessor define will drive the build, also used in the *.pc files:
if("${SIMDHWYHASH_LIBRARY_TYPE}" STREQUAL "SHARED")
  set(DLLEXPORT_TO_DEFINE "SIMDHWYHASH_SHARED_DEFINE")
else()
  set(DLLEXPORT_TO_DEFINE "SIMDHWYHASH_STATIC_DEFINE")
endif()


if(NOT SIMDHWYHASH_HWY_CXX_FLAGS)
  set(SIMDHWYHASH_HWY_CXX_FLAGS "")
endif()

separate_arguments(SIMDHWYHASH_HWY_CXX_FLAGS)

if (MSVC)
  set(SIMDHWYHASH_FLAGS
    # fix build error C1128 in blockwise*_test & arithmetic_test
    /bigobj

    # Warnings
    /W4
    # Disable some W4 warnings.  Enable them individually after they are cleaned up.
    /wd4100
    /wd4127
    /wd4324
    /wd4456
    /wd4701
    /wd4702
    /wd4723
  )

  # This adds extra warnings for the clang-cl compiler on Windows.
  # This is the same as the sections in the else part.
  # These could be refactored.
  if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    list(APPEND SIMDHWYHASH_FLAGS
      # These are not included in Wall nor Wextra:
      -Wconversion
      -Wsign-conversion
      -Wvla
      -Wnon-virtual-dtor

      -Wfloat-overflow-conversion
      -Wfloat-zero-conversion
      -Wfor-loop-analysis
      -Wgnu-redeclared-enum
      -Winfinite-recursion
      -Wself-assign
      -Wstring-conversion
      -Wtautological-overlap-compare
      -Wthread-safety-analysis
      -Wundefined-func-template
    )
  endif()

  if (SIMDHWYHASH_WARNINGS_ARE_ERRORS)
    list(APPEND SIMDHWYHASH_FLAGS /WX)
  endif()
else()
  set(SIMDHWYHASH_FLAGS
    # Optimizations
    -fmerge-all-constants

    # Warnings
    -Wall
    -Wextra
    # These are not included in Wall nor Wextra:
    -Wconversion
    -Wsign-conversion
    -Wvla
    -Wnon-virtual-dtor
    -Wcast-align # see -Wcast-align=strict on x86
  )

  if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    list(APPEND SIMDHWYHASH_FLAGS
      -Wfloat-overflow-conversion
      -Wfloat-zero-conversion
      -Wfor-loop-analysis
      -Wgnu-redeclared-enum
      -Winfinite-recursion
      -Wself-assign
      -Wstring-conversion
      -Wtautological-overlap-compare
      -Wthread-safety-analysis
      -Wundefined-func-template

      -fno-slp-vectorize
      -fno-vectorize

      # Use color in messages
      -fdiagnostics-show-option -fcolor-diagnostics
    )
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 6.0)
      list(APPEND SIMDHWYHASH_FLAGS -Wc++2a-extensions)
    endif()
  endif()

  if (WIN32)
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
      list(APPEND SIMDHWYHASH_FLAGS
        -Wno-global-constructors
        -Wno-language-extension-token
        -Wno-used-but-marked-unused
        -Wno-shadow-field-in-constructor
        -Wno-unused-member-function
        -Wno-unused-template
        -Wno-c++98-compat-pedantic
        -Wno-used-but-marked-unused
        -Wno-zero-as-null-pointer-constant
      )
    endif()

    list(APPEND SIMDHWYHASH_FLAGS
      -Wno-cast-align
      -Wno-double-promotion
      -Wno-float-equal
      -Wno-format-nonliteral
      -Wno-shadow
      -Wno-sign-conversion
    )
  endif()  # WIN32

  # Workaround for excess precision, see #1488.
  if (HWY_CMAKE_SSE2)
    list(APPEND SIMDHWYHASH_FLAGS -msse2 -mfpmath=sse)
  endif()

  # Suppress STL iterator warnings. Supported by GCC 4.4.7 and newer, which
  # predates the C++11 we require.
  if (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    list(APPEND SIMDHWYHASH_FLAGS -Wno-psabi)
  endif()
  # Clang supports this flag from 11.0.
  if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11.0)
      list(APPEND SIMDHWYHASH_FLAGS -Wno-psabi)
    endif()
  endif()

  if (HWY_CMAKE_ARM7)
    list(APPEND SIMDHWYHASH_FLAGS
      -march=armv7-a
      -mfpu=neon-vfpv4
      -mfloat-abi=hard  # must match the toolchain specified as CXX=
    )
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
      # using GCC
      list(APPEND SIMDHWYHASH_FLAGS
        -mfp16-format=ieee  # required for vcvt_f32_f16
      )
    endif()
  endif()  # HWY_CMAKE_ARM7

  if(SIMDHWYHASH_RISCV)
    # gcc(13) and recent clang both support V, but not yet runtime dispatch, so
    # we add the gcv compiler flag, which then requires the CPU (now when using
    # either compiler) to support V.
    if(HWY_CMAKE_RVV)
      list(APPEND SIMDHWYHASH_FLAGS -march=rv64gcv1p0)
      if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        list(APPEND SIMDHWYHASH_FLAGS -menable-experimental-extensions)
      endif()
    endif()
  endif()

  if (SIMDHWYHASH_WARNINGS_ARE_ERRORS)
    list(APPEND SIMDHWYHASH_FLAGS -Werror)
  endif()
endif()  # !MSVC

list(APPEND SIMDHWYHASH_FLAGS ${SIMDHWYHASH_HWY_CXX_FLAGS})

add_library(simdhwyhash ${SIMDHWYHASH_LIBRARY_TYPE} ${SIMDHWYHASH_INCLUDES} ${SIMDHWYHASH_SOURCES})

target_compile_definitions(simdhwyhash PUBLIC "${DLLEXPORT_TO_DEFINE}")
target_compile_options(simdhwyhash PRIVATE ${SIMDHWYHASH_FLAGS})
set_property(TARGET simdhwyhash PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(simdhwyhash PROPERTIES VERSION ${LIBRARY_VERSION} SOVERSION ${LIBRARY_SOVERSION})
target_include_directories(simdhwyhash PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_include_directories(simdhwyhash PUBLIC ${PROJECT_SOURCE_DIR}/include)

# simdhwyhash requires C++17 support
target_compile_features(simdhwyhash PUBLIC cxx_std_17)

set_target_properties(simdhwyhash PROPERTIES PUBLIC_HEADER "${SIMDHWYHASH_INCLUDES}")

set(SIMDHWYHASH_SYSTEM_HIGHWAY OFF CACHE BOOL "Use pre-installed highway?")

get_property(SIMDHWYHASH_IS_MULTICONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

include(ExternalProject)

if(SIMDHWYHASH_SYSTEM_HIGHWAY)
  find_package(hwy 1.1.0)
  if (NOT HWY_FOUND)
    message(FATAL_ERROR
        "Highway library (hwy) not found. Highway is required to build "
        "simdhwyhash.")
  endif()

  set(SIMDHWYHASH_HWY_LIBS "hwy::hwy")
else()
  configure_file(highway-download.CMakeLists.txt.in highway-download/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/highway-download )
  if(result)
    message(FATAL_ERROR "CMake step for highway failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/highway-download )
  if(result)
    message(FATAL_ERROR "Build step for highway failed: ${result}")
  endif()

  set(SIMDHWYHASH_HWY_BUILD_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  list(APPEND SIMDHWYHASH_HWY_BUILD_CXX_FLAGS ${SIMDHWYHASH_HWY_CXX_FLAGS})
  string(REPLACE ";" " " SIMDHWYHASH_HWY_BUILD_CXX_FLAGS "${SIMDHWYHASH_HWY_BUILD_CXX_FLAGS}")

  set(SIMDHWYHASH_HWY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/highway-build")
  set(SIMDHWYHASH_HWY_LIB_FILENAME "${CMAKE_STATIC_LIBRARY_PREFIX}hwy${CMAKE_STATIC_LIBRARY_SUFFIX}")

  if(SIMDHWYHASH_IS_MULTICONFIG)
    set(SIMDHWYHASH_HWY_LIB_PATH "${SIMDHWYHASH_HWY_BUILD_DIR}/$<CONFIG>/${SIMDHWYHASH_HWY_LIB_FILENAME}")
  else()
    set(SIMDHWYHASH_HWY_LIB_PATH "${SIMDHWYHASH_HWY_BUILD_DIR}/${SIMDHWYHASH_HWY_LIB_FILENAME}")
  endif()

  set(SIMDHWYHASH_HWY_CMAKE_ARGS "")
  foreach (var_name IN ITEMS CMAKE_C_COMPILER CMAKE_CXX_COMPILER
           CMAKE_C_COMPILER_TARGET CMAKE_CXX_COMPILER_TARGET
           CMAKE_CROSSCOMPILING_EMULATOR CMAKE_BUILD_TYPE CMAKE_TOOLCHAIN_FILE
           CMAKE_CXX_STANDARD HWY_CMAKE_ARM7 HWY_CMAKE_SSE2 HWY_CMAKE_RVV)
    if(DEFINED ${var_name})
      string(REPLACE ";" "^^" simdhwyhash_hwy_cmake_arg "${${var_name}}")
      list (APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-D${var_name}=${simdhwyhash_hwy_cmake_arg}")
    endif()
  endforeach()

  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DCMAKE_CXX_FLAGS=${SIMDHWYHASH_HWY_BUILD_CXX_FLAGS}")
  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/highway-prefix")
  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DBUILD_SHARED_LIBS=OFF")
  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DHWY_ENABLE_CONTRIB=OFF")
  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DHWY_ENABLE_EXAMPLES=OFF")
  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DHWY_ENABLE_INSTALL=OFF")
  list(APPEND SIMDHWYHASH_HWY_CMAKE_ARGS "-DHWY_ENABLE_TESTS=OFF")

  ExternalProject_Add(highway
    LIST_SEPARATOR   ^^
    SOURCE_DIR       "${CMAKE_CURRENT_BINARY_DIR}/highway-src"
    BINARY_DIR       "${SIMDHWYHASH_HWY_BUILD_DIR}"
    PREFIX           "${CMAKE_CURRENT_BINARY_DIR}/highway-prefix"
    INSTALL_COMMAND  ""
    TEST_COMMAND     ""
    CMAKE_ARGS       ${SIMDHWYHASH_HWY_CMAKE_ARGS}
    BUILD_BYPRODUCTS ${SIMDHWYHASH_HWY_LIB_PATH})

  add_library(highway-lib STATIC IMPORTED)
  add_dependencies(highway-lib highway)

  if(SIMDHWYHASH_IS_MULTICONFIG)
    foreach(output_config IN ITEMS ${CMAKE_CONFIGURATION_TYPES} ${CMAKE_BUILD_TYPE})
      string(TOUPPER ${output_config} uppercase_output_config)
      set_target_properties(highway-lib
         PROPERTIES IMPORTED_LOCATION_${uppercase_output_config}
         "${SIMDHWYHASH_HWY_BUILD_DIR}/${output_config}/${SIMDHWYHASH_HWY_LIB_FILENAME}")
    endforeach()
  else()
    set_target_properties(highway-lib
       PROPERTIES IMPORTED_LOCATION ${SIMDHWYHASH_HWY_LIB_PATH}
    )
  endif()

  set_target_properties(highway-lib
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/highway-src")

  add_dependencies(simdhwyhash highway)
  set(SIMDHWYHASH_HWY_LIBS "highway-lib")
endif()

target_link_libraries(simdhwyhash PRIVATE ${SIMDHWYHASH_HWY_LIBS})

# -------------------------------------------------------- install library
if (SIMDHWYHASH_ENABLE_INSTALL)

install(TARGETS simdhwyhash EXPORT simdhwyhash_targets
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

set (SIMDHWYHASH_PKGCONFIG_REQUIRES_PRIVATE "")
set (SIMDHWYHASH_PKGCONFIG_EXTRA_LIBS "")

if ("${SIMDHWYHASH_LIBRARY_TYPE}" STREQUAL "STATIC")
  if(SIMDHWYHASH_SYSTEM_HIGHWAY)
    set(SIMDHWYHASH_PKGCONFIG_REQUIRES_PRIVATE "libhwy >= 1.1.0")
  else()
    set(SIMDHWYHASH_PKGCONFIG_EXTRA_LIBS "-lsimdhwyhash_hwy")
    install(FILES ${SIMDHWYHASH_HWY_LIB_PATH} DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            RENAME "${CMAKE_STATIC_LIBRARY_PREFIX}simdhwyhash_hwy${CMAKE_STATIC_LIBRARY_SUFFIX}")
  endif()
endif()

set(SIMDHWYHASH_LIBRARY_VERSION "${CMAKE_PROJECT_VERSION}")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/libsimdhwyhash.pc.in"
               "libsimdhwyhash.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libsimdhwyhash.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

endif()  # SIMDHWYHASH_ENABLE_INSTALL

# -------------------------------------------------------- Tests

include(CTest)

if(BUILD_TESTING AND SIMDHWYHASH_ENABLE_TESTS)

enable_testing()
include(GoogleTest)

set(SIMDHWYHASH_SYSTEM_GTEST OFF CACHE BOOL "Use pre-installed googletest?")
if(SIMDHWYHASH_SYSTEM_GTEST)
  find_package(GTest REQUIRED)
else()
  # Download and unpack googletest at configure time
  configure_file(googletest-download.CMakeLists.txt.in googletest-download/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
  if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
  if(result)
      message(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  add_library(simdhwyhash_gtest STATIC ${CMAKE_CURRENT_BINARY_DIR}/googletest-src/googletest/src/gtest-all.cc)
  target_include_directories(simdhwyhash_gtest PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/googletest-src/googletest/include)
  target_include_directories(simdhwyhash_gtest PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/googletest-src/googletest)
endif()  # SIMDHWYHASH_SYSTEM_GTEST

set(SIMDHWYHASH_TEST_FILES
  ${PROJECT_SOURCE_DIR}/tests/simdhwyhash_test.cc
)

set(SIMDHWYHASH_TEST_LIBS simdhwyhash)
if("${SIMDHWYHASH_LIBRARY_TYPE}" STREQUAL "STATIC")
  list(APPEND SIMDHWYHASH_TEST_LIBS ${SIMDHWYHASH_HWY_LIBS})
endif()

if(SIMDHWYHASH_SYSTEM_GTEST)
  if (CMAKE_VERSION VERSION_LESS 3.20)
    set(SIMDHWYHASH_GTEST_LIBS GTest::GTest)
  else()
    set(SIMDHWYHASH_GTEST_LIBS GTest::gtest)
  endif()
else()
  set(SIMDHWYHASH_GTEST_LIBS simdhwyhash_gtest)
endif()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests)

foreach (TESTFILE IN LISTS SIMDHWYHASH_TEST_FILES)
  # The TESTNAME is the name without the extension or directory.
  get_filename_component(TESTNAME ${TESTFILE} NAME_WE)
  add_executable(${TESTNAME} ${TESTFILE})
  target_compile_options(${TESTNAME} PRIVATE ${SIMDHWYHASH_FLAGS})

  target_link_libraries(${TESTNAME} PRIVATE ${SIMDHWYHASH_TEST_LIBS} ${SIMDHWYHASH_GTEST_LIBS})
  # Output test targets in the test directory.
  set_target_properties(${TESTNAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "tests")

  if(${CMAKE_VERSION} VERSION_LESS "3.10.3")
    gtest_discover_tests(${TESTNAME} TIMEOUT 60)
  else ()
    gtest_discover_tests(${TESTNAME} DISCOVERY_TIMEOUT 60)
  endif ()
endforeach ()

endif()  # BUILD_TESTING

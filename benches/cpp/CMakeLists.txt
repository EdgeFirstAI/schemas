cmake_minimum_required(VERSION 3.20)
project(edgefirst_cpp_benchmarks LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
# FetchContent_Populate() with declared content is deprecated in CMake 3.28.
# We use it deliberately for cyclonedds_cxx to download source without running
# its CMake build system (which cannot build on all platforms without ddsc).
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

# Force static linking for all FetchContent dependencies so the deployed
# benchmark binaries are self-contained — no libbenchmark.so / libfastcdr.so
# / libfoonathan_memory.so to ship alongside each binary.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# Pin versions to ROS 2 Kilted's bundled releases (adjust as needed).
FetchContent_Declare(foonathan_memory
    GIT_REPOSITORY https://github.com/foonathan/memory.git
    GIT_TAG        v0.7-3
    GIT_SHALLOW    TRUE)
set(FOONATHAN_MEMORY_BUILD_EXAMPLES OFF CACHE BOOL "")
set(FOONATHAN_MEMORY_BUILD_TESTS    OFF CACHE BOOL "")
set(FOONATHAN_MEMORY_BUILD_TOOLS    OFF CACHE BOOL "")

FetchContent_Declare(fastcdr
    GIT_REPOSITORY https://github.com/eProsima/Fast-CDR.git
    GIT_TAG        v2.2.5
    GIT_SHALLOW    TRUE)

FetchContent_Declare(googlebench
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.9.1
    GIT_SHALLOW    TRUE)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "")
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "")

# Eclipse Cyclone DDS C library — MakeAvailable so its configure step generates
# the platform-specific headers (dds/features.h, dds/config.h, dds/export.h,
# dds/version.h) that the CDR C++ headers transitively include.
# Build options: disable everything that doesn't affect header generation.
set(BUILD_IDLC         OFF CACHE BOOL "" FORCE)
set(BUILD_DDSPERF      OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING      OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES     OFF CACHE BOOL "" FORCE)
set(ENABLE_SECURITY    OFF CACHE BOOL "" FORCE)
# ENABLE_SSL=OFF avoids pulling OpenSSL into the cross-compile (the host's
# /usr/include/openssl headers leak in otherwise and fail under aarch64 zig).
set(ENABLE_SSL         OFF CACHE BOOL "" FORCE)
# We only need cyclonedds's configure step to *generate headers* (features.h,
# config.h, export.h, version.h). We don't link against ddsc — the
# cyclonedds_cdr static library below builds the codec from cyclonedds-cxx
# sources directly. Excluding ddsc from the default build avoids cross-
# compile issues with its networking code.
set(CMAKE_DISABLE_FIND_PACKAGE_OpenSSL TRUE)
FetchContent_Declare(cyclonedds
    GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
    GIT_TAG        0.10.5
    GIT_SHALLOW    TRUE)

# Eclipse Cyclone DDS C++ binding — source tree only (no add_subdirectory).
# We build a minimal cyclonedds_cdr static library from the 5 CDR .cpp files,
# bypassing the full ddscxx build which requires DDS participant infrastructure.
FetchContent_Declare(cyclonedds_cxx
    GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
    GIT_TAG        0.10.5
    GIT_SHALLOW    TRUE)

FetchContent_MakeAvailable(foonathan_memory fastcdr googlebench cyclonedds)

# Populate cyclonedds_cxx source tree without running its CMake build system.
FetchContent_GetProperties(cyclonedds_cxx)
if(NOT cyclonedds_cxx_POPULATED)
    FetchContent_Populate(cyclonedds_cxx)
endif()

# Paths into the two source trees.
set(CDDS_SRC_DIR   "${cyclonedds_SOURCE_DIR}")
set(CDDS_BIN_DIR   "${cyclonedds_BINARY_DIR}")       # contains generated .h files
set(CDDS_CXX_SRC   "${cyclonedds_cxx_SOURCE_DIR}/src/ddscxx")

# Generate stub headers required by the cyclonedds-cxx CDR layer:
#   dds/features.hpp  — C++ feature flags (all optional features disabled)
#   dds/core/detail/export.hpp — OMG_DDS_API visibility macro (empty for static builds)
# The C-library generated headers (dds/features.h, dds/config.h, dds/export.h) are
# produced by cyclonedds's own configure step and found via CDDS_BIN_DIR below.
set(CDDS_STUB_DIR "${CMAKE_CURRENT_BINARY_DIR}/cyclonedds_gen")
file(MAKE_DIRECTORY "${CDDS_STUB_DIR}/dds/core/detail")
file(WRITE "${CDDS_STUB_DIR}/dds/features.hpp"
"#ifndef __OMG_DDS_DDSCXX_FEATURES_HPP__\n"
"#define __OMG_DDS_DDSCXX_FEATURES_HPP__\n"
"/* DDSCXX_HAS_SHM not set */\n"
"/* DDSCXX_HAS_TYPE_DISCOVERY not set */\n"
"/* DDSCXX_HAS_TOPIC_DISCOVERY not set */\n"
"/* DDSCXX_USE_BOOST not set */\n"
"#endif\n")
file(WRITE "${CDDS_STUB_DIR}/dds/core/detail/export.hpp"
"#ifndef OMG_DDS_API_DETAIL_H\n"
"#define OMG_DDS_API_DETAIL_H\n"
"#define OMG_DDS_API_DETAIL\n"
"#define OMG_DDS_API_DETAIL_NO_EXPORT\n"
"#define OMG_DDS_API_DETAIL_DEPRECATED\n"
"#define OMG_DDS_API_DETAIL_DEPRECATED_EXPORT\n"
"#define OMG_DDS_API_DETAIL_DEPRECATED_NO_EXPORT\n"
"#endif\n")

# Minimal CDR-only static library built from the 5 serialisation source files.
# fragchain.cpp is excluded: it pulls in dds/ddsi/q_radmin.h from the C daemon
# and is not needed for pure encode/decode (no fragment reassembly required here).
add_library(cyclonedds_cdr STATIC
    "${CDDS_CXX_SRC}/src/org/eclipse/cyclonedds/core/cdr/cdr_stream.cpp"
    "${CDDS_CXX_SRC}/src/org/eclipse/cyclonedds/core/cdr/entity_properties.cpp"
    "${CDDS_CXX_SRC}/src/org/eclipse/cyclonedds/core/cdr/basic_cdr_ser.cpp"
    "${CDDS_CXX_SRC}/src/org/eclipse/cyclonedds/core/cdr/extended_cdr_v1_ser.cpp"
    "${CDDS_CXX_SRC}/src/org/eclipse/cyclonedds/core/cdr/extended_cdr_v2_ser.cpp")
target_include_directories(cyclonedds_cdr PUBLIC
    "${CDDS_STUB_DIR}"                              # C++ stubs: features.hpp, export.hpp (first to override)
    "${CDDS_CXX_SRC}/include"                       # cyclonedds-cxx C++ headers
    "${CDDS_BIN_DIR}/src/ddsrt/include"             # generated: dds/features.h, dds/config.h, dds/version.h
    "${CDDS_BIN_DIR}/src/core/include"              # generated: dds/export.h (DDS_INLINE_EXPORT)
    "${CDDS_SRC_DIR}/src/ddsrt/include"             # source: dds/ddsrt/endian.h, md5.h
    "${CDDS_SRC_DIR}/src/core/ddsi/include"         # source: dds/ddsi/ddsi_keyhash.h, ddsi_serdata.h
    "${CDDS_SRC_DIR}/src/core/ddsc/include"         # source: dds/ddsc/dds_public_qosdefs.h (via ddsi_xqos.h)
    "${CDDS_SRC_DIR}/src/include")
target_compile_features(cyclonedds_cdr PUBLIC cxx_std_17)

# Generated Cyclone CDR type sources — compiled directly into each consuming
# target (parallel to how fastcdr skips the generated .cxx files).
set(CDDS_GENERATED_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/types/cyclonedds")
set(CDDS_GENERATED_SOURCES
    types/cyclonedds/builtin_interfaces/Time.cpp
    types/cyclonedds/std_msgs/Header.cpp
    types/cyclonedds/geometry_msgs/Vector3.cpp
    types/cyclonedds/geometry_msgs/Point.cpp
    types/cyclonedds/geometry_msgs/Quaternion.cpp
    types/cyclonedds/geometry_msgs/Pose.cpp
    types/cyclonedds/edgefirst_msgs/DmaBuffer.cpp
    types/cyclonedds/edgefirst_msgs/Mask.cpp
    types/cyclonedds/edgefirst_msgs/RadarCube.cpp
    types/cyclonedds/sensor_msgs/Image.cpp
    types/cyclonedds/sensor_msgs/PointField.cpp
    types/cyclonedds/sensor_msgs/PointCloud2.cpp
    types/cyclonedds/foxglove_msgs/CompressedVideo.cpp)

# Helper to register a benchmark binary.
function(add_benchmark_executable name)
    cmake_parse_arguments(BE "" "" "SOURCES;LINK" ${ARGN})
    add_executable(${name} ${BE_SOURCES})
    target_link_libraries(${name} PRIVATE benchmark::benchmark ${BE_LINK})
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

# Locate the edgefirst-schemas C++ wrappers (header-only) and the underlying
# Rust shared library. Built by the parent crate; run `cargo build --release`
# from the repo root before configuring this CMake project.
set(EF_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(EF_INCLUDE   "${EF_REPO_ROOT}/include")
set(EF_LIB_DIR   "${EF_REPO_ROOT}/target/release")
if(DEFINED ENV{EF_LIB_DIR_OVERRIDE})
    set(EF_LIB_DIR "$ENV{EF_LIB_DIR_OVERRIDE}")
endif()
find_library(EF_SCHEMAS_LIB
    NAMES edgefirst_schemas
    PATHS "${EF_LIB_DIR}"
    NO_DEFAULT_PATH
    REQUIRED)

add_benchmark_executable(bench_edgefirst
    SOURCES bench_edgefirst.cpp
    LINK    ${EF_SCHEMAS_LIB})
target_include_directories(bench_edgefirst PRIVATE ${EF_INCLUDE})

# fastddsgen places generated files under types/fastcdr/idl/<module>/<Type>.{hpp,cxx,...}
# (one level of nesting beneath the explicit -d output root). Internal includes
# resolve relative to types/fastcdr/idl, so that's the include path we expose.
#
# Note: the generated `*PubSubTypes.cxx` and `*TypeObjectSupport.cxx` files require
# the full Fast DDS SDK at link time (not just Fast-CDR). The codec-only benchmarks
# here use only the inline `*.hpp` / `*.ipp` definitions (CdrAux templates), so we
# deliberately do NOT compile the generated `.cxx` files into any target. Regen
# via scripts/regen_fastcdr_types.sh.
set(FASTCDR_GENERATED_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/types/fastcdr/idl")

add_benchmark_executable(bench_fastcdr
    SOURCES bench_fastcdr.cpp
    LINK    fastcdr)
target_include_directories(bench_fastcdr PRIVATE ${FASTCDR_GENERATED_INCLUDES})

# idlc generates one .hpp + one .cpp per IDL type. The .cpp files define
# get_type_props<T>() which is required by the write/read/move/max free functions.
# Regen via scripts/regen_cyclonedds_types.sh.
add_benchmark_executable(bench_cyclonedds
    SOURCES bench_cyclonedds.cpp ${CDDS_GENERATED_SOURCES}
    LINK    cyclonedds_cdr)
target_include_directories(bench_cyclonedds PRIVATE ${CDDS_GENERATED_INCLUDES})

enable_testing()

# parity_cyclonedds.cpp must be compiled with ONLY the cyclonedds generated type
# headers (CDDS_GENERATED_INCLUDES), not the Fast-CDR ones, because both sets of
# generated types occupy the same C++ namespaces (std_msgs::msg etc.) and cannot
# coexist in one translation unit. We use an OBJECT library to isolate the
# include paths per source file.
add_library(parity_cdds_obj OBJECT
    tests/parity_cyclonedds.cpp
    ${CDDS_GENERATED_SOURCES})
target_include_directories(parity_cdds_obj PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CDDS_GENERATED_INCLUDES})
target_link_libraries(parity_cdds_obj PRIVATE cyclonedds_cdr)

add_executable(parity_test tests/parity_test.cpp)
target_sources(parity_test PRIVATE $<TARGET_OBJECTS:parity_cdds_obj>)
target_link_libraries(parity_test PRIVATE
    ${EF_SCHEMAS_LIB}
    fastcdr
    cyclonedds_cdr)
target_include_directories(parity_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EF_INCLUDE}
    ${FASTCDR_GENERATED_INCLUDES})

add_test(NAME parity COMMAND parity_test)
# Append (don't overwrite) LD_LIBRARY_PATH so toolchain / sanitizer / custom
# libstdc++ paths the user already set in their environment continue to work.
set_tests_properties(parity PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${EF_LIB_DIR}:$ENV{LD_LIBRARY_PATH}")


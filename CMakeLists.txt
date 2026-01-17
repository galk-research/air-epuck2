cmake_minimum_required(VERSION 3.8.0)
project(argos_air_resistance_plugins)

# ---------------------------
# Build type (default Release)
# ---------------------------
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..."
      FORCE)
endif()

# ---------------------------
# C++ standard
# ---------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MACOSX_RPATH 0)

# ---------------------------
# ARGoS
# ---------------------------
find_package(ARGoS REQUIRED)

include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})

# ARGOS_LDFLAGS is often a single string; split it safely into a list
if(DEFINED ARGOS_LDFLAGS)
  separate_arguments(ARGOS_LDFLAGS_LIST NATIVE_COMMAND "${ARGOS_LDFLAGS}")
  link_libraries(${ARGOS_LDFLAGS_LIST})
endif()

# Put all shared libraries under build/lib by default (subdirs may override)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---------------------------
# Core project (your real plugins)
# ---------------------------
add_subdirectory(src)

# ---------------------------
# Optional example controllers (tutorial code)
# ---------------------------
option(BUILD_EXAMPLE_CONTROLLERS "Build example controllers under examples/controllers" ON)
if(BUILD_EXAMPLE_CONTROLLERS)
  add_subdirectory(examples/controllers/wind_aware)
endif()


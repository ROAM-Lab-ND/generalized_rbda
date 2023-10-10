cmake_minimum_required(VERSION 3.2)

project(generalized_rbda)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_COLOR_MAKEFILE   ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG_MODE)
endif()

#------------------------------------------------------------------------
# Compiler flags
#------------------------------------------------------------------------
option(M1_BUILD "set compile options for Mac M1 chip" OFF)
if(M1_BUILD AND CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "-mcpu=apple-m1")
else()
  set(CMAKE_CXX_FLAGS "-march=native")
endif()

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Wno-sign-compare -Wno-unused-const-variable \
  -Wno-unused-parameter -Wno-unused-variable -Wno-uninitialized")

#------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------
# TODO(@MatthewChignoli): Maybe want to check out the TOLRAS repo for examples on how to setup this CMake in a more general way

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS "Eigen3 found")
  include_directories(${EIGEN3_INCLUDE_DIR}/..)
else()
  message(FATAL_ERROR "Eigen3 not found. Please install it or specify its location.")
endif()


find_package(casadi REQUIRED)
if(casadi_FOUND)
  message(STATUS "CasADi found")
  set(CASADI_INCLUDE_DIR "/usr/local/include/casadi" CACHE PATH "Path to Casadi include directory")
  include_directories(${CASADI_INCLUDE_DIR}/..)
  find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
  if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
  endif()
else()
  message(FATAL_ERROR "CasADi not found. Please install it or specify its location.")
endif()



#------------------------------------------------------------------------
# Directory setup
#------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Dynamics)

add_subdirectory(Utils)
add_subdirectory(Dynamics)
add_subdirectory(Robots)
add_subdirectory(Benchmarking)

# Check if the project is being built as a subdirectory
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  set(SUBMOD_MESSAGE "GRBDA compiled as main project")
  
  set(THIS_COM "${PROJECT_SOURCE_DIR}/" )
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

  add_subdirectory(UnitTests)
else()
  set(SUBMOD_MESSAGE "GRBDA compiled as sub-module")
endif()

#------------------------------------------------------------------------
# Printout
#------------------------------------------------------------------------
message ("")
message ("--------------------------------------------------------------")
message (" *** ${SUBMOD_MESSAGE} *** ")
message ("")
message (" The project name/ Build Type: ${PROJECT_NAME}/${CMAKE_BUILD_TYPE}")
message (" The system name: ${CMAKE_SYSTEM_NAME}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${CMAKE_BINARY_DIR}")
message ("--------------------------------------------------------------")
message ("")

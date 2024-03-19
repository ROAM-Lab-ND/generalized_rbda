cmake_minimum_required(VERSION 3.13)

project(grbda VERSION 0.1.0)

message(STATUS "=============  !Generalized Rigid-Body Dynamics Algorithms!  =============")

################################################################################
# Options
################################################################################
option(BUILD_TESTS "Building the tests" ON)
option(BUILD_BENCHMARKS "Building the benchmarks" OFF)
option(M1_BUILD "set compile options for Mac M1 chip" OFF)
option(MARCH_NATIVE "set compile options to optimize for native architecture" OFF)
option(ASAN "set compile options for Address Sanitizer" OFF)

################################################################################
# Finding Dependencies
################################################################################
message(STATUS "======> Find Dependencies")

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

################################################################################
# Setup Compilation
################################################################################
message(STATUS "======> Setup Compilation")

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

if (M1_BUILD AND MARCH_NATIVE)
  message(SEND_ERROR "Cannot enable M1 build and optimize for native architecture")
elseif(M1_BUILD AND CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "-mcpu=apple-m1")
elseif(MARCH_NATIVE)
  set(CMAKE_CXX_FLAGS "-march=native")
  set(CMAKE_C_FLAGS "-march=native")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Wno-sign-compare -Wno-unused-const-variable \
  -Wno-unused-parameter -Wno-unused-variable -Wno-uninitialized")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -ggdb -std=gnu99 -I.")

if(ASAN)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

################################################################################
# Specify Build Resources
################################################################################
message(STATUS "======>  Setup Build  ")

include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB_RECURSE GRBDA_SRCS "src/*.cpp")

################################################################################
# Setup Build
################################################################################

if(NOT GRBDA_SRCS)
  message(FATAL_ERROR "No source files found. Please add something")
	set(LIBRARY_NAME)
else()
  add_library(${PROJECT_NAME} ${GRBDA_SRCS})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${CASADI_LIBRARIES})

  set(LIBRARY_NAME ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE) 

  install(TARGETS ${PROJECT_NAME}
          EXPORT ${PROJECT_NAME}Targets
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          INCLUDES DESTINATION include)

  install(DIRECTORY include/ DESTINATION include)

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
  )

  configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  )

  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME})

  install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTS)
  message(STATUS "======>  Setup Tests  ")
  enable_testing()
  add_subdirectory(UnitTests)
endif()

if(BUILD_BENCHMARKS)
  message(STATUS "======>  Setup Benchmarks  ")
  add_subdirectory(Benchmarking)
endif()

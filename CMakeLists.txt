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

#set(CMAKE_VERBOSE_MAKEFILE ON)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

option(CMAKE_BUILD_TYPE "compile type. default: Debug" Debug)
option(M1_BUILD "set compile options for Mac M1 chip" OFF)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Werror -Wno-aligned-new -Wno-unused-but-set-variable \
  -Wno-sign-compare")

elseif(APPLE)
  if(M1_BUILD)
    set(CMAKE_CXX_FLAGS "-mcpu=apple-m1")
  else()
    set(CMAKE_CXX_FLAGS "-march=native")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo ")
  include_directories("/usr/local/include/")
endif()

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

#------------------------------------------------------------------------
# Directory setup
#------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Systems)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LCM-Types/cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd-parties)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd-parties/ParamHandler)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd-parties/URDF)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd-parties/CasadiGen)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/DynamicsEngine)
include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})

find_package(lcm REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-const-variable \
  -Wno-unused-parameter -Wno-unused-variable -Wno-uninitialized")

add_subdirectory(Utils)
add_subdirectory(DynamicsEngine)
#add_subdirectory(Systems)
add_subdirectory(Robots)
add_subdirectory(3rd-parties)
add_subdirectory(Benchmarking)

# Check if the project is being built as a subdirectory
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    # This is the main project

    # Your main project code here
    set(THIS_COM "${PROJECT_SOURCE_DIR}/" )
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
      ${CMAKE_BINARY_DIR}/Configuration.h)

    add_subdirectory(UnitTests)
else()
    # This is a subdirectory

    # Your subdirectory-specific code here
endif()

#------------------------------------------------------------------------
# Printout
#------------------------------------------------------------------------
message ("")
message ("--------------------------------------------------------------")
message (" The project name/ Build Type: ${PROJECT_NAME}/${CMAKE_BUILD_TYPE}")
message (" The system name: ${CMAKE_SYSTEM_NAME}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${CMAKE_BINARY_DIR}")
message ("--------------------------------------------------------------")
message ("")

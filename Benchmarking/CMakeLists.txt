find_package(pinocchio REQUIRED)
if(pinocchio_FOUND)
  message(STATUS "Pinocchio found")
  set(PINOCCHIO_INCLUDE_DIR "/usr/local/include/pinocchio" CACHE PATH "Path to pinocchio include directory")
  include_directories(${PINOCCHIO_INCLUDE_DIR}/..)
  find_library(PINOCCHIO_LIBRARY
    NAMES pinocchio
    HINTS ${PINOCCHIO_INCLUDE_DIR}/../lib $ENV{PINOCCHIO_PREFIX}/lib)
  if(PINOCCHIO_LIBRARY)
    set(PINOCCHIO_LIBRARIES ${PINOCCHIO_LIBRARIES} ${PINOCCHIO_LIBRARY})
  endif()
  find_library(PINOCCHIO_PARSERS_LIBRARY
    NAMES pinocchio_parsers
    HINTS ${PINOCCHIO_INCLUDE_DIR}/../lib $ENV{PINOCCHIO_PREFIX}/lib)
  if(PINOCCHIO_PARSERS_LIBRARY)
    set(PINOCCHIO_LIBRARIES ${PINOCCHIO_LIBRARIES} ${PINOCCHIO_PARSERS_LIBRARY})
  endif()
  find_library(PINOCCHIO_CASADI_LIBRARY
    NAMES pinocchio_casadi
    HINTS ${PINOCCHIO_INCLUDE_DIR}/../lib $ENV{PINOCCHIO_PREFIX}/lib)
  if(PINOCCHIO_CASADI_LIBRARY)
    set(PINOCCHIO_LIBRARIES ${PINOCCHIO_LIBRARIES} ${PINOCCHIO_CASADI_LIBRARY})
  endif()
else()
  message(FATAL_ERROR "Pinocchio not found. Please install it or specify its location.")
endif()

file(GLOB_RECURSE HELPER_SRCS "src/*Helpers.cpp")
add_library(benchmarkHelpers ${HELPER_SRCS})
target_link_libraries(benchmarkHelpers ${PROJECT_NAME} ${PINOCCHIO_LIBRARIES} console_bridge)

add_executable(pinocchio_benchmark src/pinocchioBenchmark.cpp)
target_link_libraries(pinocchio_benchmark benchmarkHelpers)
target_link_libraries(pinocchio_benchmark gtest gmock_main)

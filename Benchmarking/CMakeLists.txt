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

add_executable(parallel_chains_benchmark src/parallelChainsBenchmark.cpp)
target_link_libraries(parallel_chains_benchmark ${PROJECT_NAME} ${PINOCCHIO_LIBRARIES})
target_link_libraries(parallel_chains_benchmark gtest gmock_main)

add_executable(parallel_chains_benchmark_v2 src/parallelChainsBenchmark_v2.cpp)
target_link_libraries(parallel_chains_benchmark_v2 ${PROJECT_NAME} ${PINOCCHIO_LIBRARIES})
target_link_libraries(parallel_chains_benchmark_v2 gtest gmock_main)

add_executable(rotors_benchmark src/rotorsBenchmark.cpp)
target_link_libraries(rotors_benchmark ${PROJECT_NAME} ${PINOCCHIO_LIBRARIES})
target_link_libraries(rotors_benchmark gtest gmock_main)

add_executable(approx_benchmark src/approximateBenchmark.cpp)
target_link_libraries(approx_benchmark ${PROJECT_NAME} ${PINOCCHIO_LIBRARIES})
target_link_libraries(approx_benchmark gtest gmock_main)

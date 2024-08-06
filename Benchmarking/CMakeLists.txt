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
else()
  message(FATAL_ERROR "Pinocchio not found. Please install it or specify its location.")
endif()

add_executable(time_benchmark_fd src/timeBenchmarkForwardDynamics.cpp)
target_link_libraries(time_benchmark_fd ${PROJECT_NAME})

add_executable(time_benchmark_iosim src/timeBenchmarkInverseOSIM.cpp)
target_link_libraries(time_benchmark_iosim ${PROJECT_NAME})

add_executable(time_benchmark_robots src/timeBenchmarkRobots.cpp)
target_link_libraries(time_benchmark_robots ${PROJECT_NAME})

add_executable(accuracy_benchmark_fd src/accuracyBenchmarkForwardDynamics.cpp)
target_link_libraries(accuracy_benchmark_fd ${PROJECT_NAME})

add_executable(accuracy_benchmark_id src/accuracyBenchmarkInverseDynamics.cpp)
target_link_libraries(accuracy_benchmark_id ${PROJECT_NAME})

add_executable(accuracy_benchmark_atf src/accuracyBenchmarkApplyTestForce.cpp)
target_link_libraries(accuracy_benchmark_atf ${PROJECT_NAME})

add_executable(accuracy_benchmark_robots src/accuracyBenchmarkRobots.cpp)
target_link_libraries(accuracy_benchmark_robots ${PROJECT_NAME})

add_executable(accuracy_benchmark_ee_tracking src/accuracyBenchmarkEndEffectorTracking.cpp)
target_link_libraries(accuracy_benchmark_ee_tracking ${PROJECT_NAME})

add_executable(pinocchio_benchmark src/pinocchioBenchmark.cpp)
target_link_libraries(pinocchio_benchmark ${PROJECT_NAME} ${PINOCCHIO_LIBRARIES} pinocchio_parsers)

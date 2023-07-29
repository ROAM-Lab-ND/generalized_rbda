add_executable(time_benchmark timeBenchmarkForwardDynamics.cpp)
target_link_libraries(time_benchmark dyna-utils robots)

add_executable(time_benchmark_robots timeBenchmarkRobots.cpp)
target_link_libraries(time_benchmark_robots dyna-utils robots)

add_executable(accuracy_benchmark_fd accuracyBenchmarkForwardDynamics.cpp)
target_link_libraries(accuracy_benchmark_fd dyna-utils robots)

add_executable(accuracy_benchmark_id accuracyBenchmarkInverseDynamics.cpp)
target_link_libraries(accuracy_benchmark_id dyna-utils robots)

add_executable(accuracy_benchmark_atf accuracyBenchmarkApplyTestForce.cpp)
target_link_libraries(accuracy_benchmark_atf dyna-utils robots)

add_executable(accuracy_benchmark_robots accuracyBenchmarkRobots.cpp)
target_link_libraries(accuracy_benchmark_robots dyna-utils robots)

if(TIMING_STATS)
    add_executable(lg_mult_timing_analysis lagrangeMultiplierMethodTimingAnalysis.cpp)
    target_link_libraries(lg_mult_timing_analysis dyna-utils robots)

    add_executable(cluster_model_timing_analysis clusterModelTimingAnalysis.cpp)
    target_link_libraries(cluster_model_timing_analysis dyna-utils robots)

    add_executable(cluster_model_timing_analysis_tello clusterModelTimingAnalysisTello.cpp)
    target_link_libraries(cluster_model_timing_analysis_tello dyna-utils robots)
endif()

file(GLOB BENCH_SRCS "*.cpp")

foreach(bench_source ${BENCH_SRCS})
    get_filename_component(bench_name ${bench_source} NAME_WE)
    add_executable(${bench_name} ${bench_source})
    target_link_libraries(${bench_name} PRIVATE ${PROJECT_NAME})
endforeach()

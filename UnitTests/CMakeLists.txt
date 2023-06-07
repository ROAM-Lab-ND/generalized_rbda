add_subdirectory(googletest-src)

file(GLOB_RECURSE test_sources "*.cpp")
add_executable(unit_tests ${test_sources})

target_link_libraries(unit_tests gtest gmock_main)
target_link_libraries(unit_tests dyna-urdf dyna-utils robots)

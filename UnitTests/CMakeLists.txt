add_subdirectory(googletest-src)

# Create single executable for all tests
file(GLOB_RECURSE test_sources "*.cpp")
add_executable(unit_tests ${test_sources})

target_link_libraries(unit_tests gtest gmock_main)
target_link_libraries(unit_tests dyna-urdf dyna-utils robots)

# Create a test for each test file
foreach(test_source ${test_sources})
    # Extract the file name without the extension
    get_filename_component(test_name ${test_source} NAME_WE)

    # Create an executable for each test file
    add_executable(${test_name} ${test_source})

    # Link the necessary libraries
    target_link_libraries(${test_name} gtest gmock_main)
    target_link_libraries(${test_name} dyna-urdf dyna-utils robots)
endforeach()

add_subdirectory(googletest-src)

# Create single executable for all tests
file(GLOB_RECURSE test_sources "*.cpp")

# Create a test for each test file
foreach(test_source ${test_sources})
    # Extract the file name without the extension
    get_filename_component(test_name ${test_source} NAME_WE)

    # Create an executable for each test file
    add_executable(${test_name} ${test_source})

    # Link the necessary libraries
    target_link_libraries(${test_name} PRIVATE gtest gmock_main ${PROJECT_NAME})

    # Add the test to CTest
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

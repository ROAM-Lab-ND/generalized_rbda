# Create single executable for all tests
file(GLOB_RECURSE test_sources "*.cpp")

# Tests that provide their own main() — link gtest only (no gmock_main)
set(TESTS_WITH_CUSTOM_MAIN
    testInverseDynamicsDerivativesComplexStep
)

# Create a test for each test file
foreach(test_source ${test_sources})
    # Extract the file name without the extension
    get_filename_component(test_name ${test_source} NAME_WE)

    # Create an executable for each test file
    add_executable(${test_name} ${test_source})

    # Link the necessary libraries
    if(${test_name} IN_LIST TESTS_WITH_CUSTOM_MAIN)
        target_link_libraries(${test_name} PRIVATE gtest ${PROJECT_NAME})
    else()
        target_link_libraries(${test_name} PRIVATE gtest gmock_main ${PROJECT_NAME})
    endif()

    # Add the test to CTest
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

file(GLOB_RECURSE sources "*.cpp")
add_library(robots SHARED ${sources})
target_link_libraries(robots dyna-engine casadigen)

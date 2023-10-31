file(GLOB_RECURSE sources "*.cpp")
add_library(grbda-robots SHARED ${sources})
target_link_libraries(grbda-robots grbda casadigen)

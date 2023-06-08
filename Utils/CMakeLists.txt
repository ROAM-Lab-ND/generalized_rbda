file(GLOB_RECURSE sources "*.cpp")  
add_library(dyna-utils SHARED ${sources})
target_link_libraries(dyna-utils lcm)


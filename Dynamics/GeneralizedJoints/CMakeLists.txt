file(GLOB_RECURSE sources *.cpp)
add_library(gen-joints SHARED ${sources})
target_link_libraries(gen-joints dyna-utils casadigen)

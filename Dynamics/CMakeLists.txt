add_subdirectory(ClusterJoints)
add_subdirectory(Nodes)

file(GLOB sources *.cpp)
add_library(grbda SHARED ${sources})
target_link_libraries(grbda nodes)

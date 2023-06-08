add_subdirectory(GeneralizedJoints)
add_subdirectory(Nodes)

# TODO(@MatthewChignoli): Change the name of this library to generalized_rbda

file(GLOB sources *.cpp)
add_library(dyna-engine SHARED ${sources})
target_link_libraries(dyna-engine nodes dyna-urdf)

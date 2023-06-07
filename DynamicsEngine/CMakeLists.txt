add_subdirectory(GeneralizedJoints)
add_subdirectory(Nodes)

# file(GLOB_RECURSE sources *.cpp)
set(sources TestEngine.cpp  Factorization.cpp TreeModel.cpp
    RigidBodyTreeModel.cpp RigidBodyTreeDynamics.cpp 
    ClusterTreeModel.cpp ClusterTreeDynamics.cpp
    ReflectedInertiaTreeModel.cpp)
add_library(dyna-engine SHARED ${sources})
target_link_libraries(dyna-engine nodes dyna-urdf)

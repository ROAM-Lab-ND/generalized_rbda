include_directories("./include")

file(GLOB_RECURSE sources "src/*.cpp")
add_library(manipulator_system SHARED ${sources})
target_link_libraries(manipulator_system 
  lcm dyna-utils dyna-engine)


add_executable(run_manipulator main.cpp)
target_link_libraries(run_manipulator manipulator_system)


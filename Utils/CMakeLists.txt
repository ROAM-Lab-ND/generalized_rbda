#find_package(ZLIB REQUIRED)
#include_directories(${ZLIB_INCLUDE_DIRS})

file(GLOB_RECURSE sources "*.cpp")  
add_library(dyna-utils SHARED ${sources})
#target_link_libraries(dyna-utils lcm daros_param_handler ${ZLIB_LIBRARIES})
target_link_libraries(dyna-utils lcm daros_param_handler)


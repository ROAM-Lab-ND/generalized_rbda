file(GLOB_RECURSE sources *.cpp)
add_library(nodes SHARED ${sources})
target_link_libraries(nodes gen-joints)

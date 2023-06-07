file(GLOB sources source/*.cpp)

add_library(casadigen SHARED ${sources})
target_link_libraries(casadigen dyna-utils)
target_include_directories(casadigen PUBLIC "header/")

file(GLOB sources source/*.cpp)
add_library(casadi_codegen SHARED ${sources})
target_include_directories(casadi_codegen PRIVATE "header/" "${CMAKE_SOURCE_DIR}/include/")
target_link_libraries(casadi_codegen PRIVATE ${CASADI_LIBRARIES})

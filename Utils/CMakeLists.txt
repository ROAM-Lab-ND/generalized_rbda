file(GLOB sources "*.cpp")  
add_library(dyna-utils SHARED ${sources})
target_link_libraries(dyna-utils ${CASADI_LIBRARIES})

add_subdirectory(CasadiGen)

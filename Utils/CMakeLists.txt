file(GLOB sources "*.cpp")  
add_library(dyna-utils SHARED ${sources})
add_subdirectory(CasadiGen)

add_library(customLibrary add.cpp)
install (TARGETS customLibrary DESTINATION bin)
install (FILES add.h DESTINATION include)

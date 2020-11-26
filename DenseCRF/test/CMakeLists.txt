SET(targetname test_denseCRF)

ADD_EXECUTABLE(${targetname} test.cpp)
target_link_libraries (${targetname} PUBLIC DenseCRF)
set_property(TARGET ${targetname} PROPERTY CXX_STANDARD 14)
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)

set(targetname PytorchCppTools)

###############################################################################
SET(sources
    PytorchCpp.cpp
)
SET(headers
	include/PytorchCpp/PytorchCpp.h
)


INCLUDE(SetLibTarget)
INCLUDE(LinkPytorch)
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)
target_include_directories(${targetname} PUBLIC ./include )
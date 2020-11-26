cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

UNSET(COMPILE_WITH_CUDA)
###############################################################################
### -- 3rd party
INCLUDE(UseCUDA)
INCLUDE(UseOpenCV)
INCLUDE(UseEigen3)

###########################################################################################
SET(targetname ImageLoader)
file(GLOB_RECURSE srcs *.cpp)
file(GLOB_RECURSE headers *.h *.hpp)
SET(sources
	${srcs}
)

SET(headers
	${headers}
)

INCLUDE(SetCUDALibTarget)
target_link_libraries(${targetname}
		PUBLIC CxxTools
		PUBLIC ${OpenCV_LIBS}
)
INCLUDE(LinkOpenCV)

target_include_directories(${targetname} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 11)
target_include_directories(${targetname} PUBLIC ./include)

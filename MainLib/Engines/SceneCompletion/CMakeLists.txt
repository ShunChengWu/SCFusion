#################################
# SceneCompletionEngine LIBRARY #
#################################
set(targetnamebase SceneCompletionEngine)
file(GLOB_RECURSE ${targetnamebase}_headers *.h)
file(GLOB_RECURSE ${targetnamebase}_sources
        *.c
        *.cpp
        *.cu
        *.tpp
        *.tcu)


FOREACH(i RANGE ${VOXELCOUNTS} )
	LIST(GET VOXELNAMES ${i} name)
	LIST(GET VOXELTYPES ${i} type)
	SET(FIELD_TYPE VOXELTYPE=${type})
	SET(NAME_SUFFIX _${name})
	SET(targetname ${targetnamebase}${NAME_SUFFIX})
	ADD_LIBRARY(${targetname} OBJECT ${${targetnamebase}_sources} ${${targetnamebase}_headers})
	if(TENSORFLOW_FOUND)
		INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkTensorflow.cmake)
	endif()
	TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC VOXELTYPE=${type})
	SET_TARGET_PROPERTIES(${targetname} PROPERTIES CXX_STANDARD 14)
ENDFOREACH()

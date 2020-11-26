###############################################################################
### -- 3rd party
SET(CUDA_NVCC_FLAGS)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
####################################### SCFusion ###############################################
set(targetname SCFUSION)

file(GLOB headers *.hpp *.h)
file(GLOB cuda src/*.cu)

SET(${targetname}_sources
    SLAM.cpp
    ${cuda}
    ${glad}
)
SET(${targetname}_headers
    ${headers}
)

FOREACH(i RANGE ${VOXELCOUNTS} )
	LIST(GET VOXELNAMES ${i} name)
	LIST(GET VOXELTYPES ${i} type)
	SET(FIELD_TYPE VOXELTYPE=${type})
	SET(NAME_SUFFIX _${name})
	SET(targetname_suffix ${targetname}${NAME_SUFFIX})
	ADD_LIBRARY(${targetname_suffix}
			${${targetname}_sources} ${${targetname}_headers}
			)

	TARGET_LINK_LIBRARIES(${targetname_suffix}
			PUBLIC MeshingEngine${NAME_SUFFIX}
			PUBLIC PointCloudEngine${NAME_SUFFIX}
			PUBLIC BasicSLAM${NAME_SUFFIX}
			PUBLIC SceneCompletion${NAME_SUFFIX}
			)
	if(WITH_PYTORCH)
		target_link_libraries(${targetname_suffix} PRIVATE PytorchCppTools)
	endif()
	if(WITH_TENSORFLOW)
		target_link_libraries(${targetname_suffix} PUBLIC TFCppTools)
	endif()
	target_include_directories(${targetname_suffix} PUBLIC ../)
	SET_TARGET_PROPERTIES(${targetname_suffix} PROPERTIES CXX_STANDARD 14)
	TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC VOXELTYPE=${type})
ENDFOREACH()
###############################################################################
### -- 3rd party
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

######################################### SceneNet To Numpy ############################################
set(targetname exe_ExtractPoseFromScanNet)
SET(sources
	main.cpp
        )
SET(headers)
INCLUDE(UseEigen3)
INCLUDE(SetAppTarget)
target_link_libraries(${targetname}
        PRIVATE ImageLoader
		PRIVATE CxxTools
        )
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)
target_compile_definitions(${targetname} PUBLIC -DCOMPILE_WITHOUT_CUDA)
###############################################################################
### -- 3rd party
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
###########################################################################################
set(targetname SLAMBase)

file(GLOB srcs *.cpp *.tpp)
file(GLOB headers *.hpp *.h)
file(GLOB cuda src/*.cu)

SET(${targetname}_sources
    ${srcs}
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
			)
	target_include_directories(${targetname_suffix} PUBLIC ../)
	TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC VOXELTYPE=${type})
ENDFOREACH()
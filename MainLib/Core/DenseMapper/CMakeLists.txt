#######################
# DenseMapper LIBRARY #
#######################
set(targetname DenseMapper)
file(GLOB_RECURSE ${targetname}_headers *.h)
file(GLOB_RECURSE ${targetname}_sources
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
	SET(targetname_suffix ${targetname}${NAME_SUFFIX})
	ADD_LIBRARY(${targetname_suffix}
			${${targetname}_sources}
			${${targetname}_headers}
			)
	TARGET_LINK_LIBRARIES(${targetname_suffix}
			PUBLIC ReconstructionEngine${NAME_SUFFIX}
			PUBLIC SwappingEngine${NAME_SUFFIX}
			)
	TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC VOXELTYPE=${type})
ENDFOREACH()
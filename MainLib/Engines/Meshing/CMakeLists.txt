#########################
# Meshing LIBRARY #
#########################
set(targetname MeshingEngine)
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
	message("targetname_suffix " ${targetname_suffix} )

	ADD_LIBRARY(${targetname_suffix} OBJECT ${${targetname}_sources} ${${targetname}_headers})
	TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC VOXELTYPE=${type})
ENDFOREACH()

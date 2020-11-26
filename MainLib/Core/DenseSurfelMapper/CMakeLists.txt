#######################
# DenseMapper LIBRARY #
#######################
set(targetname SurfelDenseMapper)
file(GLOB_RECURSE ${targetname}_headers *.h)
file(GLOB_RECURSE ${targetname}_sources
        *.c
        *.cpp
        *.cu
        *.tpp
        *.tcu)

FOREACH(i RANGE ${SURFELCOUNTS} )
	LIST(GET SURFELNAMES ${i} name)
	LIST(GET SURFELTYPES ${i} type)
	SET(FIELD_TYPE SURFELTYPE=${type})
	SET(NAME_SUFFIX _${name})
	SET(targetname_suffix ${targetname}${NAME_SUFFIX})

	ADD_LIBRARY(${targetname_suffix}
			${${targetname}_sources}
			${${targetname}_headers}
			)
	TARGET_LINK_LIBRARIES(${targetname_suffix}
			PUBLIC SurfelReconstructionEngine${NAME_SUFFIX}
			PUBLIC SurfelVisualizationEngine${NAME_SUFFIX}
			)
	TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC SURFELTYPE=${type})
ENDFOREACH()
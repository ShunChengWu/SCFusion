############################
# VisualizationEngine LIBRARY #
############################
set(targetname VisualizationEngine)
file(GLOB_RECURSE ${targetname}_headers
		CPU/ITMMultiVisualisationEngine_CPU.h
		CPU/ITMVisualisationEngine_CPU.h
		CUDA/ITMMultiVisualisationEngine_CUDA.h
		CUDA/ITMVisualisationEngine_CUDA.h
		CUDA/ITMVisualisationHelpers_CUDA.h
		Interface/ITMMultiVisualisationEngine.h
		Interface/ITMVisualisationEngine.h
		Shared/ITMVisualisationEngine_Shared.h
)
file(GLOB_RECURSE ${targetname}_sources
		CPU/ITMMultiVisualisationEngine_CPU.tpp
		CPU/ITMVisualisationEngine_CPU.tpp
		CUDA/ITMMultiVisualisationEngine_CUDA.tcu
		CUDA/ITMVisualisationEngine_CUDA.tcu
		CUDA/ITMVisualisationHelpers_CUDA.cu
		Interface/ITMVisualisationEngine.cpp
		CPUVoxelInstantiations.cpp
		CUDAVoxelInstantiations.cu
)


FOREACH(i RANGE ${VOXELCOUNTS} )
	LIST(GET VOXELNAMES ${i} name)
	LIST(GET VOXELTYPES ${i} type)
	SET(FIELD_TYPE VOXELTYPE=${type})
	SET(NAME_SUFFIX _${name})
	SET(targetname_suffix ${targetname}${NAME_SUFFIX})
	ADD_LIBRARY(${targetname_suffix} OBJECT ${${targetname}_sources} ${${targetname}_headers})
	TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC VOXELTYPE=${type})
ENDFOREACH()

############################
# SurfelVisualizationEngine LIBRARY #
############################
IF(SURFELCOUNTS)
	SET(targetname SurfelVisualizationEngine)
	SET(${targetname}_headers
			CPU/ITMSurfelVisualisationEngine_CPU.h
			CUDA/ITMSurfelVisualisationEngine_CUDA.h
			Interface/ITMSurfelVisualisationEngine.h
			Shared/ITMSurfelVisualisationEngine_Settings.h
			Shared/ITMSurfelVisualisationEngine_Shared.h
			ITMSurfelVisualisationEngineFactory.h
			)
	SET(${targetname}_sources
			CPU/ITMSurfelVisualisationEngine_CPU.tpp
			CUDA/ITMSurfelVisualisationEngine_CUDA.tcu
			CUDA/ITMVisualisationHelpers_CUDA.cu
			Interface/ITMSurfelVisualisationEngine.tpp
			Interface/ITMVisualisationEngine.cpp
			ITMSurfelVisualisationEngineFactory.tpp
			CPUSurfelInstantiations.cpp
			CUDASurfelInstantiations.cu
			)
	FOREACH(i RANGE ${SURFELCOUNTS} )
		LIST(GET SURFELNAMES ${i} name)
		LIST(GET SURFELTYPES ${i} type)
		SET(FIELD_TYPE SURFELTYPE=${type})
		SET(NAME_SUFFIX _${name})
		SET(targetname_suffix ${targetname}${NAME_SUFFIX})
		ADD_LIBRARY(${targetname_suffix} OBJECT ${${targetname}_sources} ${${targetname}_headers})
		TARGET_COMPILE_DEFINITIONS(${targetname_suffix} PUBLIC SURFELTYPE=${type})
	ENDFOREACH()
ENDIF(SURFELCOUNTS)
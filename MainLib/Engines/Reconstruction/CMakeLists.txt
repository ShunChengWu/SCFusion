################################
# ReconstructionEngine LIBRARY #
################################
set(targetname ReconstructionEngine)

file(GLOB_RECURSE ${targetname}_headers
		CPU/ITMSceneReconstructionEngine_CPU.h
		CUDA/ITMSceneReconstructionEngine_CUDA.h
		Interface/ITMSceneReconstructionEngine.h
		Shared/ITMSceneReconstructionEngine_Shared.h
		ITMSceneReconstructionEngineFactory.h
)
file(GLOB_RECURSE ${targetname}_sources
		CPU/ITMSceneReconstructionEngine_CPU.tpp
		CUDA/ITMSceneReconstructionEngine_CUDA.tcu
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
# Surfel ReconstructionEngine LIBRARY #
############################
IF(SURFELCOUNTS)
	SET(targetname SurfelReconstructionEngine)
	SET(${targetname}_headers
			CPU/ITMSurfelSceneReconstructionEngine_CPU.h
			CUDA/ITMSurfelSceneReconstructionEngine_CUDA.h
			Interface/ITMSurfelSceneReconstructionEngine.h
			Shared/ITMSurfelSceneReconstructionEngine_Shared.h
			ITMSurfelSceneReconstructionEngineFactory.h
	)
	SET(${targetname}_sources
			CPU/ITMSceneReconstructionEngine_CPU.tpp
			CUDA/ITMSurfelSceneReconstructionEngine_CUDA.tcu
			Interface/ITMSurfelSceneReconstructionEngine.tpp
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
// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "ITMLibDefines.h"
#include "Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu"
#include "Engines/PointExtraction/CUDA/PointCloudExtractionEngine_CUDA.tcu"
#include "Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.tcu"
#include "Engines/Reconstruction/CUDA/ITMSurfelSceneReconstructionEngine_CUDA.tcu"
#include "Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu"
#include "Engines/Visualisation/CUDA/ITMSurfelVisualisationEngine_CUDA.tcu"
#include "Engines/Visualisation/CUDA/ITMVisualisationEngine_CUDA.tcu"
#include "Objects/Scene/UnitMapSaver/CUDA/UnitMapSaver_CUDA.tcu"


// Engines
template class ITMLib::ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class SCSLAM::PointCloudExtractionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSurfelSceneReconstructionEngine_CUDA<ITMLib::ITMSurfel_grey>;
template class ITMLib::ITMSurfelSceneReconstructionEngine_CUDA<ITMLib::ITMSurfel_rgb>;
template class ITMLib::ITMSwappingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSurfelVisualisationEngine_CUDA<ITMLib::ITMSurfel_grey>;
template class ITMLib::ITMSurfelVisualisationEngine_CUDA<ITMLib::ITMSurfel_rgb>;
template class SCSLAM::UnitMapSave_CUDA<ITMVoxel, ITMVoxelIndex>;

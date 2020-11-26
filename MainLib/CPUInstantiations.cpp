// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "ITMLibDefines.h"
#include "Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Engines/PointExtraction/CPU/PointCloudExtractionEngine_CPU.tpp"
#include "Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Engines/Reconstruction/CPU/ITMSurfelSceneReconstructionEngine_CPU.tpp"
#include "Engines/Reconstruction/Interface/ITMSurfelSceneReconstructionEngine.tpp"
#include "Engines/Reconstruction/ITMSurfelSceneReconstructionEngineFactory.tpp"
#include "Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Engines/Visualisation/CPU/ITMSurfelVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/MainEngine.tpp"
#include "Core/MeshEngine.tpp"
#include "Core/PointCloudEngine.tpp"

// Engines
template class ITMLib::ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class SCSLAM::PointCloudExtractionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSurfelSceneReconstructionEngine<ITMLib::ITMSurfel_grey>;
template class ITMLib::ITMSurfelSceneReconstructionEngine<ITMLib::ITMSurfel_rgb>;
template class ITMLib::ITMSurfelSceneReconstructionEngine_CPU<ITMLib::ITMSurfel_grey>;
template class ITMLib::ITMSurfelSceneReconstructionEngine_CPU<ITMLib::ITMSurfel_rgb>;
template struct ITMLib::ITMSurfelSceneReconstructionEngineFactory<ITMLib::ITMSurfel_grey>;
template struct ITMLib::ITMSurfelSceneReconstructionEngineFactory<ITMLib::ITMSurfel_rgb>;
template class ITMLib::ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSurfelVisualisationEngine<ITMLib::ITMSurfel_grey>;
template class ITMLib::ITMSurfelVisualisationEngine<ITMLib::ITMSurfel_rgb>;
template class ITMLib::ITMSurfelVisualisationEngine_CPU<ITMLib::ITMSurfel_grey>;
template class ITMLib::ITMSurfelVisualisationEngine_CPU<ITMLib::ITMSurfel_rgb>;
//Core
template class ITMLib::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
template class SCSLAM::MainEngine<ITMVoxel, ITMVoxelIndex>;
template class SCSLAM::MeshEngine<ITMVoxel, ITMVoxelIndex>;
template class SCSLAM::PointCloudEngine<ITMVoxel,ITMVoxelIndex>;

//Main
//template class SCFusion::SLAM<ITMVoxel, ITMVoxelIndex>;
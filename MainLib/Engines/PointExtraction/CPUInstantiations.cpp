// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "../../ITMLibDefines.h"
#include "CPU/PointCloudExtractionEngine_CPU.tpp"
#include "PointCloudEngine.tpp"

// Engines
template class SCFUSION::PointCloudExtractionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class SCFUSION::PointCloudEngine<ITMVoxel,ITMVoxelIndex>;

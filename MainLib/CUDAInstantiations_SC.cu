// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "ITMLibDefines.h"
#include "Engines/SceneCompletion/CUDA/SceneCompletionEngine_CUDA.tcu"

// Engines
template class SCSLAM::SceneCompletionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

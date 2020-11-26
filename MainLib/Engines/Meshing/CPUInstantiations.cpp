// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "../../ITMLibDefines.h"
#include "CPU/ITMMeshingEngine_CPU.tpp"
#include "MeshEngine.tpp"
// Engines
template class ITMLib::ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class SCFUSION::MeshEngine<ITMVoxel, ITMVoxelIndex>;

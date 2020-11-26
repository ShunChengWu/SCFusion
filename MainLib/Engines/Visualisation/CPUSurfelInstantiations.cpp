// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "../../ITMLibDefines.h"
#include "CPU/ITMSurfelVisualisationEngine_CPU.tpp"
#include "CPU/ITMVisualisationEngine_CPU.tpp"
#include "Interface/ITMSurfelVisualisationEngine.tpp"
#include "ITMSurfelVisualisationEngineFactory.tpp"
// Engines
//template class ITMLib::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMLib::ITMSurfelVisualisationEngine<ITMSurfelT>;
template class ITMLib::ITMSurfelVisualisationEngine_CPU<ITMSurfelT>;
template struct ITMLib::ITMSurfelVisualisationEngineFactory<ITMSurfelT>;
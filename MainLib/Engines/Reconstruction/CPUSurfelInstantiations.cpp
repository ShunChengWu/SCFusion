// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#include "../../ITMLibDefines.h"
#include "CPU/ITMSurfelSceneReconstructionEngine_CPU.tpp"
#include "Interface/ITMSurfelSceneReconstructionEngine.tpp"
#include "ITMSurfelSceneReconstructionEngineFactory.tpp"
// Engines
template class ITMLib::ITMSurfelSceneReconstructionEngine<ITMSurfelT>;
template class ITMLib::ITMSurfelSceneReconstructionEngine_CPU<ITMSurfelT>;
template struct ITMLib::ITMSurfelSceneReconstructionEngineFactory<ITMSurfelT>;
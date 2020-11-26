// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//#include "CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/SceneCompletionEngine_CUDA.h"
#include "../../Utils/ITMLibSettings.h"
#endif


namespace SCFUSION
{

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct SceneCompletionEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a scene reconstruction engine.
   *
   * \param deviceType  The device on which the scene reconstruction engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static SceneCompletionEngine<TVoxel,TIndex> *MakeSceneCompletionEngine(SceneCompletionMethod sceneCompletionMethod,
          const ITMLib::ITMLibSettings::DeviceType &deviceType)
  {
      SCFUSION::SceneCompletionEngine<TVoxel,TIndex> *sceneCompletionEngine = NULL;

    switch(deviceType)
    {
      case ITMLib::ITMLibSettings::DEVICE_CPU:
          //sceneCompletion = new SCFusion::SceneCompletion_CUDA<TVoxel,TIndex>;
//          std::runtime_error("Not Implemented");
        break;
      case ITMLib::ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
            sceneCompletionEngine = new SCFUSION::SceneCompletionEngine_CUDA<TVoxel,TIndex>(sceneCompletionMethod);
#endif
        break;
      case ITMLib::ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel,TIndex>;
#endif
        break;
    }

    return sceneCompletionEngine;
  }

//    template <typename TVoxel, typename TIndex>
//    static SceneCompletionEngine<TVoxel,TIndex> *MakeSceneCompletionEngine(
//            const ITMLib::ITMLibSettings::DeviceType &deviceType, const std::string& path_meta, const std::string& path_ckpt, const std::string& inputTensorName, const std::string& outputTensorName,
//            const std::vector<int>& inputDims, const std::vector<int>& outputDims, float gpu_fraction)
//    {
//        SCFusion::SceneCompletionEngine<TVoxel,TIndex> *sceneCompletionEngine = NULL;
//
//        switch(deviceType)
//        {
//            case ITMLib::ITMLibSettings::DEVICE_CPU:
//                //sceneCompletion = new SCFusion::SceneCompletion_CUDA<TVoxel,TIndex>;
////          std::runtime_error("Not Implemented");
//                break;
//            case ITMLib::ITMLibSettings::DEVICE_CUDA:
//#ifndef COMPILE_WITHOUT_CUDA
//                sceneCompletionEngine = new SCFusion::SceneCompletionEngine_CUDA<TVoxel,TIndex>(
//                        path_meta, path_ckpt, inputTensorName, outputTensorName, inputDims, outputDims, gpu_fraction
//                );
//#endif
//                break;
//            case ITMLib::ITMLibSettings::DEVICE_METAL:
//#ifdef COMPILE_WITH_METAL
//                sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel,TIndex>;
//#endif
//                break;
//        }
//
//        return sceneCompletionEngine;
//    }
};



}

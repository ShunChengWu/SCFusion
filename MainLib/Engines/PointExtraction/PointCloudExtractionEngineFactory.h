// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once


#include "../../Objects/Scene/ITMVoxelTypes.h"
#include "../../Utils/ITMLibSettings.h"
#include "CPU/PointCloudExtractionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/PointCloudExtractionEngine_CUDA.h"
#endif

namespace SCFUSION
{

	/**
	 * \brief This struct provides functions that can be used to construct meshing engines.
	 */
	struct PointCloudEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a meshing engine.
		 *
		 * \param deviceType  The device on which the meshing engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static PointCloudExtractionEngine<TVoxel, TIndex> *MakeMeshingEngine(ITMLib::ITMLibSettings::DeviceType deviceType)
		{
            PointCloudExtractionEngine<TVoxel, TIndex> *engine = NULL;

			switch (deviceType)
			{
			case ITMLib::ITMLibSettings::DEVICE_CPU:
                engine = new PointCloudExtractionEngine_CPU<TVoxel, TIndex> ();
				break;
			case ITMLib::ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
                    engine = new PointCloudExtractionEngine_CUDA<TVoxel, TIndex> ();
#endif
				break;
			case ITMLib::ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				engine = new ITMMeshingEngine_CPU<TVoxel, TIndex>;
#endif
				break;
			}

			return engine;
		}
	};
}
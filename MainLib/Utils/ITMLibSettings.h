// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "SceneCompletionParams.h"
#include <ORUtils/MemoryDeviceType.h>



namespace ITMLib
{
	class ITMLibSettings
	{
	public:
		/// The device used to run the DeviceAgnostic code
		typedef enum {
			DEVICE_CPU,
			DEVICE_CUDA,
			DEVICE_METAL
		} DeviceType;

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;
        
		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_BASIC_SURFELS,
			LIBMODE_LOOPCLOSURE
		} LibMode;

		/// Select the type of device to use
		DeviceType deviceType;
		/// Use Scene Complete
        bool useSC;
        /// Skip Input Frame. Image counter % useSkipFrame == 0? fuse : skip
        int useSkipFrame;

		bool useApproximateRaycast;

		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;

		bool createPointExtractionEngine;

		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		std::string trackerConfig;

		std::string labelColorPath;

		/// Further, scene specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;
		SCFUSION::SceneCompletionParams scParams;

		ITMLibSettings();
		virtual ~ITMLibSettings() {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);

		MemoryDeviceType GetMemoryType() const;


	};
}

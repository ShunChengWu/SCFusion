// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"
using namespace ITMLib;

#include <climits>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <sstream>

//#define TESTSIZE 4

ITMLibSettings::ITMLibSettings()
:	//sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, false),
//float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
//        sceneParams(0.015f, 100, 0.005f, 0.2f, 100.0f, false),
    sceneParams(0.141f, 100, 0.047f, 0.1f, 100.0f, true, false, SCFUSION::Policy::Integrate_WEIGHTED),
//	surfelSceneParams(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true),
    surfelSceneParams(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.05f, 0.1f, 3.5f, 10.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true),
    scParams("/home/deep/scslam/Model/ForkNet_20190424Occupancy.pb", "", "", "Placeholder_1", "div_6", 14, false, true, 0.05,
            /*80,48,80*/std::vector<int>{1,64,64,64}, std::vector<int>{1,64,64,64}, -1, 0.5, sceneParams.voxelSize,
             SCFUSION::Policy::FuseTwo::FuseTwo_OCCUPIED, SCFUSION::SceneCompletionMethod::SceneCompletionMethod_SceneInpainting,
             0.5, 4.8, 3, 0)
{
    // enable Scene Completion
    useSC = true;

    useSkipFrame = 0;

	// skips every other point when using the colour renderer for creating a point cloud
	skipPoints = true;

	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	createMeshingEngine = false;

    createPointExtractionEngine = false;

#ifndef COMPILE_WITHOUT_CUDA
	deviceType = DEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = DEVICE_METAL;
#else
	deviceType = DEVICE_CPU;
#endif
#endif

	//deviceType = DEVICE_CPU;

	/// how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version
	swappingMode = SWAPPINGMODE_DISABLED;
//    swappingMode = SWAPPINGMODE_ENABLED;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering
	useBilateralFilter = false;

	/// what to do on tracker failure: ignore, relocalise or stop integration - not supported in loop closure version
    behaviourOnFailure = FAILUREMODE_IGNORE;
    behaviourOnFailure = FAILUREMODE_RELOCALISE;

	/// switch between various library modes - basic, with loop closure, etc.
	libMode = LIBMODE_BASIC;
    
//	libMode = LIBMODE_LOOPCLOSURE;
//    libMode = LIBMODE_BASIC_SURFELS;

	//// Default ICP tracking
	//trackerConfig = "type=icp,levels=rrrbb,minstep=1e-3,"
	//				"outlierC=0.01,outlierF=0.002,"
	//				"numiterC=10,numiterF=2,failureDec=5.0"; // 5 for normal, 20 for loop closure

	// Depth-only extended tracker:
	trackerConfig = "type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
					  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
					  "numiterC=20,numiterF=50,tukeyCutOff=8,"
					  "framesToSkip=20,framesToWeight=50,failureDec=20.0";

	//// For hybrid intensity+depth tracking:
	trackerConfig = "type=extended,levels=bbb,useDepth=1,useColour=1,"
					  "colourWeight=0.3,minstep=1e-4,"
					  "outlierColourC=0.175,outlierColourF=0.005,"
					  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
					  "numiterC=20,numiterF=50,tukeyCutOff=8,"
					  "framesToSkip=20,framesToWeight=50,failureDec=20.0";

	// Colour only tracking, using rendered colours
	//trackerConfig = "type=rgb,levels=rrbb";

	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
	//trackerConfig = "type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

    labelColorPath = "./LabelColorList.txt";

	// Surfel tracking
	if(libMode == LIBMODE_BASIC_SURFELS)
	{
		trackerConfig = "extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0";
	}
}

MemoryDeviceType ITMLibSettings::GetMemoryType() const
{
	return deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
}
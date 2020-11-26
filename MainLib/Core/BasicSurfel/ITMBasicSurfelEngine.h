// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../DenseSurfelMapper/ITMDenseSurfelMapper.h"
#include "../ITMMainEngine.h"
#include "../ITMTrackingController.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../../Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h"
#include "../../Objects/Misc/ITMIMUCalibrator.h"
#include "../../FernRelocLib//Relocaliser.cpp"

namespace ITMLib
{
	template <typename TSurfel>
	class ITMBasicSurfelEngine : public ITMMainEngine
	{
    protected:
		const ITMLibSettings *settings;

		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		ITMLowLevelEngine *lowLevelEngine;
		ITMSurfelVisualisationEngine<TSurfel> *surfelVisualisationEngine;

		ITMViewBuilder *viewBuilder;
		ITMDenseSurfelMapper<TSurfel> *denseSurfelMapper;
		ITMTrackingController *trackingController;

		ITMSurfelScene<TSurfel> *surfelScene;
		ITMSurfelRenderState *surfelRenderState_live;
		ITMSurfelRenderState *surfelRenderState_freeview;

		ITMTracker *tracker;
		ITMIMUCalibrator *imuCalibrator;

		FernRelocLib::Relocaliser<float> *relocaliser;
		ITMUChar4Image *kfRaycast;

		/// Pointer for storing the current input frame
		ITMView *view;

		/// Pointer to the current camera pose and additional tracking information
		ITMTrackingState *trackingState;

		static typename ITMSurfelVisualisationEngine<TSurfel>::RenderImageType ToSurfelImageType(GetImageType getImageType);

        std::unique_ptr<ORUtils::MemoryBlock<Vector4f>> LabelColorList_;
	public:
		ITMView* GetView(void) override { return view; }
		ITMTrackingState* GetTrackingState(void) override { return trackingState; }

		virtual ITMTrackingState::TrackingResult ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                      ORUtils::Matrix4<float> *customPose, ITMLib::ITMIMUMeasurement *imuMeasurement, ITMUShortImage *imgLabel);

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName);

		/// save and load the full scene and relocaliser (if any) to/from file
        void SaveToFile(const std::string &pth_to_folder) override ;
		void LoadFromFile();

		/// Get a result image as output
		Vector2i GetImageSize(void) const override ;

		ORUtils::MemoryBlock<Vector4f> * GetLabelColorList() {return LabelColorList_? LabelColorList_.get():nullptr;}

		virtual void renderImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, ITMIntrinsics *intrinsics = nullptr,
                         ITMLib::IITMVisualisationEngine::RenderMode renderMode = ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE) = 0;

		/// switch for turning tracking on/off
		void turnOnTracking();
		void turnOffTracking();

		/// switch for turning integration on/off
		void turnOnIntegration();
		void turnOffIntegration();

		/// switch for turning main processing on/off
		void turnOnMainProcessing();
		void turnOffMainProcessing();

		/// resets the scene and the tracker
		void resetAll();

		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		ITMBasicSurfelEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib);
		~ITMBasicSurfelEngine();
	};
}

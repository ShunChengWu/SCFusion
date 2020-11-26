// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Objects/Misc/ITMIMUMeasurement.h"
#include "../Trackers/Interface/ITMTracker.h"
#include "../Utils/ITMLibSettings.h"

/** \mainpage
    This is the API reference documentation for InfiniTAM. For a general
    overview additional documentation can be found in the included Technical
    Report.

    For use of ITMLib in your own project, the class
    @ref ITMLib::Engine::ITMMainEngine should be the main interface and entry
    point to the library.
*/

namespace ITMLib
{
	/** \brief
	    Main engine, that instantiates all the other engines and
	    provides a simplified interface to them.

	    This class is the main entry point to the ITMLib library
	    and basically performs the whole KinectFusion algorithm.
	    It stores the latest image internally, as well as the 3D
	    world model and additionally it keeps track of the camera
	    pose.

	    The intended use is as follows:
	    -# Create an ITMMainEngine specifying the internal settings,
	       camera parameters and image sizes
	    -# Get the pointer to the internally stored images with
	       @ref GetView() and write new image information to that
	       memory
	    -# Call the method @ref ProcessFrame() to track the camera
	       and integrate the new information into the world model
	    -# Optionally access the rendered reconstruction or another
	       image for visualisation using @ref GetImage()
	    -# Iterate the above three steps for each image in the
	       sequence

	    To access the internal information, look at the member
	    variables @ref trackingState and @ref scene.
	*/
	class ITMMainEngine
	{
	public:
        enum GTPoseMODE {
            GTPOSEMODE_IGNORE,
            GTPOSEMODE_ASSIST,
            GTPOSEMODE_TACKOVER,
        };
        enum GetImageType {
            InfiniTAM_IMAGE_ORIGINAL_RGB,
            InfiniTAM_IMAGE_ORIGINAL_DEPTH,
            InfiniTAM_IMAGE_SCENERAYCAST,
            InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,
            InfiniTAM_IMAGE_COLOUR_FROM_NORMAL,
            InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE,
            InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL,
            InfiniTAM_IMAGE_LABEL_FROM_NORMAL,
            InfiniTAM_IMAGE_FREECAMERA_SHADED,
            InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME,
            InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL,
            InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE,
            InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_IMAGENORMAL,
            InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL,
            InfiniTAM_IMAGE_UNKNOWN,
        };
        static std::string getGetImageTypeString(size_t i){
            if(i>=GetImageType ::InfiniTAM_IMAGE_UNKNOWN) return "InfiniTAM_IMAGE_UNKNOWN";
            auto imageType = static_cast<GetImageType>(i);
            switch (imageType){
                case InfiniTAM_IMAGE_ORIGINAL_RGB:
                    return "InfiniTAM_IMAGE_ORIGINAL_RGB";
                case InfiniTAM_IMAGE_ORIGINAL_DEPTH:
                    return "InfiniTAM_IMAGE_ORIGINAL_DEPTH";
                case InfiniTAM_IMAGE_SCENERAYCAST:
                    return "InfiniTAM_IMAGE_SCENERAYCAST";
                case InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
                    return "InfiniTAM_IMAGE_COLOUR_FROM_VOLUME";
                case InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
                    return "InfiniTAM_IMAGE_COLOUR_FROM_NORMAL";
                case InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
                    return "InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE";
                case InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL:
                    return "InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL";
                case InfiniTAM_IMAGE_LABEL_FROM_NORMAL:
                    return "InfiniTAM_IMAGE_LABEL_FROM_NORMAL";
                case InfiniTAM_IMAGE_FREECAMERA_SHADED:
                    return "InfiniTAM_IMAGE_FREECAMERA_SHADED";
                case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
                    return "InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME";
                case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
                    return "InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL";
                case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
                    return "InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE";
                case InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_IMAGENORMAL:
                    return "InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_IMAGENORMAL";
                case InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL:
                    return "InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL";
                case InfiniTAM_IMAGE_UNKNOWN:
                    return "InfiniTAM_IMAGE_UNKNOWN";
            }
            return "";
        }

        virtual void setGTPoseMode (GTPoseMODE option) {gtPoseMode=option;}

		/// Gives access to the current input frame
		virtual ITMView* GetView(void) = 0;

		/// Gives access to the current camera pose and additional tracking information
		virtual ITMTrackingState* GetTrackingState(void) = 0;

		/// Process a frame with rgb and depth images and optionally a corresponding imu measurement
        virtual ITMTrackingState::TrackingResult ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                              ORUtils::Matrix4<float> *customPose=nullptr, ITMLib::ITMIMUMeasurement *imuMeasurement=nullptr, ITMUShortImage *imgLabel=nullptr) = 0;

		/// Get a result image as output
		virtual Vector2i GetImageSize(void) const = 0;

		virtual void renderImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, ITMIntrinsics *intrinsics = NULL,
                                 ITMLib::IITMVisualisationEngine::RenderMode renderMode = ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE) = 0;

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		virtual void SaveSceneToMesh(const char *fileName) { };

		/// save and load the full scene and relocaliser (if any) to/from file
		virtual void SaveToFile(const std::string &pth_to_folder) { };
		virtual void LoadFromFile() { };

		virtual ~ITMMainEngine() {}

	protected:
        GTPoseMODE gtPoseMode;
	};
}

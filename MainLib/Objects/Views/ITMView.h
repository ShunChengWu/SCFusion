// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Camera/ITMCalibIO.h"
#include "../../Utils/ITMImageTypes.h"
#include <memory>

namespace ITMLib
{
	/** \brief
	    Represents a single "view", i.e. RGB and depth images along
	    with all intrinsic and relative calibration information
	*/
	class ITMView
	{
	public:
		/// Intrinsic calibration information for the view.
		const ITMRGBDCalib calib;

		/// RGB colour image for the current frame.
		std::unique_ptr<ITMUChar4Image> rgb;

		/// RGB colour image for the previous frame.
        std::unique_ptr<ITMUChar4Image> rgb_prev;

        /// Float valued vertex image
        std::unique_ptr<ITMFloat3Image> vertex;
        
		/// Float valued depth image, if available according to @ref inputImageType.
        std::unique_ptr<ITMFloatImage> depth;

		/// surface normal of depth image
		// allocated when needed
        std::unique_ptr<ITMFloat4Image> depthNormal;

		/// uncertainty (std) in each pixel of depth value based on sensor noise model
		/// allocated when needed
        std::unique_ptr<ITMFloatImage> depthUncertainty;

		/// confidence based on distance from center
        std::unique_ptr<ITMFloatImage> depthConfidence;
        
        /// Label image
        std::unique_ptr<ITMUShortImage> label;
        ITMIntrinsics intrinsics_label;

        /// Semantic Label image
        std::unique_ptr<ITMUShortImage> semanticlabel;

		ITMView(const ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
		: calib(calibration)
		{
		    Reset(imgSize_rgb, imgSize_d, useGPU);
		}

		void Reset(Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU){
            this->rgb.reset( new ITMUChar4Image(imgSize_rgb, true, useGPU) );
            this->rgb_prev = nullptr;
            this->depth.reset( new ITMFloatImage(imgSize_d, true, useGPU) );
            this->depthNormal = nullptr;
            this->depthUncertainty = nullptr;
            this->depthConfidence.reset( new ITMFloatImage(imgSize_d, true, useGPU) );
            this->label = nullptr;
            this->semanticlabel = nullptr;
		}

		virtual ~ITMView() = default;

		// Suppress the default copy constructor and assignment operator
		ITMView(const ITMView&);
		ITMView& operator=(const ITMView&);
	};
}

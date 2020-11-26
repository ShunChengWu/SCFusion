// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Camera/ITMRGBDCalib.h"
#include "../../../Objects/Views/ITMViewIMU.h"
#include <memory>
//TODO: add an option here
//#include <InSegLabelling/InSegLabeling.h>

namespace ITMLib
{
	/** \brief
	*/
	class ITMViewBuilder
	{
	protected:
        const ITMRGBDCalib calib;
        std::unique_ptr<ITMShortImage> shortImage;
        std::unique_ptr<ITMFloatImage> floatImage;
        std::unique_ptr<ITMUShortImage> labelImage;
        float scaleLabel;
	public:
        virtual void setStream(void *stream) = 0;
        virtual void syncStream() = 0;

		virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
			Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) = 0;

		virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
        virtual void ComputeVertex(ITMFloat3Image *vertex_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0;
		virtual void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloat3Image *vertex_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;
        virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;
        virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, ITMUShortImage *labelImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;
		virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;
        virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;

        void SetLabelImgScale(float scale) { scaleLabel = scale;}
        float GetLabelImgScale(){return scaleLabel;}

		ITMViewBuilder(const ITMRGBDCalib& calib_)
		: calib(calib_), scaleLabel(1.f)
		{
			this->shortImage = nullptr;
			this->floatImage = nullptr;
            this->labelImage = nullptr;
		}

		virtual ~ITMViewBuilder() = default;
	};
}

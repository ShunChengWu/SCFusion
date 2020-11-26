// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMViewBuilder.h"
//#include "../../Segmentation/CUDA/InSegLabeling_CUDA.h"
#include <iostream>
namespace ITMLib
{
	class ITMViewBuilder_CUDA : public ITMViewBuilder
	{
    private:
	    cudaStream_t stream_;
	public:
        void setStream(void *stream) override {stream_ = static_cast<cudaStream_t>(stream);}
        void syncStream() override {cudaStreamSynchronize(stream_);};

		void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics, 
			Vector2f disparityCalibParams) override;
		void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) override;

		void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) override;
        
        void ComputeVertex(ITMFloat3Image *vertex_out, const ITMFloatImage *depth_in, Vector4f intrinsic) override;
		void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloat3Image *vertex_in, Vector4f intrinsic) override;

		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true)override ;
        void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) override;
        void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, ITMUShortImage *labelImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true)  override;
		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) override;
        void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise = false, bool storePreviousImage = true) override;

		explicit ITMViewBuilder_CUDA(const ITMRGBDCalib& calib);
		~ITMViewBuilder_CUDA() override;

	private:
	    void BuildLabelImage(ITMView **view_ptr, Vector2i dims, bool useCPU, bool useCUDA);
	};
}

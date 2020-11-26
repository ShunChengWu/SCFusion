// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../Shared/ITMViewBuilder_Shared.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib& calib):ITMViewBuilder(calib) {}
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{ 
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		this->shortImage.reset( new ITMShortImage(rawDepthImage->noDims, true, false) );
		this->floatImage.reset( new ITMFloatImage(rawDepthImage->noDims, true, false) );

		if (modelSensorNoise || useLabelImage)
		{
            (*view_ptr)->vertex.reset( new ITMFloat3Image(rawDepthImage->noDims, true, false) );
			(*view_ptr)->depthNormal.reset( new ITMFloat4Image(rawDepthImage->noDims, true, false) );
			(*view_ptr)->depthUncertainty.reset( new ITMFloatImage(rawDepthImage->noDims, true, false) );
		}
        
        if (useLabelImage) {
            (*view_ptr)->label.reset( new ITMUShortImage(rawDepthImage->noDims, true, false) );
            (*view_ptr)->semanticlabel.reset( new ITMUShortImage(rawDepthImage->noDims, true, false) );
        }
	}
	ITMView *view = *view_ptr;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev.reset( new ITMUChar4Image(rgbImage->noDims, true, false) );
		else view->rgb_prev->SetFrom(view->rgb.get(), MemoryBlock<Vector4u>::CPU_TO_CPU);
	}

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);

	switch (view->calib.disparityCalib.GetType())
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth.get(), this->shortImage.get(), &(view->calib.intrinsics_d), view->calib.disparityCalib.GetParams());
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth.get(), this->shortImage.get(), view->calib.disparityCalib.GetParams());
		break;
	default:
		break;
	}

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage.get(), view->depth.get());
		this->DepthFiltering(view->depth.get(), this->floatImage.get());
		this->DepthFiltering(this->floatImage.get(), view->depth.get());
		this->DepthFiltering(view->depth.get(), this->floatImage.get());
		this->DepthFiltering(this->floatImage.get(), view->depth.get());
		view->depth->SetFrom(this->floatImage.get(), MemoryBlock<float>::CPU_TO_CPU);
	}

	if (modelSensorNoise || useLabelImage)
	{
        this->ComputeVertex(view->vertex.get(), view->depth.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
		this->ComputeNormalAndWeights(view->depthNormal.get(), view->depthUncertainty.get(), view->vertex.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
	}
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
    if (*view_ptr == nullptr)
    {
        *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
        this->shortImage.reset( new ITMShortImage(rawDepthImage->noDims, true, false) );
        this->floatImage.reset( new ITMFloatImage(rawDepthImage->noDims, true, false) );

        if (modelSensorNoise || useLabelImage)
        {
            (*view_ptr)->vertex.reset( new ITMFloat3Image(rawDepthImage->noDims, true, false) );
            (*view_ptr)->depthNormal.reset( new ITMFloat4Image(rawDepthImage->noDims, true, false) );
            (*view_ptr)->depthUncertainty.reset( new ITMFloatImage(rawDepthImage->noDims, true, false) );
        }

        if (useLabelImage) {
            (*view_ptr)->label.reset( new ITMUShortImage(rawDepthImage->noDims, true, false) );
            (*view_ptr)->semanticlabel.reset( new ITMUShortImage(rawDepthImage->noDims, true, false) );
        }
    }
    ITMView *view = *view_ptr;

    if (storePreviousImage)
    {
        if (!view->rgb_prev) view->rgb_prev.reset( new ITMUChar4Image(rgbImage->noDims, true, false) );
        else view->rgb_prev->SetFrom(view->rgb.get(), MemoryBlock<Vector4u>::CPU_TO_CPU);
    }

    view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
    view->depth->SetFrom(rawDepthImage, MemoryBlock<float>::CPU_TO_CPU);

    if (useBilateralFilter)
    {
        //5 steps of bilateral filtering
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        view->depth->SetFrom(this->floatImage.get(), MemoryBlock<float>::CPU_TO_CPU);
    }

    if (modelSensorNoise || useLabelImage)
    {
        this->ComputeVertex(view->vertex.get(), view->depth.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
        this->ComputeNormalAndWeights(view->depthNormal.get(), view->depthUncertainty.get(), view->vertex.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
    }
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, ITMUShortImage *labelImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage) {
    if(labelImage != nullptr) useLabelImage = true;
    if (*view_ptr == nullptr)
    {
        if(rgbImage != nullptr)
            *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
        else if (rawDepthImage != nullptr)
            *view_ptr = new ITMView(calib, rawDepthImage->noDims, rawDepthImage->noDims, false);
        else
            throw std::runtime_error("Must give at least a depth image or a rgb image\n");
//        *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
//        this->shortImage.reset( new ITMShortImage(rawDepthImage->noDims, true, false) );
        this->floatImage.reset( new ITMFloatImage(rawDepthImage->noDims, true, false) );

        if (modelSensorNoise || useLabelImage)
        {
            (*view_ptr)->vertex.reset( new ITMFloat3Image(rawDepthImage->noDims, true, false) );
            (*view_ptr)->depthNormal.reset( new ITMFloat4Image(rawDepthImage->noDims, true, false) );
            (*view_ptr)->depthUncertainty.reset( new ITMFloatImage(rawDepthImage->noDims, true, false) );
        }

        if (useLabelImage) {
            (*view_ptr)->label.reset( new ITMUShortImage(rawDepthImage->noDims, true, false) );
            (*view_ptr)->semanticlabel.reset( new ITMUShortImage(rawDepthImage->noDims, true, false) );
        }
    }
    ITMView *view = *view_ptr;

    if (storePreviousImage && rgbImage)
    {
        if (!view->rgb_prev) view->rgb_prev.reset( new ITMUChar4Image(rgbImage->noDims, true, false) );
        else view->rgb_prev->SetFrom(view->rgb.get(), MemoryBlock<Vector4u>::CPU_TO_CPU);
    }

    if(rgbImage != nullptr) view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
    if(rawDepthImage != nullptr) view->depth->SetFrom(rawDepthImage, MemoryBlock<float>::CPU_TO_CPU);

    if (useBilateralFilter)
    {
        //5 steps of bilateral filtering
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        view->depth->SetFrom(this->floatImage.get(), MemoryBlock<float>::CPU_TO_CPU);
    }

    if (modelSensorNoise || useLabelImage)
    {
        view->label->SetFrom(labelImage, MemoryBlock<ushort>::MemoryCopyDirection::CPU_TO_CPU);
        this->ComputeVertex(view->vertex.get(), view->depth.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
        this->ComputeNormalAndWeights(view->depthNormal.get(), view->depthUncertainty.get(), view->vertex.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
    }
}


void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
		this->shortImage.reset( new ITMShortImage(depthImage->noDims, true, false));
		this->floatImage.reset(new ITMFloatImage(depthImage->noDims, true, false));

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal.reset( new ITMFloat4Image(depthImage->noDims, true, false));
			(*view_ptr)->depthUncertainty.reset( new ITMFloatImage(depthImage->noDims, true, false));
		}
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
    if (*view_ptr == NULL)
    {
        *view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
        this->shortImage.reset( new ITMShortImage(depthImage->noDims, true, false));
        this->floatImage.reset( new ITMFloatImage(depthImage->noDims, true, false));

        if (modelSensorNoise)
        {
            (*view_ptr)->depthNormal.reset( new ITMFloat4Image(depthImage->noDims, true, false));
            (*view_ptr)->depthUncertainty.reset( new ITMFloatImage(depthImage->noDims, true, false));
        }
    }

    ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
    imuView->imu->SetFrom(imuMeasurement);

    this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

void ITMViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		filterDepth(imout, imin, x, y, imgSize);
}

void ITMViewBuilder_CPU::ComputeVertex(ITMFloat3Image *vertex_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
    Vector2i imgDims = depth_in->noDims;
    const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);
    
    Vector3f *vertexData_out = vertex_out->GetData(MEMORYDEVICE_CPU);
    
    for (int y = 0; y < imgDims.y; y++) for (int x = 0; x < imgDims.x; x++)
        computeVertex(depthData_in, vertexData_out, x, y, imgDims, intrinsic);
}

void ITMViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloat3Image *vertex_in, Vector4f intrinsic)
{
	Vector2i imgDims = vertex_in->noDims;

	const Vector3f *vertexData_in = vertex_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(vertexData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}


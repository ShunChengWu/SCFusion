// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CUDA.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include <ORUtils/CUDADefines.h>
#include <ORUtils/MemoryBlock.h>
#include <ORUtils/Logging.h>

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CUDA::ITMViewBuilder_CUDA(const ITMRGBDCalib& calib):ITMViewBuilder(calib), stream_(0){}
ITMViewBuilder_CUDA::~ITMViewBuilder_CUDA(void) { }

//---------------------------------------------------------------------------
//
// kernel function declaration 
//
//---------------------------------------------------------------------------


__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);
__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams);
__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void ComputeVertex_device(const float* depth_in, Vector3f* vertex_out, Vector2i imgDims, Vector4f intrinsic);
__global__ void ComputeNormalAndWeight_device(const Vector3f* depth_in, Vector4f* normal_out, float *sigmaL_out, int2 imgDims, float4 intrinsic);

//---------------------------------------------------------------------------
//
// host methods
//
//---------------------------------------------------------------------------

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == nullptr)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
		this->shortImage.reset( new ITMShortImage(rawDepthImage->noDims, true, true) );
		this->floatImage.reset( new ITMFloatImage(rawDepthImage->noDims, true, true) );

        if (modelSensorNoise || useLabelImage)
        {
            (*view_ptr)->vertex.reset( new ITMFloat3Image(rawDepthImage->noDims, true, true) );
			(*view_ptr)->depthNormal.reset( new ITMFloat4Image(rawDepthImage->noDims, true, true) );
			(*view_ptr)->depthUncertainty.reset( new ITMFloatImage(rawDepthImage->noDims, true, true) );
		}

        if (useLabelImage) {
            (*view_ptr)->label.reset( new ITMUShortImage(rawDepthImage->noDims, true, true) );
            (*view_ptr)->semanticlabel.reset( new ITMUShortImage(rawDepthImage->noDims, true, true) );
        }
	}

	ITMView *view = *view_ptr;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev.reset( new ITMUChar4Image(rgbImage->noDims, true, true) );
		else view->rgb_prev->SetFrom(view->rgb.get(), MemoryBlock<Vector4u>::CUDA_TO_CUDA, stream_);
	}

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA, stream_);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA, stream_);

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
//        SCLOG(VERBOSE) << "Bilateral filtering. 5 steps.";

		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage.get(), view->depth.get());
		this->DepthFiltering(view->depth.get(), this->floatImage.get());
		this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
		view->depth->SetFrom(this->floatImage.get(), MemoryBlock<float>::CUDA_TO_CUDA, stream_);
	}

	if (modelSensorNoise || useLabelImage)
	{
        this->ComputeVertex(view->vertex.get(), view->depth.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
		this->ComputeNormalAndWeights(view->depthNormal.get(), view->depthUncertainty.get(), view->vertex.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
	}
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
    if (*view_ptr == nullptr)
    {
        if(rgbImage != nullptr)
            *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
        else if (rawDepthImage != nullptr)
            *view_ptr = new ITMView(calib, rawDepthImage->noDims, rawDepthImage->noDims, true);
        else
            throw std::runtime_error("Must give at least a depth image or a rgb image\n");
//        if (this->shortImage != nullptr) delete this->shortImage;
//        this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
        this->floatImage.reset( new ITMFloatImage(rawDepthImage->noDims, true, true) );

        if (modelSensorNoise || useLabelImage)
        {
            (*view_ptr)->vertex.reset( new ITMFloat3Image(rawDepthImage->noDims, true, true) );
            (*view_ptr)->depthNormal.reset( new ITMFloat4Image(rawDepthImage->noDims, true, true) );
            (*view_ptr)->depthUncertainty.reset( new ITMFloatImage(rawDepthImage->noDims, true, true) );
        }

        if (useLabelImage) {
            BuildLabelImage(view_ptr, rawDepthImage->noDims, true,true);
        }
    }

    ITMView *view = *view_ptr;

    if (storePreviousImage && rgbImage)
    {
        if (!view->rgb_prev) view->rgb_prev.reset( new ITMUChar4Image(rgbImage->noDims, true, true) );
        else view->rgb_prev->SetFrom(view->rgb.get(), MemoryBlock<Vector4u>::CUDA_TO_CUDA, stream_);
    }

    if(rgbImage != nullptr) view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA, stream_);
    if(rawDepthImage != nullptr) view->depth->SetFrom(rawDepthImage, MemoryBlock<float>::CPU_TO_CUDA, stream_);

    if (useBilateralFilter)
    {
        SCLOG(VERBOSE) << "Bilateral filtering. 5 steps.";
        //5 steps of bilateral filtering
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        view->depth->SetFrom(this->floatImage.get(), MemoryBlock<float>::CUDA_TO_CUDA, stream_);
    }

    if (modelSensorNoise || useLabelImage)
    {
        this->ComputeVertex(view->vertex.get(), view->depth.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
        this->ComputeNormalAndWeights(view->depthNormal.get(), view->depthUncertainty.get(), view->vertex.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
    }
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, ITMUShortImage *labelImage, bool useBilateralFilter, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage) {
    if(labelImage != nullptr) useLabelImage = true;
    if (*view_ptr == nullptr)
    {
        if(rgbImage != nullptr)
            *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
        else if (rawDepthImage != nullptr)
            *view_ptr = new ITMView(calib, rawDepthImage->noDims, rawDepthImage->noDims, true);
        else
            throw std::runtime_error("Must give at least a depth image or a rgb image\n");
//        if (this->shortImage != nullptr) delete this->shortImage;
//        this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
        this->floatImage.reset( new ITMFloatImage(rawDepthImage->noDims, true, true) );

        if (modelSensorNoise || useLabelImage)
        {
            (*view_ptr)->vertex.reset( new ITMFloat3Image(rawDepthImage->noDims, true, true) );
            (*view_ptr)->depthNormal.reset( new ITMFloat4Image(rawDepthImage->noDims, true, true) );
            (*view_ptr)->depthUncertainty.reset( new ITMFloatImage(rawDepthImage->noDims, true, true) );
        }

        if (useLabelImage) {
            BuildLabelImage(view_ptr, rawDepthImage->noDims, true,true);
        }
    }

    ITMView *view = *view_ptr;

    if (storePreviousImage && rgbImage)
    {
        if (!view->rgb_prev) view->rgb_prev.reset( new ITMUChar4Image(rgbImage->noDims, true, true) );
        else view->rgb_prev->SetFrom(view->rgb.get(), MemoryBlock<Vector4u>::CUDA_TO_CUDA, stream_);
    }

    if(rgbImage != nullptr) view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA, stream_);
    if(rawDepthImage != nullptr) view->depth->SetFrom(rawDepthImage, MemoryBlock<float>::CPU_TO_CUDA, stream_);

    if (useBilateralFilter)
    {
//        SCLOG(VERBOSE) << "Bilateral filtering. 5 steps.";

        //5 steps of bilateral filtering
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());
        this->DepthFiltering(view->depth.get(), this->floatImage.get());
        this->DepthFiltering(this->floatImage.get(), view->depth.get());

//        for(size_t i=0;i<10;++i){
//            this->DepthFiltering(this->floatImage.get(), view->depth.get());
//            this->DepthFiltering(view->depth.get(), this->floatImage.get());
//            this->DepthFiltering(this->floatImage.get(), view->depth.get());
//            this->DepthFiltering(view->depth.get(), this->floatImage.get());
//            this->DepthFiltering(this->floatImage.get(), view->depth.get());
//        }

        view->depth->SetFrom(this->floatImage.get(), MemoryBlock<float>::CUDA_TO_CUDA, stream_);
    }

    if (modelSensorNoise || useLabelImage)
    {
        if(labelImage != nullptr) {
            view->label->SetFrom(labelImage, MemoryBlock<ushort>::MemoryCopyDirection::CPU_TO_CPU, stream_);
            view->label->UpdateDeviceFromHost(true, stream_);
        }
        this->ComputeVertex(view->vertex.get(), view->depth.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
        this->ComputeNormalAndWeights(view->depthNormal.get(), view->depthUncertainty.get(), view->vertex.get(), view->calib.intrinsics_d.projectionParamsSimple.all);
    }
}


void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == nullptr)
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, true);
		this->shortImage.reset( new ITMShortImage(depthImage->noDims, true, true) );
		this->floatImage.reset( new ITMFloatImage(depthImage->noDims, true, true) );

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal.reset(  new ITMFloat4Image(depthImage->noDims, true, true) );
			(*view_ptr)->depthUncertainty.reset( new ITMFloatImage(depthImage->noDims, true, true) );
		}
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool useLabelImage, bool modelSensorNoise, bool storePreviousImage)
{
    if (*view_ptr == nullptr)
    {
        *view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, true);
        this->shortImage.reset(new ITMShortImage(depthImage->noDims, true, true) );
        this->floatImage.reset( new ITMFloatImage(depthImage->noDims, true, true) );

        if (modelSensorNoise)
        {
            (*view_ptr)->depthNormal.reset( new ITMFloat4Image(depthImage->noDims, true, true) );
            (*view_ptr)->depthUncertainty.reset( new ITMFloatImage(depthImage->noDims, true, true) );
        }
    }

    ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
    imuView->imu->SetFrom(imuMeasurement);

    this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CUDA::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDisparityToDepth_device << <gridSize, blockSize, 0, stream_ >> >(d_out, d_in, disparityCalibParams, fx_depth, imgSize);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDepthAffineToFloat_device << <gridSize, blockSize, 0, stream_ >> >(d_out, d_in, imgSize, depthCalibParams);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgDims = image_in->noDims;

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	filterDepth_device << <gridSize, blockSize, 0, stream_ >> >(imageData_out, imageData_in, imgDims);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ComputeVertex(ITMFloat3Image *vertex_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
    Vector2i imgDims = depth_in->noDims;
    const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CUDA);
    Vector3f *vertexData_out = vertex_out->GetData(MEMORYDEVICE_CUDA);
    
    dim3 blockSize(16, 16);
    dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

    ComputeVertex_device <<< gridSize, blockSize, 0, stream_ >>>(depthData_in, vertexData_out, imgDims, intrinsic);
    ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloat3Image *vertex_in, Vector4f intrinsic)
{
	Vector2i imgDims = vertex_in->noDims;

	const Vector3f *vertexData_in = vertex_in->GetData(MEMORYDEVICE_CUDA);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	ComputeNormalAndWeight_device << <gridSize, blockSize, 0, stream_ >> >(vertexData_in, normalData_out, sigmaZData_out, make_int2(imgDims.x,imgDims.y), make_float4(intrinsic.x,intrinsic.y,intrinsic.z,intrinsic.w));

	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::BuildLabelImage(ITMView **view_ptr, Vector2i dims, bool useCPU, bool useCUDA) {
    (*view_ptr)->intrinsics_label = calib.intrinsics_d.MakeRescaled(scaleLabel);
    Vector2i scaledDims = {(int)std::round(dims.width*scaleLabel), (int)std::round(dims.height*scaleLabel)};
    (*view_ptr)->label.reset( new ITMUShortImage(scaledDims, useCPU, useCUDA) );

    //TOOD: maybe change semanticLabel to scaledDims as well?
    (*view_ptr)->semanticlabel.reset( new ITMUShortImage(dims, useCPU, useCUDA) );
}

//---------------------------------------------------------------------------
//
// kernel function implementation
//
//---------------------------------------------------------------------------

__global__ void convertDisparityToDepth_device(float *d_out, const short *d_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2) return;

	filterDepth(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void ComputeVertex_device(const float* depth_in, Vector3f* vertex_out, Vector2i imgDims, Vector4f intrinsic)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    if (x < 0 || x > imgDims.x || y < 0 || y > imgDims.y)
        return;
    else
        computeVertex(depth_in, vertex_out, x, y, imgDims, intrinsic);
}

__global__ void ComputeNormalAndWeight_device(const Vector3f* vertex_in, Vector4f* normal_out, float *sigmaZ_out, int2 imgDims, float4 intrinsic)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	int idx = x + y * imgDims.x;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}
	else
	{
		computeNormalAndWeight(vertex_in, normal_out, sigmaZ_out, x, y, Vector2i(imgDims.x,imgDims.y), Vector4f(intrinsic.x,intrinsic.y,intrinsic.z,intrinsic.w));
	}
}


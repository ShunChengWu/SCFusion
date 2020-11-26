#include <SLAMGUI/gui_kernel.hpp>
//#include <MainEngine/ITMLibDefines.h>
//#include "../../SCFusion/ITMLibDefines.h"

#define CUDA_1D_LOOP(i, n)                       \
for (int i = blockIdx.x * blockDim.x + threadIdx.x;     \
i < (n);                                                \
i += blockDim.x * gridDim.x)
namespace {
    int GET_1D_BLOCKS(int n, int num_thread = 512) {
        return (n + num_thread - 1) / num_thread;
    }

    int threadPerBlock = 512;
}

_CPU_AND_GPU_CODE_ void HSVtoRGB(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV) {
    float fC = fV * fS; // Chroma
    float fHPrime = fmod(fH / 60.f, 6.f);
    float fX = fC * (1 - fabs(fmod(fHPrime, 2.f) - 1));
    float fM = fV - fC;

    if(0 <= fHPrime && fHPrime < 1) {
        fR = fC;
        fG = fX;
        fB = 0;
    } else if(1 <= fHPrime && fHPrime < 2) {
        fR = fX;
        fG = fC;
        fB = 0;
    } else if(2 <= fHPrime && fHPrime < 3) {
        fR = 0;
        fG = fC;
        fB = fX;
    } else if(3 <= fHPrime && fHPrime < 4) {
        fR = 0;
        fG = fX;
        fB = fC;
    } else if(4 <= fHPrime && fHPrime < 5) {
        fR = fX;
        fG = 0;
        fB = fC;
    } else if(5 <= fHPrime && fHPrime < 6) {
        fR = fC;
        fG = 0;
        fB = fX;
    } else {
        fR = 0;
        fG = 0;
        fB = 0;
    }

    fR += fM;
    fG += fM;
    fB += fM;
}
_CPU_AND_GPU_CODE_ void float2RGB_shared(const float &value, const float &nearPlane, const float &farPlane, ORUtils::Vector4<unsigned char> *RGB) {
    float H,s=1,v=1,r,g,b;
    if(value <= nearPlane || value > farPlane - nearPlane) {
        RGB->x = 0;
        RGB->y = 0;
        RGB->z = 0;
        return;
    }
    H = 250 * (value - nearPlane)/(farPlane - nearPlane);
    HSVtoRGB(r,g,b,H,s,v);
    RGB->x = r*255;
    RGB->y = g*255;
    RGB->z = b*255;
    RGB->w = 255;
}
__global__ void float2RGB_kernel(int width, int height, const float *data, ORUtils::Vector4<unsigned char> *output, float nearP, float farP){
    /// Inversely
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
    if( x >= width || y >= height ){
        return;
    }
    int H = height-1-y;

    int index = H * width + x;
    float2RGB_shared(data[y*width+x], nearP, farP, &output[index]);
}

void float2RGB(const ORUtils::Image<float> *depthImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream){
    dim3 threads(16,16);
    dim3 blocks( (image->noDims.x + threads.x-1) / threads.x, (image->noDims.y + threads.y-1) / threads.y );
    float2RGB_kernel<<< blocks, threads, 0, stream >>> (image->noDims.x, image->noDims.y, depthImage->GetDataConst(MEMORYDEVICE_CUDA), image->GetData(MEMORYDEVICE_CUDA,false), nearPlane, farPlane);
    cudaStreamSynchronize(stream);
}

void float2RGB_cpu(const ORUtils::Image<float> *depthImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream){
    uint width = depthImage->noDims.x, height = depthImage->noDims.y;
    auto *data = depthImage->GetData(MEMORYDEVICE_CPU);
    ORUtils::Vector4<unsigned char> *output = image->GetData(MEMORYDEVICE_CPU);
    for(uint x=0;x<depthImage->noDims.x; ++x) for (uint y=0;y<depthImage->noDims.y; ++y)
    {
        if( x >= width || y >= height ){
            continue;
        }
        int H = height-1-y;

        int index = H * width + x;
        float2RGB_shared(data[y*width+x], nearPlane, farPlane, &output[index]);
    }
    cudaStreamSynchronize(stream);
}

_CPU_AND_GPU_CODE_ void float2Height_shared(int w, int h, const float &value, const float &render_low_bound, const float &render_high_bound, ORUtils::Vector4<unsigned char> *RGB) {
    float H,s=1,v=1,r,g,b;
    if(value <= render_low_bound || value >= render_high_bound) {
        RGB->x = 0;
        RGB->y = 0;
        RGB->z = 0;
        return;
    } else {
        H = 240 * (value - render_low_bound) /
             (render_high_bound - render_low_bound);
    }
    HSVtoRGB(r,g,b,H,s,v);
    RGB->x = r*255;
    RGB->y = g*255;
    RGB->z = b*255;
    RGB->w = 255;
}

__global__ void float3ToHeight_kernel(int width, int height, ORUtils::Vector3<float> *data, ORUtils::Vector4<unsigned char> *output, float nearP, float farP){
    /// Inversely
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
    if( x >= width || y >= height ){
        return;
    }

    int H = height-1-y;

    int index = H * width + x;
    int index_vert = y*width+x;

    output[index].w = 255;

    if(data[index_vert ].x==0&&data[index_vert ].y==0&&data[index_vert ].z==0){
        output[index].x = 0;
        output[index].y = 0;
        output[index].z = 0;
        return;
    }

    float2Height_shared(x, H, data[index_vert ].y, nearP, farP, &output[index]);
}

void float3ToHeight(ORUtils::Image<ORUtils::Vector3<float>> *vertexImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream){
    dim3 threads(16,16);
    dim3 blocks( (image->noDims.x + threads.x-1) / threads.x, (image->noDims.y + threads.y-1) / threads.y );
    float3ToHeight_kernel<<< blocks, threads, 0, stream >>> (image->noDims.x, image->noDims.y, vertexImage->GetData(MEMORYDEVICE_CUDA), image->GetData(MEMORYDEVICE_CUDA), nearPlane, farPlane);
    cudaStreamSynchronize(stream);
}
void float3ToHeight_cpu(ORUtils::Image<ORUtils::Vector3<float>> *vertexImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream){
    uint width = image->noDims.x, height = image->noDims.y;
    auto *output = image->GetData(MEMORYDEVICE_CPU);
    auto *data   = vertexImage->GetData(MEMORYDEVICE_CPU);
    for(uint x=0;x<width;++x) for (uint y=0;y<height;++y) {
            if( x >= width || y >= height ){
                continue;
            }

            int H = height-1-y;

            int index = H * width + x;
            int index_vert = y*width+x;

            output[index_vert].w = 255;

            if(data[index_vert ].x==0&&data[index_vert ].y==0&&data[index_vert ].z==0){
                output[index].x = 0;
                output[index].y = 0;
                output[index].z = 0;
                continue;
            }
            float2Height_shared(x, H, data[index_vert ].y, nearPlane, farPlane, &output[index]);
    }
}

//__global__ void InvSensorModel2Occupancy_kernel(int size, float *value)
//{
//    CUDA_1D_LOOP(i,size){
//        value[i] = 1.f - 1.f/(1.f+expf(value[i]));
//    }
//}

//void InvSensorModel2Occupancy(float *data, int size, cudaStream_t stream)
//{
//    //int size = (int)volume->volumeSize();
//    InvSensorModel2Occupancy_kernel<<<GET_1D_BLOCKS(size, threadPerBlock), threadPerBlock, 0, stream>>>(size, data);
//}


template <SCFUSION::IntegrateType T>
__global__ void occupaiedScan_kernel(int3 dims, float *data, uint *occupied){
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x  >= dims.x - 1 || y >= dims.y - 1) return;

    for (int z = 0; z < dims.z - 1; ++z) {
        int idx = (z*dims.y+y)*dims.x+x;
        float value = data[idx];
        occupied[idx] = 0;
        switch (T) {
//            case IntegrateType::Binary: {
//                if(value >= 1)
//                    occupied[idx] = 1;
//                break;
//            }
            case SCFUSION::IntegrateType::IntegrateType_OFusion: {
                float Probability = 1 - 1/(1+exp(value));
                if (Probability > 0.5f){
                    occupied[idx] = 1;
                }
                if(value > 0) occupied[idx] = 1;
                break;
            }
            case SCFUSION::IntegrateType::IntegrateType_TSDF: {
                if(value==0 || value <= -0.99 || value >= 0.99 )
                    continue;

                // Check Zero Crossing
                //DEBUG: turn on me again, or set a new display case
                if(false)
                {
                    float tx = data[(z*dims.y+y)*dims.x+x+1];
                    float ty = data[(z*dims.y+y+1)*dims.x+x];
                    float tz = data[((z+1)*dims.y+y)*dims.x+x];

                    // Check zero-crossing
                    bool isSurface = ( (value > 0) && (tx < 0 || ty < 0 || tz < 0 ) ) || ( (value < 0) && (tx > 0 || ty > 0 || tz > 0 ) );
                    if( isSurface  ){
                        occupied[idx] = 1;
                    }
                }
                else
                    occupied[idx] = 1;
                break;
            }
//            case IntegrateType ::InvertTSDF: {
//                if (abs(value) > 0.8f && abs(value) < 1.0f) {
//                    occupied[idx] = 1;
//                }
//                break;
//            }
        }
    }
}





template <typename  MaskType>
__global__ void copyDataWithMask_kernel(int size, float *data_from, MaskType *mask_from, float *data_to) {
    CUDA_1D_LOOP(i,size){
        data_to[i]=mask_from[i]>0?data_from[i]:0;
    }
}
void copyDataWithMask(float *data_from, float *mask_from, float *data_to, int size, cudaStream_t stream){
    copyDataWithMask_kernel<float> <<< GET_1D_BLOCKS(size,threadPerBlock),threadPerBlock,0,stream >>>(size, data_from,mask_from,data_to);
    ORcudaSafeCall(cudaStreamSynchronize(stream));
}

#include <thrust/device_vector.h>
#include <thrust/scan.h>

template <typename T>
void  volumeChecker (std::string name, T *volume, size_t size) {
    size_t counter =0;
    size_t counter_inf = 0;
    size_t counter_nan = 0;
    size_t counter_zero = 0;
    for(size_t  i=0;i<size;++i){
        if(volume[i]>0) counter++;
        if(isnanf(volume[i])) counter_nan++;
        if(isinff(volume[i])) counter_inf++;
        if(volume[i] == 0) counter_zero++;
    }
    printf("[%s] value:%zu(%f) NAN:%zu(%f) INF:%zu(%f) ZERO:%zu(%f) \n", name.c_str(),
           counter, float(counter)/float(size),
           counter_nan, float(counter_nan)/float(size),
           counter_inf, float(counter_inf)/float(size),
           counter_zero, float(counter_zero)/float(size));
}

void Volume2PointCloudExtractor::occupancyScan() {
    dim3 threads(16, 16);
    dim3 blocks( (dims_.x + threads.x-1) / threads.x, (dims_.y + threads.y-1) / threads.y );
    occupied_->Clear(0, 1, stream_);
    switch (type_){
        case SCFUSION::IntegrateType_TSDF:
            occupaiedScan_kernel<SCFUSION::IntegrateType_TSDF><<< blocks, threads, 0 , stream_ >>> (dims_, data_, occupied_->GetData(MEMORYDEVICE_CUDA));
            break;
//        case InvertTSDF:
////            occupaiedScan_kernel<InvertTSDF><<< blocks, threads, 0, stream_ >>> (dims_, data_, occupied_->GetData(MEMORYDEVICE_CUDA));
//            break;
        case SCFUSION::IntegrateType_OFusion:
            occupaiedScan_kernel<SCFUSION::IntegrateType_OFusion><<< blocks, threads, 0, stream_ >>> (dims_, data_, occupied_->GetData(MEMORYDEVICE_CUDA));
            break;
//        case Binary:
////            occupaiedScan_kernel<Binary><<< blocks, threads, 0 ,stream_ >>> (dims_, data_, occupied_->GetData(MEMORYDEVICE_CUDA));
//            break;
//        case Label:
//            throw "Inpute data type Label doesn't support yet.";
    }
    ORcudaSafeCall(cudaStreamSynchronize(stream_));

//    occupied_->UpdateHostFromDevice();
//    volumeChecker("occupied", occupied_->data<Util::CPU>(), occupied_->dataSize());
}

__global__ void maskUnInitialized_kernel(int size, float*data_in, float *mask, float *data_out){
    CUDA_1D_LOOP(i,size){
        data_out[i] = mask[i]>0? data_in[i]:0;
    }
}
void maskUnInitialized(int size, float *data_in, float *weight_in, float *data_out, cudaStream_t stream){
    maskUnInitialized_kernel<<< GET_1D_BLOCKS(size, threadPerBlock), threadPerBlock, 0, stream >>>(
            size,data_in,weight_in,data_out
    );
}

/// Count total valid entries and allocate memory
void Volume2PointCloudExtractor::compactIndicesAllocation(){
    ORcudaSafeCall(cudaStreamSynchronize(stream_));
    thrust::exclusive_scan(thrust::device_ptr<unsigned int>(occupied_->GetData(MEMORYDEVICE_CUDA)),
                           thrust::device_ptr<unsigned int>(occupied_->GetData(MEMORYDEVICE_CUDA) + occupied_->dataSize),
                           thrust::device_ptr<unsigned int>(voxelOccupiedScan_->GetData(MEMORYDEVICE_CUDA))
    );

    uint activeVoxels;
    {
        uint lastElement, lastScanElement;
        ORcudaSafeCall(cudaMemcpyAsync((void *) &lastElement,
                                    (void *)(occupied_->GetData(MEMORYDEVICE_CUDA) + occupied_->dataSize-1),
                                    sizeof(uint), cudaMemcpyDeviceToHost, stream_));
        ORcudaSafeCall(cudaMemcpyAsync((void *) &lastScanElement,
                                    (void *)(voxelOccupiedScan_->GetData(MEMORYDEVICE_CUDA) + voxelOccupiedScan_->dataSize-1),
                                    sizeof(uint), cudaMemcpyDeviceToHost, stream_));
        ORcudaSafeCall(cudaStreamSynchronize(stream_));
        activeVoxels = lastElement + lastScanElement;
        activeVoxels_ = activeVoxels;
    }
    if(activeVoxels_==0) return;

    if(compactedVoxelArray_) {
        compactedVoxelArray_->Resize(activeVoxels);
    }
    else {
        compactedVoxelArray_.reset( new ORUtils::MemoryBlock<uint>(activeVoxels, false, true) );
    }
}


__global__ void generateCompactVoxelIndexArray_kernel (int size, uint *voxelOccupied, uint *voxelOccupiedScan, uint *compactedVoxelArray) {
    CUDA_1D_LOOP(i, size) {
        if ( ((uint)i < size))
        {
            if(voxelOccupied[i])
                if(voxelOccupiedScan[i] < size)
                    compactedVoxelArray[ voxelOccupiedScan[i] ] = i;
        }
    }
}

/// copy indices from sparse array to dense array
void Volume2PointCloudExtractor::generateCompactVoxelIndexArray(){
    generateCompactVoxelIndexArray_kernel <<< GET_1D_BLOCKS(occupied_->dataSize, threadPerBlock), threadPerBlock, 0, stream_ >>>
       (
               occupied_->dataSize, occupied_->GetData(MEMORYDEVICE_CUDA), voxelOccupiedScan_->GetData(MEMORYDEVICE_CUDA), compactedVoxelArray_->GetData(MEMORYDEVICE_CUDA)
       );
    ORcudaSafeCall(cudaStreamSynchronize(stream_));
//    compactedVoxelArray_->UpdateHostFromDevice(1, stream_);
//    for(size_t i=0; i < compactedVoxelArray_->dataSize; ++i){
//        printf("compactedVoxelArray_[%zu]: %d\n", i, compactedVoxelArray_->data<Util::CPU>()[i]);
//    }
}


template<Volume2PointCloudExtractor::COLORMODE  mode>
__global__ void indices2PointCloudWithColor_kernel (int size, float3 base, int3 dims, float voxel_size,
                                                    uint* indices_in, float *data_in, int *labels_in, float4 *labelColors_, float4 *points_out, float4 *colors_out, SCFUSION::IntegrateType type) {
    CUDA_1D_LOOP(i, size) {
        int idx = indices_in[i];
        int label = labels_in[idx];

        switch (mode) {
            case Volume2PointCloudExtractor::COLORMODE ::LABEL_COLOR:{
                colors_out[i] = labelColors_[label];
                break;
            }
            case Volume2PointCloudExtractor::COLORMODE ::VALUE_COLOR:{
                if(type == SCFUSION::IntegrateType_OFusion) {
                    ORUtils::Vector4<unsigned char> colorout;
                    float2RGB_shared( (1 - 1/(1+exp(data_in[idx])) - 0.5) * 2, -1.0, 1.0, &colorout);
                    colors_out[i].x = float(colorout.x) / 255.0;
                    colors_out[i].y = float(colorout.y) / 255.0;
                    colors_out[i].z = float(colorout.z) / 255.0;
                    colors_out[i].w = float(colorout.w);
                } else {
                    ORUtils::Vector4<unsigned char> colorout;
                    float2RGB_shared(data_in[idx], -1.0, 1.0, &colorout);
                    colors_out[i].x = float(colorout.x) / 255.0;
                    colors_out[i].y = float(colorout.y) / 255.0;
                    colors_out[i].z = float(colorout.z) / 255.0;
                    colors_out[i].w = float(colorout.w);
//                colors_out[i].w = 1 - data_in[idx];
                }
                break;
            }
            case Volume2PointCloudExtractor::COLORMODE ::POS_NEG:{
                colors_out[i].x = data_in[idx] > 0 ? 1.0 : 0;
                colors_out[i].y = 0;
                colors_out[i].z = data_in[idx] > 0 ? 0 : 1;
                colors_out[i].w = 1 - data_in[idx];
//                colors_out[i].w = 1;
                break;
            }
        }

        int z = floorf(idx / (dims.x * dims.y));
        int y = floorf((idx - (z * dims.x * dims.y)) / dims.x);
        int x = idx - (z * dims.x * dims.y) - (y * dims.x);

        points_out[i].x = base.x + voxel_size * x;
        points_out[i].y = base.y + voxel_size * y;
        points_out[i].z = base.z + voxel_size * z;
        points_out[i].w = 1;
    }
}

void Volume2PointCloudExtractor::indices2PointCloudWithColor(){
    switch (renderingMode_){
        case COLORMODE::LABEL_COLOR:
            indices2PointCloudWithColor_kernel<LABEL_COLOR> <<<GET_1D_BLOCKS(activeVoxels_, threadPerBlock), threadPerBlock, 0, stream_>>>
             ( activeVoxels_, base_, dims_, voxel_size_, compactedVoxelArray_->GetData(MEMORYDEVICE_CUDA), data_, label_, labelColor_, points_out_, colors_out_, type_);
            break;
        case COLORMODE ::VALUE_COLOR:
            indices2PointCloudWithColor_kernel<VALUE_COLOR> <<<GET_1D_BLOCKS(activeVoxels_, threadPerBlock), threadPerBlock, 0, stream_>>>
             ( activeVoxels_, base_, dims_, voxel_size_, compactedVoxelArray_->GetData(MEMORYDEVICE_CUDA), data_, label_, labelColor_, points_out_, colors_out_, type_);
            break;
        case COLORMODE ::POS_NEG:
            indices2PointCloudWithColor_kernel<POS_NEG> <<<GET_1D_BLOCKS(activeVoxels_, threadPerBlock), threadPerBlock, 0, stream_>>>
             ( activeVoxels_, base_, dims_, voxel_size_, compactedVoxelArray_->GetData(MEMORYDEVICE_CUDA), data_, label_, labelColor_, points_out_, colors_out_, type_);
            break;
    }

    ORcudaSafeCall(cudaStreamSynchronize(stream_));
};


void Volume2PointCloudExtractor::compute() {
    occupancyScan();
    compactIndicesAllocation();
    generateCompactVoxelIndexArray();
    indices2PointCloudWithColor();
}
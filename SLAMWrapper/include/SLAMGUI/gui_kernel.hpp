#pragma once
#include <cuda_runtime.h>
#include <memory>
//#include <Utilities/MemoryBlock.hpp>
//#include <Utilities/Image.h>
#include "../../../ORUtils/Image.h"
#include "../../../MainLib/Objects/Scene/ITMVoxelTypes.h"

void float2RGB(const ORUtils::Image<float> *depthImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream);
void float2RGB_cpu(const ORUtils::Image<float> *depthImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream);
void float3ToHeight(ORUtils::Image<ORUtils::Vector3<float>> *vertexImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream);
void float3ToHeight_cpu(ORUtils::Image<ORUtils::Vector3<float>> *vertexImage, ORUtils::Image<ORUtils::Vector4<unsigned char>> *image, float nearPlane, float farPlane, cudaStream_t stream);
void InvSensorModel2Occupancy(float *data, int size, cudaStream_t stream);

void maskUnInitialized(int size, float *data_in, float *weight_in, float *data_out, cudaStream_t stream);

void copyDataWithMask(float *data_from, float *mask_from, float *data_to, int size, cudaStream_t stream);

class Volume2PointCloudExtractor {
public:
    enum COLORMODE{
        LABEL_COLOR, VALUE_COLOR, POS_NEG
    };

    Volume2PointCloudExtractor(float *data_in, int *label_in, float4 *labelColor_in, float4 *point_out, float4 *color_out):
    data_(data_in),label_(label_in), labelColor_(labelColor_in),points_out_(point_out), colors_out_(color_out), activeVoxels_(0){
        occupied_.reset(new ORUtils::MemoryBlock<uint>(1,false, true));
        voxelOccupiedScan_.reset(new ORUtils::MemoryBlock<uint>(1,false, true));
        compactedVoxelArray_.reset(new ORUtils::MemoryBlock<uint>(1,false, true));
    }
    void setParams(float3 base, int3 dims, float voxel_size, SCFUSION::IntegrateType type, cudaStream_t stream){
        base_ = base;
        dims_ = dims;
        voxel_size_ = voxel_size;
        type_ = type;
        stream_ = stream;
        occupied_->Resize(dims.x*dims.y*dims.z);
        voxelOccupiedScan_->Resize(dims.x*dims.y*dims.z);
    }
    void setMode(COLORMODE mode) {
        renderingMode_ = mode;
    }
    void compute();
    uint getNumVerts(){return activeVoxels_;}
private:
    float *data_;
    int *label_;
    float4 *labelColor_;
    float4 *points_out_;
    float4 *colors_out_;
    float3 base_;
    int3 dims_;
    float voxel_size_;
    SCFUSION::IntegrateType type_;
    std::unique_ptr<ORUtils::MemoryBlock<uint>> occupied_, voxelOccupiedScan_, compactedVoxelArray_;
    cudaStream_t stream_;
    uint activeVoxels_;
    COLORMODE  renderingMode_;

    void occupancyScan();
    void compactIndicesAllocation();
    void generateCompactVoxelIndexArray();
    void indices2PointCloudWithColor();
};
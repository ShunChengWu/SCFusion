#pragma once

#include <cstdlib>
#include <cmath>
#include <cuda_runtime.h>
#include <cassert>
#include <stdexcept>
#include <vector>

#ifdef __DRIVER_TYPES_H__
static const char *_cudaGetErrorEnum(cudaError_t error) {
    return cudaGetErrorName(error);
}
#endif
template <typename T>
void check(T result, char const *const func, const char *const file,
           int const line) {
    if (result) {
        fprintf(stderr, "CUDA error at %s:%d code=%d(%s) \"%s\" \n", file, line,
                static_cast<unsigned int>(result), _cudaGetErrorEnum(result), func);
        exit(EXIT_FAILURE);
    }
}
#define checkCudaErrors(val) check((val), #val, __FILE__, __LINE__)

struct VoxelInfo {
    float3 bbox_max,bbox_min;
    uint3 gridSize;
    size_t n_triangles;
    float3 unit;

    VoxelInfo(float3 bbox_min, float3 bbox_max, float voxelSize, unsigned int numTriangles)
            : bbox_max(bbox_max),bbox_min(bbox_min), n_triangles(numTriangles){
        unit = make_float3(voxelSize,voxelSize,voxelSize);
        gridSize.x = std::ceil((bbox_max.x - bbox_min.x) / voxelSize);
        gridSize.y = std::ceil((bbox_max.y - bbox_min.y) / voxelSize);
        gridSize.z = std::ceil((bbox_max.z - bbox_min.z) / voxelSize);
        assert(bbox_min.x + voxelSize*gridSize.x >= bbox_max.x);
        assert(bbox_min.y + voxelSize*gridSize.y >= bbox_max.y);
        assert(bbox_min.z + voxelSize*gridSize.z >= bbox_max.z);
    }
};

class MeshVoxelizer {

public:
    MeshVoxelizer(bool useThrustPath = false, bool forceCPU=false):
    vtable(nullptr),ltable(nullptr), itable(nullptr), voxelInfo(nullptr), useThrustPath(useThrustPath), forceCPU(forceCPU){}
    ~MeshVoxelizer(){
        if(forceCPU) {
            if (vtable) delete[]vtable;
            if(ltable) delete[]ltable;
            if(itable) delete[]itable;
        } else {
            cudaFree(vtable);
            cudaFree(ltable);
            cudaFree(itable);
        }
        if(voxelInfo) delete voxelInfo;
    }
    /**
     * @param points should be a contiguous float points of each triangle meshes.
     * @param size the total size of input points. e.g. You have 3 triangle meshes which each consists of three points: size = 3 * 3
     * @param bbox_min The range of the input points
     * @param bbox_max The range of the input points
     * @param voxelSize Voxel resolution
     */
    void compute(float *points, size_t size, const float3 &bbox_min, const float3 &bbox_max, float voxelSize);
    /**
     * @param points should be a contiguous float points of each triangle meshes.
     * @param labels The size of labels should size 1/9 of the size of points. i.e. each triangle has one label.
     * @param size the total size of input points. e.g. You have 3 triangle meshes which each consists of three points: size = 3 * 3
     * @param bbox_min The range of the input points
     * @param bbox_max The range of the input points
     * @param voxelSize Voxel resolution
     */
    void compute(float *points, unsigned int *labels, size_t size, const float3 &bbox_min, const float3 &bbox_max, float voxelSize);
    void compute(float *points, unsigned int *labels,unsigned int *instances, size_t size, const float3 &bbox_min, const float3 &bbox_max, float voxelSize);

    void getPointCloud(std::vector<float3> &points);
    void getLabels(std::vector<unsigned int> &labels);
    void getOutput(std::vector<float3> &points, std::vector<unsigned int> &labels);
    void getOutput(std::vector<float3> &points, std::vector<unsigned int> &labels, std::vector<unsigned int> &instances);

    unsigned int *getVolumeTable(){return vtable;}
    unsigned int *getLabelTable(){return ltable;}
    unsigned int *getInstanceTable(){return itable;}
    VoxelInfo * getVoxelInfo(){return voxelInfo;}
private:
    unsigned int *vtable;
    unsigned int *ltable;
    unsigned int *itable;
    VoxelInfo *voxelInfo;
    bool useThrustPath;
    bool forceCPU;

//    bool initCuda(bool verbose = false);
    void voxelize(const VoxelInfo& v, float* triangle_data, unsigned int* vtable, bool useThrustPath);
    void voxelize(const VoxelInfo& v, float* triangle_data, unsigned int *label_data, unsigned int* vtable, unsigned  int* ltable, bool useThrustPath);
    void voxelize(const VoxelInfo& v, float* triangle_data, unsigned int *label_data, unsigned int *instance_data, unsigned int* vtable, unsigned  int* ltable, unsigned  int* itable, bool useThrustPath);
};
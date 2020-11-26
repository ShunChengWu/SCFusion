#include "MeshVoxelizer.h"
//#include <stdio.h>
#include <cstring>
#include "helper_math.h"
//#include <Utilities/helper_math.h>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgproc.hpp>


void MeshVoxelizer::compute(float *points, size_t size, const float3 &bbox_min, const float3 &bbox_max, float voxelSize){
    voxelInfo = new VoxelInfo (bbox_min,bbox_max,voxelSize, size/9); 
    size_t vtable_size = static_cast<size_t>(voxelInfo->gridSize.x)* static_cast<size_t>(voxelInfo->gridSize.y)* static_cast<size_t>(voxelInfo->gridSize.z) *
                         sizeof(unsigned int);

    if(!forceCPU) {
        float* device_triangles;
        // Transfer triangles to GPU using either thrust or managed cuda memory
        if (useThrustPath) {
            throw std::runtime_error("Voxelize mesh with thrust is not supported yet.\n");
            //device_triangles = meshToGPU_thrust(themesh);
        } else {
            checkCudaErrors(cudaMallocManaged((void**) &device_triangles, sizeof(float)*size)); // managed memory
            cudaMemcpy(device_triangles, points, sizeof(float)*size,cudaMemcpyHostToDevice);
        }
        if(useThrustPath) {

        } else {
            checkCudaErrors(cudaMallocManaged((void**)&vtable, vtable_size));
        }
        voxelize(*voxelInfo, device_triangles, vtable, useThrustPath);
    } else {
//            if (!forceCPU) { fprintf(stdout, "[Info] No suitable CUDA GPU was found: Falling back to CPU voxelization\n"); }
//            else { fprintf(stdout, "[Info] Doing CPU voxelization (forced using command-line switch -cpu)\n"); }
//            vtable = (unsigned int*) calloc(1, vtable_size);
//            cpu_voxelizer::cpu_voxelize_mesh(voxelInfo, themesh, vtable, (outputformat == OutputFormat::output_morton));
    }
}

void MeshVoxelizer::compute(float *points, uint *labels, size_t size, const float3 &bbox_min, const float3 &bbox_max, float voxelSize){
    voxelInfo = new VoxelInfo (bbox_min,bbox_max,voxelSize, size/9);
    size_t vtable_size = static_cast<size_t>(voxelInfo->gridSize.x)* static_cast<size_t>(voxelInfo->gridSize.y)* static_cast<size_t>(voxelInfo->gridSize.z) *
                         sizeof(unsigned int);
    if(!forceCPU) {
        float* device_triangles;
        uint* device_labels;
        // Transfer triangles to GPU using either thrust or managed cuda memory
        if (useThrustPath) {
            throw std::runtime_error("Voxelize mesh with thrust is not supported yet.\n");
            //device_triangles = meshToGPU_thrust(themesh);
        } else {
            checkCudaErrors(cudaMallocManaged((void**) &device_triangles, sizeof(float)*size)); // managed memory
            checkCudaErrors(cudaMallocManaged((void**) &device_labels, sizeof(uint)*size/9)); // managed memory
            cudaMemcpy(device_triangles, points, sizeof(float)*size,cudaMemcpyHostToDevice);
            cudaMemcpy(device_labels, labels, sizeof(uint)*size/9, cudaMemcpyHostToDevice);
        }
        if(useThrustPath) {

        } else {
            checkCudaErrors(cudaMallocManaged((void**)&vtable, vtable_size));
            checkCudaErrors(cudaMallocManaged((void**)&ltable, vtable_size));
        }
//        voxelize(*voxelInfo, device_triangles, vtable, useThrustPath);
        voxelize(*voxelInfo,device_triangles,device_labels,vtable,ltable,useThrustPath);

        cudaFree(device_triangles);
        cudaFree(device_labels);
    } else {
//            if (!forceCPU) { fprintf(stdout, "[Info] No suitable CUDA GPU was found: Falling back to CPU voxelization\n"); }
//            else { fprintf(stdout, "[Info] Doing CPU voxelization (forced using command-line switch -cpu)\n"); }
//            vtable = (unsigned int*) calloc(1, vtable_size);
//            cpu_voxelizer::cpu_voxelize_mesh(voxelInfo, themesh, vtable, (outputformat == OutputFormat::output_morton));
    }
}

void MeshVoxelizer::compute(float *points, uint *labels, uint *instances, size_t size, const float3 &bbox_min, const float3 &bbox_max, float voxelSize){
    voxelInfo = new VoxelInfo (bbox_min,bbox_max,voxelSize, size/9);
    size_t vtable_size = static_cast<size_t>(voxelInfo->gridSize.x)* static_cast<size_t>(voxelInfo->gridSize.y)* static_cast<size_t>(voxelInfo->gridSize.z) *
                         sizeof(unsigned int);
    if(!forceCPU) {
        float* device_triangles;
        uint* device_labels;
        uint* device_instances;
        // Transfer triangles to GPU using either thrust or managed cuda memory
        if (useThrustPath) {
            throw std::runtime_error("Voxelize mesh with thrust is not supported yet.\n");
            //device_triangles = meshToGPU_thrust(themesh);
        } else {
            checkCudaErrors(cudaMallocManaged((void**) &device_triangles, sizeof(float)*size)); // managed memory
            checkCudaErrors(cudaMallocManaged((void**) &device_labels, sizeof(uint)*size/9)); // managed memory
            checkCudaErrors(cudaMallocManaged((void**) &device_instances, sizeof(uint)*size/9)); // managed memory
            cudaMemcpy(device_triangles, points, sizeof(float)*size,cudaMemcpyHostToDevice);
            cudaMemcpy(device_labels, labels, sizeof(uint)*size/9, cudaMemcpyHostToDevice);
            cudaMemcpy(device_instances, instances, sizeof(uint)*size/9, cudaMemcpyHostToDevice);
        }
        if(useThrustPath) {

        } else {
            checkCudaErrors(cudaMallocManaged((void**)&vtable, vtable_size));
            checkCudaErrors(cudaMallocManaged((void**)&ltable, vtable_size));
            checkCudaErrors(cudaMallocManaged((void**)&itable, vtable_size));
        }
        cudaDeviceSynchronize();
        voxelize(*voxelInfo,device_triangles,device_labels,device_instances,vtable,ltable,itable,useThrustPath);

        cudaFree(device_triangles);
        cudaFree(device_labels);
        cudaFree(device_instances);
    } else {
//            if (!forceCPU) { fprintf(stdout, "[Info] No suitable CUDA GPU was found: Falling back to CPU voxelization\n"); }
//            else { fprintf(stdout, "[Info] Doing CPU voxelization (forced using command-line switch -cpu)\n"); }
//            vtable = (unsigned int*) calloc(1, vtable_size);
//            cpu_voxelizer::cpu_voxelize_mesh(voxelInfo, themesh, vtable, (outputformat == OutputFormat::output_morton));
    }
}


void MeshVoxelizer::getPointCloud(std::vector<float3> &points) {
    if(!vtable) throw std::runtime_error("did not compute yet!\n");
    for(size_t x=0; x < voxelInfo->gridSize.x; ++x){
        for(size_t y=0;y<voxelInfo->gridSize.y;++y){
            for(size_t z=0;z<voxelInfo->gridSize.z;++z){
                if(vtable[(z*voxelInfo->gridSize.y+y)*voxelInfo->gridSize.x+x] == 0) continue;
                points.emplace_back(make_float3(x*voxelInfo->unit.x,y*voxelInfo->unit.y,z*voxelInfo->unit.z) + voxelInfo->bbox_min);
            }
        }
    }
}

void MeshVoxelizer::getLabels(std::vector<unsigned int> &labels) {
    if(!ltable) throw std::runtime_error("did not compute yet!\n");
    labels.clear();
    labels.reserve(voxelInfo->gridSize.x*voxelInfo->gridSize.y*voxelInfo->gridSize.z);
    for(size_t x=0; x < voxelInfo->gridSize.x; ++x){
        for(size_t y=0;y<voxelInfo->gridSize.y;++y){
            for(size_t z=0;z<voxelInfo->gridSize.z;++z){
                uint idx = (z*voxelInfo->gridSize.y+y)*voxelInfo->gridSize.x+x;
                if(vtable[idx] == 0) continue;
                labels.emplace_back(ltable[idx]);
            }
        }
    }
}

void MeshVoxelizer::getOutput(std::vector<float3> &points, std::vector<unsigned int> &labels){
    if(!vtable) throw std::runtime_error("did not compute yet!\n");
    if(!ltable) throw std::runtime_error("did not compute yet!\n");
    points.clear();
    labels.clear();
    points.reserve(voxelInfo->gridSize.x*voxelInfo->gridSize.y*voxelInfo->gridSize.z);
    labels.reserve(voxelInfo->gridSize.x*voxelInfo->gridSize.y*voxelInfo->gridSize.z);
    for(size_t x=0; x < voxelInfo->gridSize.x; ++x){
        for(size_t y=0;y<voxelInfo->gridSize.y;++y){
            for(size_t z=0;z<voxelInfo->gridSize.z;++z){
                uint idx = (z*voxelInfo->gridSize.y+y)*voxelInfo->gridSize.x+x;
                if(vtable[idx] == 0) continue;
                points.emplace_back(make_float3(x*voxelInfo->unit.x,y*voxelInfo->unit.y,z*voxelInfo->unit.z) + voxelInfo->bbox_min);
                labels.emplace_back(ltable[idx]);
            }
        }
    }
}

void MeshVoxelizer::getOutput(std::vector<float3> &points, std::vector<unsigned int> &labels, std::vector<unsigned int> &instances){
    if(!vtable) throw std::runtime_error("did not compute yet!\n");
    if(!ltable) throw std::runtime_error("did not compute yet!\n");
    if(!itable) throw std::runtime_error("did not compute yet!\n");
    points.clear();
    labels.clear();
    instances.clear();
    points.reserve(voxelInfo->gridSize.x*voxelInfo->gridSize.y*voxelInfo->gridSize.z);
    labels.reserve(voxelInfo->gridSize.x*voxelInfo->gridSize.y*voxelInfo->gridSize.z);
    instances.reserve(voxelInfo->gridSize.x*voxelInfo->gridSize.y*voxelInfo->gridSize.z);
    for(size_t x=0; x < voxelInfo->gridSize.x; ++x){
        for(size_t y=0;y<voxelInfo->gridSize.y;++y){
            for(size_t z=0;z<voxelInfo->gridSize.z;++z){
                uint idx = (z*voxelInfo->gridSize.y+y)*voxelInfo->gridSize.x+x;
                if(vtable[idx] == 0) continue;
                points.emplace_back(make_float3(x*voxelInfo->unit.x,y*voxelInfo->unit.y,z*voxelInfo->unit.z) + voxelInfo->bbox_min);
                labels.emplace_back(ltable[idx]);
                instances.emplace_back(itable[idx]);
            }
        }
    }
}
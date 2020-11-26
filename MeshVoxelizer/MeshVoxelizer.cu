#include "MeshVoxelizer.h"
//#include <cuda_runtime.h>
//#include <cuda_runtime_api.h>
//#define GLM_FORCE_CUDA
//#define GLM_FORCE_PURE
//#include <glm/glm.hpp>
//#include <Utilities/helper_math.h>
#include "helper_math.h"
//
//float3 make_float3(float x, float y, float z){
//    float3 t;
//    t.x = x;
//    t.y = y;
//    t.z = z;
//    return t;
//}
//float3 operator -(float3 a, float3 b){
//    return make_float3(a.x-b.x,a.y-b.x,a.z-b.z);
//}

__global__ void voxelize_triangle(VoxelInfo info, float* triangle_data, unsigned int* voxel_table);
__global__ void voxelize_triangle_and_label(VoxelInfo info, float* triangle_data, unsigned int* label_data, unsigned int* voxel_table,
                                  unsigned int *label_table);
__global__ void voxelize_triangle_and_label_and_instance(VoxelInfo info, float* triangle_data, unsigned int* label_data, unsigned int *instance_data, unsigned int* voxel_table,
                                                         unsigned int *label_table, unsigned int *instance_table);
__global__ void voxelize_assign_instance(VoxelInfo info, float* triangle_data, unsigned int* label_data,
                                         unsigned int *instance_data, unsigned int* voxel_table, unsigned int *label_table, unsigned int *instance_table);

void MeshVoxelizer::voxelize(const VoxelInfo& v, float* triangle_data, unsigned int* vtable, bool useThrustPath) {
	float   elapsedTime;

	// These are only used when we're not using UNIFIED memory
	unsigned int* dev_vtable; // DEVICE pointer to voxel_data
	size_t vtable_size; // vtable size

	// Create timers, set start time
	cudaEvent_t start_vox, stop_vox;
	checkCudaErrors(cudaEventCreate(&start_vox));
	checkCudaErrors(cudaEventCreate(&stop_vox));

	// Estimate best block and grid size using CUDA Occupancy Calculator
	int blockSize;   // The launch configurator returned block size
	int minGridSize; // The minimum grid size needed to achieve the  maximum occupancy for a full device launch
	int gridSize;    // The actual grid size needed, based on input size
	cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, voxelize_triangle, 0, 0);
	// Round up according to array size
	gridSize = (v.n_triangles + blockSize - 1) / blockSize;

	if (useThrustPath) { // We're not using UNIFIED memory
		vtable_size = ((size_t)v.gridSize.x * v.gridSize.y * v.gridSize.z) / (size_t) 8.0;
//		fprintf(stdout, "[Voxel Grid] Allocating %llu kB of DEVICE memory for Voxel Grid\n", size_t(vtable_size / 1024.0f));
		checkCudaErrors(cudaMalloc(&dev_vtable, vtable_size));
		checkCudaErrors(cudaMemset(dev_vtable, 0, vtable_size));
		// Start voxelization
		checkCudaErrors(cudaEventRecord(start_vox, 0));
		voxelize_triangle << <gridSize, blockSize >> > (v, triangle_data, dev_vtable);
	}
	else { // UNIFIED MEMORY
		checkCudaErrors(cudaEventRecord(start_vox, 0));
		voxelize_triangle << <gridSize, blockSize >> > (v, triangle_data, vtable);
	}

	cudaDeviceSynchronize();
	checkCudaErrors(cudaEventRecord(stop_vox, 0));
	checkCudaErrors(cudaEventSynchronize(stop_vox));
	checkCudaErrors(cudaEventElapsedTime(&elapsedTime, start_vox, stop_vox));
//	printf("[Perf] Voxelization GPU time: %.1f ms\n", elapsedTime);

	// If we're not using UNIFIED memory, copy the voxel table back and free all
	if (useThrustPath){
//		fprintf(stdout, "[Voxel Grid] Copying %llu kB to page-locked HOST memory\n", size_t(vtable_size / 1024.0f));
		checkCudaErrors(cudaMemcpy((void*)vtable, dev_vtable, vtable_size, cudaMemcpyDefault));
//		fprintf(stdout, "[Voxel Grid] Freeing %llu kB of DEVICE memory\n", size_t(vtable_size / 1024.0f));
		checkCudaErrors(cudaFree(dev_vtable));
	}

	// SANITY CHECKS
#ifdef _DEBUG
	size_t debug_n_triangles, debug_n_voxels_marked, debug_n_voxels_tested;
	checkCudaErrors(cudaMemcpyFromSymbol((void*)&(debug_n_triangles),debug_d_n_triangles, sizeof(debug_d_n_triangles), 0, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpyFromSymbol((void*)&(debug_n_voxels_marked), debug_d_n_voxels_marked, sizeof(debug_d_n_voxels_marked), 0, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpyFromSymbol((void*) & (debug_n_voxels_tested), debug_d_n_voxels_tested, sizeof(debug_d_n_voxels_tested), 0, cudaMemcpyDeviceToHost));
	printf("[Debug] Processed %llu triangles on the GPU \n", debug_n_triangles);
	printf("[Debug] Tested %llu voxels for overlap on GPU \n", debug_n_voxels_tested);
	printf("[Debug] Marked %llu voxels as filled (includes duplicates!) \n", debug_n_voxels_marked);
#endif

	// Destroy timers
	checkCudaErrors(cudaEventDestroy(start_vox));
	checkCudaErrors(cudaEventDestroy(stop_vox));
}

void MeshVoxelizer::voxelize(const VoxelInfo& v, float* triangle_data, uint *label_data, unsigned int* vtable, unsigned  int* ltable, bool useThrustPath) {
    float elapsedTime;


    // Create timers, set start time
    cudaEvent_t start_vox, stop_vox;
    checkCudaErrors(cudaEventCreate(&start_vox));
    checkCudaErrors(cudaEventCreate(&stop_vox));

    // Estimate best block and grid size using CUDA Occupancy Calculator
    int blockSize;   // The launch configurator returned block size
    int minGridSize; // The minimum grid size needed to achieve the  maximum occupancy for a full device launch
    int gridSize;    // The actual grid size needed, based on input size
    cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, voxelize_triangle_and_label, 0, 0);
    // Round up according to array size
    gridSize = (v.n_triangles + blockSize - 1) / blockSize;


    checkCudaErrors(cudaEventRecord(start_vox, 0));
    voxelize_triangle_and_label << < gridSize, blockSize >> > (v, triangle_data,label_data, vtable,ltable);


    checkCudaErrors(cudaDeviceSynchronize());
    checkCudaErrors(cudaEventRecord(stop_vox, 0));
    checkCudaErrors(cudaEventSynchronize(stop_vox));
    checkCudaErrors(cudaEventElapsedTime(&elapsedTime, start_vox, stop_vox));

    // SANITY CHECKS
#ifdef _DEBUG
    size_t debug_n_triangles, debug_n_voxels_marked, debug_n_voxels_tested;
checkCudaErrors(cudaMemcpyFromSymbol((void*)&(debug_n_triangles),debug_d_n_triangles, sizeof(debug_d_n_triangles), 0, cudaMemcpyDeviceToHost));
checkCudaErrors(cudaMemcpyFromSymbol((void*)&(debug_n_voxels_marked), debug_d_n_voxels_marked, sizeof(debug_d_n_voxels_marked), 0, cudaMemcpyDeviceToHost));
checkCudaErrors(cudaMemcpyFromSymbol((void*) & (debug_n_voxels_tested), debug_d_n_voxels_tested, sizeof(debug_d_n_voxels_tested), 0, cudaMemcpyDeviceToHost));
printf("[Debug] Processed %llu triangles on the GPU \n", debug_n_triangles);
printf("[Debug] Tested %llu voxels for overlap on GPU \n", debug_n_voxels_tested);
printf("[Debug] Marked %llu voxels as filled (includes duplicates!) \n", debug_n_voxels_marked);
#endif

    // Destroy timers
    checkCudaErrors(cudaEventDestroy(start_vox));
    checkCudaErrors(cudaEventDestroy(stop_vox));
}

void MeshVoxelizer::voxelize(const VoxelInfo& v, float* triangle_data, uint *label_data,uint* instance_data,
        unsigned int* vtable, unsigned  int* ltable, unsigned int *itable, bool useThrustPath) {
    float elapsedTime;


    // Create timers, set start time
    cudaEvent_t start_vox, stop_vox;
    checkCudaErrors(cudaEventCreate(&start_vox));
    checkCudaErrors(cudaEventCreate(&stop_vox));

    // Estimate best block and grid size using CUDA Occupancy Calculator
    int blockSize;   // The launch configurator returned block size
    int minGridSize; // The minimum grid size needed to achieve the  maximum occupancy for a full device launch
    int gridSize;    // The actual grid size needed, based on input size
    cudaOccupancyMaxPotentialBlockSize(&minGridSize, &blockSize, voxelize_triangle_and_label, 0, 0);
    // Round up according to array size
    gridSize = (v.n_triangles + blockSize - 1) / blockSize;


    checkCudaErrors(cudaEventRecord(start_vox, 0));
    // Cannot call it all together, some how this will broke cuda. Create a sepearte function to assign instance
//    voxelize_triangle_and_label_and_instance << < gridSize, blockSize >> > (v, triangle_data,label_data,instance_data, vtable,ltable, itable);
    voxelize_triangle_and_label << < gridSize, blockSize >> > (v, triangle_data,label_data, vtable,ltable);
    voxelize_assign_instance<< < gridSize, blockSize >> > (v, triangle_data,label_data,instance_data, vtable,ltable, itable);

    checkCudaErrors(cudaDeviceSynchronize());
    checkCudaErrors(cudaEventRecord(stop_vox, 0));
    checkCudaErrors(cudaEventSynchronize(stop_vox));
    checkCudaErrors(cudaEventElapsedTime(&elapsedTime, start_vox, stop_vox));

    // SANITY CHECKS
#ifdef _DEBUG
    size_t debug_n_triangles, debug_n_voxels_marked, debug_n_voxels_tested;
checkCudaErrors(cudaMemcpyFromSymbol((void*)&(debug_n_triangles),debug_d_n_triangles, sizeof(debug_d_n_triangles), 0, cudaMemcpyDeviceToHost));
checkCudaErrors(cudaMemcpyFromSymbol((void*)&(debug_n_voxels_marked), debug_d_n_voxels_marked, sizeof(debug_d_n_voxels_marked), 0, cudaMemcpyDeviceToHost));
checkCudaErrors(cudaMemcpyFromSymbol((void*) & (debug_n_voxels_tested), debug_d_n_voxels_tested, sizeof(debug_d_n_voxels_tested), 0, cudaMemcpyDeviceToHost));
printf("[Debug] Processed %llu triangles on the GPU \n", debug_n_triangles);
printf("[Debug] Tested %llu voxels for overlap on GPU \n", debug_n_voxels_tested);
printf("[Debug] Marked %llu voxels as filled (includes duplicates!) \n", debug_n_voxels_marked);
#endif

    // Destroy timers
    checkCudaErrors(cudaEventDestroy(start_vox));
    checkCudaErrors(cudaEventDestroy(stop_vox));
}

// Main triangle voxelization method
__global__ void voxelize_triangle(VoxelInfo info, float* triangle_data, unsigned int* voxel_table){
    size_t thread_id = threadIdx.x + blockIdx.x * blockDim.x;
    size_t stride = blockDim.x * gridDim.x;

    // Common variables used in the voxelization process
    float3 delta_p = make_float3(info.unit.x, info.unit.y, info.unit.z);
//    glm::vec3 delta_p(info.unit.x, info.unit.y, info.unit.z);
    uint3 grid_max = make_uint3(info.gridSize.x - 1, info.gridSize.y - 1, info.gridSize.z - 1);
//    glm::vec3 grid_max(info.gridSize.x - 1, info.gridSize.y - 1, info.gridSize.z - 1); // grid max (grid runs from 0 to gridSize-1)

    while (thread_id < info.n_triangles){ // every thread works on specific triangles in its stride
        size_t t = thread_id * 9; // triangle contains 9 vertices

        // COMPUTE COMMON TRIANGLE PROPERTIES
        // Move vertices to origin using bbox
        float3 infoBboxMin = make_float3(info.bbox_min.x,info.bbox_min.y,info.bbox_min.z);
//        glm::vec3 infoBboxMin (info.bbox_min.x,info.bbox_min.y,info.bbox_min.z);
        float3 v0 = make_float3(triangle_data[t], triangle_data[t + 1], triangle_data[t + 2]) - infoBboxMin;
        float3 v1 = make_float3(triangle_data[t+3], triangle_data[t + 4], triangle_data[t + 5]) - infoBboxMin;
        float3 v2 = make_float3(triangle_data[t+6], triangle_data[t + 7], triangle_data[t + 8]) - infoBboxMin;
//        glm::vec3 v0 = glm::vec3(triangle_data[t], triangle_data[t + 1], triangle_data[t + 2]) - infoBboxMin;
//        glm::vec3 v1 = glm::vec3(triangle_data[t + 3], triangle_data[t + 4], triangle_data[t + 5]) - infoBboxMin;
//        glm::vec3 v2 = glm::vec3(triangle_data[t + 6], triangle_data[t + 7], triangle_data[t + 8]) - infoBboxMin;
        // Edge vectors
        float3 e0 = v1-v0;
        float3 e1 = v2-v1;
        float3 e2 = v0-v2;
//        glm::vec3 e0 = v1 - v0;
//        glm::vec3 e1 = v2 - v1;
//        glm::vec3 e2 = v0 - v2;

        // Normal vector pointing up from the triangle
        float3 n = normalize(cross(e0,e1));
//        glm::vec3 n = glm::normalize(glm::cross(e0, e1));

        // COMPUTE TRIANGLE BBOX IN GRID
        // Triangle bounding box in world coordinates is min(v0,v1,v2) and max(v0,v1,v2)
        // Triangle bounding box in voxel grid coordinates is the world bounding box divided by the grid unit vector
        float3 bbox_min = clamp(fminf(v0,fminf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));
        float3 bbox_max = clamp(fmaxf(v0,fmaxf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));
//        auto bbox_min = glm::clamp(glm::min(v0, glm::min(v1, v2)) / info.unit, glm::vec3(0.0f, 0.0f, 0.0f), grid_max);
//        auto bbox_max = glm::clamp(glm::max(v0, glm::max(v1, v2)) / info.unit, glm::vec3(0.0f, 0.0f, 0.0f), grid_max);

//        AABox<glm::ivec3> t_bbox_grid;
//        t_bbox_grid.min =
//        t_bbox_grid.max = glm::clamp(glm::max(v0, glm::max(v1, v2)) / info.unit, glm::vec3(0.0f, 0.0f, 0.0f), grid_max);

        // PREPARE PLANE TEST PROPERTIES
        float3 c = make_float3(0);
//        glm::vec3 c(0.0f, 0.0f, 0.0f);
        if (n.x > 0.0f) { c.x = info.unit.x; }
        if (n.y > 0.0f) { c.y = info.unit.y; }
        if (n.z > 0.0f) { c.z = info.unit.z; }
        float d1 = dot(n,(c-v0));// glm::dot(n, (c - v0));
        float d2 = dot(n,delta_p-c-v0);//glm::dot(n, ((delta_p - c) - v0));

        // PREPARE PROJECTION TEST PROPERTIES
        // XY plane
        float2 n_xy_e0 = make_float2(-1.f*e0.y,e0.x);
        float2 n_xy_e1 = make_float2(-1.f*e1.y,e1.x);
        float2 n_xy_e2 = make_float2(-1.f*e2.y,e2.x);
//        float2 n_xy_e0(-1.0f*e0.y, e0.x);
//        float2 n_xy_e1(-1.0f*e1.y, e1.x);
//        float2 n_xy_e2(-1.0f*e2.y, e2.x);
        if (n.z < 0.0f) {
            n_xy_e0 = -n_xy_e0;
            n_xy_e1 = -n_xy_e1;
            n_xy_e2 = -n_xy_e2;
        }
        
        float d_xy_e0 = (-1.0f * dot(n_xy_e0, make_float2(v0.x, v0.y))) + max(0.0f, info.unit.x*n_xy_e0.x) + max(0.0f, info.unit.y*n_xy_e0.y);
        float d_xy_e1 = (-1.0f * dot(n_xy_e1, make_float2(v1.x, v1.y))) + max(0.0f, info.unit.x*n_xy_e1.x) + max(0.0f, info.unit.y*n_xy_e1.y);
        float d_xy_e2 = (-1.0f * dot(n_xy_e2, make_float2(v2.x, v2.y))) + max(0.0f, info.unit.x*n_xy_e2.x) + max(0.0f, info.unit.y*n_xy_e2.y);
        // YZ plane
        float2 n_yz_e0 = make_float2(-1.0f*e0.z, e0.y);
        float2 n_yz_e1 = make_float2(-1.0f*e1.z, e1.y);
        float2 n_yz_e2 = make_float2(-1.0f*e2.z, e2.y);
        if (n.x < 0.0f) {
            n_yz_e0 = -n_yz_e0;
            n_yz_e1 = -n_yz_e1;
            n_yz_e2 = -n_yz_e2;
        }
        float d_yz_e0 = (-1.0f * dot(n_yz_e0, make_float2(v0.y, v0.z))) + max(0.0f, info.unit.y*n_yz_e0.x) + max(0.0f, info.unit.z*n_yz_e0.y);
        float d_yz_e1 = (-1.0f * dot(n_yz_e1, make_float2(v1.y, v1.z))) + max(0.0f, info.unit.y*n_yz_e1.x) + max(0.0f, info.unit.z*n_yz_e1.y);
        float d_yz_e2 = (-1.0f * dot(n_yz_e2, make_float2(v2.y, v2.z))) + max(0.0f, info.unit.y*n_yz_e2.x) + max(0.0f, info.unit.z*n_yz_e2.y);
        // ZX plane
        float2 n_zx_e0 = make_float2(-1.0f*e0.x, e0.z);
        float2 n_zx_e1 = make_float2(-1.0f*e1.x, e1.z);
        float2 n_zx_e2 = make_float2(-1.0f*e2.x, e2.z);
        if (n.y < 0.0f) {
            n_zx_e0 = -n_zx_e0;
            n_zx_e1 = -n_zx_e1;
            n_zx_e2 = -n_zx_e2;
        }
        float d_xz_e0 = (-1.0f * dot(n_zx_e0, make_float2(v0.z, v0.x))) + max(0.0f, info.unit.x*n_zx_e0.x) + max(0.0f, info.unit.z*n_zx_e0.y);
        float d_xz_e1 = (-1.0f * dot(n_zx_e1, make_float2(v1.z, v1.x))) + max(0.0f, info.unit.x*n_zx_e1.x) + max(0.0f, info.unit.z*n_zx_e1.y);
        float d_xz_e2 = (-1.0f * dot(n_zx_e2, make_float2(v2.z, v2.x))) + max(0.0f, info.unit.x*n_zx_e2.x) + max(0.0f, info.unit.z*n_zx_e2.y);

        // test possible grid boxes for overlap
        for (int z = bbox_min.z; z <= bbox_max.z; z++){
            for (int y = bbox_min.y; y <= bbox_max.y; y++){
                for (int x = bbox_min.x; x <= bbox_max.x; x++){
                    // size_t location = x + (y*info.gridSize) + (z*info.gridSize*info.gridSize);
                    // if (checkBit(voxel_table, location)){ continue; }
#ifdef _DEBUG
                    atomicAdd(&debug_d_n_voxels_tested, 1);
#endif
                    // TRIANGLE PLANE THROUGH BOX TEST
                    float3 p = make_float3(x*info.unit.x, y*info.unit.y, z*info.unit.z);
                    float nDOTp = dot(n, p);
                    if ((nDOTp + d1) * (nDOTp + d2) > 0.0f) { continue; }

                    // PROJECTION TESTS
                    // XY
                    float2 p_xy = make_float2(p.x, p.y);
                    if ((dot(n_xy_e0, p_xy) + d_xy_e0) < 0.0f){ continue; }
                    if ((dot(n_xy_e1, p_xy) + d_xy_e1) < 0.0f){ continue; }
                    if ((dot(n_xy_e2, p_xy) + d_xy_e2) < 0.0f){ continue; }

                    // YZ
                    float2 p_yz = make_float2(p.y, p.z);
                    if ((dot(n_yz_e0, p_yz) + d_yz_e0) < 0.0f){ continue; }
                    if ((dot(n_yz_e1, p_yz) + d_yz_e1) < 0.0f){ continue; }
                    if ((dot(n_yz_e2, p_yz) + d_yz_e2) < 0.0f){ continue; }

                    // XZ	
                    float2 p_zx = make_float2(p.z, p.x);
                    if ((dot(n_zx_e0, p_zx) + d_xz_e0) < 0.0f){ continue; }
                    if ((dot(n_zx_e1, p_zx) + d_xz_e1) < 0.0f){ continue; }
                    if ((dot(n_zx_e2, p_zx) + d_xz_e2) < 0.0f){ continue; }

#ifdef _DEBUG
                    atomicAdd(&debug_d_n_voxels_marked, 1);
#endif

                    size_t location = static_cast<size_t>(x) + (static_cast<size_t>(y)* static_cast<size_t>(info.gridSize.x)) + (static_cast<size_t>(z)* static_cast<size_t>(info.gridSize.y)* static_cast<size_t>(info.gridSize.x));
                    atomicOr(&(voxel_table[location]), 1);
                    continue;
                }
            }
        }
#ifdef _DEBUG
        atomicAdd(&debug_d_n_triangles, 1);
#endif
        thread_id += stride;
    }
}

__global__ void voxelize_triangle_and_label(VoxelInfo info, float* triangle_data, unsigned int* label_data,
        unsigned int* voxel_table, unsigned int *label_table){
    size_t thread_id = threadIdx.x + blockIdx.x * blockDim.x;
    size_t stride = blockDim.x * gridDim.x;

    // Common variables used in the voxelization process
    float3 delta_p = make_float3(info.unit.x, info.unit.y, info.unit.z);
    uint3 grid_max = make_uint3(info.gridSize.x - 1, info.gridSize.y - 1, info.gridSize.z - 1);

    while (thread_id < info.n_triangles){ // every thread works on specific triangles in its stride
        size_t t = thread_id * 9; // triangle contains 9 vertices

        // COMPUTE COMMON TRIANGLE PROPERTIES
        // Move vertices to origin using bbox
        float3 infoBboxMin = make_float3(info.bbox_min.x,info.bbox_min.y,info.bbox_min.z);
        float3 v0 = make_float3(triangle_data[t], triangle_data[t + 1], triangle_data[t + 2]) - infoBboxMin;
        float3 v1 = make_float3(triangle_data[t+3], triangle_data[t + 4], triangle_data[t + 5]) - infoBboxMin;
        float3 v2 = make_float3(triangle_data[t+6], triangle_data[t + 7], triangle_data[t + 8]) - infoBboxMin;
        uint label = label_data[thread_id];
        // Edge vectors
        float3 e0 = v1-v0;
        float3 e1 = v2-v1;
        float3 e2 = v0-v2;
        // Normal vector pointing up from the triangle
        float3 n = normalize(cross(e0,e1));
//        glm::vec3 n = glm::normalize(glm::cross(e0, e1));

        // COMPUTE TRIANGLE BBOX IN GRID
        // Triangle bounding box in world coordinates is min(v0,v1,v2) and max(v0,v1,v2)
        // Triangle bounding box in voxel grid coordinates is the world bounding box divided by the grid unit vector
        float3 bbox_min = clamp(fminf(v0,fminf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));
        float3 bbox_max = clamp(fmaxf(v0,fmaxf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));

        // PREPARE PLANE TEST PROPERTIES
        float3 c = make_float3(0);
        if (n.x > 0.0f) { c.x = info.unit.x; }
        if (n.y > 0.0f) { c.y = info.unit.y; }
        if (n.z > 0.0f) { c.z = info.unit.z; }
        float d1 = dot(n,(c-v0));// glm::dot(n, (c - v0));
        float d2 = dot(n,delta_p-c-v0);//glm::dot(n, ((delta_p - c) - v0));

        // PREPARE PROJECTION TEST PROPERTIES
        // XY plane
        float2 n_xy_e0 = make_float2(-1.f*e0.y,e0.x);
        float2 n_xy_e1 = make_float2(-1.f*e1.y,e1.x);
        float2 n_xy_e2 = make_float2(-1.f*e2.y,e2.x);
//        float2 n_xy_e0(-1.0f*e0.y, e0.x);
//        float2 n_xy_e1(-1.0f*e1.y, e1.x);
//        float2 n_xy_e2(-1.0f*e2.y, e2.x);
        if (n.z < 0.0f) {
            n_xy_e0 = -n_xy_e0;
            n_xy_e1 = -n_xy_e1;
            n_xy_e2 = -n_xy_e2;
        }

        float d_xy_e0 = (-1.0f * dot(n_xy_e0, make_float2(v0.x, v0.y))) + max(0.0f, info.unit.x*n_xy_e0.x) + max(0.0f, info.unit.y*n_xy_e0.y);
        float d_xy_e1 = (-1.0f * dot(n_xy_e1, make_float2(v1.x, v1.y))) + max(0.0f, info.unit.x*n_xy_e1.x) + max(0.0f, info.unit.y*n_xy_e1.y);
        float d_xy_e2 = (-1.0f * dot(n_xy_e2, make_float2(v2.x, v2.y))) + max(0.0f, info.unit.x*n_xy_e2.x) + max(0.0f, info.unit.y*n_xy_e2.y);
        // YZ plane
        float2 n_yz_e0 = make_float2(-1.0f*e0.z, e0.y);
        float2 n_yz_e1 = make_float2(-1.0f*e1.z, e1.y);
        float2 n_yz_e2 = make_float2(-1.0f*e2.z, e2.y);
        if (n.x < 0.0f) {
            n_yz_e0 = -n_yz_e0;
            n_yz_e1 = -n_yz_e1;
            n_yz_e2 = -n_yz_e2;
        }
        float d_yz_e0 = (-1.0f * dot(n_yz_e0, make_float2(v0.y, v0.z))) + max(0.0f, info.unit.y*n_yz_e0.x) + max(0.0f, info.unit.z*n_yz_e0.y);
        float d_yz_e1 = (-1.0f * dot(n_yz_e1, make_float2(v1.y, v1.z))) + max(0.0f, info.unit.y*n_yz_e1.x) + max(0.0f, info.unit.z*n_yz_e1.y);
        float d_yz_e2 = (-1.0f * dot(n_yz_e2, make_float2(v2.y, v2.z))) + max(0.0f, info.unit.y*n_yz_e2.x) + max(0.0f, info.unit.z*n_yz_e2.y);
        // ZX plane
        float2 n_zx_e0 = make_float2(-1.0f*e0.x, e0.z);
        float2 n_zx_e1 = make_float2(-1.0f*e1.x, e1.z);
        float2 n_zx_e2 = make_float2(-1.0f*e2.x, e2.z);
        if (n.y < 0.0f) {
            n_zx_e0 = -n_zx_e0;
            n_zx_e1 = -n_zx_e1;
            n_zx_e2 = -n_zx_e2;
        }
        float d_xz_e0 = (-1.0f * dot(n_zx_e0, make_float2(v0.z, v0.x))) + max(0.0f, info.unit.x*n_zx_e0.x) + max(0.0f, info.unit.z*n_zx_e0.y);
        float d_xz_e1 = (-1.0f * dot(n_zx_e1, make_float2(v1.z, v1.x))) + max(0.0f, info.unit.x*n_zx_e1.x) + max(0.0f, info.unit.z*n_zx_e1.y);
        float d_xz_e2 = (-1.0f * dot(n_zx_e2, make_float2(v2.z, v2.x))) + max(0.0f, info.unit.x*n_zx_e2.x) + max(0.0f, info.unit.z*n_zx_e2.y);

        // test possible grid boxes for overlap
        for (int z = bbox_min.z; z <= bbox_max.z; z++){
            for (int y = bbox_min.y; y <= bbox_max.y; y++){
                for (int x = bbox_min.x; x <= bbox_max.x; x++){
                    // size_t location = x + (y*info.gridSize) + (z*info.gridSize*info.gridSize);
                    // if (checkBit(voxel_table, location)){ continue; }
#ifdef _DEBUG
                    atomicAdd(&debug_d_n_voxels_tested, 1);
#endif
                    // TRIANGLE PLANE THROUGH BOX TEST
                    float3 p = make_float3(x*info.unit.x, y*info.unit.y, z*info.unit.z);
                    float nDOTp = dot(n, p);
                    if ((nDOTp + d1) * (nDOTp + d2) > 0.0f) { continue; }

                    // PROJECTION TESTS
                    // XY
                    float2 p_xy = make_float2(p.x, p.y);
                    if ((dot(n_xy_e0, p_xy) + d_xy_e0) < 0.0f){ continue; }
                    if ((dot(n_xy_e1, p_xy) + d_xy_e1) < 0.0f){ continue; }
                    if ((dot(n_xy_e2, p_xy) + d_xy_e2) < 0.0f){ continue; }

                    // YZ
                    float2 p_yz = make_float2(p.y, p.z);
                    if ((dot(n_yz_e0, p_yz) + d_yz_e0) < 0.0f){ continue; }
                    if ((dot(n_yz_e1, p_yz) + d_yz_e1) < 0.0f){ continue; }
                    if ((dot(n_yz_e2, p_yz) + d_yz_e2) < 0.0f){ continue; }

                    // XZ
                    float2 p_zx = make_float2(p.z, p.x);
                    if ((dot(n_zx_e0, p_zx) + d_xz_e0) < 0.0f){ continue; }
                    if ((dot(n_zx_e1, p_zx) + d_xz_e1) < 0.0f){ continue; }
                    if ((dot(n_zx_e2, p_zx) + d_xz_e2) < 0.0f){ continue; }

#ifdef _DEBUG
                    atomicAdd(&debug_d_n_voxels_marked, 1);
#endif

                    size_t location = static_cast<size_t>(x) + (static_cast<size_t>(y)* static_cast<size_t>(info.gridSize.x)) + (static_cast<size_t>(z)* static_cast<size_t>(info.gridSize.y)* static_cast<size_t>(info.gridSize.x));
                    atomicOr(&(voxel_table[location]), 1);
                    atomicMax(&label_table[location],label);
                    continue;
                }
            }
        }
#ifdef _DEBUG
        atomicAdd(&debug_d_n_triangles, 1);
#endif
        thread_id += stride;
    }
}


__global__ void voxelize_triangle_and_label_and_instance(VoxelInfo info, float* triangle_data, unsigned int* label_data,
        unsigned int *instance_data, unsigned int* voxel_table, unsigned int *label_table, unsigned int *instance_table){
    size_t thread_id = threadIdx.x + blockIdx.x * blockDim.x;
    size_t stride = blockDim.x * gridDim.x;

    // Common variables used in the voxelization process
    float3 delta_p = make_float3(info.unit.x, info.unit.y, info.unit.z);
    uint3 grid_max = make_uint3(info.gridSize.x - 1, info.gridSize.y - 1, info.gridSize.z - 1);

    while (thread_id < info.n_triangles){ // every thread works on specific triangles in its stride
        size_t t = thread_id * 9; // triangle contains 9 vertices

        // COMPUTE COMMON TRIANGLE PROPERTIES
        // Move vertices to origin using bbox
        float3 infoBboxMin = make_float3(info.bbox_min.x,info.bbox_min.y,info.bbox_min.z);
        float3 v0 = make_float3(triangle_data[t], triangle_data[t + 1], triangle_data[t + 2]) - infoBboxMin;
        float3 v1 = make_float3(triangle_data[t+3], triangle_data[t + 4], triangle_data[t + 5]) - infoBboxMin;
        float3 v2 = make_float3(triangle_data[t+6], triangle_data[t + 7], triangle_data[t + 8]) - infoBboxMin;
        uint label = label_data[thread_id];
        uint instance = instance_data[thread_id];
        // Edge vectors
        float3 e0 = v1-v0;
        float3 e1 = v2-v1;
        float3 e2 = v0-v2;
        // Normal vector pointing up from the triangle
        float3 n = normalize(cross(e0,e1));
//        glm::vec3 n = glm::normalize(glm::cross(e0, e1));

        // COMPUTE TRIANGLE BBOX IN GRID
        // Triangle bounding box in world coordinates is min(v0,v1,v2) and max(v0,v1,v2)
        // Triangle bounding box in voxel grid coordinates is the world bounding box divided by the grid unit vector
        float3 bbox_min = clamp(fminf(v0,fminf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));
        float3 bbox_max = clamp(fmaxf(v0,fmaxf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));

        // PREPARE PLANE TEST PROPERTIES
        float3 c = make_float3(0);
        if (n.x > 0.0f) { c.x = info.unit.x; }
        if (n.y > 0.0f) { c.y = info.unit.y; }
        if (n.z > 0.0f) { c.z = info.unit.z; }
        float d1 = dot(n,(c-v0));// glm::dot(n, (c - v0));
        float d2 = dot(n,delta_p-c-v0);//glm::dot(n, ((delta_p - c) - v0));

        // PREPARE PROJECTION TEST PROPERTIES
        // XY plane
        float2 n_xy_e0 = make_float2(-1.f*e0.y,e0.x);
        float2 n_xy_e1 = make_float2(-1.f*e1.y,e1.x);
        float2 n_xy_e2 = make_float2(-1.f*e2.y,e2.x);
//        float2 n_xy_e0(-1.0f*e0.y, e0.x);
//        float2 n_xy_e1(-1.0f*e1.y, e1.x);
//        float2 n_xy_e2(-1.0f*e2.y, e2.x);
        if (n.z < 0.0f) {
            n_xy_e0 = -n_xy_e0;
            n_xy_e1 = -n_xy_e1;
            n_xy_e2 = -n_xy_e2;
        }

        float d_xy_e0 = (-1.0f * dot(n_xy_e0, make_float2(v0.x, v0.y))) + max(0.0f, info.unit.x*n_xy_e0.x) + max(0.0f, info.unit.y*n_xy_e0.y);
        float d_xy_e1 = (-1.0f * dot(n_xy_e1, make_float2(v1.x, v1.y))) + max(0.0f, info.unit.x*n_xy_e1.x) + max(0.0f, info.unit.y*n_xy_e1.y);
        float d_xy_e2 = (-1.0f * dot(n_xy_e2, make_float2(v2.x, v2.y))) + max(0.0f, info.unit.x*n_xy_e2.x) + max(0.0f, info.unit.y*n_xy_e2.y);
        // YZ plane
        float2 n_yz_e0 = make_float2(-1.0f*e0.z, e0.y);
        float2 n_yz_e1 = make_float2(-1.0f*e1.z, e1.y);
        float2 n_yz_e2 = make_float2(-1.0f*e2.z, e2.y);
        if (n.x < 0.0f) {
            n_yz_e0 = -n_yz_e0;
            n_yz_e1 = -n_yz_e1;
            n_yz_e2 = -n_yz_e2;
        }
        float d_yz_e0 = (-1.0f * dot(n_yz_e0, make_float2(v0.y, v0.z))) + max(0.0f, info.unit.y*n_yz_e0.x) + max(0.0f, info.unit.z*n_yz_e0.y);
        float d_yz_e1 = (-1.0f * dot(n_yz_e1, make_float2(v1.y, v1.z))) + max(0.0f, info.unit.y*n_yz_e1.x) + max(0.0f, info.unit.z*n_yz_e1.y);
        float d_yz_e2 = (-1.0f * dot(n_yz_e2, make_float2(v2.y, v2.z))) + max(0.0f, info.unit.y*n_yz_e2.x) + max(0.0f, info.unit.z*n_yz_e2.y);
        // ZX plane
        float2 n_zx_e0 = make_float2(-1.0f*e0.x, e0.z);
        float2 n_zx_e1 = make_float2(-1.0f*e1.x, e1.z);
        float2 n_zx_e2 = make_float2(-1.0f*e2.x, e2.z);
        if (n.y < 0.0f) {
            n_zx_e0 = -n_zx_e0;
            n_zx_e1 = -n_zx_e1;
            n_zx_e2 = -n_zx_e2;
        }
        float d_xz_e0 = (-1.0f * dot(n_zx_e0, make_float2(v0.z, v0.x))) + max(0.0f, info.unit.x*n_zx_e0.x) + max(0.0f, info.unit.z*n_zx_e0.y);
        float d_xz_e1 = (-1.0f * dot(n_zx_e1, make_float2(v1.z, v1.x))) + max(0.0f, info.unit.x*n_zx_e1.x) + max(0.0f, info.unit.z*n_zx_e1.y);
        float d_xz_e2 = (-1.0f * dot(n_zx_e2, make_float2(v2.z, v2.x))) + max(0.0f, info.unit.x*n_zx_e2.x) + max(0.0f, info.unit.z*n_zx_e2.y);

        // test possible grid boxes for overlap
        for (int z = bbox_min.z; z <= bbox_max.z; z++){
            for (int y = bbox_min.y; y <= bbox_max.y; y++){
                for (int x = bbox_min.x; x <= bbox_max.x; x++){
                    // size_t location = x + (y*info.gridSize) + (z*info.gridSize*info.gridSize);
                    // if (checkBit(voxel_table, location)){ continue; }
#ifdef _DEBUG
                    atomicAdd(&debug_d_n_voxels_tested, 1);
#endif
                    // TRIANGLE PLANE THROUGH BOX TEST
                    float3 p = make_float3(x*info.unit.x, y*info.unit.y, z*info.unit.z);
                    float nDOTp = dot(n, p);
                    if ((nDOTp + d1) * (nDOTp + d2) > 0.0f) { continue; }

                    // PROJECTION TESTS
                    // XY
                    float2 p_xy = make_float2(p.x, p.y);
                    if ((dot(n_xy_e0, p_xy) + d_xy_e0) < 0.0f){ continue; }
                    if ((dot(n_xy_e1, p_xy) + d_xy_e1) < 0.0f){ continue; }
                    if ((dot(n_xy_e2, p_xy) + d_xy_e2) < 0.0f){ continue; }

                    // YZ
                    float2 p_yz = make_float2(p.y, p.z);
                    if ((dot(n_yz_e0, p_yz) + d_yz_e0) < 0.0f){ continue; }
                    if ((dot(n_yz_e1, p_yz) + d_yz_e1) < 0.0f){ continue; }
                    if ((dot(n_yz_e2, p_yz) + d_yz_e2) < 0.0f){ continue; }

                    // XZ
                    float2 p_zx = make_float2(p.z, p.x);
                    if ((dot(n_zx_e0, p_zx) + d_xz_e0) < 0.0f){ continue; }
                    if ((dot(n_zx_e1, p_zx) + d_xz_e1) < 0.0f){ continue; }
                    if ((dot(n_zx_e2, p_zx) + d_xz_e2) < 0.0f){ continue; }

#ifdef _DEBUG
                    atomicAdd(&debug_d_n_voxels_marked, 1);
#endif

                    size_t location = static_cast<size_t>(x) + (static_cast<size_t>(y)* static_cast<size_t>(info.gridSize.x)) + (static_cast<size_t>(z)* static_cast<size_t>(info.gridSize.y)* static_cast<size_t>(info.gridSize.x));
                    atomicMax(&(voxel_table[location]), 1);
                    atomicMax(&label_table[location],label);
                    atomicMax(&instance_table[location], instance);
                    continue;
                }
            }
        }
#ifdef _DEBUG
        atomicAdd(&debug_d_n_triangles, 1);
#endif
        thread_id += stride;
    }
}

__global__ void voxelize_assign_instance(VoxelInfo info, float* triangle_data, unsigned int* label_data,
                                                         unsigned int *instance_data, unsigned int* voxel_table, unsigned int *label_table, unsigned int *instance_table){
    size_t thread_id = threadIdx.x + blockIdx.x * blockDim.x;
    size_t stride = blockDim.x * gridDim.x;

    // Common variables used in the voxelization process
    uint3 grid_max = make_uint3(info.gridSize.x - 1, info.gridSize.y - 1, info.gridSize.z - 1);

    while (thread_id < info.n_triangles){ // every thread works on specific triangles in its stride
        size_t t = thread_id * 9; // triangle contains 9 vertices
        uint label = label_data[thread_id];
        uint instance = instance_data[thread_id];

        // COMPUTE COMMON TRIANGLE PROPERTIES
        // Move vertices to origin using bbox
        float3 infoBboxMin = make_float3(info.bbox_min.x,info.bbox_min.y,info.bbox_min.z);
        float3 v0 = make_float3(triangle_data[t], triangle_data[t + 1], triangle_data[t + 2]) - infoBboxMin;
        float3 v1 = make_float3(triangle_data[t+3], triangle_data[t + 4], triangle_data[t + 5]) - infoBboxMin;
        float3 v2 = make_float3(triangle_data[t+6], triangle_data[t + 7], triangle_data[t + 8]) - infoBboxMin;

        // COMPUTE TRIANGLE BBOX IN GRID
        // Triangle bounding box in world coordinates is min(v0,v1,v2) and max(v0,v1,v2)
        // Triangle bounding box in voxel grid coordinates is the world bounding box divided by the grid unit vector
        float3 bbox_min = clamp(fminf(v0,fminf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));
        float3 bbox_max = clamp(fmaxf(v0,fmaxf(v1,v2)) / info.unit, make_float3(0), make_float3(grid_max));

        // test possible grid boxes for overlap
        for (int z = bbox_min.z; z <= bbox_max.z; z++){
            for (int y = bbox_min.y; y <= bbox_max.y; y++){
                for (int x = bbox_min.x; x <= bbox_max.x; x++){
                    size_t location = static_cast<size_t>(x) + (static_cast<size_t>(y)* static_cast<size_t>(info.gridSize.x)) + (static_cast<size_t>(z)* static_cast<size_t>(info.gridSize.y)* static_cast<size_t>(info.gridSize.x));
                    if(voxel_table[location]) {
                        atomicMax(&label_table[location],label);
                        atomicMax(&instance_table[location], instance);
                    }
                    continue;
                }
            }
        }
#ifdef _DEBUG
        atomicAdd(&debug_d_n_triangles, 1);
#endif
        thread_id += stride;
    }
}
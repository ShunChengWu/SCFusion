#include "TrainingDataGenerator.h"
#include "../../ORUtils/Logging.h"
#include <thrust/device_vector.h>
#include <thrust/scan.h>
#include <Utils/ITMCUDAUtils.h>
#include <Objects/Scene/ITMRepresentationAccess.h>
#include <sstream>


using namespace SCFUSION;
namespace {

    template<class TVoxel>
    __device__ inline bool checkVoxelState_Training(const TVoxel &voxel) {
        if (TVoxel::integrateType == SCFUSION::IntegrateType_Invert_TSDF)
            return TVoxel::valueToFloat(voxel.sdf) > 0.8 && voxel.w_depth>0;
        if (TVoxel::integrateType == SCFUSION::IntegrateType_OFusion)
            return TVoxel::valueToFloat(voxel.sdf) >= 0 && voxel.w_depth>0 ;
        if (TVoxel::integrateType == SCFUSION::IntegrateType_TSDF)
            return TVoxel::valueToFloat(voxel.sdf) < 0.2 && voxel.w_depth>0;
    }

    template<int dummy>
    __global__ void
    findAllocateBlocks(Vector4s *visibleBlockGlobalPos, const ITMHashEntry *hashTable, int noTotalEntries,
                       Vector3s blockDims, unsigned int *visibleBlockSize) {
        int entryId = threadIdx.x + blockIdx.x * blockDim.x;
        if (entryId > noTotalEntries - 1) return;

        const ITMHashEntry &currentHashEntry = hashTable[entryId];

        if (currentHashEntry.ptr >= 0) {
            for (short z = 0; z < blockDims.z; ++z) {
                for (short y = 0; y < blockDims.y; ++y) {
                    for (short x = 0; x < blockDims.x; ++x) {
                        int idx = atomicAdd(visibleBlockSize, 1);
                        visibleBlockGlobalPos[idx]
                                = Vector4s(currentHashEntry.pos.x * blockDims.x + x,
                                           currentHashEntry.pos.y * blockDims.y + y,
                                           currentHashEntry.pos.z * blockDims.z + z, 1);
                    }
                }
            }
        }
    }

    template<class TVoxel>
    __global__ void findBoundaries_device(const ITMHashEntry *hashTable, int noTotalEntries, const TVoxel *localVBA,
                                          Vector3s blockDims, int floor_label, int *boundaries){
        int entryId = threadIdx.x + blockIdx.x * blockDim.x;
        if (entryId > noTotalEntries - 1) return;
        const ITMHashEntry &currentHashEntry = hashTable[entryId];
        if (currentHashEntry.ptr >= 0) {
            for (short z = 0; z < blockDims.z; ++z) {
                for (short y = 0; y < blockDims.y; ++y) {
                    for (short x = 0; x < blockDims.x; ++x) {
                        Vector3i pt(currentHashEntry.pos.x * blockDims.x + x,
                                    currentHashEntry.pos.y * blockDims.y + y,
                                    currentHashEntry.pos.z * blockDims.z + z);

                        int vmIndex;
                        TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
                        if(checkVoxelState_Training(voxel)){
                            atomicMin(&boundaries[0], pt.x);
                            atomicMax(&boundaries[1], pt.x);
                            atomicMin(&boundaries[2], pt.y);
                            atomicMax(&boundaries[3], pt.y);
                            atomicMin(&boundaries[4], pt.z);
                            atomicMax(&boundaries[5], pt.z);
                            if(floor_label>=0)
                                if(voxel.label == floor_label) atomicMin(&boundaries[6], pt.y);
                        }
                    }
                }
            }
        }
    }

    template<class TVoxel>
    __global__ void
    CheckVoxelData_device(int size, unsigned int *occupancy, Vector3i origin, Vector3s dims,
                      const TVoxel *localVBA, const ITMHashEntry *hashTable) {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x, y, z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            if(checkVoxelState_Training(voxel))
                atomicAdd(occupancy, 1);
        }
    }

    template<class TVoxel>
    __global__ void
    CountLabels_device(int size, int *labels, Vector3i origin, Vector3s dims,
                          const TVoxel *localVBA, const ITMHashEntry *hashTable) {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x, y, z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            atomicAdd(&labels[voxel.label], 1);
        }
    }

    template<class TVoxel>
    __global__ void
    CountSufaceAndLabelOnSurface(int size, unsigned int *occupancy, unsigned int *labels, Vector3i origin, Vector3s dims,
                      const TVoxel *localVBA, const ITMHashEntry *hashTable) {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x, y, z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            if(checkVoxelState_Training(voxel)) {
                atomicAdd(occupancy, 1);
                atomicAdd(&labels[voxel.label], 1);
            }
        }
    }

    template<class TVoxel>
    __global__ void CopyValueDirect(int size, float *output_data, Vector3i origin, Vector3s dims, const TVoxel *localVBA,
            const ITMHashEntry *hashTable, float truncationMargin, bool ITSDF, bool outputVoxelDistance, bool makeInitValueInf, bool ofuNorm)
    {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x,y,z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            if(TVoxel::integrateType == SCFUSION::IntegrateType_TSDF) {
                output_data[i] = TVoxel::valueToFloat(voxel.sdf);
            } else if(TVoxel::integrateType == SCFUSION::IntegrateType_OFusion) {
                // Convert Ofu to probability
                float value;
                if (ofuNorm) {
                    value = 1.f - 1.f / (1.f + exp(TVoxel::valueToFloat(voxel.sdf)));
                    value = value * 2.f - 1.f;
                    value = MAX(MIN(1.f,value),-1.f);
                } else
                    value = TVoxel::valueToFloat(voxel.sdf) / LOGODD_MAX;

                if(ITSDF) {// treat it as inverted tsdf
                    if (voxel.w_depth == 0) {
                        output_data[i] = 0;
                        continue;
                    }
                    output_data[i] = 1 - ABS(value);
                } else {
                        output_data[i] = value;
                }
            } else if (TVoxel::integrateType == SCFUSION::IntegrateType_Invert_TSDF) {
                output_data[i] = 1 - ABS(TVoxel::valueToFloat(voxel.sdf));
            }

            if(makeInitValueInf)
                if(voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue())
                    output_data[i] = INFINITY;
            if(outputVoxelDistance)
                    output_data[i] *= truncationMargin;
        }
    }

    template<class TVoxel>
    __global__ void CopyMaskDirect(int size, bool *output_mask, Vector3i origin, Vector3s dims, const TVoxel *localVBA, const ITMHashEntry *hashTable)
    {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x,y,z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            /*if(TVoxel::integrateType == SCFUSION::IntegrateType_Invert_TSDF) {
                output_mask[i] = true;
            } else if(TVoxel::integrateType == SCFUSION::IntegrateType_OFusion) {
                output_mask[i] = (voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue());
            } else if (TVoxel::integrateType == SCFUSION::IntegrateType_TSDF)*/
                output_mask[i] = voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue();
        }
    }

    /// Copy only labels on surface
    template<class TVoxel, class Tout>
    __global__ void CopyLabelDirect(int size, Tout *output_label, Vector3i origin, Vector3s dims, const TVoxel *localVBA, const ITMHashEntry *hashTable)
    {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x,y,z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            output_label[i] = Tout(voxel.label);
        }
    }
    template<class TVoxel, class Tout>
    __global__ void CopyLabelDirectFromRC(int size, Tout *output_label, Vector3i origin, Vector3s dims, const TVoxel *localVBA, const ITMHashEntry *hashTable)
    {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x,y,z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            if(TVoxel::valueToFloat(voxel.sdf) >= 0 && voxel.w_depth > 0)
                output_label[i] = Tout(voxel.label);
            else
                output_label[i] = 0;
        }
    }

    template<class TVoxel, class Tout>
    __global__ void CopyInstanceDirect(int size, Tout *output_instance, Vector3i origin, Vector3s dims, const TVoxel *localVBA, const ITMHashEntry *hashTable)
    {
        CUDA_1D_LOOP(i, size) {
            int z = (int) floor(double(i) / double(dims.x * dims.y));
            int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
            int x = i - (z * dims.x * dims.y) - (y * dims.x);
            Vector3i pt = origin + Vector3i(x,y,z);

            int vmIndex;
            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
            output_instance[i] = Tout(voxel.semantic);
        }
    }
}

template<class TVoxelRC, class TVoxelGT>
TrainingDataGenerator<TVoxelRC, TVoxelGT>::TrainingDataGenerator(bool verbose):verbose_(verbose) {
    visibleBlockGlobalPos.reset(new ORUtils::MemoryBlock<Vector4s>(visibleBlockPos_size, true, true));
    totalVisibleBlockPos.reset(new ORUtils::MemoryBlock<unsigned int>(1,true,true));
    stream_ = 0;
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::findBoundaries(Vector3i &pos_min, Vector3i &pos_max,
        const ITMLib::ITMLocalVBA<TVoxelGT> *localVBA, const ITMLib::ITMVoxelBlockHash *index, float factor,
    int floorLabel){
    //FIXME: this is not very accurate. somethings the floor should be 0 but get 1 instead.
    int noTotalEntries = index->noTotalEntries;
    const ITMHashEntry *hashTable = index->GetEntries();
    visibleBlockGlobalPos->Clear(0, true, stream_);
    totalVisibleBlockPos->Clear(0,true,stream_);

    auto visibleBlockGlobalPos_device = visibleBlockGlobalPos->GetDataConst(MEMORYDEVICE_CUDA);
    auto visibleBlockGlobalPos_cpu = visibleBlockGlobalPos->GetDataConst(MEMORYDEVICE_CPU);
    auto totalVisibleBlockPos_device = totalVisibleBlockPos->GetDataConst(MEMORYDEVICE_CUDA);
    auto totalVisibleBlockPos_cpu = totalVisibleBlockPos->GetDataConst(MEMORYDEVICE_CPU);

    {
        ORUtils::MemoryBlock<int> boundaries(7,true,true);
        boundaries.Clear(0);
        boundaries.GetData(MEMORYDEVICE_CPU)[6]=100;
        boundaries.UpdateDeviceFromHost();
        findBoundaries_device<<<GET_1D_BLOCKS(noTotalEntries), threadPerBlock, 0, stream_>>>(hashTable, noTotalEntries, localVBA->GetVoxelBlocks(), Vector3s(SDF_BLOCK_SIZE,SDF_BLOCK_SIZE,SDF_BLOCK_SIZE),
                floorLabel, boundaries.GetData(MEMORYDEVICE_CUDA));
        boundaries.UpdateHostFromDevice();
//        printf("boundaries.GetData(MEMORYDEVICE_CPU)[6]: %d\n", boundaries.GetData(MEMORYDEVICE_CPU)[6]);
        if(floorLabel>=0){
            if(boundaries.GetData(MEMORYDEVICE_CPU)[6] != 100)
                boundaries.GetData(MEMORYDEVICE_CPU)[2] = boundaries.GetData(MEMORYDEVICE_CPU)[6];
        }
//        for(size_t i=0;i<boundaries.dataSize;++i)
//            printf("[%zu] %d\n", i, boundaries.GetData(MEMORYDEVICE_CPU)[i]);
        pos_min.x = boundaries.GetData(MEMORYDEVICE_CPU)[0];
        pos_max.x = boundaries.GetData(MEMORYDEVICE_CPU)[1];
        pos_min.y = boundaries.GetData(MEMORYDEVICE_CPU)[2];
        pos_max.y = boundaries.GetData(MEMORYDEVICE_CPU)[3];
        pos_min.z = boundaries.GetData(MEMORYDEVICE_CPU)[4];
        pos_max.z = boundaries.GetData(MEMORYDEVICE_CPU)[5];
    }

    /// Create Contiguous Map
//    Vector3s dims;
//    ORUtils::MemoryBlock<float> *tmp;
//    Vector3i pos_min(0,0,0), pos_max(0,0,0);
#if false
    if(0)
    {
        visibleBlockGlobalPos->UpdateHostFromDevice();
        for(size_t i=0; i < *totalVisibleBlockPos_cpu; ++i){
            if(visibleBlockGlobalPos_cpu[i].w == 0) continue;
            if(visibleBlockGlobalPos_cpu[i].x < pos_min.x) pos_min.x = visibleBlockGlobalPos_cpu[i].x;
            if(visibleBlockGlobalPos_cpu[i].y < pos_min.y) pos_min.y = visibleBlockGlobalPos_cpu[i].y;
            if(visibleBlockGlobalPos_cpu[i].z < pos_min.z) pos_min.z = visibleBlockGlobalPos_cpu[i].z;
            if(visibleBlockGlobalPos_cpu[i].x > pos_max.x) pos_max.x = visibleBlockGlobalPos_cpu[i].x;
            if(visibleBlockGlobalPos_cpu[i].y > pos_max.y) pos_max.y = visibleBlockGlobalPos_cpu[i].y;
            if(visibleBlockGlobalPos_cpu[i].z > pos_max.z) pos_max.z = visibleBlockGlobalPos_cpu[i].z;
        }

        printf("min: %d %d %d max: %d %d %d \n", pos_min.x,pos_min.y,pos_min.z,pos_max.x,pos_max.y,pos_max.z);

        auto dims = Vector3s((pos_max.x-pos_min.x),(pos_max.y-pos_min.y),(pos_max.z-pos_min.z));

        /// Allocate Memory
//        tmp = new ORUtils::MemoryBlock<float>(dims.x*dims.y*dims.z, true, true);
    }
#endif
}


template<class TVoxelRC, class TVoxelGT>
bool TrainingDataGenerator<TVoxelRC, TVoxelGT>::CheckLabelandDistribution(Vector3i origin, Vector3s dims,
                                                      ITMLib::ITMScene<TVoxelRC, ITMLib::ITMVoxelBlockHash> *scene_rc,
                                                      ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt,
                                                      uint labelNum, float threshold_ocu, uint threshold_labelNum,
                                                      float threshold_max_domainate,
                                                      float threshold_occupancy_rc) {
    auto localVBA_rc = scene_rc->localVBA.get();
    auto localVBA_gt = scene_gt->localVBA.get();
    const ITMHashEntry *hashTable_rc = scene_rc->index->GetEntries();
    const ITMHashEntry *hashTable_gt = scene_gt->index->GetEntries();

    /// Check
    bool checkState = false;
    {
        size_t totalSize = dims.x * dims.y * dims.z;
        ORUtils::MemoryBlock<uint> labels_rc(labelNum, true, true);
        ORUtils::MemoryBlock<uint> occupancy(1, true, true);
        auto label_cpu = labels_rc.GetData(MEMORYDEVICE_CPU, false);
        auto occupancy_cpu = occupancy.GetData(MEMORYDEVICE_CPU, false);
        auto label_gpu = labels_rc.GetData(MEMORYDEVICE_CUDA, false);
        auto occupancy_gpu = occupancy.GetData(MEMORYDEVICE_CUDA, false);

        float distr_ocu_rc = 0, distr_ocu_gt = 0;
        float distr_labels[labelNum];
        /// Calculate occupancy in reconstructed scene
        labels_rc.Clear(0);
        occupancy.Clear(0);
        CheckVoxelData_device<TVoxelRC> << < GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_ >> >
              (totalSize, occupancy_gpu, origin, dims, localVBA_rc->GetVoxelBlocks(), hashTable_rc);
        labels_rc.UpdateHostFromDevice();
        occupancy.UpdateHostFromDevice();
        ORcudaSafeCall(cudaStreamSynchronize(stream_));
        distr_ocu_rc = float(occupancy_cpu[0]) / float(totalSize);

        /// Calcualte occupancy and label distribution in GT scene
        labels_rc.Clear(0);
        occupancy.Clear(0);
        CountSufaceAndLabelOnSurface<TVoxelGT> << < GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_ >> >
              (totalSize, occupancy_gpu, label_gpu, origin, dims, localVBA_gt->GetVoxelBlocks(), hashTable_gt);
        labels_rc.UpdateHostFromDevice();
        occupancy.UpdateHostFromDevice();
        ORcudaSafeCall(cudaStreamSynchronize(stream_));
        distr_ocu_gt = float(occupancy_cpu[0]) / float(totalSize);
        size_t label_total = 0, observed_label_num=0;
        for (size_t i = 0; i < labelNum; ++i) {
            label_total += label_cpu[i];
        }
        for (size_t i = 0; i < labelNum; ++i) {
            distr_labels[i] = float(label_cpu[i]) / float(totalSize);
            if(distr_labels[i]>0) observed_label_num++;
        }
        distr_labels[0] = 1.f-label_total/float(totalSize);

//        printf("blockHashOrigin: %d %d %d\n", origin.x,origin.y,origin.z);
//        printf("distr_rc: %f distr_gt: %f\n", distr_ocu_rc, distr_ocu_gt);
        float highestPercentage = 0;
        auto without_0 = 1 - distr_labels[0];
        for (size_t i = 1; i < labelNum; ++i)
            if (distr_labels[i] / without_0 > highestPercentage)
                highestPercentage = distr_labels[i] / without_0;

        if (verbose_) {
            printf("occupancy[rc, gt]: %f %f \n", distr_ocu_rc, distr_ocu_gt);
            printf("distr_label: [");
            for (float distr_label : distr_labels)printf("%5.2f ", distr_label);
            printf("]\n");
            printf("highest: %f\n", highestPercentage);
        }

        if (highestPercentage> 0 && highestPercentage < threshold_max_domainate &&
        !std::isnan(distr_labels[0]) &&
        distr_labels[0] <= threshold_ocu &&
        observed_label_num >= threshold_labelNum &&
        distr_ocu_rc > threshold_occupancy_rc) {
            checkState = true;
        }

        try {
            labels_rc.Free();
        } catch (std::runtime_error &e){
            std::stringstream s;
            s << __FILE__ << ":" << __LINE__ << "[labels_rc.Free()]" << e.what();
            throw std::runtime_error(s.str());
        }

        try {
            occupancy.Free();
        } catch (std::runtime_error &e){
            std::stringstream s;
            s << __FILE__ << ":" << __LINE__ << "[occupancy.Free()]" << e.what();
            throw std::runtime_error(s.str());
        }
    }

    return checkState;
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::ExtractToUniMap(ORUtils::MemoryBlock<float> *data, ORUtils::MemoryBlock<float> *label, ORUtils::MemoryBlock<bool> *mask,
        const Vector3i &origin, const Vector3s &dims,
    ITMLib::ITMScene<TVoxelRC, ITMLib::ITMVoxelBlockHash> *scene_rc, ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt,
    bool ITSDF, bool outputVoxelDistance, bool makeInitValueInf, bool ofuNorm, bool gtFromRC){
    auto localVBA_rc = scene_rc->localVBA.get();
    auto localVBA_gt = scene_gt->localVBA.get();
    const ITMHashEntry *hashTable_rc = scene_rc->index->GetEntries();
    const ITMHashEntry *hashTable_gt = scene_gt->index->GetEntries();
    float mu = scene_rc->sceneParams->mu;
    /// Create Contiguous Map
    size_t totalSize = dims.x*dims.y*dims.z;
    if(data->dataSize != totalSize) data->Resize(totalSize);
    if(label->dataSize != totalSize) label->Resize(totalSize);

    /// Copy Memory
    {
//        printf("blockHashOrigin: %d %d %d\n", origin.x,origin.y,origin.z);
        CopyValueDirect<TVoxelRC><<<GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_>>>
              (totalSize, data->GetData(MEMORYDEVICE_CUDA), origin, dims,
                      localVBA_rc->GetVoxelBlocks(), hashTable_rc, mu, ITSDF, outputVoxelDistance, makeInitValueInf, ofuNorm);
        CopyMaskDirect<TVoxelRC><<<GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_>>>
                (totalSize, mask->GetData(MEMORYDEVICE_CUDA), origin, dims, localVBA_rc->GetVoxelBlocks(), hashTable_rc);
        if(gtFromRC){
            CopyLabelDirectFromRC<TVoxelRC, float><<<GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_>>>
                    (totalSize, label->GetData(MEMORYDEVICE_CUDA), origin, dims, localVBA_rc->GetVoxelBlocks(), hashTable_rc);
        } else
            CopyLabelDirect<TVoxelGT, float><<<GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_>>>
                (totalSize, label->GetData(MEMORYDEVICE_CUDA), origin, dims, localVBA_gt->GetVoxelBlocks(), hashTable_gt);


        ORcudaSafeCall(cudaStreamSynchronize(stream_));

#ifndef NDEBUG
        if(verbose_) {
            data->UpdateHostFromDevice();
            auto data_value = data->GetDataConst(MEMORYDEVICE_CPU);
            float avg = 0, max = 0, min = 0, stdev = 0;
            for (size_t i = 0; i < data->dataSize; ++i) {
                float value = data_value[i];
                avg += value;
                if (value > max)max = value;
                if (value < min)min = value;
            }
            avg /= float(data->dataSize);

            float diff = 0;
            for (size_t i = 0; i < data->dataSize; ++i) diff = std::pow(avg - data_value[i], 2);
            diff /= float(data->dataSize);
            stdev = std::sqrt(diff);
            printf("\t volume value distribution: mean(%f),max(%f),min(%f), stdev(%f)\n", avg, max, min, stdev);
        }
#endif
    }
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::ExtractToUniMap(ORUtils::MemoryBlock<unsigned short> *label,
                     const Vector3i &origin, const Vector3s &dims,
                     ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt){
        auto localVBA_gt = scene_gt->localVBA.get();
        const ITMHashEntry *hashTable_gt = scene_gt->index->GetEntries();
        /// Create Contiguous Map
        size_t totalSize = dims.x*dims.y*dims.z;
        if(label->dataSize != totalSize) label->Resize(totalSize);

        /// Copy Memory
        {
            CopyLabelDirect<TVoxelGT, unsigned short><<<GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_>>>
                 (totalSize, label->GetData(MEMORYDEVICE_CUDA), origin, dims, localVBA_gt->GetVoxelBlocks(), hashTable_gt);
            ORcudaSafeCall(cudaStreamSynchronize(stream_));
        }
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::ExtractInstanceToUniMap(ORUtils::MemoryBlock<unsigned short> *instance,
                             const Vector3i &origin, const Vector3s &dims,
                             ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt){
    auto localVBA_gt = scene_gt->localVBA.get();
    const ITMHashEntry *hashTable_gt = scene_gt->index->GetEntries();
    /// Create Contiguous Map
    size_t totalSize = dims.x*dims.y*dims.z;
    if(instance->dataSize != totalSize) instance->Resize(totalSize);

    /// Copy Memory
    {
        CopyInstanceDirect<TVoxelGT, unsigned short><<<GET_1D_BLOCKS(totalSize, threadPerBlock), threadPerBlock, 0, stream_>>>
             (totalSize, instance->GetData(MEMORYDEVICE_CUDA), origin, dims, localVBA_gt->GetVoxelBlocks(), hashTable_gt);
        ORcudaSafeCall(cudaStreamSynchronize(stream_));
    }
}


template <class TVoxelRC, class TVoxelGT>
__global__ void calculateIoUOfInstances_device(Vector2i *instancesIoU, uint instance_limit,
                                    const ITMHashEntry *hashTable_rc, int noTotalEntries_rc, const TVoxelRC *localVBA_rc,
                                    const ITMHashEntry *hashTable_gt, int noTotalEntries_gt, const TVoxelGT *localVBA_gt) {
    int entryId = threadIdx.x + blockIdx.x * blockDim.x;
    if (entryId < noTotalEntries_gt) {
        const ITMHashEntry &currentHashEntry_gt = hashTable_gt[entryId];
        if (currentHashEntry_gt.ptr >= 0) {
            for (short z = 0; z < SDF_BLOCK_SIZE; ++z) {
                for (short y = 0; y < SDF_BLOCK_SIZE; ++y) {
                    for (short x = 0; x < SDF_BLOCK_SIZE; ++x) {
                        auto pt = Vector3i(currentHashEntry_gt.pos.x * SDF_BLOCK_SIZE + x,
                                           currentHashEntry_gt.pos.y * SDF_BLOCK_SIZE + y,
                                           currentHashEntry_gt.pos.z * SDF_BLOCK_SIZE + z);
                        int vmIndex;
                        int semantic;
                        /// get label GT
                        {
                            TVoxelGT voxel = readVoxel(localVBA_gt, hashTable_gt, pt, vmIndex);
                            semantic = voxel.semantic;
                            if(semantic==0 || voxel.sdf < 1 || semantic >= instance_limit) continue;
                            atomicAdd(&instancesIoU[semantic].y,1);
                        }
                        /// get label RC
                        {
                            TVoxelRC voxel = readVoxel(localVBA_rc, hashTable_rc, pt, vmIndex);
                            if(/*voxel.sdf >= -2 && */voxel.w_depth>0) atomicAdd(&instancesIoU[semantic].x,1);
                        }
                    }
                }
            }
        }
    }
}

template <class TVoxel, class TIndex>
__global__ void removeTargetInstance_device(Vector2i *instancesIoU, uint instance_limit,
                                            const typename TIndex::IndexData *hashTable, int noTotalEntries,
                                            TVoxel *localVBA, float threshold) {
    int entryId = threadIdx.x + blockIdx.x * blockDim.x;
    if (entryId < noTotalEntries) {
        const ITMHashEntry &currentHashEntry_gt = hashTable[entryId];
        if (currentHashEntry_gt.ptr >= 0) {
            for (short z = 0; z < SDF_BLOCK_SIZE; ++z) {
                for (short y = 0; y < SDF_BLOCK_SIZE; ++y) {
                    for (short x = 0; x < SDF_BLOCK_SIZE; ++x) {
                        auto pt = Vector3i(currentHashEntry_gt.pos.x * SDF_BLOCK_SIZE + x,
                                           currentHashEntry_gt.pos.y * SDF_BLOCK_SIZE + y,
                                           currentHashEntry_gt.pos.z * SDF_BLOCK_SIZE + z);
                        int vmIndex;
                        int semantic;
                        {
                            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
                            semantic = voxel.semantic;
                            if(semantic==0 || voxel.sdf < 1 || semantic >= instance_limit) continue;
                            if(instancesIoU[semantic].y == 0) continue;
                            if(float(instancesIoU[semantic].x)/float(instancesIoU[semantic].y) <= threshold){
                                int vmIndex;
                                typename TIndex::IndexCache cache;
                                int idx_to = findVoxel(hashTable,pt,vmIndex, cache);
                                TVoxel &voxel_to = localVBA[idx_to];
                                voxel_to = TVoxel();
//                                voxel_to.sdf = 0;
//                                voxel_to.w_label = 0;
//                                voxel_to.semantic = 0;
                            }
                        }
                    }
                }
            }
        }
    }
}

template <class TVoxel, class TIndex>
__global__ void keepOnlyTargetInstance_device(int instanceSize, ushort *instances,
                                            const typename TIndex::IndexData *hashTable, int noTotalEntries,
                                            TVoxel *localVBA) {
    int entryId = threadIdx.x + blockIdx.x * blockDim.x;
    if (entryId < noTotalEntries) {
        const ITMHashEntry &currentHashEntry_gt = hashTable[entryId];
        if (currentHashEntry_gt.ptr >= 0) {
            for (short z = 0; z < SDF_BLOCK_SIZE; ++z) {
                for (short y = 0; y < SDF_BLOCK_SIZE; ++y) {
                    for (short x = 0; x < SDF_BLOCK_SIZE; ++x) {
                        auto pt = Vector3i(currentHashEntry_gt.pos.x * SDF_BLOCK_SIZE + x,
                                           currentHashEntry_gt.pos.y * SDF_BLOCK_SIZE + y,
                                           currentHashEntry_gt.pos.z * SDF_BLOCK_SIZE + z);
                        int vmIndex;
                        int semantic;
                        {
                            TVoxel voxel = readVoxel(localVBA, hashTable, pt, vmIndex);
                            if(!vmIndex) continue;

                            semantic = voxel.semantic;
                            bool found = false;
                            for(int i=0;i<instanceSize;++i)
                                if(instances[i] == semantic) {
                                    found = true;
                                    break;
                                }

                            if(!found){
                                typename TIndex::IndexCache cache;
                                int idx_to = findVoxel(hashTable,pt,vmIndex, cache);
                                TVoxel &voxel_to = localVBA[idx_to];
                                voxel_to = TVoxel();
                            }
                        }
                    }
                }
            }
        }
    }
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::FilterOutBaseOnInstances(const ITMLib::ITMScene<TVoxelRC, ITMLib::ITMVoxelBlockHash> *scene_rc,
        ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt, uint num_instances, float threshold){
    ORUtils::MemoryBlock<Vector2i> numInstances(num_instances,true,true);
    numInstances.Clear(0);
    calculateIoUOfInstances_device<TVoxelRC, TVoxelGT><<<GET_1D_BLOCKS(scene_gt->index->noTotalEntries, threadPerBlock), threadPerBlock>>>
            (numInstances.GetData(MEMORYDEVICE_CUDA), num_instances,
                    scene_rc->index->GetEntries(),scene_rc->index->noTotalEntries,scene_rc->localVBA->GetVoxelBlocks(),
                    scene_gt->index->GetEntries(),scene_gt->index->noTotalEntries,scene_gt->localVBA->GetVoxelBlocks());
    numInstances.UpdateHostFromDevice();

#ifndef NDEBUG
    printf("Before instance filtering\n");
    for(size_t i=0;i<num_instances;++i)
        printf("%zu: %d/%d(%f)\n",i,numInstances.GetData(MEMORYDEVICE_CPU)[i].x,
               numInstances.GetData(MEMORYDEVICE_CPU)[i].y,
               (float)numInstances.GetData(MEMORYDEVICE_CPU)[i].x/(float)numInstances.GetData(MEMORYDEVICE_CPU)[i].y);
#endif

    removeTargetInstance_device<TVoxelGT,ITMVoxelIndex><<<GET_1D_BLOCKS(scene_gt->index->noTotalEntries, threadPerBlock), threadPerBlock>>>
            (numInstances.GetData(MEMORYDEVICE_CUDA),num_instances,
                                scene_gt->index->GetEntries(),scene_gt->index->noTotalEntries,scene_gt->localVBA->GetVoxelBlocks(), threshold);

#ifndef NDEBUG
    // Check again
    numInstances.Clear(0);
    calculateIoUOfInstances_device<TVoxelRC, TVoxelGT><<<GET_1D_BLOCKS(scene_gt->index->noTotalEntries, threadPerBlock), threadPerBlock>>>
           (numInstances.GetData(MEMORYDEVICE_CUDA), num_instances,
                   scene_rc->index->GetEntries(),scene_rc->index->noTotalEntries,scene_rc->localVBA->GetVoxelBlocks(),
                   scene_gt->index->GetEntries(),scene_gt->index->noTotalEntries,scene_gt->localVBA->GetVoxelBlocks());
    numInstances.UpdateHostFromDevice();
    printf("After instance filtering\n");
    for(size_t i=0;i<num_instances;++i)
        printf("%zu: %d/%d(%f)\n",i,numInstances.GetData(MEMORYDEVICE_CPU)[i].x,
               numInstances.GetData(MEMORYDEVICE_CPU)[i].y,
               (float)numInstances.GetData(MEMORYDEVICE_CPU)[i].x/(float)numInstances.GetData(MEMORYDEVICE_CPU)[i].y);
#endif

}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::RemoveOtherInstance(
                         ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt, size_t num_instances, ushort *instances){
    keepOnlyTargetInstance_device<TVoxelGT,ITMVoxelIndex><<<GET_1D_BLOCKS(scene_gt->index->noTotalEntries, threadPerBlock), threadPerBlock>>>
    (num_instances,instances,scene_gt->index->GetEntries(),scene_gt->index->noTotalEntries,scene_gt->localVBA->GetVoxelBlocks());
    cudaDeviceSynchronize();
}


template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void FuseCloud2Scene_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                      float oneOverVoxelSize, const Vector3f *points, const unsigned short *label){

    Vector3f pt = points[i] * oneOverVoxelSize;

    typename TIndex::IndexCache cache;
    int vmIndex;
    int idx_to = findVoxel(voxelIndex,Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z)),vmIndex, cache);
    if( !vmIndex  ) return;
    TVoxel &voxel_to = voxelData[idx_to];
    atomicMax(&voxel_to.sdf, LOGODD_OCU);
    voxel_to.w_depth = 1;
    voxel_to.label = label[i];
}

template<class TVoxel, class TIndex>
__global__ void FuseCloudLabel2Map_device(int size, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                          float oneOverVoxelSize, const Vector3f *value_from, const unsigned short *label)
{
    CUDA_1D_LOOP(i, size) {
        FuseCloud2Scene_shared<TVoxel, ITMVoxelIndex>(i, voxelData, voxelIndex, oneOverVoxelSize, value_from, label);
    }
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void FuseCloud2Scene_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                      float oneOverVoxelSize, const Vector3f *points, const unsigned short *label,
                                                      const unsigned int *instance){

    Vector3f pt = points[i] * oneOverVoxelSize;

    typename TIndex::IndexCache cache;
    int vmIndex;
    int idx_to = findVoxel(voxelIndex,Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z)),vmIndex, cache);
    if( !vmIndex  ) return;
    TVoxel &voxel_to = voxelData[idx_to];
    atomicMax(&voxel_to.sdf, LOGODD_OCU);
    voxel_to.w_depth = 1;
    voxel_to.label = label[i];
    voxel_to.semantic = instance[i];
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void FuseOccupancy2Map_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                        float oneOverVoxelSize, const Vector3f *points){

    Vector3f pt = points[i] * oneOverVoxelSize;

    typename TIndex::IndexCache cache;
    int vmIndex;
    int idx_to = findVoxel(voxelIndex,Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z)),vmIndex, cache);
    if( !vmIndex  ) return;
    TVoxel &voxel_to = voxelData[idx_to];
    voxel_to.sdf = LOGODD_OCU;
    voxel_to.w_depth = 1;
}

template<class TVoxel, class TIndex>
__global__ void FuseCloudLabel2Map_device(int size, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                          float oneOverVoxelSize, const Vector3f *value_from, const unsigned short *label,
                                          const unsigned int *instances)
{
    CUDA_1D_LOOP(i, size) {
        FuseCloud2Scene_shared<TVoxel, ITMVoxelIndex>(i, voxelData, voxelIndex, oneOverVoxelSize, value_from, label, instances);
    }
}


template<class TVoxel, class TIndex>
__global__ void FuseOccupancy2Map_device(int size, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                         float oneOverVoxelSize, const Vector3f *value_from)
{
    CUDA_1D_LOOP(i, size) {
        FuseOccupancy2Map_shared<TVoxel, ITMVoxelIndex>(i, voxelData, voxelIndex, oneOverVoxelSize, value_from);
    }
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::FuseCloud2Scene(const ORUtils::MemoryBlock<ORUtils::Vector3<float>> *points,
                                      const ORUtils::MemoryBlock<unsigned short> *labels,
                                      const ORUtils::MemoryBlock<unsigned int> *instances,
                                      ITMLib::ITMScene<TVoxelGT, ITMVoxelIndex> *scene, bool occupancyOnly) {
    assert(points->dataSize == labels->dataSize);
    assert(points->dataSize == instances->dataSize);
    float oneOverVoxelSize = 1.f / scene->sceneParams->voxelSize;

    if (occupancyOnly)
        FuseOccupancy2Map_device<TVoxelGT, ITMVoxelIndex> << <
    GET_1D_BLOCKS(points->dataSize, threadPerBlock), threadPerBlock >> >
                                                     (points->dataSize, scene->localVBA->GetVoxelBlocks(), scene->index->getIndexData(), oneOverVoxelSize,
                                                             points->GetData(MEMORYDEVICE_CUDA));
    else {
//        DEBUG("scene->index->noTotalEntries: %d\n",scene->index->noTotalEntries);
//        ORUtils::MemoryBlock<uint> counter(2,true,true);
//        counter.Clear(0);
//        FillAllocatedBlocks<TVoxel,ITMVoxelIndex><<<GET_1D_BLOCKS(scene->index->noTotalEntries, threadPerBlock), threadPerBlock>>>
//                (scene->index->GetEntries(),scene->index->noTotalEntries, scene->localVBA->GetVoxelBlocks(), counter.GetData(MEMORYDEVICE_CUDA));
//
//        counter.UpdateHostFromDevice();
//        for(size_t i=0;i<counter.dataSize;++i)
//            printf("counter[%zu]: %d\n", i, counter.GetData(MEMORYDEVICE_CPU)[i]);

        FuseCloudLabel2Map_device<TVoxelGT, ITMVoxelIndex> << <
        GET_1D_BLOCKS(points->dataSize, threadPerBlock), threadPerBlock >> >
                                                         (points->dataSize, scene->localVBA->GetVoxelBlocks(), scene->index->getIndexData(), oneOverVoxelSize,
                                                                 points->GetData(MEMORYDEVICE_CUDA), labels->GetData(
                                                                 MEMORYDEVICE_CUDA),
                                                                 instances->GetData(MEMORYDEVICE_CUDA));

//        FindMaxLabel_device<TVoxel, ITMVoxelIndex> << <
//        GET_1D_BLOCKS(points->dataSize, threadPerBlock), threadPerBlock >> >
//                                                         (points->dataSize, scene->localVBA->GetVoxelBlocks(), scene->index->getIndexData(),
//                                                                 oneOverVoxelSize, points->GetData(
//                                                                 MEMORYDEVICE_CUDA), labels->GetData(
//                                                                 MEMORYDEVICE_CUDA));
    }
}

template<class TVoxelRC, class TVoxelGT>
void TrainingDataGenerator<TVoxelRC, TVoxelGT>::FuseCloud2Scene(const ORUtils::MemoryBlock<ORUtils::Vector3<float>> *points,
                                            const ORUtils::MemoryBlock<unsigned short> *labels,
                                            ITMLib::ITMScene<TVoxelGT, ITMVoxelIndex> *scene, bool occupancyOnly) {
    assert(points->dataSize == labels->dataSize);
    float oneOverVoxelSize = 1.f / scene->sceneParams->voxelSize;

    if (occupancyOnly)
        FuseOccupancy2Map_device<TVoxelGT, ITMVoxelIndex> << <
        GET_1D_BLOCKS(points->dataSize, threadPerBlock), threadPerBlock >> >
                                                         (points->dataSize, scene->localVBA->GetVoxelBlocks(), scene->index->getIndexData(), oneOverVoxelSize,
                                                                 points->GetData(MEMORYDEVICE_CUDA));
    else {
//        DEBUG("scene->index->noTotalEntries: %d\n",scene->index->noTotalEntries);
//        ORUtils::MemoryBlock<uint> counter(2,true,true);
//        counter.Clear(0);
//        FillAllocatedBlocks<TVoxel,ITMVoxelIndex><<<GET_1D_BLOCKS(scene->index->noTotalEntries, threadPerBlock), threadPerBlock>>>
//                (scene->index->GetEntries(),scene->index->noTotalEntries, scene->localVBA->GetVoxelBlocks(), counter.GetData(MEMORYDEVICE_CUDA));
//
//        counter.UpdateHostFromDevice();
//        for(size_t i=0;i<counter.dataSize;++i)
//            printf("counter[%zu]: %d\n", i, counter.GetData(MEMORYDEVICE_CPU)[i]);

        FuseCloudLabel2Map_device<TVoxelGT, ITMVoxelIndex> << <
        GET_1D_BLOCKS(points->dataSize, threadPerBlock), threadPerBlock >> >
                                                         (points->dataSize, scene->localVBA->GetVoxelBlocks(), scene->index->getIndexData(), oneOverVoxelSize,
                                                                 points->GetData(MEMORYDEVICE_CUDA), labels->GetData(
                                                                 MEMORYDEVICE_CUDA));

//        FindMaxLabel_device<TVoxel, ITMVoxelIndex> << <
//        GET_1D_BLOCKS(points->dataSize, threadPerBlock), threadPerBlock >> >
//                                                         (points->dataSize, scene->localVBA->GetVoxelBlocks(), scene->index->getIndexData(),
//                                                                 oneOverVoxelSize, points->GetData(
//                                                                 MEMORYDEVICE_CUDA), labels->GetData(
//                                                                 MEMORYDEVICE_CUDA));
    }
}
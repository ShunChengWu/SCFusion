#pragma once


#include "../Interface/PointCloudExtractionEngine.h"
#include "../../../Objects/PointCloud/PointCloud.h"
#include "../../../Objects/Scene/ITMVoxelTypes.h"

namespace SCFUSION {
    template<class TVoxel, class TIndex>
    class PointCloudExtractionEngine_CUDA : public PointCloudExtractionEngine <TVoxel, TIndex>{
    public:
        void ExtractPointCloud(SCFUSION::PointCloud *pointcloud, const ITMLib::ITMScene<TVoxel,TIndex> *scene){};
        void setStream(void *stream){}
        void syncStream() {};
    };

    template <class TVoxel>
    class PointCloudExtractionEngine_CUDA<TVoxel, ITMLib::ITMVoxelBlockHash> : public PointCloudExtractionEngine<TVoxel, ITMLib::ITMVoxelBlockHash> {
        std::unique_ptr<ORUtils::MemoryBlock<uint>> voxelOccupied_, voxelOccupiedScan_, compactedVoxelArray_;
        cudaStream_t stream_;
        std::unique_ptr<ORUtils::MemoryBlock<Vector4s>> visibleBlockGlobalPos;
        const size_t visibleBlockPos_size = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
    public:
        explicit PointCloudExtractionEngine_CUDA();

        void ExtractPointCloud(SCFUSION::PointCloud *pointcloud, const ITMLib::ITMScene<TVoxel,ITMLib::ITMVoxelBlockHash> *scene);
        void setStream(void *stream) {stream_ = static_cast<cudaStream_t>(stream);}
        void syncStream() {cudaStreamSynchronize(stream_);};
    private:

    };
}
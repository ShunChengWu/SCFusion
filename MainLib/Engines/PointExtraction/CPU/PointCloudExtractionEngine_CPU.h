#pragma once
#include "../Interface/PointCloudExtractionEngine.h"
#include "../../../Utils/Exceptions.h"

namespace SCFUSION {
    template <class TVoxel, class TIndex>
    class PointCloudExtractionEngine_CPU: public PointCloudExtractionEngine<TVoxel, TIndex> {
        explicit PointCloudExtractionEngine_CPU(){};
        void setStream(void *stream) {}
        void syncStream() {};
        void ExtractPointCloud(SCFUSION::PointCloud *pointcloud, const ITMLib::ITMScene<TVoxel,TIndex> *scene){
            throw ERROR_NotImplemented();
        };
    };

    template <class TVoxel>
    class PointCloudExtractionEngine_CPU<TVoxel, ITMLib::ITMVoxelBlockHash> : public PointCloudExtractionEngine<TVoxel, ITMLib::ITMVoxelBlockHash> {
        std::unique_ptr<ORUtils::MemoryBlock<uint>> voxelOccupied_, voxelOccupiedScan_, compactedVoxelArray_;
        cudaStream_t stream_;
        std::unique_ptr<ORUtils::MemoryBlock<Vector4s>> visibleBlockGlobalPos;
        const size_t visibleBlockPos_size = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
    public:
        explicit PointCloudExtractionEngine_CPU(){};

        void ExtractPointCloud(SCFUSION::PointCloud *pointcloud, const ITMLib::ITMScene<TVoxel,ITMLib::ITMVoxelBlockHash> *scene){
            throw ERROR_NotImplemented();
        };
        void setStream(void *stream) {stream_ = static_cast<cudaStream_t>(stream);}
        void syncStream() {};
    private:

    };
}
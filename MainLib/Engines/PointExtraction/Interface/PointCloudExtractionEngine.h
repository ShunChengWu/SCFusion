#pragma once

#include "../../../Objects/PointCloud/PointCloud.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Scene/ITMScene.h"

namespace SCFUSION {
    template<class TVoxel, class TIndex>
    class PointCloudExtractionEngine {
    public:
        virtual void ExtractPointCloud(SCFUSION::PointCloud *pointcloud, const ITMLib::ITMScene<TVoxel,ITMLib::ITMVoxelBlockHash> *scene) = 0;
        virtual void setStream(void *stream){}
        virtual void syncStream() = 0;
        explicit PointCloudExtractionEngine() = default;
        virtual ~PointCloudExtractionEngine() = default;
    private:

    };
}
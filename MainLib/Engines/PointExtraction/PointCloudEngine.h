#pragma once
#include "../../Utils/ITMLibSettings.h"
#include "../../Objects/PointCloud/PointCloud.h"
#include "Interface/PointCloudExtractionEngine.h"
#include "../../ITMLibDefines.h"
namespace SCFUSION {
    template<class TVoxel, class TIndex>
    class PointCloudEngine {
    public:
        PointCloudEngine(const ITMLib::ITMLibSettings *itmLibSettings);

        virtual void computePointCloud(ITMLib::ITMScene<TVoxel, ITMVoxelIndex>* itmScene);

        SCFUSION::PointCloud* getPointCloud();

//        void setLabelColorList(Vector4f *pointer) {
//            pointCloudExtractionEngine->setLabelColorListPtr(pointer);
//        }

        void SyncPointExtractionEngine(){ pointCloudExtractionEngine->syncStream(); }

        void reset(){
            pointcloud->noTotalPoints=0;
            pointcloud->points->Clear(0);
            pointcloud->colors->Clear(0);
        }
    protected:
        std::unique_ptr<SCFUSION::PointCloudExtractionEngine<ITMVoxel, ITMVoxelIndex>> pointCloudExtractionEngine;
        std::unique_ptr<SCFUSION::PointCloud> pointcloud;
    };
}
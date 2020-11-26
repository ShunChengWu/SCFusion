#include "PointCloudEngine.h"
#include "PointCloudExtractionEngineFactory.h"

using namespace SCFUSION;
template<class TVoxel, class TIndex>
PointCloudEngine<TVoxel,TIndex>::PointCloudEngine(const ITMLib::ITMLibSettings *itmSettings) {
    if(itmSettings->createPointExtractionEngine)
        pointcloud.reset(new SCFUSION::PointCloud(itmSettings->GetMemoryType()));

    if(itmSettings->createPointExtractionEngine) {
        pointCloudExtractionEngine.reset(
                PointCloudEngineFactory::MakeMeshingEngine<ITMVoxel, ITMVoxelIndex>(itmSettings->deviceType));
    }
}
template<class TVoxel, class TIndex>
SCFUSION::PointCloud* PointCloudEngine<TVoxel,TIndex>::getPointCloud() {
    return pointcloud.get();
}

template<class TVoxel, class TIndex>
void PointCloudEngine<TVoxel,TIndex>::computePointCloud(ITMLib::ITMScene<TVoxel, ITMVoxelIndex>* itmScene) {
    if(pointCloudExtractionEngine == nullptr)
        throw std::runtime_error("Did not enable point extraction!!\n");
    pointCloudExtractionEngine->ExtractPointCloud(pointcloud.get(), itmScene);
}
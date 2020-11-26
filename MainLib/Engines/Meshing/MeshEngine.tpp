#include "MeshEngine.h"
#include "ITMMeshingEngineFactory.h"
using namespace SCFUSION;
template<class TVoxel, class TIndex>
MeshEngine<TVoxel,TIndex>::MeshEngine(const ITMLib::ITMLibSettings *itmSettings){
    if (itmSettings->createMeshingEngine)
        itmMesh.reset(new ITMLib::ITMMesh(itmSettings->GetMemoryType()));
    if (itmSettings->createMeshingEngine) {
        itmMeshingEngine.reset(ITMLib::ITMMeshingEngineFactory::MakeMeshingEngine<ITMVoxel, ITMVoxelIndex>(
                itmSettings->deviceType,
                ITMVoxel::integrateType == SCFUSION::IntegrateType_TSDF ? 0 : LOGODD_SURFACE));
    }
}

template<class TVoxel, class TIndex>
bool MeshEngine<TVoxel,TIndex>::computeMesh(ITMLib::ITMScene<TVoxel, ITMVoxelIndex>* itmScene, bool labelOnly, bool checkState) {
    if (itmMeshingEngine == nullptr) {
        std::cout << "Did not enable meshing!!\n";
        return false;
    }
    switch (ITMVoxel::integrateType) {
        case SCFUSION::IntegrateType_TSDF:
            itmMeshingEngine->setIsoValue(0.f);
            break;
        case SCFUSION::IntegrateType_OFusion:
            itmMeshingEngine->setIsoValue(LOGODD_SURFACE);
            break;
    };
    if (labelOnly)
        itmMeshingEngine->MeshSceneLabel(itmMesh.get(), itmScene, checkState);
    else
        itmMeshingEngine->MeshScene(itmMesh.get(), itmScene, checkState);
    return true;
}
template<class TVoxel, class TIndex>
ITMLib::ITMMesh* MeshEngine<TVoxel,TIndex>::getMesh() { return itmMesh.get(); }


template<class TVoxel, class TIndex>
void MeshEngine<TVoxel,TIndex>::computeMesh(ORUtils::MemoryBlock<float> *data, const Vector3f &origin, const Vector3s &dims, float voxelSize, float isoValue){
    if(itmMeshingEngine == nullptr) {
        std::cout << "Did not enable meshing!!\n";
        return;
    }
    itmMeshingEngine->setIsoValue(isoValue);
    itmMeshingEngine->MeshScene(itmMesh.get(), data, origin, dims, voxelSize);
}


template<class TVoxel, class TIndex>
void MeshEngine<TVoxel,TIndex>::saveSceneToMesh(ITMLib::ITMScene <TVoxel, ITMVoxelIndex> *itmScene, const std::string &pth_to_directory,
                     bool labelOnly, bool checkState){

    if(!computeMesh(itmScene, labelOnly, checkState))return;
//    std::string saveOutputDirectory = tools::PathTool::CheckEnd(pth_to_directory);
    std::string outputPLYDirectory = pth_to_directory + "/mesh.ply";
//    tools::PathTool::check_and_create_folder(saveOutputDirectory);
    itmMesh->WritePLY(outputPLYDirectory.c_str());
}
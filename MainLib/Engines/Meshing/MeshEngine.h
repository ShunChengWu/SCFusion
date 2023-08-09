#pragma once

#include <ORUtils/Matrix.h>
#include "../../ITMLibDefines.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Meshing/ITMMesh.h"
#include "Interface/ITMMeshingEngine.h"
#include "../../Utils/ITMLibSettings.h"

namespace SCFUSION {
    template<class TVoxel, class TIndex>
    class MeshEngine {
    public:
        MeshEngine(const ITMLib::ITMLibSettings *itmLibSettings);

        virtual bool computeMesh(ITMLib::ITMScene <TVoxel, ITMVoxelIndex> *itmScene, bool labelOnly, bool checkState);

        virtual void
        computeMesh(ORUtils::MemoryBlock<float> *data, const Vector3f &origin, const Vector3s &dims, float voxelSize,
                    float isoValue);

        void saveSceneToMesh(ITMLib::ITMScene <TVoxel, ITMVoxelIndex> *itmScene, const std::string &pth_to_directory,
                             bool labelOnly=false, bool checkState=true);

        ITMLib::ITMMesh *getMesh();

        void setLabelColorList(Vector4f *pointer) {
            itmMeshingEngine->setLabelColorListPtr(pointer);
        }

        void SyncMeshingEngine(){ itmMeshingEngine->syncStream(); }

        void reset(){
            itmMeshingEngine->reset();
            itmMesh->noTotalTriangles = 0;
            itmMesh->triangles->Clear(0);
            if(itmMesh->hasColor) itmMesh->colors->Clear(0);
            if(itmMesh->hasNormal) itmMesh->normals->Clear(0);
        }
    protected:
        std::unique_ptr <ITMLib::ITMMeshingEngine<ITMVoxel, ITMVoxelIndex>> itmMeshingEngine;
        std::unique_ptr <ITMLib::ITMMesh> itmMesh;
    };
}
#pragma once
#include "../../../Objects/Scene/ITMVoxelTypes.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/SceneCompletion/SceneCompletionBuffer.h"
#include <ORUtils/MemoryBlock.h>
#include <vector>
//#include "SceneCompletion.h"

namespace SCFUSION{

    template <class TVoxel, class TIndex>
    class SceneCompletionEngine {
    protected:
        SceneCompletionMethod sceneCompletionMethod_;
    public:
        SceneCompletionEngine(SceneCompletionMethod sceneCompletionMethod):sceneCompletionMethod_(sceneCompletionMethod){};
        virtual ~SceneCompletionEngine()= default;

        /// Check should do completion or not
        virtual float CriteriaCheck(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene) = 0;

        /// Extract sub-map
        virtual void Extraction(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene) = 0;

        /// Fuse sub-map
        virtual void Fusion(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, const SCBuffer *scBuffer) = 0;

        /// Extract label and confidence
        virtual void ExtractLabelAndConfidnece(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene) = 0;

        /// Fuse label and confidence
        virtual void FusionLabelAndConfidence(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, const SCBuffer *scBuffer) = 0;

        virtual void ResetSCBuffer(SCBuffer *scBuffer) = 0;

        virtual void setStream(void *stream)=0;
        virtual void syncStream() = 0;
    };

    class NetworkBackEnd {
    public:
        NetworkBackEnd()=default;
        virtual ~NetworkBackEnd()=default;
    };
}
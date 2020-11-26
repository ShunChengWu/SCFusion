#pragma once

#include "../Interface/SceneCompletionEngine.h"
#include "../../Reconstruction/Interface/ITMSceneReconstructionEngine.h"

namespace SCFUSION {
    template<class TVoxel, class TIndex>
    class SceneCompletionEngine_CUDA : public SceneCompletionEngine<TVoxel, TIndex>{};

    template<class TVoxel>
    class SceneCompletionEngine_CUDA <TVoxel, ITMLib::ITMVoxelBlockHash> : public SceneCompletionEngine<TVoxel, ITMLib::ITMVoxelBlockHash> {
    private:
        cudaStream_t stream_;
    public:
        void ResetSCBuffer(SCBuffer *scBuffer);

        float CriteriaCheck(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene);

        void Extraction(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene);

        void Fusion(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, const SCBuffer *scBuffer);

        /// Extract label and confidence
        void ExtractLabelAndConfidnece(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene);

        /// Fuse label and confidence
        void FusionLabelAndConfidence(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, const SCBuffer *scBuffer);

        void setStream(void *stream) { stream_ = static_cast<cudaStream_t >(stream); }
        void syncStream() {cudaStreamSynchronize(stream_);}

        SceneCompletionEngine_CUDA(SceneCompletionMethod sceneCompletionMethod);
        ~SceneCompletionEngine_CUDA();
    };
}
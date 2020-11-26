// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMVisualisationEngine.h"

struct RenderingBlock;

namespace ITMLib {
    template<class TVoxel, class TIndex>
    class ITMVisualisationEngine_CUDA : public ITMVisualisationEngine<TVoxel, TIndex> {
    private:
        uint *noTotalPoints_device;
        cudaStream_t stream_;
    public:
        using ITMVisualisationEngine<TVoxel, TIndex>::labelColorList;

        explicit ITMVisualisationEngine_CUDA();

        ~ITMVisualisationEngine_CUDA();

        void setStream(void *stream) { stream_ = static_cast<cudaStream_t>(stream); }

        void syncStream() { cudaStreamSynchronize(stream_); }

        ITMRenderState *CreateRenderState(const ITMScene <TVoxel, TIndex> *scene, const Vector2i &imgSize) const;

        void FindVisibleBlocks(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                               const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

        int
        CountVisibleBlocks(const ITMScene <TVoxel, TIndex> *scene, const ITMRenderState *renderState, int minBlockId,
                           int maxBlockId) const;

        void CreateExpectedDepths(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                                  const ITMIntrinsics *intrinsics, const Vector2f &viewFrustumRange,
                                  ITMRenderState *renderState) const;

        void RenderImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics,
                         const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                         ITMUChar4Image *outputImage,
                         IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
                         IITMVisualisationEngine::RenderRaycastSelection raycastType = IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const override;

        void GetDepthImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                           const ITMIntrinsics *intrinsics,
                           const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                           ITMFloatImage *outputImage) const;

        void GetVertexImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics,
                            const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                            ITMFloat3Image *outputImage) const;

        void GetVertexImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics,
                            const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                            ITMFloat4Image *outputImage) const;

        void GetNormalImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics,
                            const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                            ITMFloat4Image *outputImage) const;

        void GetLabelImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                           const ITMIntrinsics *intrinsics,
                           const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                           ITMUShortImage *outputImage) const;

        void GetSemanticImage(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                              const ITMIntrinsics *intrinsics,
                              const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                              ITMUShortImage *outputImage) const;

        void FindSurface(const ITMScene <TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                         ITMLib::IITMVisualisationEngine::RenderMode renderMode) const;

        void
        CreatePointCloud(const ITMScene <TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                         ITMRenderState *renderState, bool skipPoints) const;

        void CreateICPMaps(const ITMScene <TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                           ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode) const;

        void ForwardRender(const ITMScene <TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                           ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode) const;
    };

    template<class TVoxel>
    class ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>
            : public ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash> {
    private:
        uint *noTotalPoints_device;
        RenderingBlock *renderingBlockList_device;
        uint *noTotalBlocks_device;
        int *noVisibleEntries_device;
        void *allocationTempData_device;
        void *allocationTempData_host;
        cudaStream_t stream_;
    public:
        using ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash>::labelColorList;

        explicit ITMVisualisationEngine_CUDA();

        ~ITMVisualisationEngine_CUDA();

        void setStream(void *stream) { stream_ = static_cast<cudaStream_t>(stream); }

        void syncStream() { cudaStreamSynchronize(stream_); };

        ITMRenderState_VH *
        CreateRenderState(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const Vector2i &imgSize) const;

        void FindVisibleBlocks(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                               const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

        int CountVisibleBlocks(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState,
                               int minBlockId, int maxBlockId) const;

        void CreateExpectedDepths(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                                  const ITMIntrinsics *intrinsics, const Vector2f &viewFrustumRange,
                                  ITMRenderState *renderState) const;

        void RenderImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                         ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                         ITMUChar4Image *outputImage,
                         IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
                         IITMVisualisationEngine::RenderRaycastSelection raycastType = IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;

        void GetDepthImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                           const ITMIntrinsics *intrinsics,
                           const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                           ITMFloatImage *outputImage) const;

        void GetVertexImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics,
                            const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                            ITMFloat3Image *outputImage) const;

        void GetVertexImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics,
                            const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                            ITMFloat4Image *outputImage) const;

        void GetNormalImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics,
                            const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                            ITMFloat4Image *outputImage) const;

        void GetLabelImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                           const ITMIntrinsics *intrinsics,
                           const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                           ITMUShortImage *outputImage) const;

        void GetSemanticImage(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                              const ITMIntrinsics *intrinsics,
                              const ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                              ITMUShortImage *outputImage) const;

        void FindSurface(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics,
                         const ITMRenderState *renderState,
                         ITMLib::IITMVisualisationEngine::RenderMode renderMode) const;

        void CreatePointCloud(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                              ITMTrackingState *trackingState,
                              ITMRenderState *renderState, ITMLib::IITMVisualisationEngine::RenderMode renderMode,
                              bool skipPoints) const override;

        void CreateICPMaps(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                           ITMTrackingState *trackingState,
                           ITMRenderState *renderState,
                           ITMLib::IITMVisualisationEngine::RenderMode renderMode) const override;

        void ForwardRender(const ITMScene <TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                           ITMTrackingState *trackingState,
                           ITMRenderState *renderState,
                           ITMLib::IITMVisualisationEngine::RenderMode renderMode) const override;
    };
}

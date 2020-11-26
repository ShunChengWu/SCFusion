// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSceneReconstructionEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSceneReconstructionEngine_CUDA : public ITMSceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, ITMVoxelBlockHash >
	{
    protected:
		void *allocationTempData_device;
		void *allocationTempData_host;
		unsigned char *entriesAllocType_device;
		Vector4s *blockCoords_device;
        cudaStream_t stream_;
	public:
        void setStream(void *stream) override {stream_ = static_cast<cudaStream_t>(stream);}
        void syncStream() override {cudaStreamSynchronize(stream_);};

		void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

        void AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const Matrix4f &M_d,
                                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState);
        void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const Matrix4f &M_d,
                                const ITMRenderState *renderState);
        void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState,
                                        const ORUtils::MemoryBlock<Vector3f> *points,
                                        const ORUtils::MemoryBlock<unsigned short> *labels,
                                const ORUtils::MemoryBlock<unsigned short> *instances);

		void AllocationSceneFromVolume(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const float *data, const Vector3f &volume_start, const Vector3s &blockDims,
                                       const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

        void AllocationSceneFromPoints(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::MemoryBlock<Vector3f> *points,
                                       const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		ITMSceneReconstructionEngine_CUDA();
		~ITMSceneReconstructionEngine_CUDA();
	};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CUDA<TVoxel, ITMPlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, ITMPlainVoxelArray >
	{
    private:
	    cudaStream_t stream_;
	public:
        using ITMSceneReconstructionEngine<TVoxel,ITMVoxelBlockHash>::integratePolicy;

        void setStream(void *stream) {stream_ = static_cast<cudaStream_t>(stream);}

		void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

		void AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState);
	};
}

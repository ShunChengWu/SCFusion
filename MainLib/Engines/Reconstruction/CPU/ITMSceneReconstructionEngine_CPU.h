// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSceneReconstructionEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSceneReconstructionEngine_CPU : public ITMSceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSceneReconstructionEngine < TVoxel, ITMVoxelBlockHash >
	{
	protected:
		ORUtils::MemoryBlock<unsigned char> *entriesAllocType;
		ORUtils::MemoryBlock<Vector4s> *blockCoords;

	public:
        /**
           Reset Hash table and all the voxel blocks

         @param ITMScene<TVoxel ITMScene<TVoxel, ITMVoxelBlockHash> scene
         */
		void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);;

        void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const Matrix4f &M_d,
                                    const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);;

        void AllocationSceneFromVolume(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const float *data, const Vector3f &volume_start, const Vector3s &blockNums,
                                       const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false){}

        void AllocationSceneFromPoints(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::MemoryBlock<Vector3f> *points,
                                               const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) {};

		void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState);

        void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const Matrix4f &M_d,
                                const ITMRenderState *renderState);
        void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState,
                                const ORUtils::MemoryBlock<Vector3f> *points,
                                const ORUtils::MemoryBlock<unsigned short> *labels,
                                const ORUtils::MemoryBlock<unsigned short> *instances);

		ITMSceneReconstructionEngine_CPU();
		~ITMSceneReconstructionEngine_CPU();

		void setStream(void *stream) override {};
        void syncStream() override {};
	};

	template<class TVoxel>
	class ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray> : public ITMSceneReconstructionEngine < TVoxel, ITMPlainVoxelArray >
	{
	public:
        using ITMSceneReconstructionEngine<TVoxel,ITMVoxelBlockHash>::integratePolicy;

		void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

		void AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState);

		ITMSceneReconstructionEngine_CPU();
		~ITMSceneReconstructionEngine_CPU();
	};
}

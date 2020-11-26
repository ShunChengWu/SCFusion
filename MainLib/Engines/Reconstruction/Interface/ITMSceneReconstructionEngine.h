// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <cmath>

#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Scene/IntegratePolicies.h"


namespace ITMLib
{
    
	/** \brief
	    Interface to engines implementing the main KinectFusion
	    depth integration process.

	    These classes basically manage
	    an ITMLib::Objects::ITMScene and fuse new image information
	    into them.
	*/
	template<class TVoxel, class TIndex>
	class ITMSceneReconstructionEngine
	{
	public:
		/** Clear and reset a scene to set up a new empty
		    one.
{
    AllocateSceneFromDepth(scene, view, trackingState->pose_d->GetM(), renderState, onlyUpdateVisibleList, resetVisibleList);
}

		*/
		virtual void ResetScene(ITMScene<TVoxel, TIndex> *scene) = 0;

		/** Given a view with a new depth image, compute the
		    visible blocks, allocate them and update the hash
		    table so that the new image data can be integrated.
		*/
		virtual void AllocateSceneFromDepth(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

        virtual void AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const Matrix4f &M_d,
                                    const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

        virtual void AllocationSceneFromVolume(ITMScene<TVoxel, TIndex> *scene, const float *data, const Vector3f &volume_start, const Vector3s &blockDims,
                                       const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

        virtual void AllocationSceneFromPoints(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::MemoryBlock<Vector3f> *points,
                                       const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

		/** Update the voxel blocks by integrating depth and
		    possibly colour information from the given view.
		*/
		virtual void IntegrateIntoScene(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState) = 0;

        virtual void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const Matrix4f &M_d,
                                        const ITMRenderState *renderState) = 0;
        virtual void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState,
                                        const ORUtils::MemoryBlock<Vector3f> *points,
                                        const ORUtils::MemoryBlock<unsigned short> *labels,
                                        const ORUtils::MemoryBlock<unsigned short> *instances) = 0;


		virtual void setStream(void *stream) = 0;
        virtual void syncStream() = 0;

		ITMSceneReconstructionEngine()= default;
		virtual ~ITMSceneReconstructionEngine() = default;
	};
}

// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>
#include <memory>
#include "../../../Objects/Meshing/ITMMesh.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/Scene/ITMVoxelTypes.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine
	{
    protected:
        std::unique_ptr<ORUtils::MemoryBlock<uint>> voxelVert_, voxelVertScan_, voxelOccupied_, voxelOccupiedScan_, compactedVoxelArray_;
        float isoValue_;
        const Vector4f *labelColorPtr_;
	public:
	    void setIsoValue(float isoValue){this->isoValue_ = isoValue;}
		virtual void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel,TIndex> *scene, bool checkVoxelState) = 0;
        virtual void MeshSceneLabel(ITMMesh *mesh, const ITMScene<TVoxel,TIndex> *scene, bool checkVoxelState) = 0;
		virtual void MeshScene(ITMMesh *mesh, const ORUtils::MemoryBlock<float> *localVBA, const Vector3f &origin, const Vector3s &dims, float voxelSize) =0;
        virtual void setStream(void *stream) = 0;
        virtual void syncStream() = 0;
        void setLabelColorListPtr(const Vector4f *labelColorPtr) { labelColorPtr_ = labelColorPtr; };
        virtual void reset() {
            if(voxelVert_)voxelVert_->Clear();
            if(voxelVertScan_)voxelVertScan_->Clear();
            if(voxelOccupied_)voxelOccupied_->Clear();
            if(voxelOccupiedScan_)voxelOccupiedScan_->Clear();
            if(compactedVoxelArray_)compactedVoxelArray_->Clear();
        };

		ITMMeshingEngine(const float &isoValue):isoValue_(isoValue), labelColorPtr_(NULL) { }
		virtual ~ITMMeshingEngine() { }
	};
}

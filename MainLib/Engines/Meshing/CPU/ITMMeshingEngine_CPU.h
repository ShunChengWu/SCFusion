// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAMsetStream

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine_CPU : public ITMMeshingEngine < TVoxel, TIndex >
	{
	public:
        using ITMMeshingEngine<TVoxel, TIndex>::isoValue_;
		void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, TIndex> *scene, bool checkVoxelState) { }
        void MeshSceneLabel(ITMMesh *mesh, const ITMScene<TVoxel,TIndex> *scene, bool checkVoxelState) {}
        void MeshScene(ITMMesh *mesh, const ORUtils::MemoryBlock<float> *localVBA, const Vector3f &origin, const Vector3s &dims, float voxelSize){}
        void setStream(void *stream){};
        void syncStream() {};
        void reset() override {};
        explicit ITMMeshingEngine_CPU(const float &isovalue=0.f):ITMMeshingEngine<TVoxel, TIndex>(isovalue) { }
        ~ITMMeshingEngine_CPU() = default;
	};

	template<class TVoxel>
	class ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine < TVoxel, ITMVoxelBlockHash >
	{
	public:
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::isoValue_;
		void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, bool checkVoxelState);
        void MeshSceneLabel(ITMMesh *mesh, const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, bool checkVoxelState) {}
        void MeshScene(ITMMesh *mesh, const ORUtils::MemoryBlock<float> *localVBA, const Vector3f &origin, const Vector3s &dims, float voxelSize){}
        void setStream(void *stream){};
        void syncStream() {};
        void reset() override {};
		explicit ITMMeshingEngine_CPU(const float &isovalue=0.f):ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>(isovalue) { }
		~ITMMeshingEngine_CPU() = default;
	};
}

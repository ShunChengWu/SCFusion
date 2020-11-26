// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../../Objects/Scene/ITMVoxelTypes.h"
#include "../../../Objects/Scene/SCPlainVoxelArray.h"
#include <memory>

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine_CUDA : public ITMMeshingEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class ITMMeshingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine < TVoxel, ITMVoxelBlockHash >
	{
	private:
		std::unique_ptr<ORUtils::MemoryBlock<Vector4s>> visibleBlockGlobalPos;
		const size_t visibleBlockPos_size = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
        cudaStream_t  stream_;
	public:
	    using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::isoValue_;
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::voxelVert_;
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::voxelVertScan_;
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::voxelOccupied_;
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::voxelOccupiedScan_;
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::compactedVoxelArray_;
        using ITMMeshingEngine<TVoxel, ITMVoxelBlockHash>::labelColorPtr_;

        void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, bool checkVoxelState);
        void MeshSceneLabel(ITMMesh *mesh, const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, bool checkVoxelState);
        void MeshScene(ITMMesh *mesh, const ORUtils::MemoryBlock<float> *localVBA, const Vector3f &origin, const Vector3s &dims, float voxelSize);
        void reset() override;
        void setStream(void *stream) override {stream_ = static_cast<cudaStream_t>(stream);}
        void syncStream() override {cudaStreamSynchronize(stream_);};
		explicit ITMMeshingEngine_CUDA(const float &isoValue=0.f);
		~ITMMeshingEngine_CUDA();
	};

	template<class TVoxel>
	class ITMMeshingEngine_CUDA<TVoxel, ITMPlainVoxelArray> : public ITMMeshingEngine < TVoxel, ITMPlainVoxelArray >
	{
	private:
        std::unique_ptr<ORUtils::MemoryBlock<Vector4s>> visibleBlockGlobalPos;
        const size_t visibleBlockPos_size = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
        cudaStream_t  stream_;
	public:
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::isoValue_;
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::voxelVert_;
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::voxelVertScan_;
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::voxelOccupied_;
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::voxelOccupiedScan_;
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::compactedVoxelArray_;
        using ITMMeshingEngine<TVoxel, ITMPlainVoxelArray>::labelColorPtr_;

        void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene);
        void MeshSceneLabel(ITMMesh *mesh, const ITMScene<TVoxel,ITMPlainVoxelArray> *scene, bool checkVoxelState) {}
        void MeshScene(ITMMesh *mesh, const ORUtils::MemoryBlock<float> *localVBA, const Vector3f &origin, const Vector3s &dims, float voxelSize);
        void reset() override {};
        void setStream(void *stream) {stream_ = static_cast<cudaStream_t>(stream);}

		explicit ITMMeshingEngine_CUDA(const float &isoValue=0.f);
		~ITMMeshingEngine_CUDA();
	};

#if 0
    template<class TVoxel>
    class ITMMeshingEngine_CUDA<TVoxel, SCFUSION::SCPlainVoxelArray> : public ITMMeshingEngine < TVoxel, SCFUSION::SCPlainVoxelArray >
    {
    private:
        const size_t visibleBlockPos_size = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
        cudaStream_t  stream_;
    public:
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::isoValue_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::dataType_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::voxelVert_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::voxelVertScan_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::voxelOccupied_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::voxelOccupiedScan_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::compactedVoxelArray_;
        using ITMMeshingEngine<TVoxel, SCFUSION::SCPlainVoxelArray>::labelColorPtr_;

        void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, SCFUSION::SCPlainVoxelArray> *scene);
        void setStream(void *stream) {stream_ = static_cast<cudaStream_t>(stream);}

        explicit ITMMeshingEngine_CUDA(const SCFUSION::IntegrateType &datatype=SCFUSION::IntegrateType_OFusion, const float &isoValue=0.f);
        ~ITMMeshingEngine_CUDA();
    };
#endif
}

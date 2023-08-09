#pragma once
#include <memory>
#include <cuda_runtime.h>
#include <Utils/ITMMath.h>
#include <ORUtils/MemoryBlock.h>
#include <Objects/Scene/ITMVoxelTypes.h>
#include <Objects/Scene/ITMLocalVBA.h>
#include <Objects/Scene/ITMVoxelBlockHash.h>
#include <Objects/Scene/ITMScene.h>
#include <ITMLibDefines.h>
#include "../../MainLib/Core/DenseMapper/ITMDenseMapper.h"

namespace SCFUSION {
    template<class TVoxelRC, class TVoxelGT>
    class TrainingDataGenerator {
        std::unique_ptr<ORUtils::MemoryBlock<Vector4s>> visibleBlockGlobalPos;
        const size_t visibleBlockPos_size = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
        std::unique_ptr<ORUtils::MemoryBlock<unsigned int>> totalVisibleBlockPos;
        cudaStream_t  stream_;
    public:
        TrainingDataGenerator(bool verbose);
        void findBoundaries(Vector3i &b_min, Vector3i &b_max, const ITMLib::ITMLocalVBA<TVoxelGT> *localVBA, const ITMLib::ITMVoxelBlockHash *index, float factor, int floorLabel=-1);
        ///
        /// \param origin
        /// \param dims
        /// \param scene_rc
        /// \param scene_gt
        /// \param labelNum the number of labels
        /// \param threshold_ocu At least the percentage of the empty voxels should below this value
        /// \param threshold_labelNum The maximum percentage of one label should be LOWER than this value.
        /// \param threshold_max_domainate One label cannot have more than this percentage
        /// \param threshold_occupancy_rc The percentage of occupied voxel in scene reconstruction should exceed this value.
        /// \return
        bool CheckLabelandDistribution(Vector3i origin, Vector3s dims,
                ITMLib::ITMScene<TVoxelRC, ITMLib::ITMVoxelBlockHash> *scene_rc,
                ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt,
                uint labelNum, float threshold_ocu, uint threshold_labelNum, float threshold_max_domainate,
                float threshold_occupancy_rc=0.02);

        void ExtractToUniMap(ORUtils::MemoryBlock<float> *data, ORUtils::MemoryBlock<float> *label, ORUtils::MemoryBlock<bool> *mask,
                const Vector3i &origin, const Vector3s &dims,
                             ITMLib::ITMScene<TVoxelRC, ITMLib::ITMVoxelBlockHash> *scene_rc, ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt,
                             bool ITSDF, bool outputVoxelDistance, bool makeInitValueInf, bool ofuNorm, bool gtFromRC);
        void ExtractToUniMap(ORUtils::MemoryBlock<unsigned short> *label,
                             const Vector3i &origin, const Vector3s &dims,
                             ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt);
        void ExtractInstanceToUniMap(ORUtils::MemoryBlock<unsigned short> *instance,
                             const Vector3i &origin, const Vector3s &dims,
                             ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt);

        /// Remove unseen instances from the ground truth model.
        /// The voxels with w_depth == 0 is seen as unseen.
        /// \param scene_rc
        /// \param scene_gt
        /// \param num_instances
        /// \param threshold
        void FilterOutBaseOnInstances(const ITMLib::ITMScene<TVoxelRC, ITMLib::ITMVoxelBlockHash> *scene_rc,
                ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt, uint num_instances, float threshold);

        void RemoveOtherInstance(ITMLib::ITMScene<TVoxelGT, ITMLib::ITMVoxelBlockHash> *scene_gt, size_t num_instances, ushort *instances);


        void FuseToScene(ITMLib::ITMDenseMapper<TVoxelGT, ITMVoxelIndex> *itmMapper,
                         ITMLib::ITMScene<TVoxelGT, ITMVoxelIndex> *itmScene,
                         ITMLib::ITMRenderState *renderState,
                        const ORUtils::MemoryBlock<Vector3f> *points,
                        const ORUtils::MemoryBlock<unsigned short> *labels,
                        const ORUtils::MemoryBlock<unsigned int> *instances,
                        bool occupancyOnly = false){
            /// Allocate Scene
            for(size_t i=0;i<100;++i)
                itmMapper->getSceneRecoEngine()->AllocationSceneFromPoints(itmScene, points, renderState);
            cudaDeviceSynchronize();

            /// Integrate
            FuseCloud2Scene(points, labels, instances, itmScene, occupancyOnly);
            cudaDeviceSynchronize();
        }

        void FuseToScene(ITMLib::ITMDenseMapper<TVoxelGT, ITMVoxelIndex> *itmMapper,
                         ITMLib::ITMScene<TVoxelGT, ITMVoxelIndex> *itmScene,
                         ITMLib::ITMRenderState *renderState,
                         const ORUtils::MemoryBlock<Vector3f> *points,
                         const ORUtils::MemoryBlock<unsigned short> *labels,
                         bool occupancyOnly = false){
            /// Allocate Scene
            for(size_t i=0;i<100;++i)
                itmMapper->getSceneRecoEngine()->AllocationSceneFromPoints(itmScene, points, renderState);
            cudaDeviceSynchronize();

            /// Integrate
            FuseCloud2Scene(points, labels, itmScene, occupancyOnly);
            cudaDeviceSynchronize();
        }
    private:
        void FuseCloud2Scene(const ORUtils::MemoryBlock<ORUtils::Vector3<float>> *points,
                             const ORUtils::MemoryBlock<unsigned short> *labels,
                             const ORUtils::MemoryBlock<unsigned int> *instances,
                             ITMLib::ITMScene<TVoxelGT, ITMVoxelIndex> *scene,
                             bool occupancyOnly);
        void FuseCloud2Scene(const ORUtils::MemoryBlock<ORUtils::Vector3<float>> *points,
                             const ORUtils::MemoryBlock<unsigned short> *labels,
                             ITMLib::ITMScene<TVoxelGT, ITMVoxelIndex> *scene,
                             bool occupancyOnly);
    private:
        bool verbose_;
    };
}
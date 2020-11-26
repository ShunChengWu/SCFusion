// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once
#include <memory>
#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"
#include "../../Utils/ITMSceneParams.h"

//#include "UnitMapSaver/UnitMapSaverFactory.h"

namespace ITMLib
{
	/** \brief
	Represents the 3D world model as a hash of small voxel
	blocks
	*/
	template<class TVoxel, class TIndex>
	class ITMScene
	{
//        std::unique_ptr<SCFusion::UnitMapSave<TVoxel,TIndex>> unitMapSaver;
        MemoryDeviceType memoryType;
	public:
        /** Scene parameters like voxel size etc. */
        const ITMLib::ITMSceneParams *sceneParams;

        /** Hash table to reference the 8x8x8 blocks */
        std::unique_ptr<TIndex> index;

        /** Current local content of the 8x8x8 voxel blocks -- stored host or device */
        std::unique_ptr<ITMLib::ITMLocalVBA<TVoxel>> localVBA;

        /** Global content of the 8x8x8 voxel blocks -- stored on host only */
        std::unique_ptr<ITMLib::ITMGlobalCache<TVoxel>> globalCache;

        void SaveToDirectory(const std::string &outputDirectory) const
        {
            localVBA->SaveToDirectory(outputDirectory);
            index->SaveToDirectory(outputDirectory);
        }

//        void SaveToUnitMap(const std::string &outputDirectory) {
//            if(unitMapSaver == nullptr ) unitMapSaver.reset(SCFusion::UnitMapSaverFactory::MakeUnitMapSaver<TVoxel,TIndex>(memoryType));
//            unitMapSaver->SaveToDirectory(localVBA.get(), index.get(), sceneParams->voxelSize, outputDirectory);
//        }
//
//        void SaveToUnitMap(const std::string &outputDirectory, const Vector3f &origin, const Vector3s &dims) {
//            if(unitMapSaver == nullptr ) unitMapSaver.reset(SCFusion::UnitMapSaverFactory::MakeUnitMapSaver<TVoxel,TIndex>(memoryType));
//            unitMapSaver->SaveToDirectory(origin, dims, localVBA.get(), index.get(), sceneParams->voxelSize, outputDirectory);
//        }

        void LoadFromDirectory(const std::string &outputDirectory)
        {
            localVBA->LoadFromDirectory(outputDirectory);
            index->LoadFromDirectory(outputDirectory);
        }

        ITMScene(const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType)
                : memoryType(_memoryType), sceneParams(_sceneParams)
        {
            index.reset(new TIndex(_memoryType));
            localVBA.reset(new ITMLib::ITMLocalVBA<TVoxel>(_memoryType, index->getNumAllocatedVoxelBlocks(), index->getVoxelBlockSize()));
            if (_useSwapping) globalCache.reset( new ITMLib::ITMGlobalCache<TVoxel>() );
            else globalCache = NULL;
        }

        ITMScene(const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, TIndex *_index)
                : memoryType(_memoryType), sceneParams(_sceneParams)
        {
            this->index.reset(_index);
            localVBA.reset(new ITMLib::ITMLocalVBA<TVoxel>(_memoryType, index->getNumAllocatedVoxelBlocks(), index->getVoxelBlockSize()));
            if (_useSwapping) globalCache.reset( new ITMLib::ITMGlobalCache<TVoxel>() );
            else globalCache = NULL;
        }

        ~ITMScene()=default;

        // Suppress the default copy constructor and assignment operator
        ITMScene(const ITMScene&);
        ITMScene& operator=(const ITMScene&);
	};
}

// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "VoxelIndexInterface.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"




namespace SCFUSION
{
    struct SCVoxelArrayInfo {
        /// Size in voxels
        Vector3s size;
        /// offset of the lower left front corner of the volume in voxels
        Vector3f offset;

        SCVoxelArrayInfo()
        {
            size.x = 80;
            size.y = 48;
            size.z = 80;
            offset.x = 0;
            offset.y = 0;
            offset.z = 0;
        }
    };

	/** \brief
	This is the central class for the original fixed size volume
	representation. It contains the data needed on the CPU and
	a pointer to the data structure on the GPU.
	*/
    class SCPlainVoxelArray : public ITMLib::VoxelIndex
	{
	public:


		typedef SCVoxelArrayInfo IndexData;
		struct IndexCache {};

	private:
		ORUtils::MemoryBlock<IndexData> *indexData;

	public:
        SCPlainVoxelArray(MemoryDeviceType memoryType) : VoxelIndex(memoryType)
		{

			if (memoryType == MEMORYDEVICE_CUDA) indexData = new ORUtils::MemoryBlock<IndexData>(1, true, true);
			else indexData = new ORUtils::MemoryBlock<IndexData>(1, true, false);

			indexData->GetData(MEMORYDEVICE_CPU)[0] = IndexData();
			indexData->UpdateDeviceFromHost();
		}

		~SCPlainVoxelArray()
		{
			delete indexData;
		}

		/** Maximum number of total entries. */
		int getNumAllocatedVoxelBlocks() { return 1; }
		int getVoxelBlockSize()
		{ 
			return indexData->GetData(MEMORYDEVICE_CPU)->size.x * 
				indexData->GetData(MEMORYDEVICE_CPU)->size.y * 
				indexData->GetData(MEMORYDEVICE_CPU)->size.z;
		}

		const Vector3s getVolumeSize() { return indexData->GetData(MEMORYDEVICE_CPU)->size; }

		const IndexData* getIndexData() const { return indexData->GetData(memoryType); }

		void SaveToDirectory(const std::string &outputDirectory) const
		{
		}

		void LoadFromDirectory(const std::string &outputDirectory)
		{
		}

#ifdef COMPILE_WITH_METAL
		const void *getIndexData_MB() const { return indexData->GetMetalBuffer(); }
#endif

		// Suppress the default copy constructor and assignment operator
        SCPlainVoxelArray(const SCPlainVoxelArray&);
        SCPlainVoxelArray& operator=(const SCPlainVoxelArray&);
	};
}

#endif

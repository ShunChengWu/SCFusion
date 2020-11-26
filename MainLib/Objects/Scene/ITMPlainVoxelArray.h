// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "VoxelIndexInterface.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"




namespace ITMLib
{
	/** \brief
	This is the central class for the original fixed size volume
	representation. It contains the data needed on the CPU and
	a pointer to the data structure on the GPU.
	*/
	class ITMPlainVoxelArray : public VoxelIndex
	{
	public:
		struct ITMVoxelArrayInfo {
			/// Size in voxels
			Vector3i size;
			/// offset of the lower left front corner of the volume in voxels
			Vector3i offset;

			ITMVoxelArrayInfo()
			{
				size.x = size.y = size.z = 512;
				offset.x = -256;
				offset.y = -256;
				offset.z = 0;
			}
		};

		typedef ITMVoxelArrayInfo IndexData;
		struct IndexCache {};

	private:
		ORUtils::MemoryBlock<IndexData> *indexData;

	public:
		explicit ITMPlainVoxelArray(MemoryDeviceType memoryType) : VoxelIndex(memoryType)
		{

            indexData = new ORUtils::MemoryBlock<IndexData>(1, true, memoryType == MEMORYDEVICE_CUDA);

			indexData->GetData(MEMORYDEVICE_CPU)[0] = IndexData();
			indexData->UpdateDeviceFromHost();
		}

        explicit ITMPlainVoxelArray(Vector3i offset, Vector3i size, MemoryDeviceType memoryType) : VoxelIndex(memoryType)
        {

            indexData = new ORUtils::MemoryBlock<IndexData>(1, true, memoryType == MEMORYDEVICE_CUDA);

            indexData->GetData(MEMORYDEVICE_CPU)[0].size = size;
            indexData->GetData(MEMORYDEVICE_CPU)[0].offset = offset;
            indexData->UpdateDeviceFromHost();
        }

		~ITMPlainVoxelArray()
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

		const Vector3i getVolumeSize() { return indexData->GetData(MEMORYDEVICE_CPU)->size; }

		const IndexData* getIndexData() const { return indexData->GetData(memoryType); }

		void setIndexOffset(Vector3i offset) {
		    indexData->GetData(MEMORYDEVICE_CPU)->offset = offset;
		    indexData->UpdateDeviceFromHost();
		}

		// do not allow resize
//		void setIndexSize(Vector3i size){
//		    indexData->GetData(MEMORYDEVICE_CPU)->size = size;
//		    indexData->UpdateDeviceFromHost();
//		}

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
		ITMPlainVoxelArray(const ITMPlainVoxelArray&);
		ITMPlainVoxelArray& operator=(const ITMPlainVoxelArray&);
	};
}

#endif

#pragma once

#include <stdlib.h>
#include "../../../ORUtils/MemoryBlock.h"
namespace ITMLib {
    class VoxelIndex {
    public:
        explicit VoxelIndex(MemoryDeviceType memoryType) : memoryType(memoryType) {}

        virtual int getNumAllocatedVoxelBlocks() = 0;

        virtual int getVoxelBlockSize() = 0;

        virtual void SaveToDirectory(const std::string &outputDirectory) const = 0;

        virtual void LoadFromDirectory(const std::string &outputDirectory) = 0;

    protected:
        MemoryDeviceType memoryType{};
    };
}
#pragma once

#include <ORUtils/MemoryBlock.h>
#include <iostream>

namespace ORUtils{
    class PlainVolume{
    public:
        float3 base;
        int3 dims;
        float voxel_size;
        bool hasCPU, hasCUDA;
        MemoryBlock<float> *data;
        MemoryBlock<int> *weight;

        /**
         Create a volume to store data

         @param bx The location of the first index of x
         @param by The location of the first index of y
         @param bz The location of the first index of z
         @param sx The number of voxels along x dimension
         @param sy The number of voxels along y dimension
         @param sz The number of voxels along z dimension
         @param voxel_size The size of each voxel
         @param cpu Whether allocate CPU memoery
         @param cuda Whether allocate CUDA memory
         */
        PlainVolume(float bx, float by, float bz,
                size_t sx, size_t sy, size_t sz,
                float voxel_size, bool cpu, bool cuda):
        voxel_size(voxel_size), hasCPU(cpu), hasCUDA(cuda){
            setParams(bx,by,bz,sx,sy,sz);
            data = new MemoryBlock<float> (sx*sy*sz, cpu, cuda);
            weight = new MemoryBlock<int> (sx*sy*sz, cpu, cuda);
        }
        ~PlainVolume(){
            if(data != NULL)
                delete data;
            if(weight != NULL)
                delete weight;
        }
        
        void setParams(float cx,float cy, float cz, int sx, int sy, int sz) {
            base = make_float3(cx,cy,cz);
            dims = make_int3(sx,sy,sz);
        }
        
        size_t volumeSize() const {return data->dataSize();}
        
        void init(float value = 0.f, bool async = false, void *stream = 0){
            if(hasCPU){
                data->Clear(value, async, stream);
                weight->Clear(0, async, stream);
            }
            if(hasCUDA){
                data->Clear(value, async, stream);
                weight->Clear(0, async, stream);
            }
        }
        
        void UpdateHostFromDevice(bool async=false, cudaStream_t stream=0) const {
            data->UpdateHostFromDevice(async,stream);
            weight->UpdateHostFromDevice(async,stream);
        }
        
        void UpdateDeviceFromHost(bool async=false, cudaStream_t stream=0) const {
            data->UpdateDeviceFromHost(async,stream);
            weight->UpdateDeviceFromHost(async,stream);
        }

        PlainVolume(PlainVolume& other){
            operator=(other);
        }
        PlainVolume& operator = (PlainVolume& other){
            //The destructor shall do the job. just in case
            std::swap(this->base, other.base);
            std::swap(this->dims, other.dims);
            std::swap(this->voxel_size, other.voxel_size);
            std::swap(this->hasCPU, other.hasCPU);
            std::swap(this->hasCUDA, other.hasCUDA);
            std::swap(this->data, other.data);
            return *this;
        }
        
        
        friend std::ostream& operator<<(std::ostream& os, const PlainVolume& info)
        {
            os << "voxel_size: " << info.voxel_size << "\n";
            os << "hasCPU: " << info.hasCPU << ", hasCUDA: " << info.hasCUDA << "\n";
            os << "base: " << info.base.x << ", " << info.base.y << ", " << info.base.z << "\n";
            os << "dims: " << info.dims.x << ", " << info.dims.y << ", " << info.dims.z << "\n";
            os << "Real world boundaries: \n";
            os << "x range: " << info.base.x << ", " <<  info.base.x+info.dims.x*info.voxel_size << "\n";
            os << "y range: " << info.base.y << ", " <<  info.base.y+info.dims.y*info.voxel_size << "\n";
            os << "z range: " << info.base.z << ", " <<  info.base.z+info.dims.z*info.voxel_size << "\n";
            return os;
        }
    };


}

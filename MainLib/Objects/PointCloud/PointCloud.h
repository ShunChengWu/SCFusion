#pragma once

#include <ORUtils/Image.h>
#include "../../Objects/Scene/ITMVoxelBlockHash.h"
#include <memory>
namespace SCFUSION
{
    class PointCloud
    {
    public:
        MemoryDeviceType memoryType;
        uint noTotalPoints;
        uint noMaxPoints;
        static const uint noMaxPoint_default = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
        std::unique_ptr<ORUtils::MemoryBlock<Vector4f>> points;
        std::unique_ptr<ORUtils::MemoryBlock<Vector4f>> colors;

        explicit PointCloud(MemoryDeviceType memoryType_, uint maxPoints = noMaxPoint_default)
        {
            this->memoryType = memoryType_;
            this->noTotalPoints = 0;
            this->noMaxPoints = maxPoints;
            points.reset(new ORUtils::MemoryBlock<Vector4f> (this->noMaxPoints, this->memoryType));
            colors.reset(new ORUtils::MemoryBlock<Vector4f> (this->noMaxPoints, this->memoryType));
        }

        void WriteOBJ(const char *fileName) //TODO: TESTME
        {
            ORUtils::MemoryBlock<Vector4f> *cpu_points; bool shoulDelete = false;
            if (memoryType == MEMORYDEVICE_CUDA)
            {
                cpu_points = new ORUtils::MemoryBlock<Vector4f>(noMaxPoints, MEMORYDEVICE_CPU);
                cpu_points->SetFrom(points.get(), ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CPU);
                shoulDelete = true;
            }
            else cpu_points = points.get();

            Vector4f *pointArray = cpu_points->GetData(MEMORYDEVICE_CPU);

            FILE *f = fopen(fileName, "w+");
            if (f != nullptr)
            {
                for (uint i = 0; i < noTotalPoints; i++)
                {
                    fprintf(f, "v %f %f %f\n", pointArray[i].x, pointArray[i].y, pointArray[i].z);
                }
                fclose(f);
            }

            if (shoulDelete) delete cpu_points;
        }

        void WritePLY(const char *fileName)
        {
            ORUtils::MemoryBlock<Vector4f> *cpu_points;
            bool shoulDelete = false;
            if (memoryType == MEMORYDEVICE_CUDA)
            {
                cpu_points = new ORUtils::MemoryBlock<Vector4f>(noMaxPoints, MEMORYDEVICE_CPU);
                cpu_points->SetFrom(points.get(), ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CPU);
                shoulDelete = true;
            }
            else {
                cpu_points = points.get();
            }

            Vector4f *pointArray = cpu_points->GetData(MEMORYDEVICE_CPU);
            
            {
                std::ofstream fout;
                fout.open(fileName);
                if (fout.fail()){
                    std::string errmsg = "Cannot open file at ";
                    throw errmsg;
                }
                fout << "ply" << std::endl;
                fout << "format ascii 1.0" << std::endl;
                fout << "element vertex " << noTotalPoints*3 << std::endl;
                fout << "property float x" << std::endl;
                fout << "property float y" << std::endl;
                fout << "property float z" << std::endl;
                //TODO: add color output
                fout << "property list uchar int vertex_indices" << std::endl;
                fout << "end_header" << std::endl;

                for (size_t i=0;i<noTotalPoints; ++i){
                    const Vector4f &triangle = pointArray[i];
                    fout << triangle.x << " " << triangle.y << " " << triangle.z << "\n";
                }

                fout.close();
            }
            if (shoulDelete) delete cpu_points;
        }
        // Suppress the default copy constructor and assignment operator
        PointCloud(const PointCloud&);
        PointCloud& operator=(const PointCloud&);
    };
}
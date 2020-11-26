#pragma once
#include <Eigen/Dense>
#include "../../ConnectedComponent/ConnectedComponent/CPU/ConnectedComponent_CPU.h"
#include "../../MeshVoxelizer/MeshVoxelizer.h"
void loadCloud2Array(std::vector<float> *vertices, std::vector<unsigned int> *pointLabels,
        std::vector<unsigned int> *pointInstances, std::vector<int> *objIndices,
        ORUtils::MemoryBlock<ORUtils::Vector3<float>> *points, ORUtils::MemoryBlock<unsigned short> *labels, ORUtils::MemoryBlock<unsigned int> *instances,
        float voxelSize, bool OccupancyOnly = false, bool Fill = true, const std::string &pth_cam_extrinsic = ""){
    assert(vertices->size() / 9 == pointLabels->size());
    assert(vertices->size() / 9 == pointInstances->size());

    TICK("Rasterization");
    std::vector<float3> points_;
    std::vector<unsigned int> labels_, instances_;
    DEBUG("Voxelize Mesh\n");
    DEBUG("\tThere are %zu objects.\n", objIndices->size());
//    for(size_t i=0;i<objIndices->size();++i)
    for(int i=objIndices->size()-1;i>=0;--i)
    {
        DEBUG("\n\t [Object %d]\n", i);

        int vertice_start, vertice_end;
        if(i==0){
            vertice_start = 0;
            vertice_end = objIndices->at(i);
        } else {
            vertice_start = objIndices->at(i-1);
            vertice_end = objIndices->at(i);
        }
        assert(vertice_start%3==0);
        DEBUG("\tProcess start index (%d) to end index(%d).\n", vertice_start, vertice_end);

        Eigen::Vector3f boundary_min(0,0,0), boundary_max(0,0,0);
        for(int j=vertice_start; j < vertice_end; ++j) {
            auto check=[](const Eigen::Map<const Eigen::Vector3f> &value, Eigen::Vector3f &low, Eigen::Vector3f &height){
                for(size_t i=0;i<3;++i)
                    if(std::isinf(value.data()[i])) assert(false) ;
                for(size_t i=0;i<3;++i)
                    if(value.data()[i] < low.data()[i]) low.data()[i] = value.data()[i];
                for(size_t i=0;i<3;++i)
                    if(value.data()[i] > height.data()[i]) height.data()[i] = value.data()[i];
            };
            auto eigenMap = Eigen::Map<const Eigen::Vector3f> (&vertices->at(j * 3));
            check(eigenMap, boundary_min, boundary_max);
        }
        float3 bbox_min = make_float3(boundary_min.x(),boundary_min.y(),boundary_min.z()),bbox_max = make_float3(boundary_max.x(),boundary_max.y(),boundary_max.z());
        DEBUG("\tObject boundary found from min (%f,%f,%f) to max (%f,%f,%f).\n",
              boundary_min.x(),boundary_min.y(),boundary_min.z(), boundary_max.x(),boundary_max.y(),boundary_max.z());
        {
            bool allZero=true;
            for(size_t i=0;i<3;++i) if(boundary_min.data()[i] != 0) allZero = false;
            for(size_t i=0;i<3;++i) if(boundary_max.data()[i] != 0) allZero = false;
            if(allZero) {
                DEBUG("\tBounding box was invalid (all zeros)! Skip.\n");
                continue;
            }
            auto totalSize = (int)std::floor((bbox_max.x-bbox_min.x)*(bbox_max.y-bbox_min.y)*(bbox_max.z-bbox_min.z)/voxelSize);
            DEBUG("totalSize: %d (%f MB)\n", totalSize, float(totalSize)/1024.f/1024.f);
            if(totalSize>1e6){
                DEBUG("Object size too big. Skip!\n");
                continue;
            }
        }



        MeshVoxelizer meshVoxelizer;
        meshVoxelizer.compute(vertices->data()+vertice_start*3, pointLabels->data()+vertice_start/3,
                pointInstances->data()+vertice_start/3, (vertice_end-vertice_start)*3,bbox_min,bbox_max,voxelSize);
        std::vector<float3> points_tmp;
        std::vector<unsigned int> labels_tmp, instances_tmp;
        meshVoxelizer.getOutput(points_tmp, labels_tmp, instances_tmp);

        /// Fill objects by using connected component
        if(Fill){
            std::map<uint,uint> labelCounter, instanceCounter;
            for(size_t l=0; l < labels_tmp.size(); ++l){
                if(labelCounter.find(labels_tmp[l]) == labelCounter.end())
                    labelCounter[labels_tmp[l]] = 1;
                else
                    labelCounter[labels_tmp[l]]++;
                if (instanceCounter.find(instances_tmp[l]) == instanceCounter.end())
                    instanceCounter[instances_tmp[l]] = 1;
                else
                    instanceCounter[instances_tmp[l]]++;
            }
#ifndef NDEBUG
            for(auto l :labelCounter) printf("Before CC:[%d][%d]\n",l.first,l.second);
//        DEBUG("\tbefore CC there are %zu labels\n", labelCounter.size());
#endif

            if(labelCounter.size() == 1 && instanceCounter.size() == 1) {
                DEBUG("\tPerforming CC to fill out voxels\n");
                unsigned int *volumeTable = meshVoxelizer.getVolumeTable();
                unsigned int *labelTable = meshVoxelizer.getLabelTable();
                unsigned int *instanceTable = meshVoxelizer.getInstanceTable();
                VoxelInfo *voxelInfo = meshVoxelizer.getVoxelInfo();
                uint size = voxelInfo->gridSize.x * voxelInfo->gridSize.y *
                            voxelInfo->gridSize.z;
                auto *volume_out = new unsigned int[size];
                uint min_size = std::min(voxelInfo->gridSize.x,std::min(voxelInfo->gridSize.y,voxelInfo->gridSize.z));
                uint3 blockDims = {min_size, min_size, min_size};
                SCFUSION::ConnectedComponent_CPU cc;
                cc.process(volumeTable, volume_out, &voxelInfo->gridSize.x, &blockDims.x, 3);

                // calculate size of each segment
                std::map<uint,uint> ccMap;
                for(uint l=0;l<size;++l) {
                    if (ccMap.find(volume_out[l]) == ccMap.end())
                        ccMap[volume_out[l]] = 0;
                    ccMap[volume_out[l]]++;
                }

                if(ccMap.size() > 2) {
                    // locate the target segment
                    size_t idx_target_segment=0, idx_max_segment = 0,max_segment_count = 0;
                    for(const auto &segment:ccMap) {
                        if (segment.second == labelCounter.begin()->second)
                            idx_target_segment = segment.first;
                        if (segment.second > max_segment_count) {
                            idx_max_segment = segment.first;
                            max_segment_count = segment.second;
                        }
                    }
                    DEBUG("target_segment: %zu\n", idx_target_segment);
                    DEBUG("max_segment: %zu\n", idx_max_segment);

                    for(uint l=0;l<size;++l) {
                        if(volume_out[l] != idx_target_segment && volume_out[l] != idx_max_segment) {
                            volumeTable[l]=1;
                            labelTable[l] = labelCounter.begin()->first;
                            instanceTable[l]=instanceCounter.begin()->first;
                        }
                    }

#ifndef NDEBUG
                    printf("[before] cc: \n");
                    for(auto l : ccMap)
                        printf("\t[%d]:[%d]\n",l.first,l.second);

                    ccMap.clear();
                    cc.process(volumeTable, volume_out, &voxelInfo->gridSize.x, &blockDims.x, 3);
                    for(uint l=0;l<size;++l) {
                        if(ccMap.find(volume_out[l]) == ccMap.end())
                            ccMap[volume_out[l]] = 1;
                        else
                            ccMap[volume_out[l]]++;
                    }
                    printf("[after] cc: \n");
                    for(auto l : ccMap)
                        printf("\t[%d]:[%d]\n",l.first,l.second);
#endif
                }

                delete []volume_out;
            } else
                DEBUG("\tMore than one label were found. Skip CC\n");

            meshVoxelizer.getOutput(points_tmp, labels_tmp, instances_tmp);

#ifndef NDEBUG
            labelCounter.clear();instanceCounter.clear();
            for(size_t l=0; l < labels_tmp.size(); ++l){
                if(labelCounter.find(labels_tmp[l]) == labelCounter.end())
                    labelCounter[labels_tmp[l]] = 1;
                else
                    labelCounter[labels_tmp[l]]++;
                if (instanceCounter.find(instances_tmp[l]) == instanceCounter.end())
                    instanceCounter[instances_tmp[l]] = 1;
                else
                    instanceCounter[instances_tmp[l]]++;
            }
            for(auto l :labelCounter) printf("After CC:[%d][%d]\n",l.first,l.second);
//        DEBUG("\tAfter CC there are %zu labels\n", labelCounter.size());
#endif
        }

        points_.insert(points_.end(), make_move_iterator(points_tmp.begin()), make_move_iterator(points_tmp.end()));
        labels_.insert(labels_.end(), make_move_iterator(labels_tmp.begin()), make_move_iterator(labels_tmp.end()));
        instances_.insert(instances_.end(), make_move_iterator(instances_tmp.begin()), make_move_iterator(instances_tmp.end()));
    }
    TOCK("Rasterization");

    TICK("Copy Container to Vector3f");
    points->Resize(points_.size());
    labels->Resize(points_.size());
    instances->Resize(points_.size());

    auto points_cpu = points->GetData(MEMORYDEVICE_CPU);
    auto labels_cpu = labels->GetData(MEMORYDEVICE_CPU);
    auto instances_cpu = instances->GetData(MEMORYDEVICE_CPU);
    for(size_t i=0; i < points_.size(); ++i){
        points_cpu[i].x = points_[i].x;
        points_cpu[i].y = points_[i].y;
        points_cpu[i].z = points_[i].z;

        if(OccupancyOnly) {
            labels_cpu[i] = 1;
            instances_cpu[i]=0;
        } else {
            labels_cpu[i] = labels_[i];
            instances_cpu[i] = instances_[i];
        }

    }
    points->UpdateDeviceFromHost();
    labels->UpdateDeviceFromHost();
    instances->UpdateDeviceFromHost();
    TOCK("Copy Container to Vector3f");
}

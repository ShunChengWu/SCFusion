#include "ScanNetScan2CADLoader.h"
//
// Created by sc on 5/14/20.
//
#include <utility>
#include "../../Files/Label_NYU40.h"
#include "../../Files/Label_SunCG11.h"
#include "../../ConnectedComponent/ConnectedComponent/CPU/ConnectedComponent_CPU.h"
#include <pcl/kdtree/kdtree_flann.h>

ScanNetScan2CadMesh_Loader::ScanNetScan2CadMesh_Loader(std::string folder, std::string pth_shapenet, std::string pth_alignments,
                   bool occupancy_only,
                   bool fill,
                   float voxelSize,
                   bool exclude):
        counter_(0),
        bOccupancyOnly_(occupancy_only), bFill(fill),
        voxelSize_(voxelSize),
        mScanNetMeshLoader(folder),
        mScan2CadLoader(folder,pth_shapenet, pth_alignments){

}

std::shared_ptr<DataBuffer> ScanNetScan2CadMesh_Loader::get_item(int idx) {
    auto scannetholder = mScanNetMeshLoader.GetMesh(idx);
    auto name  = scannetholder->name;
    auto scan2cadholder = mScan2CadLoader.GetMeshes(name);
    if(scan2cadholder.empty())return nullptr;

    std::shared_ptr<DataBuffer> dataBuffer(new DataBuffer());

    // Build Scan
    {
        auto cloud = scannetholder->cloud;
        auto mesh  = scannetholder->mesh;

        // Calculate BBox
        float3 bbox_min, bbox_max;
        if(!BuildBBoxFromCloud(cloud, bbox_min, bbox_max))
            return nullptr;

        // Build point vector from polygon mesh
        std::vector<float> polygon_vertices;
        std::vector<unsigned int> polygon_labels;
        BuildPointVectorFromPolygonMesh(cloud,mesh,polygon_vertices,polygon_labels);

        // Voxelize
        SCLOG(VERBOSE) << "Voxelize point cloud";
        std::vector<float3> points_tmp;
        std::vector<unsigned int> labels_tmp;
        MeshVoxelizer meshVoxelizer;
        meshVoxelizer.compute(polygon_vertices.data(),polygon_labels.data(),polygon_vertices.size(),
                bbox_min, bbox_max, voxelSize_);
        meshVoxelizer.getOutput(points_tmp, labels_tmp);


        SCLOG(VERBOSE) << "Copy output to buffer";
        ORUtils::MemoryBlock<ORUtils::Vector3<float>> * points = dataBuffer->points.get();
        ORUtils::MemoryBlock<unsigned short> *labels = dataBuffer->labels.get();
        dataBuffer->subFolder = name;
        points->Resize(points_tmp.size());
        labels->Resize(points_tmp.size());
        auto points_cpu = points->GetData(MEMORYDEVICE_CPU);
        auto labels_cpu = labels->GetData(MEMORYDEVICE_CPU);
        for(size_t i=0; i < points_tmp.size(); ++i){
            points_cpu[i].x = points_tmp[i].x;
            points_cpu[i].y = points_tmp[i].y;
            points_cpu[i].z = points_tmp[i].z;

            if(bOccupancyOnly_) {
                labels_cpu[i] = 1;
            } else {
                labels_cpu[i] = labels_tmp[i];
            }

        }
        points->UpdateDeviceFromHost();
        labels->UpdateDeviceFromHost();
    }

    // Use KNN to kind nearest label to replace the object label
    // This doesn't work. When existing totally missing or mis-aligned objects, the label is wrong.
#ifdef USE_KNN_SEARCH_LABEL
    SCLOG(VERBOSE) << "Assign the label of object clouds by using KNN approach.";
    float radius = 0.02;
    auto target_cloud = scannetholder->cloud;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target_cloud);
#endif

    // Build Objects
    for(size_t i=0;i<scan2cadholder.size();++i)
    {
        auto objectbuffer = scan2cadholder[i];
        auto cloud = objectbuffer->cloud;
        auto mesh  = objectbuffer->mesh;
        auto label = objectbuffer->label;
        auto instance = objectbuffer->id;

        // Use KNN to kind nearest label to replace the object label
        // This doesn't work. When existing totally missing or mis-aligned objects, the label is wrong.
#ifdef USE_KNN_SEARCH_LABEL
        {
            SCLOG(VERBOSE) << "Processing object number: " << i;
            std::vector<int> neighbor_indice (1);
            std::vector<float> neighbor_squared_distance (1);
            // Search the maximum label appears of nearby the object cloud
            std::map<unsigned short,size_t> labelCounter;
            std::set<int> unique_indices;
            for(size_t p=0;p<cloud->size();++p){
                int neighbor_found = kdtree.radiusSearch(cloud->at(p),radius,neighbor_indice,neighbor_squared_distance);
                if(neighbor_found>1){
                    for(auto index : neighbor_indice){
                        if(unique_indices.find(index) == unique_indices.end()){
                            unique_indices.insert(index);
                            auto point = target_cloud->at(index);
                            auto LabelNYU40 = NYU40ColorLabels.at({static_cast<float>(point.r),static_cast<float>(point.g),static_cast<float>(point.b),255});
                            auto Label11    = NYU40ToSunCG11SC.at(LabelNYU40);
                            if(Label11 == 0) continue;
                            labelCounter[Label11]++;
                        }
                    }
                }
            }
            if(!labelCounter.empty()) {
                SCLOG(VERBOSE) << "All labels found:";
                for (auto pair:labelCounter) {
                    SCLOG(VERBOSE) << "[" << pair.first << "]: " << pair.second;
                }
                // Get the maximum label
                unsigned short max_label = labelCounter.rbegin()->first;
                if(label != max_label) {
                    SCLOG(VERBOSE) << "Replace object label from " << label << " to " << max_label;
                    label = max_label;
                }
            } else {
                SCLOG(VERBOSE) << "No corresponding label found.";
            }
        }
#endif

        // Calculate BBox
        float3 bbox_min, bbox_max;
        if(!BuildBBoxFromCloud(cloud, bbox_min, bbox_max))
            return nullptr;

        std::vector<float> polygon_vertices;
        std::vector<unsigned int> polygon_labels;
        BuildPointVectorFromPolygonMesh(cloud,mesh,polygon_vertices,polygon_labels, label);

        // Voxelize
        SCLOG(VERBOSE) << "Voxelize point cloud";
        std::vector<float3> points_tmp;
        std::vector<unsigned int> labels_tmp;
        MeshVoxelizer meshVoxelizer;
        meshVoxelizer.compute(polygon_vertices.data(),polygon_labels.data(),
                mesh->polygons.size()*9,bbox_min, bbox_max, voxelSize_);
        meshVoxelizer.getOutput(points_tmp, labels_tmp);

        // Fill
        if(bFill){
            FillObjWithCC(meshVoxelizer,points_tmp,labels_tmp);
        }

        SCLOG(VERBOSE) << "Copy output to buffer";
        std::unique_ptr<ORUtils::MemoryBlock<ORUtils::Vector3<float>>>
                points (new ORUtils::MemoryBlock<ORUtils::Vector3<float>>(points_tmp.size(),true,true));
        std::unique_ptr<ORUtils::MemoryBlock<unsigned short>>
                labels(new ORUtils::MemoryBlock<unsigned short>(points_tmp.size(),true,true));
        std::unique_ptr<ORUtils::MemoryBlock<unsigned int>>
                instances(new ORUtils::MemoryBlock<unsigned int>(points_tmp.size(),true,true));
        auto points_cpu = points->GetData(MEMORYDEVICE_CPU);
        auto labels_cpu = labels->GetData(MEMORYDEVICE_CPU);
        auto instances_cpu = instances->GetData(MEMORYDEVICE_CPU);

        for(size_t p=0; p < points_tmp.size(); ++p){
            points_cpu[p].x = points_tmp[p].x;
            points_cpu[p].y = points_tmp[p].y;
            points_cpu[p].z = points_tmp[p].z;

            if(bOccupancyOnly_) {
                labels_cpu[p] = 1;
            } else {
                if(labels_tmp[p] > 12){
                    SCLOG(ERROR) << "label larger than 12: " << labels_tmp[p];
                }
                labels_cpu[p] = labels_tmp[p];
            }
            instances_cpu[p] = instance;
        }

        points->UpdateDeviceFromHost();
        labels->UpdateDeviceFromHost();
        instances->UpdateDeviceFromHost();
        dataBuffer->objPoints.push_back(std::move(points));
        dataBuffer->objLabels.push_back(std::move(labels));
        dataBuffer->objInstances.push_back(std::move(instances));
    }

    // Replace duplicate points in scan.
    {
        float radius = 0.10;
        pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
        std::vector<int> neighbor_indice (1);
        std::vector<float> neighbor_squared_distance (1);

        // build new cloud from voxelized points
        SCLOG(VERBOSE) << "Building scene cloud from voxelized scene";
        pcl::PointCloud<pcl::PointXYZL>::Ptr scene_cloud (new pcl::PointCloud<pcl::PointXYZL>());
        scene_cloud->resize(dataBuffer->points->dataSize);
        const auto scene_points_data = dataBuffer->points->GetDataConst(MEMORYDEVICE_CPU);
        const auto label_buffer_data = dataBuffer->labels->GetDataConst(MEMORYDEVICE_CPU);
        for(size_t i=0; i < scene_cloud->size(); ++i){
            scene_cloud->points[i].x = scene_points_data[i].x;
            scene_cloud->points[i].y = scene_points_data[i].y;
            scene_cloud->points[i].z = scene_points_data[i].z;
            scene_cloud->points[i].label = label_buffer_data[i];
        }
        kdtree.setInputCloud(scene_cloud);



        SCLOG(VERBOSE) << "KNN search for nearby points between object points and scene points.";
        std::set<int> indices_to_remove;
        const std::set<unsigned short> label_to_ignore {1/*ceiling*/,2/*floor*/,3/*wall*/};
        for(auto & objPoints : dataBuffer->objPoints)
        {
            const auto object_points_data = objPoints->GetDataConst(MEMORYDEVICE_CPU);

            for(size_t p=0;p<objPoints->dataSize;++p){
                const auto &point = object_points_data[p];
                neighbor_indice.clear();
                neighbor_squared_distance.clear();
                pcl::PointXYZL point_;
                point_.x = point.x;
                point_.y = point.y;
                point_.z = point.z;
                int neighbor_found = kdtree.radiusSearch(point_,radius,
                        neighbor_indice,neighbor_squared_distance);
                if(neighbor_found>=1){
                    for(auto index : neighbor_indice){
                        auto point = scene_cloud->at(index);
                        if(label_to_ignore.find(point.label) != label_to_ignore.end())continue;
                        indices_to_remove.insert(index);
                    }
                }
            }
        }

        SCLOG(VERBOSE) << "Found indices to remove: " << indices_to_remove.size();
        SCLOG(VERBOSE) << "Removing points...";
        std::vector<float3> points_tmp;
        std::vector<unsigned int> labels_tmp;
        {
            points_tmp.reserve(dataBuffer->points->dataSize - indices_to_remove.size());
            labels_tmp.reserve(dataBuffer->points->dataSize - indices_to_remove.size());
            for(size_t i=0;i<dataBuffer->points->dataSize;++i){
                if(indices_to_remove.find(i) == indices_to_remove.end()){
                    points_tmp.push_back(make_float3(scene_points_data[i].x,scene_points_data[i].y,scene_points_data[i].z));
                    labels_tmp.push_back(label_buffer_data[i]);
                }
            }
        }

        SCLOG(VERBOSE) << "Copy output to buffer";
        ORUtils::MemoryBlock<ORUtils::Vector3<float>> * points = dataBuffer->points.get();
        ORUtils::MemoryBlock<unsigned short> *labels = dataBuffer->labels.get();
        dataBuffer->subFolder = name;
        points->Resize(points_tmp.size());
        labels->Resize(points_tmp.size());
        auto points_cpu = points->GetData(MEMORYDEVICE_CPU);
        auto labels_cpu = labels->GetData(MEMORYDEVICE_CPU);
        for(size_t i=0; i < points_tmp.size(); ++i){
            points_cpu[i].x = points_tmp[i].x;
            points_cpu[i].y = points_tmp[i].y;
            points_cpu[i].z = points_tmp[i].z;

            if(bOccupancyOnly_) {
                labels_cpu[i] = 1;
            } else {
                labels_cpu[i] = labels_tmp[i];
            }

        }
        points->UpdateDeviceFromHost();
        labels->UpdateDeviceFromHost();
    }


    return dataBuffer;
}

size_t ScanNetScan2CadMesh_Loader::dataSize()  {
    return mScanNetMeshLoader.size();
}

int ScanNetScan2CadMesh_Loader::next()  {
    return counter_<mScanNetMeshLoader.size()? counter_++ : -1;
}

bool ScanNetScan2CadMesh_Loader::BuildBBoxFromCloud(pcl::PointCloud<PointT>::Ptr cloud,
                        float3 &bbox_min, float3 &bbox_max){
    Eigen::Vector3f boundary_min(0,0,0), boundary_max(0,0,0);
    // Check data and calculate bounding box
    for(size_t i=0;i<cloud->size();++i){
        auto check=[](Eigen::Map<Eigen::Vector3f> value, Eigen::Vector3f &low, Eigen::Vector3f &height){
            for(size_t i=0;i<3;++i)
                if(std::isinf(value.data()[i])) assert(false) ;
            for(size_t i=0;i<3;++i)
                if(value.data()[i] < low.data()[i]) low.data()[i] = value.data()[i];
            for(size_t i=0;i<3;++i)
                if(value.data()[i] > height.data()[i]) height.data()[i] = value.data()[i];
        };
        check(cloud->points[i].getVector3fMap(), boundary_min, boundary_max);
    }

    // BBox validation
    bbox_min = make_float3(boundary_min.x(),boundary_min.y(),boundary_min.z());
    bbox_max = make_float3(boundary_max.x(),boundary_max.y(),boundary_max.z());
    SCLOG(DEBUG) << "Object boundary found from min " << boundary_min.transpose() << " to max " << boundary_max.transpose();
    {
        bool allZero=true;
        for(size_t i=0;i<3;++i) if(boundary_min.data()[i] != 0) allZero = false;
        for(size_t i=0;i<3;++i) if(boundary_max.data()[i] != 0) allZero = false;
        if(allZero) {
            SCLOG(DEBUG) << "Bounding box was invalid (all zeros)! Skip.";
            return false;
        }
        auto totalSize = (int)std::floor((bbox_max.x-bbox_min.x)*(bbox_max.y-bbox_min.y)*(bbox_max.z-bbox_min.z)/voxelSize_);
        SCLOG(DEBUG) << "totalSize: " << totalSize << " (" << float(totalSize)/1024.f/1024.f << " MB)";
        if(totalSize>1e6){
            SCLOG(DEBUG) << "Object size too big. Skip!";
            return false;
        }
    }
    return true;
}

/**
 *
 * @param cloud
 * @param mesh
 * @param polygon_vertices
 * @param polygon_labels
 * @param label If empty, retreieve label from color. Otherwise fill poitns with the label.
 */
void ScanNetScan2CadMesh_Loader::BuildPointVectorFromPolygonMesh(
        pcl::PointCloud<PointT>::Ptr cloud,
        pcl::PolygonMeshPtr mesh,
        std::vector<float> &polygon_vertices,
        std::vector<unsigned int> &polygon_labels, short label) {
    SCLOG(VERBOSE) << "Build point vector from polygon mesh";
    SCLOG(VERBOSE) << "Vertices: " << mesh->polygons.size();

    size_t validFaceCounter = 0;
    for(auto polygon : mesh->polygons){
        if(polygon.vertices.size() == 3) validFaceCounter++;
    }
    polygon_vertices.clear();
    polygon_labels.clear();

    polygon_vertices.reserve(validFaceCounter*3*3);
    polygon_labels.reserve(validFaceCounter);
    for(size_t i=0;i<mesh->polygons.size();++i){
        const auto &polygon = mesh->polygons[i];
        if(polygon.vertices.size() != 3) continue;

        // use first point in polygon to choice label
        const auto &point = cloud->points[polygon.vertices[0]];
        if(label>=0){
            polygon_labels.push_back(label);
        } else {
            ORUtils::Vector3<int> color = {point.r,point.g,point.b};
            if(NYU40ColorToLabels.find(color) == NYU40ColorToLabels.end()){
                SCLOG(WARNING) << "Cannot find corresponding label for color " << color;
                continue;
            } else {
                unsigned short LabelNYU40;
                try {
                    LabelNYU40 = NYU40ColorToLabels.at(color);
                } catch (...){
                    SCLOG(ERROR) << "Cannot find label for color("<<point.r<<","<<point.g<<","<<point.b<<")";
                }

                unsigned short Label11;
                try {
                    Label11 = NYU40ToSunCG11SC.at(LabelNYU40);
                } catch (...){
                    SCLOG(ERROR) << "Cannot map NYU label (" << LabelNYU40 << ") to Label11";;
                }

                // we load ply to polygonmesh and then converted to point cloud. during this process label information is lost
                // that's why we should recover label from color.
//                if(LabelNYU40 != point.label)
//                    SCLOG(ERROR) << "label mismatch. Expect color " << point.r << " " << point.g << " " << point.b <<
//                    " to be label " << LabelNYU40 << " but the ground truth label is " << point.label;

                if(Label11==0) continue; // skip unlabeled
                polygon_labels.push_back(Label11);
            }
        }

        auto push_back_point = [] (const PointT& point, std::vector<float> *vec){
            vec->push_back(point.x);
            vec->push_back(point.y);
            vec->push_back(point.z);
        };
        for(size_t j=0;j<3;++j)
            push_back_point(cloud->points[polygon.vertices[j]],&polygon_vertices);
    }
    assert(polygon_vertices.size() == polygon_labels.size()*9);
}

void ScanNetScan2CadMesh_Loader::FillObjWithCC(
        MeshVoxelizer &meshVoxelizer,
        std::vector<float3> &points_tmp,
        std::vector<unsigned int> &labels_tmp){

    std::map<uint,uint> labelCounter;
    for(size_t l=0; l < labels_tmp.size(); ++l){
        if(labelCounter.find(labels_tmp[l]) == labelCounter.end())
            labelCounter[labels_tmp[l]] = 1;
        else
            labelCounter[labels_tmp[l]]++;
    }
//#ifndef NDEBUG
//        for(auto l :labelCounter) printf("Before CC:[%d][%d]\n",l.first,l.second);
//#endif
    if(labelCounter.size() == 1) {
//            DEBUG("\tPerforming CC to fill out voxels\n");
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
//                DEBUG("target_segment: %zu\n", idx_target_segment);
//                DEBUG("max_segment: %zu\n", idx_max_segment);

            for(uint l=0;l<size;++l) {
                if(volume_out[l] != idx_target_segment && volume_out[l] != idx_max_segment) {
                    volumeTable[l]=1;
                    labelTable[l] = labelCounter.begin()->first;
                }
            }
        }
        delete []volume_out;
    } else {
//            DEBUG("\tMore than one label were found. Skip CC\n");
    }

    meshVoxelizer.getOutput(points_tmp, labels_tmp);

//#ifndef NDEBUG
//        labelCounter.clear();
//        for(size_t l=0; l < labels_tmp.size(); ++l){
//            if(labelCounter.find(labels_tmp[l]) == labelCounter.end())
//                labelCounter[labels_tmp[l]] = 1;
//            else
//                labelCounter[labels_tmp[l]]++;
//        }
//        for(auto l :labelCounter) printf("After CC:[%d][%d]\n",l.first,l.second);
////        DEBUG("\tAfter CC there are %zu labels\n", labelCounter.size());
//#endif
}
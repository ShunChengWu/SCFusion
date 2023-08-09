//
// Created by sc on 5/14/20.
//

#ifndef SCSLAM_PROEJCT_SCANNET_LOADER_H
#define SCSLAM_PROEJCT_SCANNET_LOADER_H

#include <set>
#include <vector>
#include <string>
#include <memory>
#include "ScanNetMeshLoader.h"
#include "Scan2CADObjectLoader.h"

#include <ORUtils/MemoryBlock.h>
#include <ORUtils/Vector.h>
#include "../../MeshVoxelizer/MeshVoxelizer.h"
#include <CxxTools/DataWorker.h>
#include "ScanNetScan2CAD_SceneToSkip.h"

//#include <utility>
//#include "../../Files/Label_NYU40.h"
//#include "../../Files/Label_SunCG11.h"
//#include "../../SceneNetRGBD2Map/SceneNet_wnid_to_13labels.h"


struct DataBuffer{
    std::unique_ptr<ORUtils::MemoryBlock<ORUtils::Vector3<float>>> points;
    std::unique_ptr<ORUtils::MemoryBlock<unsigned short>> labels;
    std::vector<std::unique_ptr<ORUtils::MemoryBlock<ORUtils::Vector3<float>>>> objPoints;
    std::vector<std::unique_ptr<ORUtils::MemoryBlock<unsigned short>>> objLabels;
    std::vector<std::unique_ptr<ORUtils::MemoryBlock<unsigned int>>> objInstances;

    std::string subFolder;
    DataBuffer(){
        points.reset (new ORUtils::MemoryBlock<ORUtils::Vector3<float>>(1,true,true));
        labels.reset(new ORUtils::MemoryBlock<unsigned short>(1,true,true));
    }
};




class ScanNetScan2CadMesh_Loader : public tools::DataLoader<DataBuffer> {
    typedef pcl::PointXYZRGB PointT;
public:
    ScanNetScan2CadMesh_Loader(std::string folder, std::string pth_shapenet, std::string pth_alignments,
                       bool occupancy_only,
                       bool fill,
                       float voxelSize,
                       bool exclude=true);

    std::shared_ptr<DataBuffer> get_item(int idx) override;

    size_t dataSize() override;

    int next() override;

private:
    int counter_;
    bool bOccupancyOnly_, bFill;
    float voxelSize_;
    ScanNetMeshLoader mScanNetMeshLoader;
    Scan2CADObjectLoader mScan2CadLoader;

    bool BuildBBoxFromCloud(pcl::PointCloud<PointT>::Ptr cloud,
                            float3 &bbox_min, float3 &bbox_max);
    /**
     *
     * @param cloud
     * @param mesh
     * @param polygon_vertices
     * @param polygon_labels
     * @param label If empty, retreieve label from color. Otherwise fill poitns with the label.
     */
    void BuildPointVectorFromPolygonMesh(
            pcl::PointCloud<PointT>::Ptr cloud,
            pcl::PolygonMeshPtr mesh,
            std::vector<float> &polygon_vertices,
            std::vector<unsigned int> &polygon_labels, short label = -1);

    void FillObjWithCC(
            MeshVoxelizer &meshVoxelizer,
            std::vector<float3> &points_tmp,
            std::vector<unsigned int> &labels_tmp);
};


#endif //SCSLAM_PROEJCT_SCANNET_LOADER_H
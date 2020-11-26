#pragma once
#include "../../ORUtils/Logging.h"
#include "Scan2CADAnnotionLoader.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Core>

class Scan2CADObjectLoader {
    typedef pcl::PointXYZRGB PointT;
public:
    struct Scan2CADMeshHolder{
        unsigned int id, label;
        pcl::PolygonMeshPtr mesh;
        pcl::PointCloud<PointT>::Ptr cloud;
    };
    Scan2CADObjectLoader(std::string pth_scannet,std::string pth_shapenet, std::string pth_alignments);

    std::vector<std::shared_ptr<Scan2CADMeshHolder>> GetMeshes(const std::string &scan_id);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    std::string msPathScanNet, msPathShapeNet, msPathAlignments;
    Scan2CADAnnotionLoader mScan2CadAnnotionLoader;
    Eigen::Matrix4d mExtrinsics;

    pcl::PolygonMeshPtr loadCloud(const std::string &pth_ply);
};
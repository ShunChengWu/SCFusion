#pragma once
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ScanNetMeshLoader {
    typedef pcl::PointXYZRGB PointT;
public:
    struct ScanNetMeshHolder{
        std::string name;
        pcl::PolygonMeshPtr mesh;
        pcl::PointCloud<PointT>::Ptr cloud;
    };
    ScanNetMeshLoader(const std::string& folder);
    std::shared_ptr<ScanNetMeshHolder> GetMesh(int idx);
    size_t size();
private:
    std::vector<std::string> msPaths;

    pcl::PolygonMeshPtr loadCloud(const std::string &pth_ply);
};

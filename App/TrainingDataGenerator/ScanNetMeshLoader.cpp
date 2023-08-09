#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include "ScanNetMeshLoader.h"
#include <ORUtils//PathTool.hpp>
#include <ORUtils/Logging.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PC;

Eigen::Matrix4f GetRotationMatrix(float angle_x, float angle_y, float angle_z) {
    // Get rotation matrix
    Eigen::Matrix4f rotmat = Eigen::Matrix4f::Identity();
    {

        Eigen::Matrix4f mat4f;
        mat4f.setIdentity();

        Eigen::Matrix3f mat3f;

        // X
        mat3f = Eigen::AngleAxisf(angle_x*EIGEN_PI, Eigen::Vector3f::UnitX());
        mat4f.topLeftCorner<3,3>() = mat3f;
        rotmat = mat4f * rotmat;

        // Y
        mat3f = Eigen::AngleAxisf(angle_y*EIGEN_PI, Eigen::Vector3f::UnitY());
        mat4f.topLeftCorner<3,3>() = mat3f;
        rotmat = mat4f * rotmat;

        // Z
        // Y
        mat3f = Eigen::AngleAxisf(angle_z*EIGEN_PI, Eigen::Vector3f::UnitZ());
        mat4f.topLeftCorner<3,3>() = mat3f;
        rotmat = mat4f * rotmat;
    }
    return rotmat;
}

ScanNetMeshLoader::ScanNetMeshLoader(const std::string& folder){
    msPaths = tools::PathTool::get_files_in_folder(folder, "", true,true);
    SCLOG(VERBOSE) << msPaths.size() << " sequences found in the given folder.";
}
std::shared_ptr<ScanNetMeshLoader::ScanNetMeshHolder> ScanNetMeshLoader::GetMesh(int idx){
    if(idx >= msPaths.size()) throw std::runtime_error("exceed datat size");
    auto sequence = msPaths.at(idx);

    auto scanNetMeshHolder = std::make_shared<ScanNetMeshHolder>();
    scanNetMeshHolder->name = tools::PathTool::getFileName(sequence);
    // load ply
    SCLOG(VERBOSE)<< "Load cloud";
//    const std::string pth_ply = sequence+"/"+scanNetMeshHolder->name+"_vh_clean_2.labels_flip.ply";
    const std::string pth_ply = sequence+"/"+scanNetMeshHolder->name+"_vh_clean_2.labels.ply";
    scanNetMeshHolder->mesh = loadCloud(pth_ply);
    scanNetMeshHolder->cloud.reset( new pcl::PointCloud<PointT> );
    pcl::fromPCLPointCloud2 (scanNetMeshHolder->mesh->cloud, *scanNetMeshHolder->cloud);
    SCLOG(VERBOSE) << "cloud loaded with size: " << scanNetMeshHolder->cloud->size();
    SCLOG(VERBOSE) << "Vertices: " << scanNetMeshHolder->mesh->polygons.size();
    return scanNetMeshHolder;
}
size_t ScanNetMeshLoader::size() {
    return msPaths.size();
}

pcl::PolygonMeshPtr ScanNetMeshLoader::loadCloud(const std::string &pth_ply){
    /* Load both math and point cloud*/
    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
    if(pth_ply.find("ply") != std::string::npos) {
        if (pcl::io::loadPLYFile(pth_ply, *mesh) < 0)
            throw std::runtime_error("Failed to load cloud in the given path: ");
    } else
        throw std::runtime_error("Failed to load cloud in the given path: ");

    PC::Ptr cloud(new PC);
    if(pth_ply.find("ply") != std::string::npos) {
        if (pcl::io::loadPLYFile(pth_ply, *cloud) < 0)
            throw std::runtime_error("Failed to load cloud in the given path: ");
    } else
        throw std::runtime_error("Failed to load cloud in the given path: ");

    /* rotate cloud*/
    const Eigen::Matrix4f &rotmat = GetRotationMatrix(-0.5,-0.5,0.0);
    transformPointCloud (*cloud, *cloud, rotmat);
    toPCLPointCloud2(*cloud, mesh->cloud);

    return mesh;
}
#include "ScanNetMeshLoader.h"
#include <pcl/io/ply_io.h>
#include <CxxTools/PathTool.hpp>
#include "../../ORUtils/Logging.h"

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
    const std::string pth_ply = sequence+"/"+scanNetMeshHolder->name+"_vh_clean_2.labels_flip.ply";
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
    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
    if(pth_ply.find("ply") != std::string::npos) {
        if (pcl::io::loadPLYFile(pth_ply, *mesh) < 0)
            throw std::runtime_error("Failed to load cloud in the given path: ");
    } else
        throw std::runtime_error("Failed to load cloud in the given path: ");
    return mesh;
}
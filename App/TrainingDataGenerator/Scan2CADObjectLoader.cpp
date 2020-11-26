#include "Scan2CADObjectLoader.h"
#include <pcl/common/transforms.h>

#include <pcl/io/ply_io.h>
#include "assimp_obj_loader.h"
//#include "../../SceneNetRGBD2Map/SceneNet_wnid_to_13labels.h"
#include "../../Files/Label_SunCG11.h"
#include "../../Files/WnidToNYU13.h"

Scan2CADObjectLoader::Scan2CADObjectLoader(std::string pth_scannet,std::string pth_shapenet, std::string pth_alignments):
        msPathScanNet(std::move(pth_scannet)), msPathShapeNet(std::move(pth_shapenet)),
        msPathAlignments(std::move(pth_alignments)),
        mScan2CadAnnotionLoader(msPathScanNet, msPathShapeNet, msPathAlignments) {
    // Get rotation matrix
    mExtrinsics = Eigen::Matrix4d::Identity();
    {
        double angle_x=-0.5;
        double angle_y=-0.5;
        double angle_z=0;

        Eigen::Matrix4d mat4d;
        mat4d.setIdentity();

        Eigen::Matrix3d mat3d;

        // X
        mat3d = Eigen::AngleAxisd(angle_x*EIGEN_PI, Eigen::Vector3d::UnitX());
        mat4d.topLeftCorner<3,3>() = mat3d;
        mExtrinsics = mat4d * mExtrinsics;

        // Y
        mat3d = Eigen::AngleAxisd(angle_y*EIGEN_PI, Eigen::Vector3d::UnitY());
        mat4d.topLeftCorner<3,3>() = mat3d;
        mExtrinsics = mat4d * mExtrinsics;

        // Z
        mat3d = Eigen::AngleAxisd(angle_z*EIGEN_PI, Eigen::Vector3d::UnitZ());
        mat4d.topLeftCorner<3,3>() = mat3d;
        mExtrinsics = mat4d * mExtrinsics;
    }
}

std::vector<std::shared_ptr<Scan2CADObjectLoader::Scan2CADMeshHolder>> Scan2CADObjectLoader::GetMeshes(const std::string &scan_id){
    bool state;
    auto jscan = mScan2CadAnnotionLoader.GetScan(scan_id, state);
    if(!state) return std::vector<std::shared_ptr<Scan2CADMeshHolder>>();

    const std::string id_scan = jscan["id_scan"].string_value();
    assert(!id_scan.empty());
    assert(id_scan == scan_id);
    const std::string scan_file = msPathScanNet + "/" + id_scan + "/" + id_scan + "_vh_clean_2.labels.ply";
    SCLOG(VERBOSE) << "id_scan: " << id_scan;
    SCLOG(VERBOSE) << "scan_file: " << scan_file;

    // trs
    Eigen::Matrix4d TMat = mScan2CadAnnotionLoader.GetTMatrix(jscan);
    SCLOG(VERBOSE) << "TMat\n"<<TMat;

    // load objects
    std::map<uint, pcl::PointCloud<PointT>::Ptr> cad_clouds;
    uint instance_num=1; // start from 1. leave 0 to the background
    SCLOG(VERBOSE) << "Number of aligned models: " << jscan["aligned_models"].array_items().size();
    std::vector<std::shared_ptr<Scan2CADMeshHolder>> output;
    for(const auto &j : jscan["aligned_models"].array_items()) {
        auto objTMat = mScan2CadAnnotionLoader.GetTMatrix(j);
        SCLOG(VERBOSE) << "ObjTMat\n" << objTMat;
        const auto &id_cad = j["id_cad"].string_value();
        SCLOG(VERBOSE) << "id_cad " << id_cad;
        const auto &catid_cad = j["catid_cad"].string_value();
        SCLOG(VERBOSE) << "catid_cad " << catid_cad;
        const auto cad_file = msPathShapeNet + "/" + catid_cad + "/" + id_cad  + "/models/model_normalized.obj";
        SCLOG(VERBOSE) << "cad_file " << cad_file;

        // find label
        // check detail label exist
        auto label40 = wnid_to_NYU40classid.at(catid_cad);
        if(cadid_to_NYU40classid.find(id_cad) != cadid_to_NYU40classid.end()){
            label40 = cadid_to_NYU40classid.at(id_cad);
        }
        auto label11 = NYU40ToSunCG11SC.at(label40);

        if(label11 == 0) continue;

        // create holder
        auto scan2CadMeshHolder = std::make_shared<Scan2CADMeshHolder>();
        scan2CadMeshHolder->id = instance_num;
        scan2CadMeshHolder->label = label11;

        // Load Obj file
        scan2CadMeshHolder->mesh = loadCloud(cad_file);
        scan2CadMeshHolder->cloud.reset(new pcl::PointCloud<PointT>());
        pcl::fromPCLPointCloud2(scan2CadMeshHolder->mesh->cloud, *scan2CadMeshHolder->cloud);

        const Eigen::Matrix4d tmp = mExtrinsics* TMat.inverse()*objTMat;

        // Transform
        pcl::transformPointCloud(*scan2CadMeshHolder->cloud,*scan2CadMeshHolder->cloud,tmp);

        pcl::toPCLPointCloud2 (*scan2CadMeshHolder->cloud, scan2CadMeshHolder->mesh->cloud);
        SCLOG(VERBOSE) << "obj cloud loaded with size: " << scan2CadMeshHolder->cloud->size();
        instance_num++;

        output.push_back(scan2CadMeshHolder);
    }
    return output;
}

pcl::PolygonMeshPtr Scan2CADObjectLoader::loadCloud(const std::string &pth_ply){
    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
    if(pth_ply.find("ply") != std::string::npos) {
        if (pcl::io::loadPLYFile(pth_ply, *mesh) < 0)
            throw std::runtime_error("Failed to load cloud in the given path: ");
    } else
    if (pth_ply.find("obj") != std::string::npos) {
        unsigned int foo=0;
        AssimpObjLoader assimpObjLoader(pth_ply,foo);

        //auto &vertices = assimpObjLoader.mvAllVertices;
        auto *vertices = assimpObjLoader.all_vertices.get();
        auto all_vertices_array_size = assimpObjLoader.all_vertices_array_size;


        // Convert to point cloud
        pcl::PointCloud<PointT>::Ptr obj_cloud (new pcl::PointCloud<PointT>());
        obj_cloud.reset(new pcl::PointCloud<PointT>());
        obj_cloud->reserve(all_vertices_array_size);
        for(size_t i=0;i<all_vertices_array_size;i+=3) {
            PointT point;
            point.x = vertices[i+0];
            point.y = vertices[i+1];
            point.z = vertices[i+2];
            point.r = 255;
            point.g = 255;
            point.b = 255;
            obj_cloud->push_back(point);
        }

        // convert to mesh
        mesh->polygons.resize(all_vertices_array_size/9);
        size_t indices_counter=0;
        for(size_t i=0;i<mesh->polygons.size();++i){
            mesh->polygons[i].vertices.push_back(indices_counter++);
            mesh->polygons[i].vertices.push_back(indices_counter++);
            mesh->polygons[i].vertices.push_back(indices_counter++);
        }
        pcl::toPCLPointCloud2 (*obj_cloud, mesh->cloud);
    }
    return mesh;
}
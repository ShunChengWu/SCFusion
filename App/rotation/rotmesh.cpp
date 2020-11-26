#include "pclloader.hpp"
#include <pcl/common/transforms.h>
#include <CxxTools/PathTool.hpp>
#include <CxxTools/parser.hpp>
#include <CxxTools/thread_pool.hpp>
using namespace pcl;

typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PC;


bool overwrite=false;
bool verbal=false;
float angle_x=0,angle_y=0,angle_z=0;
std::mutex mutex;
std::string replace_this = ".ply";
std::string replace_to = "_flip.ply";


void savePlyWithMesh(PolygonMesh::Ptr mesh, PC::Ptr cloud, std::string path){
    std::fstream file(path, std::ios::out);
    if(!file.is_open()){
        printf("cannot open file for saving (%s)",path.c_str());
        exit(-1);
    }
    
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "comment Shun-Cheng.Wu generated\n";
    file << "element vertex " << cloud->size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "property uchar alpha" << std::endl;
    file << "element face " << mesh->polygons.size() << std::endl;
    file << "property list uchar int vertex_indices\n";
    file << "end_header\n";
    
    for (size_t i=0;i<cloud->size();++i) {
        auto& point = cloud->at(i);
        file << point.x << " " << point.y << " " << point.z << " " << int(point.r) << " " << int(point.g) << " " << int(point.b) << " " << int(point.a) << "\n";
    }
    
    for (size_t i=0; i < mesh->polygons.size(); ++i) {
        auto& vertices = mesh->polygons[i].vertices;
        file << vertices.size() << " ";
        for (size_t j=0;j<vertices.size();++j) {
            file << vertices[j] << " ";
        }
        file <<"\n";
    }
    file.close();
}

void Process(const std::string &path_in, const std::string &path_out, std::fstream *log) {
    tools::PathTool ptool;
    if (!overwrite) {
        if (ptool.checkfileexist(path_out)) {
            if(verbal) printf("file %s exist. skip!\n",path_in.c_str());
            return;
        }
    }

    PCLloader loader;
    PC::Ptr cloud(new PC);
    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    if(loader.load(path_in, mesh) < 0 ) {
        if(verbal) std::cout << "Unable to open file " << path_in << "\n";
        if(log) {
            std::unique_lock<std::mutex> lock;
            *log << "Unable to open file " << path_in << "\n";
        }
        return;
    }

    if(loader.load<PointT>(path_in, cloud) <0){
        if(verbal) std::cout << "Unable to open file " << path_in << "\n";
        if(log) {
            std::unique_lock<std::mutex> lock;
            *log << "Unable to open file " << path_in << "\n";
        }
        return;
    }

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

    // Rotate
    transformPointCloud (*cloud, *cloud, rotmat);
    toPCLPointCloud2(*cloud, mesh->cloud);

    savePlyWithMesh(mesh, cloud, path_out);
//    loader.save(outPath, mesh);
    if(verbal)printf("File saved to %s\n", path_out.c_str());
}

int main(int argc, char** argv){
    std::string pth_in="", pth_out="", hint_name="_vh_clean_2.ply";
    int file_counter_max=-1;
    int threadNum = 1;
    tools::Parser parser(argc,argv);

    parser.addOption(pkgname(&pth_in), "path to point cloud file or a folder.", true);
    parser.addOption(pkgname(&pth_out),"If not given, output bin file will be saved to the corresponding input folder.", false);
    parser.addOption(pkgname(&replace_this),"Replace this from target file name to replace_to.", false);
    parser.addOption(pkgname(&replace_to),"Replace this from target file name to replace_to.", false);
    parser.addOption(pkgname(&hint_name), "The hint to find files in folder.", false);
    parser.addOption(pkgname(&angle_x), "angle to rotate in x axis.", true);
    parser.addOption(pkgname(&angle_y), "angle to rotate in y axis.", true);
    parser.addOption(pkgname(&angle_z), "angle to rotate in z axis.", true);
    parser.addOption(pkgname(&overwrite), "Enable overwrite mode.", false);
    parser.addOption(pkgname(&threadNum), "the number of threads to be used.", false);
    parser.addSwitch(pkgname(&verbal),"verbal", false);
    parser.addOption(pkgname(&file_counter_max), "The maximum file to be processed.", false);

    if(parser.showMsg(verbal)<0)
        exit(EXIT_FAILURE);

    tools::PathTool ptool;

    /// Create Folder and Log file
    if(!pth_out.empty())
        ptool.check_and_create_folder(ptool.CheckEnd(pth_out));
    std::string logfilepath;
    std::string logfileName = ptool.find_parent_folder(pth_in,1) + "/" +
            std::string(argv[0]).substr(std::string(argv[0]).find_last_of("/")+1,std::string(argv[0]).length()) + ".log";
    if(pth_out.empty())
        logfilepath = ptool.find_parent_folder(pth_in,1) + "Rotate_mesh.log";
    else
        logfilepath = ptool.CheckEnd(pth_out) + ptool.getFileName(ptool.get_executable_path()) + ".log";

//    std::fstream logfile(logfilepath,std::ios::out);
//    if(!logfile.is_open()) {
//        printf("Unable to create log file at %s\n", logfilepath.c_str());
//    } else {
//        printf("Log file created at %s\n", logfilepath.c_str());
//    }


    /// Find Files
    std::vector<std::string> target_files;
    std::vector<std::string> output_names;
    if(ptool.isFolder(pth_in)) {
        ptool.get_files_include_name_recursively(pth_in,hint_name, target_files);
    } else {
        target_files.push_back(pth_in);
    }
    std::sort(target_files.begin(),target_files.end());

    for(auto p:target_files) {
        std::string name = p;
        if (pth_out.empty()) {
            name.replace(name.find(replace_this), replace_this.size(), replace_to);
        } else {
            name = ptool.find_parent_folder(
                    name.replace(name.find(replace_this), replace_this.size(), replace_to), 0);
            name = name.substr(name.find_last_of('/') + 1, name.length());
//                printf("%s\n", name.c_str());
            name = ptool.CheckEnd(pth_out) + name;
//                printf("%s\n", paths_.out.c_str());
        }
        output_names.push_back(std::move(name));
    }


    if(verbal)
    for (size_t i=0;i<target_files.size();++i) {
        printf("[in]%s. [out]%s\n", target_files[i].c_str(), output_names[i].c_str());
    }


    int counter=0;
    if(threadNum>1) {
        tools::TaskThreadPool pool(threadNum);
        for (size_t i=0;i<target_files.size();++i) {
            pool.runTaskWithID(std::bind(Process, target_files[i], output_names[i], /*&logfile*/nullptr));
            if(file_counter_max) if (counter++ >= file_counter_max) break;
        }
        pool.waitWorkComplete();
    } else {
        for (size_t i=0;i<target_files.size();++i) {
            Process(target_files[i], output_names[i], /*&logfile*/nullptr);
            if(file_counter_max) if (counter++ >= file_counter_max) break;
        }
    }

//    logfile.close();

    return 0;
}

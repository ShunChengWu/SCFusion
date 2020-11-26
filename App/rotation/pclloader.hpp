//
//  pclloader.hpp
//  SHOT_test
//
//  Created by Shun-Cheng Wu on 05/11/2017.
//
//
#ifndef pclloader_hpp
#define pclloader_hpp

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
namespace pcl{
    /**
     This class is build for easily reading all supported files in pcl.
     */
    class PCLloader {
    public:
        PCLloader() {};
        int load (std::string path, pcl::PolygonMesh::Ptr mesh) {
            enum types {
                obj, ply
            } type;
            //obj, pcd, ply
            switch (path[path.size()-1]) {
                case 'j':
                    type = obj;
                    break;
                case 'y':
                    type = ply;
                    break;
                default:
                    //                throw "intput type doesn't support";
                    std::cout<< "input type doesn't support" <<std::endl;
                    return -1;
            }
            switch (type) {
                case obj:
                    if (pcl::io::loadOBJFile(path, *mesh) < 0){
                        std::cout<<"Error loading model cloud."<<std::endl;
                        return -1;
                    }
                    break;
                case ply:
                    if (pcl::io::loadPLYFile(path, *mesh) < 0){
                        std::cout <<"Error loading model cloud."<<std::endl;
                        return -1;
                    }
                    break;
            }
            return 1;
        }
        
        template <typename T>
        int load (std::string path, typename pcl::PointCloud<T>::Ptr cloud) {
            enum types {
                pcd, obj, ply, xyz
            } type;
            if (path == "") {
                std::cout << "PCLloader::load No output path was given." << std::endl;
                return -1;
            }
            //obj, pcd, ply
            switch (path[path.size()-1]) {
                case 'd':
                    type = pcd;
                    break;
                case 'j':
                    type = obj;
                    break;
                case 'y':
                    type = ply;
                    break;
                case 'z':
                    type = xyz;
                    break;
                default:
                    throw "intput type doesn't support";
            }
            switch (type) {
                case pcd:
                    if (pcl::io::loadPCDFile(path, *cloud) < 0){
                        std::cout<<"Error loading model cloud."<<std::endl;
                        return -1;
                    }
                    break;
                case obj:
                    if (pcl::io::loadOBJFile(path, *cloud) < 0){
                        std::cout<< "Error loading model cloud."<<std::endl;
                        return -1;
                    }
                    break;
                case ply:
                    if (pcl::io::loadPLYFile(path, *cloud) < 0){
                        std::cout<<"Error loading model cloud."<<std::endl;
                        return -1;
                    }
                    break;
                case xyz:
                {
                    /* 3 elements: xyz, 6 elements: xyzrgb */
                    std::ifstream fs;
//                    fs.open(<#const char *__s#>)
                    break;
                }
            }
            return 1;
        }
        
        void save (std::string path, pcl::PolygonMesh::Ptr mesh) {
            enum types {
                obj, ply
            } type;
            //obj, pcd, ply
            switch (path[path.size()-1]) {
                case 'j':
                    type = obj;
                    break;
                case 'y':
                    type = ply;
                    break;
                default:
                    //                throw "intput type doesn't support";
                    std::cout<< "output type doesn't support" <<std::endl;
                    exit(0);
            }
            switch (type) {
                case obj:
                    if (pcl::io::saveOBJFile(path, *mesh) < 0){
                        std::cout<<"Error loading model cloud."<<std::endl;
                        exit(0);
                    }
                    break;
                case ply:
                    if (pcl::io::saveOBJFile(path, *mesh) < 0){
                        std::cout<<"Error loading model cloud."<<std::endl;
                        exit(0);
                    }
                    break;
            }
        }
        
        template <typename T>
        void save (std::string path, typename pcl::PointCloud<T>::Ptr cloud) {
            enum types {
                pcd, ply, xyz
            } type;
            if (path == "") {
                std::cout << "PCLloader::load No output path was given." << std::endl;
                exit(-1);
            }
            //obj, pcd, ply
            switch (path[path.size()-1]) {
                case 'd':
                    type = pcd;
                    break;
                case 'y':
                    type = ply;
                    break;
                case 'z':
                    type = xyz;
                    break;
                default:
                    throw "output type doesn't support";
            }
            switch (type) {
                case pcd:
                    if (pcl::io::savePCDFile(path, *cloud) < 0){
                        std::cout<<"Error saving model cloud."<<std::endl;
                        exit(0);
                    }
                    break;
                case ply:
                    if (pcl::io::savePLYFile(path, *cloud) < 0){
                        std::cout<<"Error saving model cloud."<<std::endl;
                        exit(0);
                    }
                    break;
                case xyz:
                {
                    /* 3 elements: xyz, 6 elements: xyzrgb */
                    std::ifstream fs;
                    //                    fs.open(<#const char *__s#>)
                    break;
                }
            }
        }
    };
}
#endif /* pclloader_hpp */

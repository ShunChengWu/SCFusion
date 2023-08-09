#include "../CPU/ScanNet.hpp"
#include <ImageLoader/ImageLoader.hpp>
#include <ORUtils/EigenHelper.h>
#include <ORUtils//PathTool.hpp>

using namespace SCFUSION::IO;
namespace {
    std::string CheckEnd(std::string path){
        if (strcmp(&path[path.length() - 1], "/") != 0) {
            return path + "/";
        } else {
            return path;
        }
        return path;
    };
    std::vector<std::string> get_files_in_folder(std::string path, std::string type, bool return_full, bool sort) {
        std::vector<std::string> file_vec;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir(path.c_str())) != NULL) {
            path = CheckEnd(path);
            while ((ent = readdir(dir)) != NULL) {
                if (ent->d_name[0] != '.') {
                    /* print all the files and directories within directory */
                    //printf ("%s\n", ent->d_name);
                    file_vec.push_back(return_full ? path + ent->d_name : ent->d_name);
                }
            }
            closedir(dir);
        } else {
            /* could not open directory */
            perror("");
            exit(EXIT_FAILURE);
        }
        if (sort) std::sort(file_vec.begin(), file_vec.end());

        if (type == "") return file_vec;

        std::vector<std::string> filtered;
        for (auto name: file_vec) {
            if (name.size() > type.size()) {
                std::string tmp = name.substr(name.size() - type.size(), type.size());
                if (tmp == type) filtered.push_back(name);
            }
        }
        return filtered;
    }
    std::vector<std::string> splitLine(std::string s , char delimiter) {
        std::vector<std::string>tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter))
        {
            if(token != "")
                tokens.push_back(token);
        }
        return tokens;
    }
    std::string removeType(const std::string &name) {
        auto t = name.find_last_of('.');
        if(t != std::string::npos)
            return name.substr(0, t);
        else
            return name;
    }
}
ScanNet::ScanNet(const std::string &path_to_sens_folder) : main_path(path_to_sens_folder), loader(nullptr), img_counter(0),
                                                    img_max_counter(-1)
                                                    {
    auto name = tools::PathTool().get_current_dir_name(path_to_sens_folder);
    std::string path_to_sens = tools::PathTool().CheckEnd(path_to_sens_folder) + name + ".sens";
    if(!tools::PathTool().checkfileexist(path_to_sens)) {
        throw std::runtime_error("Cannot find any .sens file in the given path!\n");
    }
    path = path_to_sens;
}

ScanNet::~ScanNet() {
    if (loader) delete loader;
}

int ScanNet::Init() {
    if(!loader) loader = new ::ml::SensorData(path);
    paramDepth.SetFrom(loader->m_depthWidth, loader->m_depthHeight,
            loader->m_calibrationDepth.m_intrinsic._m00,loader->m_calibrationDepth.m_intrinsic._m11,
                       loader->m_calibrationDepth.m_intrinsic._m02,loader->m_calibrationDepth.m_intrinsic._m12);
    paramColor.SetFrom(loader->m_colorWidth, loader->m_colorHeight,
                       loader->m_calibrationColor.m_intrinsic._m00,loader->m_calibrationColor.m_intrinsic._m11,
                       loader->m_calibrationColor.m_intrinsic._m02,loader->m_calibrationColor.m_intrinsic._m12);
    img_max_counter = loader->m_frames.size();

//            DEBUG("ImageLoader::img_max_counter: %zu\n", img_max_counter);
    return img_max_counter;
}

int ScanNet::Next() {
    if(img_counter >= img_max_counter) return -1;
    else return img_counter++;
    img_counter++;
    if (img_counter >= img_max_counter) return 0;
    return (int) img_counter;
}

int ScanNet::NumberOfImages() {
    return img_max_counter;
}

int ScanNet::getDepth(ORUtils::Image<float> *depthptr) {
    return getDepth(img_counter-1,depthptr);
}

int ScanNet::getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr)
{
    return getColor(img_counter-1,colorptr);
}

int ScanNet::getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}

int ScanNet::getPose(ORUtils::Matrix4<float> *pose) {
    return getPose(img_counter-1,pose);
}

int ScanNet::getLabel(ORUtils::Image<ushort> *labelptr) {
    return getLabel(img_counter-1,labelptr);
}


int ScanNet::getDepth(int idx, ORUtils::Image<float> *depthptr) {
    unsigned short *depthData = loader->decompressDepthAlloc(idx);
    auto data = depthptr->GetData(MEMORYDEVICE_CPU);
    for (int r = 0; r < paramDepth.imgSize.height; ++r)
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            data[r * paramDepth.imgSize.width + c] =
                    1.f / loader->m_depthShift * static_cast<float>(depthData[r * paramDepth.imgSize.width + c]);
        }

    std::free(depthData);
    return 1;
}

int ScanNet::getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    ::ml::vec3uc *colorData = loader->decompressColorAlloc(idx);

    auto data = colorptr->GetData(MEMORYDEVICE_CPU);
    for (int r = 0; r < paramDepth.imgSize.height; ++r)
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            data[r * paramDepth.imgSize.width + c].x = colorData[r * paramDepth.imgSize.width + c].x;
            data[r * paramDepth.imgSize.width + c].y = colorData[r * paramDepth.imgSize.width + c].y;
            data[r * paramDepth.imgSize.width + c].z = colorData[r * paramDepth.imgSize.width + c].z;

//                        printf("%f ", depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c]);
        }
    //TODO: copy to our format
    std::free(colorData);
    return 0;
}

int ScanNet::getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    ::ml::vec3uc *colorData = loader->decompressColorAlloc(idx);

    auto data = colorptr->GetData(MEMORYDEVICE_CPU);
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            data[r * paramColor.imgSize.width + c].x = colorData[r * paramColor.imgSize.width + c].x;
            data[r * paramColor.imgSize.width + c].y = colorData[r * paramColor.imgSize.width + c].y;
            data[r * paramColor.imgSize.width + c].z = colorData[r * paramColor.imgSize.width + c].z;
            data[r * paramColor.imgSize.width + c].w = 255;
//                        printf("%f ", depthptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c]);
        }
    //TODO: copy to our format
    std::free(colorData);
    return 0;
}

int ScanNet::getLabel(int idx, ORUtils::Image<ushort> *labelptr) {
    if(labelptr == nullptr ) return 0;
    label_mat = cv::imread(main_path + "label-filt/" + std::to_string(idx) + ".png", -1);
    if(label_mat.empty()){
        std::cout << "Error: depth image file not read!" << std::endl;
        return 0;
    }
//            if (label_mat.type() != CV_16U){
//                label_mat.convertTo(label_mat, CV_16U);
//            }

    cv::resize(label_mat, label_mat, cv::Size(paramDepth.imgSize.width, paramDepth.imgSize.height), 0, 0,cv::InterpolationFlags::INTER_NEAREST);

    size_t counter=0;
    for (int r = 0; r < paramDepth.imgSize.height; ++r)
        for (int c = 0; c < paramDepth.imgSize.width; ++c)
            labelptr->GetData(MEMORYDEVICE_CPU)[counter++] =
                    (label_mat.at<unsigned short>(r, c));
    return 1;
}

int ScanNet::getPose(int idx, ORUtils::Matrix4<float> *pose) {
    memcpy(pose->m, loader->m_frames[idx].getCameraToWorld().matrix,
           16 * sizeof(float));
    // Rotate pose so depth is at z direction

    {
        auto pose_eigen = getEigenRowMajor<float, 4>(pose->m);//TODO: change to colMajor
        Eigen::Matrix4f mat4f;
        mat4f.setIdentity();

        Eigen::Matrix3f mat3f;
        mat3f = Eigen::AngleAxisf(-0.5 * EIGEN_PI, Eigen::Vector3f::UnitX());
//        std::cout << "mat3f\n" << mat3f << std::endl;
        mat4f.topLeftCorner<3, 3>() = mat3f;
        pose_eigen = mat4f * pose_eigen;
        mat3f = Eigen::AngleAxisf(-0.5 * EIGEN_PI, Eigen::Vector3f::UnitY());
//        std::cout << "mat3f\n" << mat3f << std::endl;
        mat4f.topLeftCorner<3, 3>() = mat3f;
        pose_eigen = mat4f * pose_eigen;
        pose_eigen.transposeInPlace();
//        std::cout << "pose_eigen\n" << *pose << std::endl;
    }
    return 1;
}
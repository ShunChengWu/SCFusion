#include "../CPU/ScanNetPose.hpp"
#include <ImageLoader/ImageLoader.hpp>
//#include <Utilities/helper_math.h>
//#include <Utilities/EigenHelper.h>
//#include <Utilities/utils.hpp>
#include <CxxTools/PathTool.hpp>

using namespace SCFUSION::IO;

ScanNetPose::ScanNetPose(const std::string &path_to_sens_folder) : main_path(path_to_sens_folder), img_counter(0),
                                                    img_max_counter(0)
                                                    {
    // check *.sens file exist
    auto name = tools::PathTool::get_current_dir_name(path_to_sens_folder);
    std::string path_to_sens = tools::PathTool::CheckEnd(path_to_sens_folder) + name + ".pose";
    if(!tools::PathTool::checkfileexist(path_to_sens)) {
        throw std::runtime_error("Cannot find any .sens file in the given path!\n");
    }
    path = path_to_sens;
}

ScanNetPose::~ScanNetPose() {
}

int ScanNetPose::Init() {
    std::fstream f(path, std::ios::in);
    f >> img_max_counter;
    mvPoses.resize(img_max_counter);

    ORUtils::Matrix4<float> pose;
    std::string line;
    std::string tmp[16];
    for(size_t i=0;i<mvPoses.size();++i){
//        if(i==1969){
//            printf("debug\n");
//        }

        f >> tmp[0] >> tmp[1] >> tmp[2] >> tmp[3]
                >> tmp[4] >> tmp[5] >> tmp[6] >> tmp[7]
                >> tmp[8] >> tmp[9] >> tmp[10] >> tmp[11]
                >> tmp[12] >> tmp[13] >> tmp[14] >> tmp[15];
        bool isNum = int(tools::PathTool::isNumber(tmp[0]));
//        if(tmp[0]!="-nan")std::cout << "isnumber: " << tmp[0] << ", " << isNum << std::endl;
        for(size_t j=0;j<16;++j){
            pose.m[j] = isNum?std::stof(tmp[j]):NAN;
        }
//        f >> pose.m[0] >> pose.m[1] >> pose.m[2] >> pose.m[3]
//            >> pose.m[4] >> pose.m[5] >> pose.m[6] >> pose.m[7]
//            >> pose.m[8] >> pose.m[9] >> pose.m[10] >> pose.m[11]
//            >> pose.m[12] >> pose.m[13] >> pose.m[14] >> pose.m[15];
        mvPoses[i] = pose;
    }

    return img_max_counter;
}

int ScanNetPose::Next() {
//    printf("debug: counter %zu/%zu\n", img_counter, img_max_counter);
    if(img_counter >= img_max_counter) return -1;
    else return img_counter++;
    img_counter++;
    if (img_counter >= img_max_counter) return 0;
    return (int) img_counter;
}

int ScanNetPose::NumberOfImages() {
    return img_max_counter;
}

int ScanNetPose::getPose(ORUtils::Matrix4<float> *pose) {
    if(img_counter==0)
        throw std::runtime_error("a Next() must be called before get Pose.");
    return getPose(img_counter-1,pose);
}

int ScanNetPose::getPose(int idx, ORUtils::Matrix4<float> *pose) {
    memcpy(pose->m, mvPoses[idx].m,
           16 * sizeof(float));
    std::cout << "pose:\n" << *pose;
//    std::cout << "pose: ";
//    for(size_t i=0;i<16;++i)
//    std::cout << mvPoses[idx].m[i]<< " ";
//    std::cout << "\n";
    return 1;
}
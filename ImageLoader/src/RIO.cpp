#ifdef WITH_OPENCV
#include "../CPU/RIO.hpp"
#include <ImageLoader/ImageLoader.hpp>
//#include <Utilities/helper_math.h>
//#include <Utilities/EigenHelper.h>
//#include <Utilities/utils.hpp>
#include <dirent.h>
//#include "../../bin/include/CxxTools/PathTool.hpp"
#include <iostream>
#include <CxxTools/PathTool.hpp>
using namespace SCFUSION::IO;


RIOReader::RIOReader(const std::string &folder)
        : folder(folder), img_counter(0), img_max_counter(-1)
          {
//              std::stringstream output_path_depth;
//              output_path_depth << folder << "/frame-" << std::setfill('0') << std::setw(6) << (a_index)
//                                << ".depth.pgm";
              scale_ = 1.f/1000.f;
          }

int RIOReader::Init() {
    cam_extrinsics.clear();
    depth_images.clear();
    color_images.clear();
    tools::PathTool::get_files_include_name(folder, ".pose.txt",cam_extrinsics,true,true);
    tools::PathTool::get_files_include_name(folder, ".depth.pgm",cam_extrinsics,true,true);
    tools::PathTool::get_files_include_name(folder, ".color.jpg",cam_extrinsics,true,true);
//    get_files_include_name(folder, ".pose.txt",cam_extrinsics,true,true);
//    get_files_include_name(folder, ".depth.pgm",depth_images,true,true);
//    get_files_include_name(folder, ".color.jpg",color_images,true,true);


    //TODO: memory issue due to un-initialized color parameters

    cv::Mat depth_mat = cv::imread(depth_images[0], -1);
    cv::Mat color_mat = cv::imread(color_images[0], -1);

    //TODO: check the size of them maybe
    img_max_counter = depth_images.size();
    paramColor.SetFrom(depth_mat.cols, depth_mat.rows, 759.273,757.959,490.82,263.451);
    paramDepth.SetFrom(color_mat.cols, color_mat.rows, 177.164,241.424,114.132,83.5809);

    // Read Image and get parameters


    return 1;
}

int RIOReader::Next() {
    if (img_counter >= img_max_counter) return -1;
    return img_counter++;
}

int RIOReader::NumberOfImages() {
    return img_max_counter;
}

int RIOReader::getDepth(ORUtils::Image<float> *depthptr) {
    return getDepth(img_counter-1,depthptr);
}
int RIOReader::getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}
int RIOReader::getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}
int RIOReader::getPose(ORUtils::Matrix4<float> *pose) {
    getPose(img_counter-1,pose);
}


int RIOReader::getDepth(int idx, ORUtils::Image<float> *depthptr) {
    if(depthptr == NULL) return -1;
    depth_mat = cv::imread(depth_images[idx], -1);
    if (depth_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;
    depthptr->ChangeDims(ORUtils::Vector2<int>(paramDepth.imgSize.width, paramDepth.imgSize.height));
    for (int r = 0; r < paramDepth.imgSize.height; ++r) {
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c] =
                    (float) (depth_mat.at<unsigned short>(r, c)) * scale_;
//            printf("%f ", depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c]);
        }
//        printf("\n");
    }
    return 1;
}
int RIOReader::getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    if(colorptr == NULL) return -1;
    color_mat = cv::imread(color_images[idx], -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
    cv::flip(color_mat, color_mat, 0);
    //TODO: copy to our format
    colorptr->ChangeDims(ORUtils::Vector2<int>(paramColor.imgSize.width, paramColor.imgSize.height));
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].x =
                    (color_mat.at<uchar3>(r, c)).x;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].y =
                    (color_mat.at<uchar3>(r, c)).y;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].z =
                    (color_mat.at<uchar3>(r, c)).z;
        }
    //FIXME: color cannt show here. but can be shown in SLAM visualizer.
    return 1;
}
int RIOReader::getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    if(colorptr == NULL) return -1;
    color_mat = cv::imread(color_images[idx], -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
    cv::flip(color_mat, color_mat, 0);
    //TODO: copy to our format
    colorptr->ChangeDims(ORUtils::Vector2<int>(paramColor.imgSize.width, paramColor.imgSize.height));
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].x =
                    (color_mat.at<uchar3>(r, c)).x;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].y =
                    (color_mat.at<uchar3>(r, c)).y;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].z =
                    (color_mat.at<uchar3>(r, c)).z;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].w = 255;
        }
    return 1;
}
int RIOReader::getPose(int idx, ORUtils::Matrix4<float> *pose) {
    if(cam_extrinsics.empty()) return -1;
    std::vector<float> base2world_vec = LoadMatrixFromFile(cam_extrinsics[idx], 4, 4);
//    base2world_vec[3] /= 1000.f;//TODO:scale not here
//    base2world_vec[7] /= 1000.f;
//    base2world_vec[11] /= 1000.f;
    memcpy(pose->m, base2world_vec.data(), sizeof(float) * 16);
    return 1;
}

#endif
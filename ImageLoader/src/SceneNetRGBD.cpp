#ifdef WITH_SCENENETRGBD
#include "../CPU/SceneNetRGBD.hpp"
#include <ImageLoader/ImageLoader.hpp>
#include <Utilities/helper_math.h>
#include <Utilities/EigenHelper.h>
#include <utility>
#include <Utilities/utils.hpp>
#include <dirent.h>
#include <CxxTools/PathTool.hpp>
//#include "../../bin/include/CxxTools/PathTool.hpp"

struct MyException : public std::exception
{
    std::string message;
    MyException(const char *file, const char *function, int line, std::basic_string<char> msg){
        message = "["+std::string(file)+"]["+std::string(function)+"]["+std::to_string(line)+"]: "+std::string(std::move(msg));
    }
    MyException(const char *file, const char *function, int line, std::string &msg){
        message = "["+std::string(file)+"]["+std::string(function)+"]["+std::to_string(line)+"]: "+msg;
    }
//    ~MyException() noexcept{}
    const char * what () const noexcept override
    {
        return message.c_str();
    }
};
#define THROW(msg) MyException(__FILE__,__FUNCTION__,__LINE__,msg)

using namespace SCFUSION::IO;

SceneNetRGBD::SceneNetRGBD(const std::string &path_folder, const std::string &path_pb)
        : img_counter(0), img_max_counter(-1),trajectory_(nullptr){
    path_depth_imgs_ = path_folder + "/depth";
    path_color_imgs_ = path_folder + "/photo";
    path_instances_ = path_folder + "/instance";
    path_pb_ = path_pb;
    scale_ = 1.f / 1000.f;

    std::string stringNum;
    auto is_number =[](const std::string& s)->bool
    {
        return !s.empty() && std::find_if(s.begin(),
                                          s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
    };

    std::string tmp;
    if(path_folder.back() == '/') tmp = path_folder.substr(0,path_folder.length()-1);
    else tmp = path_folder;
    stringNum = tmp.substr(tmp.find_last_of('/') + 1, tmp.length());
    if(!is_number(stringNum))
        throw THROW("expect to have a number at ths end of the path!\n");
    this->sequenceNumber = std::stoi(stringNum);

    tmp = tmp.substr(0,tmp.find_last_of('/'));
    stringNum = tmp.substr(tmp.find_last_of('/') + 1, tmp.length());
    if(!is_number(stringNum))
        throw THROW("expect to have a number at ths end of the path!\n");
    chunkNumber = std::stoi(stringNum);

}
SceneNetRGBD::~SceneNetRGBD(){
    if(trajectory_) delete trajectory_;
}

int SceneNetRGBD::Init() {
    //printf("[%s][%s] Warning. No ground truth pose file(s) are loaded.\n", __FILE__, __FUNCTION__);

    depth_images =
            tools::PathTool::get_files_in_folder(path_depth_imgs_, "", true, true);
    color_images =
            tools::PathTool::get_files_in_folder(path_color_imgs_, "", true, true);
    if(depth_images.size() != color_images.size())
        throw THROW("The size of depth images and color images mismatch!.\n");
    if(depth_images.empty())
        throw THROW("no depth images found!");
    if(color_images.empty())
        throw THROW("no color images found!");

    if(!path_pb_.empty()) {
        std::fstream input(path_pb_, std::ios::in | std::ios::binary);
        scenenet::Trajectories trajs;
        if (!trajs.ParseFromIstream(&input))
            throw THROW("Cannot read trajectory file! [" + path_pb_);
//        if (trajs.trajectories().size() < sequenceNumber + 1)
//            throw THROW("Trajectory file contains less trajectory than expected!\n");

        bool found = false;
        for (int i = 0; i < trajs.trajectories_size(); ++i) {
            auto renderPath = trajs.trajectories(i).render_path();
            auto tmp = std::stoi(renderPath.substr(renderPath.find_last_of('/') + 1, renderPath.length()));
            if (tmp == sequenceNumber) {
                this->trajectory_ = new scenenet::Trajectory(trajs.trajectories(i));
                found = true;
                break;
            }
        }
        if (!found)
            throw THROW("Cannot find corresponding trajectory in the given file");


        if (depth_images.size() != trajectory_->views_size())
            throw THROW("Image size and trajectory size mismatch!\n");

        for(size_t i=0;i<trajectory_->views_size();++i) {
            int view = trajectory_->views(i).frame_num();
            int depthImgNum = std::stoi(tools::PathTool::getFileName(depth_images[i]));
            int colorImgNum = std::stoi(tools::PathTool::getFileName(color_images[i]));
            if(view != depthImgNum)
                throw THROW("Pose frame_num does not match to depth frame num!");
            if(view != colorImgNum)
                throw THROW("Pose frame_num does not match to color frame num!");
        }
    }


    int pixel_width = 320;
    int pixel_height = 240;
    double hfov=60;
    double vfov=45;

    float fx = (pixel_width*0.5)/std::tan((hfov*EIGEN_PI/180/2.0));
    float fy = (pixel_height*0.5)/std::tan((vfov*EIGEN_PI/180/2.0));
    float cx = pixel_width*0.5;
    float cy = pixel_height*0.5;

    cv::Mat depth_mat = cv::imread(depth_images[0], -1);
    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;

    cv::Mat color_mat = cv::imread(color_images[0], -1);
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;

    //TODO: check the size of them maybe
    img_max_counter = depth_images.size();
    paramDepth.SetFrom(pixel_width, pixel_height, fx,fy,cx,cy);
    paramColor.SetFrom(pixel_width, pixel_height, fx,fy,cx,cy);
    return 1;
}

int SceneNetRGBD::Next() {
    if (img_counter >= img_max_counter) return -1;;
    return img_counter++;
}

int SceneNetRGBD::NumberOfImages() {
    return img_max_counter;
}

int SceneNetRGBD::getDepth(ORUtils::Image<float> *depthptr) {
    return getDepth(img_counter-1,depthptr);
}

int SceneNetRGBD::getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}

int SceneNetRGBD::getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}
int SceneNetRGBD::getPose(ORUtils::Matrix4<float> *pose) {
    return getPose(img_counter-1,pose);
}


int SceneNetRGBD::getDepth(int idx, ORUtils::Image<float> *depthptr) {
    if(depthptr == NULL) return -1;;
    depth_mat = cv::imread(depth_images[idx], -1);
    if (depth_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    depth_mat.convertTo(depth_mat,CV_32FC1);
//    cv::flip(depth_mat,depth_mat,0);
    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;
    depthptr->ChangeDims(ORUtils::Vector2<int>(paramDepth.imgSize.width, paramDepth.imgSize.height));
    double norm;
    for (int r = 0; r < paramDepth.imgSize.height; ++r) {
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            double fx = ( c - paramDepth.projectionParamsSimple.px) / paramDepth.projectionParamsSimple.fx;
            double fy = ( r - paramDepth.projectionParamsSimple.py) / paramDepth.projectionParamsSimple.fy;
            norm = 1/std::sqrt(fx*fx+fy*fy+1);
            depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c] =
                    (float) (depth_mat.at<float>(r,c)) * scale_ * norm;
//            printf("%f ",depth_mat.at<float>(r,c));
        }
//        printf("\n");
    }

//    printf("w,d: %d %d\n", depth_mat.cols,depth_mat.rows);
//    double min;
//    double max;
//    cv::minMaxIdx(depth_mat , &min, &max);
//    printf("min max: %f %f\n", min,max);
//    cv::convertScaleAbs(depth_mat , depth_mat , 255 / max);
//    imshow("d", depth_mat);
//    cv::waitKey(0);
    return 1;
}
int SceneNetRGBD::getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    if(colorptr == NULL) return -1;;
    color_mat = cv::imread(color_images[idx], -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
//    cv::flip(color_mat, color_mat, 0);
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
    return 1;
}
int SceneNetRGBD::getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    if(colorptr == NULL) return -1;;
    color_mat = cv::imread(color_images[idx], -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
//    cv::flip(color_mat, color_mat, 0);
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
int SceneNetRGBD::getPose(int idx, ORUtils::Matrix4<float> *pose) {
    if(trajectory_ == nullptr) return -1;

    const auto &camera = this->trajectory_->views(idx).shutter_close().camera();
    const auto &lookAt = this->trajectory_->views(idx).shutter_close().lookat();
    Eigen::Vector3f from (camera.x(), camera.y(),camera.z());
    Eigen::Vector3f to (lookAt.x(),lookAt.y(),lookAt.z());

    const Eigen::Vector3f up(0,1,0);
    Eigen::Matrix4f Rot = Eigen::Matrix4f::Identity(), T = Eigen::Matrix4f::Identity();
    Rot.block<1,3>(2, 0) = (to - from).normalized();
    Rot.block<1,3>(0, 0) = (Rot.block<1,3>(2, 0).cross(up)).normalized();
    Rot.block<1,3>(1, 0) = -(Rot.block<1,3>(0, 0).cross(Rot.block<1,3>(2, 0))).normalized();
    T.topRightCorner<3,1>() = -from;
    Rot = Rot * T;

//    Rot.transposeInPlace();
    Rot = Rot.inverse().eval();

    memcpy(pose->m,Rot.data(), sizeof(float) * 16);

//    std::cout << "pose_eigen\n" << *pose << std::endl;
    return true;
}
#endif //WITH_SCENENETRGBD
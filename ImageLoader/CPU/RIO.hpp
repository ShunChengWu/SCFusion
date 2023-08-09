#pragma once
//#include "../../ORUtils/Image.h"
//#include "../../ORUtils/Matrix.h"
#include <ORUtils/Image.h>
#include <ORUtils/Matrix.h>
#include <../include/ImageLoader/ImageLoader.hpp>

namespace SCFUSION {
    namespace IO{
        class RIOReader : public ImageLoader {
        public:
            RIOReader(const std::string &folder);

            int Init() override;

            int Next() override;

            int NumberOfImages() override;

            int getDepth(ORUtils::Image<float> *depthptr) override;

            int getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) override;

            int getDepth(ORUtils::Image<short> *depthptr) override  {return -1;};

            int getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) override ;

            int getPose(ORUtils::Matrix4<float> *pose) override;

            int getLabel(ORUtils::Image<ushort> *labelptr) override{return -1;}

            int getDepth(int idx, ORUtils::Image<float> *depthptr) override ;
            int getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) override ;
            int getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) override ;
            int getLabel(int idx, ORUtils::Image<ushort> *labelptr) override {return -1;}
            int getPose(int idx, ORUtils::Matrix4<float> *pose) override ;
        private:
            std::string folder;
            std::vector<std::string> cam_extrinsics;
            std::vector<std::string> depth_images;
            std::vector<std::string> color_images;
            size_t img_counter, img_max_counter;
            std::string path_K_, path_extrin_, path_depth_imgs_, path_color_imgs_;
            cv::Mat color_mat, depth_mat;
        };
    }
}

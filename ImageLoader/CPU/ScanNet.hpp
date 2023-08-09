#pragma once
#include "../include/ImageLoader/ImageLoader.hpp"
#include "ScanNet/sensorData.h"
#include <ORUtils/Image.h>
#include <ORUtils/Matrix.h>
//#include "../../ORUtils/Image.h"
//#include "../../ORUtils/Matrix.h"

namespace SCFUSION {
    namespace IO{
        class ScanNet : public ImageLoader {
        public:
            ScanNet(const std::string &path_to_sens);

            ~ScanNet() override;

            int Init() override;

            int Next() override;

            int NumberOfImages() override;

            int getDepth(ORUtils::Image<float> *depthptr) override;

            int getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) override;

            int getDepth(ORUtils::Image<short> *depthptr) override {return -1;};

            int getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) override;

            int getPose(ORUtils::Matrix4<float> *pose) override;

            int getLabel(ORUtils::Image<ushort> *labelptr) override;


            int getDepth(int idx, ORUtils::Image<float> *depthptr) override ;
            int getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) override ;
            int getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) override ;
            int getLabel(int idx, ORUtils::Image<ushort> *labelptr) override ;
            int getPose(int idx, ORUtils::Matrix4<float> *pose) override ;
        private:
            std::string main_path;
            ::ml::SensorData *loader;
            std::string path;
            size_t img_counter, img_max_counter;
            cv::Mat label_mat;
        };

    }
}

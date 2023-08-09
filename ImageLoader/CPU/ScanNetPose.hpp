#pragma once
#include "../include/ImageLoader/ImageLoader.hpp"
#include "ScanNet/sensorData.h"
#include <ORUtils/Image.h>
#include <ORUtils/Matrix.h>
//#include "../../ORUtils/Image.h"
//#include "../../ORUtils/Matrix.h"
//#include <Utilities/defines.h>
//#include <Utilities/helper_math.h>

namespace SCFUSION {
    namespace IO{
        class ScanNetPose : public ImageLoader {
        public:
            ScanNetPose(const std::string &path_to_sens);

            ~ScanNetPose() override;

            int Init() override;

            int Next() override;

            int NumberOfImages() override;

            int getDepth(ORUtils::Image<float> *depthptr) override { return 1; };

            int getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) { return 1; };

            int getDepth(ORUtils::Image<short> *depthptr) override {return -1;};

            int getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) { return 1; };

            int getPose(ORUtils::Matrix4<float> *pose) override;

            int getLabel(ORUtils::Image<ushort> *labelptr) { return 1; };


            int getDepth(int idx, ORUtils::Image<float> *depthptr) { return 1; };
            int getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) { return 1; };
            int getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) { return 1; };
            int getLabel(int idx, ORUtils::Image<ushort> *labelptr) { return 1; };
            int getPose(int idx, ORUtils::Matrix4<float> *pose) override ;
        private:
            std::string main_path;
            std::string path;
            size_t img_counter, img_max_counter;
            std::vector<ORUtils::Matrix4<float>> mvPoses;
        };

    }
}

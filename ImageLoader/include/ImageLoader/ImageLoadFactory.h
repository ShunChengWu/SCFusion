#pragma once
#include "../../../ORUtils/Logging.h"
#include "../../CPU/ScanNetPose.hpp"

#ifdef COMPILE_WITH_OPENCV
#include "../../CPU/ScanNet.hpp"
#include "../../CPU/RIO.hpp"
#endif

namespace SCFUSION {

    struct ImageLoaderFactory {
        static SCFUSION::IO::ImageLoader * MakeImageLoader(SCFUSION::IO::InputDateType::INPUTDATATYPE inputdatatype, const std::string &folder = "", const std::string &cam_K = "",
                                                           const std::string &gt = "", const std::string &depthFolderName = "", const std::string &colorFolderName = "") {
            SCFUSION::IO::ImageLoader *imageLoader = nullptr;
            switch (inputdatatype) {
                case SCFUSION::IO::InputDateType::INPUTTYPE_SCANNET: {
                    imageLoader = (new SCFUSION::IO::ScanNet(folder));
                }
                    break;
                case SCFUSION::IO::InputDateType::INPUTTYPE_RIO: {
#ifdef WITH_OPENCV
                    imageLoader = new SCFUSION::IO::RIOReader(folder );
                    break;
#else
                    throw std::runtime_error("INPUTTYPE_RIO Require OpenCV\n");
#endif
                }
                case SCFUSION::IO::InputDateType::INPUTTYPE_SCANNET_POSE: {
                    imageLoader = (new SCFUSION::IO::ScanNetPose(folder));
                }
                    break;
            }
            return imageLoader;
        }
    };

}
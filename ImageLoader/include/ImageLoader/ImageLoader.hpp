#pragma once
//#include "../../ORUtils/Image.h"
//#include "../../ORUtils/Matrix.h"
#include <ORUtils/Image.h>
#include <ORUtils/Matrix.h>
//#include <Utilities/defines.h>
//#include <Utilities/helper_math.h>
#include "../../MainLib//Objects/Camera/ITMIntrinsics.h"
#include <ORUtils//PathTool.hpp>
#include "../CPU/util.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace SCFUSION {
//    struct CameraParameters{
//        float cx, cy;
//        float fx, fy;
//        unsigned short width, height;
//    };

    namespace IO{
        class InputDateType{
        public:
            enum INPUTDATATYPE {
                INPUTTYPE_SCANNET=0, INPUTTYPE_RIO=1, INPUTTYPE_SCANNET_POSE=2,
            };
            static std::string GetAllTypesInString(){
                return "0:INPUTTYPE_SCANNET, 1:INPUTTYPE_RIO, 2:INPUTTYPE_SCANNET_POSE";
            }
            static INPUTDATATYPE ToInputDateType(int i){
                if(i > 2) throw std::runtime_error("exceed enum limit\n");
                return static_cast<INPUTDATATYPE>(i);
            }
        };

        class ImageLoader {
        public:
            explicit ImageLoader():scale_(1),flipRGB_(true),verbose_(false){}
            virtual ~ImageLoader()= default;
            virtual int Init()=0;
            /// Should be >= 0
            virtual int Next()=0;
            virtual int NumberOfImages()=0;
            virtual int getDepth(ORUtils::Image<float> *depthptr)=0;
            virtual int getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr)=0;
            virtual int getDepth(ORUtils::Image<short> *depthptr)=0;
            virtual int getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr)=0;
            virtual int getLabel(ORUtils::Image<ushort> *labelptr)=0;
            virtual int getPose(ORUtils::Matrix4<float> *pose)=0;

            virtual int getDepth(int idx, ORUtils::Image<float> *depthptr)=0;
            virtual int getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr)=0;
            virtual int getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr)=0;
            virtual int getLabel(int idx, ORUtils::Image<ushort> *labelptr)=0;
            virtual int getPose(int idx, ORUtils::Matrix4<float> *pose)=0;

            void setDepthScale(float scale) {scale_ = scale;}
            void setFlipRGB(bool option){flipRGB_=option;}

            int getColorParams(int& width, int& height, float& fx, float& fy, float& cx, float& cy);
            int getDepthParams(int& width, int& height, float& fx, float& fy, float& cx, float& cy);

            ITMLib::ITMIntrinsics getDepthCameraParams(){ return paramDepth;}
            ITMLib::ITMIntrinsics getRGBCameraParams(){ return paramColor;}
            ITMLib::ITMIntrinsics getLabelParams(){ return paramLabel;}
        protected:
            ITMLib::ITMIntrinsics paramDepth, paramColor, paramLabel;
//            CameraParameters paramDepth, paramColor, paramLabel;
            float scale_;
            bool flipRGB_;
            bool verbose_;

            void loadIntrinsics(const std::string &path);
        };

        class InheritTemplate : public ImageLoader {
        public:
            InheritTemplate(){}
            ~InheritTemplate(){}
            int Init() override;
            int Next() override;
            int NumberOfImages() override;
            int getDepth(ORUtils::Image<float> *depthptr) override;
            int getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) override;
            int getDepth(ORUtils::Image<short> *depthptr) override ;
            int getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) override ;
            int getPose(ORUtils::Matrix4<float> *pose) override;
            int getLabel(ORUtils::Image<ushort> *labelptr) override;
        };
    }
}

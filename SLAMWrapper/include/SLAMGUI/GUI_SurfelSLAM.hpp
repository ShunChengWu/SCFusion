#pragma once
#include "../../../GUI3D/GUI3D.h"
#include <ImageLoader/ImageLoader.hpp>
#include <SLAMTools/SLAMWrapper.h>
#include "../../../SCSLAM/Core/ITMMainEngine.h"

namespace SCSLAM {
    enum ProcessMode {
        STOP, STEPONCE, CONTINUE
    };

    template <typename SLAMType>
    class SurfelSLAMGUI : public SC::GUI3D {
    public:
        SurfelSLAMGUI(SLAMWrapper<SLAMType> *slamWarpper, std::string outputPath);
        ~SurfelSLAMGUI();
        virtual int initialization();
        virtual void drawUI() override ;
        virtual void drawGL() override ;
    protected:
        void registerKeys(SC::GLFWWindowContainer *window);
        virtual void process_impl();
        virtual void process_slam();
        virtual void recordImg();
        virtual void renderImgFromSLAM(SLAMType *slam_);
        void renderCamera();

        /**
         * Camera refenrece frame: x: right, y: down, z: forward
         * OpenGL Reference frame: x: left, y: up z: forward
         */
        void CameraRefenceToOpenGLReference(Eigen::Matrix4f *modelPose);

        /**
         * Camera refenrece frame: x: right, y: down, z: forward
         * OpenGL Reference frame: x: left, y: up z: forward
         */
        void OpenGLReferenceToCameraRefence(ORUtils::Matrix4<float> *modelPose);
        void OpenGLReferenceToCameraRefence(Eigen::Map<Eigen::Matrix4f> *modelPose);
        void OpenGLReferenceToCameraRefence(Eigen::Matrix4f *modelPose);
//    private:
        std::string outputPath;
        SLAMWrapper<SLAMType> *slamWarpper_;
        SLAMType *slam_;
        ITMLib::ITMIntrinsics render_intrinsics;
        ITMLib::ITMIntrinsics camDepthParam, camColorParam;
        bool bNeedUpdate, bRecordImage, bShowPose;
        ProcessMode processmode_;
        std::unique_ptr<glUtil::GLImageRenderer> glImageRenderer;
        std::map<std::string, std::vector<double>> times_;
        std::unique_ptr<ITMUChar4Image> Block_depth2RGB_;
        cv::Mat cvDepthImage, cvColorImage;
        int iterSave, slam_getImageType;
        std::string outputPath_;
        std::string pth_to_log_file, pth_to_image_folder;
    };

}
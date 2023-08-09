#pragma once
#undef __AVX2__
#include <GUI3D/GUI3D.h>
#include <ImageLoader/ImageLoader.hpp>
#include "../../../MainLib/Core/BasicSLAM/MainEngine.h"
#include "../../../MainLib/Objects/Meshing/ITMMesh.h"
#include <Utils/CUDAStreamHandler.hpp>
#include <utility>
#include "gui_kernel.hpp"
#include "../SLAMTools/SLAMWrapper.h"
#include <GUI3D/ImageDrawer.h>
#include <ft2build.h>
#include FT_FREETYPE_H


namespace SCFUSION {
    enum ProcessMode {
        STOP, STEPONCE, CONTINUE
    } ;

    template<class SLAMType>
    class SLAMGUI : public SC::GUI3D {
    public:
        SLAMGUI(SLAMWrapper<SLAMType> *slamWarpper, std::string outputPath);
        ~SLAMGUI();

        virtual int initialization();
        virtual void drawUI() override ;
        virtual void drawGL() override ;
        void run() override ;

    protected:
        bool bDrawBottomRightInfoPanel;
        virtual void DrawLeftPanel();
        virtual void DrawMiddlePanel();
        virtual void DrawTopRightPanel();
        virtual void DrawBottomRightPanel();
        virtual void DrawBottomRightOverlay();
    protected:
        const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
        std::string outputPath;
        std::string pth_to_log_file, pth_to_image_folder;
        ITMLib::ITMIntrinsics render_intrinsics;
        ITMLib::ITMIntrinsics camDepthParam, camColorParam;
        ProcessMode processmode_;
        cv::Mat cvDepthImage, cvColorImage;
        int iterSave;
        bool bRecordImage, bFollowGTPose;
        bool bShowPose,bCheckState;
        std::map<std::string, std::vector<double>> times_;
        std::fstream logFile_;

        std::unique_ptr<ITMUChar4Image> Block_ImageBuffer_;
        std::unique_ptr<glUtil::ImageDrawer> mImageDrawer;


        /// SLAM
        enum {
            camera_PoseView, camera_FreeView
        } cameraMode_;
        enum DisplayMode{
            DisplayMode_FreeView_Surface,
            DisplayMode_FreeView_SurfaceLabel,
            DisplayMode_FreeView_ColoredLabel,
            DisplayMode_FreeView_Rendered_Height,
        };
        DisplayMode displayMode_mesh;

        int displayMode_FreeView, displayMode_FollowView;
        SLAMWrapper<SLAMType> *slamWarpper_;
        SLAMType *slam_;
        int slam_getImageType;
        bool bNeedUpdate, bRenderMesh;

        CUDA_Stream_Handler<std::string> cuda_gui_streams_;

        /// Marching Cubes
        std::future<void> mc_thread_;
        std::mutex mutex_mc_;
        size_t mc_total_size;
        float mc_normal_sign_;
        std::unique_ptr<ORUtils::MemoryBlock<ITMLib::ITMMesh::Triangle>> mc_triangles;
        std::unique_ptr<ORUtils::MemoryBlock<ITMLib::ITMMesh::Normal>> mc_normals;
        std::unique_ptr<ORUtils::MemoryBlock<ITMLib::ITMMesh::Color>> mc_colors;
        std::unique_ptr<ORUtils::MemoryBlock<Vector4f>> points_location, points_color;
        std::atomic_int mc_vertices_size;
        bool bNeedUpdateSurface;
        int initMC();
        virtual int NeedUpdateSurfaceConditions();
        void copyMeshToGUI(SLAMType *slam);
        void updateVertexBuffer(size_t newSize, bool force = false);
        void drawMesh(glUtil::Shader *shader);
        void drawPointCloud(glUtil::Shader *shader);
        void ResetMCBuffer();

    public: /// Multithreading function need to be public in order to be called from inherit classes
        virtual void extractGlobalMapSurface();
        virtual void extractGlobalMapSurfaceLabel();
        virtual void extractPoints();

    protected:
        virtual void registerKeys(SC::GLFWWindowContainer *window);
        virtual void process_impl();
        virtual bool process_slam();
        virtual void recordImg();
        virtual void renderImgFromSLAM(SLAMType *slam_);
        virtual void renderCamera();
        virtual void calculateStatisticalTimes(std::ostream  &os);
        virtual void printResultOnFile(std::ostream & os);
        virtual void alignToglCam();
        int loadShaders();

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
    };
}

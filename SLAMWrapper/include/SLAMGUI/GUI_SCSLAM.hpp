#pragma once
#undef __AVX2__
#include "GUI_SLAM.hpp"
#include <ImageLoader/ImageLoader.hpp>
#include <SCFusion/SLAM.h>
#include <Utils/CUDAStreamHandler.hpp>
#include <utility>
#include "gui_kernel.hpp"
#include <SLAMTools/SLAMWrapper.h>

#include <ft2build.h>
#include FT_FREETYPE_H

namespace SCFUSION {
    struct glPoseHolder{
        float row, pitch, yaw;
        float x, y, z;
    };

    template<class SLAMType>
    class SCSLAMGUI: public SLAMGUI<SLAMType> {
        typedef SLAMGUI<SLAMType> Base;
        using SLAMGUI<SLAMType>::window_;
        using SLAMGUI<SLAMType>::displayMode_FreeView;
        using SLAMGUI<SLAMType>::bNeedUpdateSurface;
        using SLAMGUI<SLAMType>::slam_;
        using SLAMGUI<SLAMType>::slamWarpper_;
        using SLAMGUI<SLAMType>::processmode_;
        using SLAMGUI<SLAMType>::bNeedUpdate;
        using SLAMGUI<SLAMType>::cameraMode_;
//        using SLAMGUI<SLAMType>::Block_freeview_pose_;
        using SLAMGUI<SLAMType>::DisplayMode_FreeView_Surface;
        using SLAMGUI<SLAMType>::DisplayMode_FreeView_SurfaceLabel;
        using SLAMGUI<SLAMType>::DisplayMode_FreeView_ColoredLabel;
        using SLAMGUI<SLAMType>::DisplayMode_FreeView_Rendered_Height;
        using SLAMGUI<SLAMType>::glFrameBuffers;
        using SLAMGUI<SLAMType>::SHADOW_WIDTH;
        using SLAMGUI<SLAMType>::SHADOW_HEIGHT;
        using SLAMGUI<SLAMType>::glTextures;
        using SLAMGUI<SLAMType>::mc_vertices_size;
        using SLAMGUI<SLAMType>::bShowPose;
        using SLAMGUI<SLAMType>::registerKeyFunciton;
        using SLAMGUI<SLAMType>::extractPoints;
        using SLAMGUI<SLAMType>::alignToglCam;
        using SLAMGUI<SLAMType>::renderImgFromSLAM;
        using SLAMGUI<SLAMType>::add_trajectory;
        using SLAMGUI<SLAMType>::glCam;
        using SLAMGUI<SLAMType>::glShaders;
        using SLAMGUI<SLAMType>::glBuffers;
        using SLAMGUI<SLAMType>::glVertexArrays;
        using SLAMGUI<SLAMType>::fps_;
        using SLAMGUI<SLAMType>::bRenderMesh;
        using Base::bShowFPS;
        using Base::bDrawBottomRightInfoPanel;
        using SC::GUI3D::bPlotTrajectory;
        using SC::GUI_base::bDrawDemoUI;
        using Base::bShowGrid;
    public:
        SCSLAMGUI(SLAMWrapper<SLAMType> *slamWarpper, const std::string& outputPath):
        SLAMGUI<SLAMType>(slamWarpper,outputPath){
            displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL;
            bRunSceneCompletion = false;
            bShowSCGrid = false;
        }

        enum SCDisplayMode{
            Foo_DisplayMode_FreeView_Surface,
            Foo_DisplayMode_FreeView_SurfaceLabel,
            Foo_DisplayMode_FreeView_ColoredLabel,
            DisplayMode_FreeView_OBufferSurface=3,
            DisplayMode_FreeView_IBufferSurface=4,
            DisplayMode_FreeView_Surface_wo=5,
            DisplayMode_FreeView_Surface_pure_sc=6,
        };
        SCDisplayMode displayMode_mesh;

        int initialization() override;
        bool bRunSceneCompletion;
    protected:
        void registerSCKeys();
        int NeedUpdateSurfaceConditions() override;
        void process_impl();

        void extractSCIBufferSurface();
        void extractSCOBufferSurface();

        void SCShader();

        void DrawLeftPanel() override;
//        void DrawBottomRightOverlay() override;

        int rotateAlongRow=0, rotateAlongPitch=0, rotateAlongYaw=0;
        int rotateSpeed=10;
        std::vector<glPoseHolder> swagPoses;
        glPoseHolder savedPose_;
        bool bShowSCGrid;
    };
}

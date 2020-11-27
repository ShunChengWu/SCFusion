#include <fstream>
#include <CxxTools/PathTool.hpp>
#include "GUI_SLAM.hpp"
#include "gui_kernel.hpp"
#include "../../../ORUtils/EigenHelper.h"
#include <utility>
//#include <Utilities/utils.hpp>
#include "../../../ORUtils/LabelColorUtils.h"
#include <Engines/Meshing/MeshEngine.h>
#include "../../../ORUtils/Logging.h"

#ifdef WIN32 //For create/delete files
#include <direct.h>
#else
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
//#include <SCFusion/config.h>
#endif

using namespace SCFUSION;


template <class T>
void labelChecker(const std::string& name, T *data, size_t size, size_t labels) {
    ORUtils::MemoryBlock<uint> labelCounter(labels, true, false);
    auto label_data = labelCounter.GetData(MEMORYDEVICE_CPU);
    for(size_t i=0;i<size;++i){
        label_data[(int)data[i]]++;
    }
    printf("--Distribution of %s with %zu labels--\n", name.c_str(), labels);
    for(size_t i=0;i<labels;++i){
        printf("[%2zu] %u (%f)\n", i, label_data[i], float(label_data[i])/float(size));
    }
}


inline float safeACOS(float x) {
    if (x < -1.f)x = -1.f;
    else if (x > 1.f)x = 1.f;
    return acos(x);
}

Eigen::Matrix3f getRotMatfromTwoVectors(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2) {
    if (vec1 == vec2) {
        return Eigen::Matrix3f::Identity();
    }
    Eigen::Vector3f crossProduct = vec1.cross(vec2);
    Eigen::Vector3f vector_X = (crossProduct / crossProduct.norm());
    float thetaAngleRad = safeACOS(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
    Eigen::AngleAxisf aa(thetaAngleRad, vector_X);

    return aa.toRotationMatrix();
}

template<class SLAMType>
SLAMGUI<SLAMType>::SLAMGUI(SLAMWrapper<SLAMType> *slamWarpper, std::string outputPath_):
GUI3D("SLAM", 1080, 720),
outputPath(std::move(outputPath_)), slamWarpper_(slamWarpper) {
//    glCam->camera_control_ = std::make_unique<SC::ArcCameraContorl>();
//    reinterpret_cast<SC::ArcCameraContorl*>(glCam->camera_control_.get())->SetDistance(5.0);

    slam_ = slamWarpper_->getSLAM();
    camDepthParam = slamWarpper_->getCalibParam()->intrinsics_d;
    camColorParam = slamWarpper_->getCalibParam()->intrinsics_rgb;
    render_intrinsics.SetFrom(camDepthParam.imgSize.width, camDepthParam.imgSize.height, camDepthParam.projectionParamsSimple.fx, camDepthParam.projectionParamsSimple.fy,
                              camDepthParam.projectionParamsSimple.px, camDepthParam.projectionParamsSimple.py);

    processmode_ = STOP;
    bDrawBottomRightInfoPanel = false;
    bDrawDemoUI = false;
    bCheckState = false;
    bNeedUpdate = false;
    bNeedUpdateSurface = false;
    bRecordImage = false;
    bFollowGTPose = false;
    bShowGrid = false;
    bShowPose = true;
    bShowFPS = true;
    bRenderMesh = false;
    mc_vertices_size = 0;
    iterSave = 0;
    cameraMode_ = camera_PoseView;
    displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
    displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
    displayMode_mesh = DisplayMode_FreeView_Surface;

    slam_getImageType = 0;
    mImageDrawer.reset(new glUtil::ImageDrawer());
    mImageDrawer->Init(camDepthParam.imgSize.width, camDepthParam.imgSize.height,GL_RGBA);
    Block_ImageBuffer_.reset(
            new ITMUChar4Image(ORUtils::Vector2<int>(camDepthParam.imgSize.width, camDepthParam.imgSize.height), true, true));


    cuda_gui_streams_.createStream("GUI", true);
    cuda_gui_streams_.createStream("MC", true);
    cuda_gui_streams_.createStream("MC_triangle", true);
    cuda_gui_streams_.createStream("MC_normal", true);
    cuda_gui_streams_.createStream("MC_color", true);

    {
        /// Append information to the file name
        outputPath = tools::PathTool::CheckEnd(outputPath);
        outputPath.erase(outputPath.end() - 1);
        outputPath = tools::PathTool::remove_file_type(outputPath);

        static bool modify_output_name = false;
        if (modify_output_name) {
            if (ITMVoxel::integrateType == SCFUSION::IntegrateType_TSDF && ITMVoxel::hasLabelInformation) {
                outputPath += "_sdfLable";
            } else if (ITMVoxel::integrateType == SCFUSION::IntegrateType_TSDF) {
                outputPath += "_sdf";
            } else if (ITMVoxel::integrateType == SCFUSION::IntegrateType_OFusion && ITMVoxel::hasLabelInformation) {
                outputPath += "_ofu";
            } else if (ITMVoxel::integrateType == SCFUSION::IntegrateType_OFusion) {
                outputPath += "_ofuLabel";
            }
            if (slam_->getITMSetting()->useSC) outputPath += "_sc";
        }

        /// Create folder
        auto tmp = tools::PathTool::find_parent_folder(outputPath);
        tools::PathTool::check_and_create_folder(tmp);

        /// Create log file
        size_t serial_num = 0;
        pth_to_log_file = outputPath + std::to_string(serial_num) + "_log.csv";

        if (modify_output_name) {
            /// Prevent overwrite by assign serial number
            bool exist = tools::PathTool::checkfileexist(pth_to_log_file);
            while (exist) {
                serial_num++;
                pth_to_log_file = outputPath + std::to_string(serial_num) + "_log.csv";
                exist = tools::PathTool::checkfileexist(pth_to_log_file);
            }
            outputPath = outputPath + std::to_string(serial_num); // change this globally
        }

        /// Create Log File
        pth_to_log_file = outputPath + "_log.csv";
        logFile_.open(pth_to_log_file, std::ios::out);
        if (!logFile_.is_open()) {
            printf("Cannot open/create log file for writing.\n");
            exit(-1);
        }
        printf("Log file created at %s\n", pth_to_log_file.c_str());

        /// Create Folder to save images
        pth_to_image_folder = outputPath + "_images";
        tools::PathTool::check_and_delete_folder(pth_to_image_folder);
        tools::PathTool::check_and_create_folder(pth_to_image_folder);
    }

    /// Calculate fov
    fov = atan(float(camDepthParam.imgSize.height) / float(camDepthParam.projectionParamsSimple.fy) / 2) * 2 * 180 / EIGEN_PI;


    /// Mesh and Pointcloud buffer
    if (slam_->getITMSetting()->createMeshingEngine) {
        mc_triangles.reset(
                new ORUtils::MemoryBlock<ITMLib::ITMMesh::Triangle>(slam_->getMesh()->noMaxTriangles, true, false));
        if (slam_->getMesh()->hasNormal)
            mc_normals.reset(
                    new ORUtils::MemoryBlock<ITMLib::ITMMesh::Normal>(slam_->getMesh()->noMaxTriangles, true, false));
        if (slam_->getMesh()->hasColor)
            mc_colors.reset(
                    new ORUtils::MemoryBlock<ITMLib::ITMMesh::Color>(slam_->getMesh()->noMaxTriangles, true, false));
    }
    if (slam_->getITMSetting()->createPointExtractionEngine) {
        points_location.reset(new ORUtils::MemoryBlock<Vector4f>(slam_->getPointCloud()->noMaxPoints, true, false));
        points_color.reset(new ORUtils::MemoryBlock<Vector4f>(slam_->getPointCloud()->noMaxPoints, true, false));
    }
}

template<class SLAMType>
SLAMGUI<SLAMType>::~SLAMGUI() {
    logFile_.close();
    printf("LOGFILE CLOSED\n");
}

template<class SLAMType>
int SLAMGUI<SLAMType>::initialization() {
    TICK("[SLAMGUI][init]0.All");
    initMC();
    //GUI3D::initialization(camColorParam.imgSize.width,camColorParam.imgSize.height, "Main");
    loadShaders();
    registerKeys(window_);
    TOCK("[SLAMGUI][init]0.All");
    return 1;
}

/// must after initSLAM
template<class SLAMType>
int SLAMGUI<SLAMType>::initMC() {
    mc_normal_sign_ = 1.f;
    mc_total_size = SDF_LOCAL_BLOCK_NUM * 32 * 3;
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    /// Vertices
    {
        std::string name = "Vertices";
        glShaders[name] = new glUtil::Shader(shaderPath + "verticesWithNormal.vs",
                                             ITMVoxel::hasLabelInformation
                                             ? shaderPath + "verticesWithNormal.fs" :
                                             shaderPath + "verticesWithNormal_white.fs");
        glShaders[name]->use();
        glShaders[name]->set("sign", mc_normal_sign_);
        updateVertexBuffer(mc_total_size);
    }

    /// Points
    {
        std::string name = "Points";
        glShaders[name] = new glUtil::Shader(shaderPath + "circle.vs",
                                             shaderPath + "circle.fs",
                                             shaderPath + "circle.gs");
        glShaders[name]->use();
        glShaders[name]->set("radius", slam_->getITMSetting()->sceneParams.voxelSize * 0.5);
    }
    return 1;
}

template<class SLAMType>
void SLAMGUI<SLAMType>::updateVertexBuffer(size_t newSize, bool force) {
    std::string name = "Vertices";

    if (!force)
        if (mc_total_size > newSize && glVertexArrays.find(name) != glVertexArrays.end()) return;

    mc_total_size = (std::floor(newSize / 1e6) + 1) * 1e6;

    if (glVertexArrays.find(name) != glVertexArrays.end()) {
        glDeleteVertexArrays(1, &glVertexArrays.at(name));
        glDeleteBuffers(1, &glBuffers.at(name));
    }

    uint numStride = ITMVoxel::hasLabelInformation ? 12 : 8;
    size_t totalSize = mc_total_size * numStride;
    glGenVertexArrays(1, &glVertexArrays[name]);
    glGenBuffers(1, &glBuffers[name]);
    glBindVertexArray(glVertexArrays[name]);
    glBindBuffer(GL_ARRAY_BUFFER, glBuffers[name]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat),
                          (void *) (mc_total_size * 4 * sizeof(GLfloat)));
    if (ITMVoxel::hasLabelInformation) {
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat),
                              (void *) (mc_total_size * 8 * sizeof(GLfloat)));
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

template<class SLAMType>
int SLAMGUI<SLAMType>::loadShaders() {
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    const std::string fontPath = std::string(GUI_FOLDER_PATH) + "fonts/";

    // For saving image
    cvColorImage.release();
    cvDepthImage.release();

    /// Shadow
    if (0) {
        glShaders["Shadow"] = new glUtil::Shader(shaderPath + "simpleShadow.vs",
                                                 shaderPath + "simpleShadow.fs");

        unsigned int &depthMapFBO = glFrameBuffers["DepthMapFBO"];
        glGenFramebuffers(1, &depthMapFBO);

        // Create a depth buffer for framebuffer
        unsigned int &depthMap = glTextures["depthMap"];
        glGenTextures(1, &depthMap);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                     SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        // Attach the buffer to framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    return 1;
}



template <class SLAMType>
void SLAMGUI<SLAMType>::registerKeys(SC::GLFWWindowContainer *window) {
//    registerKeyFunciton(window, GLFW_KEY_1, [&]() {
//        if (displayMode_FreeView == DisplayMode_FreeView_Rendered_Height) slam_getImageType++;
//        displayMode_FreeView = DisplayMode_FreeView_Rendered_Height;
//        bNeedUpdateSurface = true;
//        bNeedUpdate = true;
//
//        if (//slam_getImageType < SCFusion::SLAM::InfiniTAM_IMAGE_FREECAMERA_SHADED ||
//                slam_getImageType == SCFusion::MainEngine<ITMVoxel, ITMVoxelIndex>::InfiniTAM_IMAGE_UNKNOWN + 1)
//            slam_getImageType = 0;//SCFusion::SLAM::InfiniTAM_IMAGE_FREECAMERA_SHADED;
//        printf("displayMode_FreeView: %s\n",
//                SCFusion::MainEngine<ITMVoxel,ITMVoxelIndex>::getGetImageTypeString(slam_getImageType).c_str());
//    });
//    registerKeyFunciton(window, GLFW_KEY_2, [&]() {
//        displayMode_FreeView = DisplayMode_FreeView_Surface;;
//        bNeedUpdateSurface = true;
//        printf("displayMode_FreeView: DisplayMode_FreeView_Surface\n");
//    });
//    registerKeyFunciton(window, GLFW_KEY_3, [&]() {
//        displayMode_FreeView = DisplayMode_FreeView_SurfaceLabel;;
//        bNeedUpdateSurface = true;
//        printf("displayMode_FreeView: DisplayMode_FreeView_SurfaceLabel\n");
//    });
//    registerKeyFunciton(window, GLFW_KEY_4, [&]() {
//        displayMode_FreeView = DisplayMode_FreeView_ColoredLabel;
//        bNeedUpdateSurface = true;
//        printf("displayMode_FreeView: DisplayMode_FreeView_ColoredLabel\n");
//    });


    registerKeyFunciton(window, GLFW_KEY_0, [&]() {
        ResetMCBuffer();
        bNeedUpdateSurface = true;
    });
/// A step once
    registerKeyFunciton(window, GLFW_KEY_A, [&]() {
        processmode_ = STEPONCE;
    });
/// S continue
    registerKeyFunciton(window, GLFW_KEY_S, [&]() {
        processmode_ = CONTINUE;
    });

/// R RESET
    registerKeyFunciton(window, GLFW_KEY_R,
                        [&]() {
                            alignToglCam();
                            bNeedUpdate = true;
                            printf("RESET VIEWPOINT!\n");
                        });

/// C Show Grid
    registerKeyFunciton(window, GLFW_KEY_C,
                        [&]() {
                            if (!bShowGrid) {
                                printf("Show Grid On\n");
                                bShowGrid = true;
                            } else {
                                printf("Show Grid Off\n");
                                bShowGrid = false;
                            }
                        });
/// V Show Pose
    registerKeyFunciton(window, GLFW_KEY_V,
                        [&]() {
                            if (!bShowPose) {
                                printf("Show Pose On\n");
                                bShowPose = true;
                            } else {
                                printf("Show Pose Off\n");
                                bShowPose = false;
                            }
                        });

/// W Change Normal Sign
    registerKeyFunciton(window, GLFW_KEY_W, [&]() {
        if (glShaders["Vertices"]) {
            mc_normal_sign_ *= -1;
            glShaders["Vertices"]->use();
            glShaders["Vertices"]->set("sign", mc_normal_sign_);
            SCLOG(INFO) << "[SLAMGUI] Flip normal direction.\n";
            //bNeedUpdateSurface = true;
        }
    });


/// F Change Mode
    registerKeyFunciton(window, GLFW_KEY_F, [&]() {
        if (cameraMode_ == camera_PoseView) {
            cameraMode_ = camera_FreeView;
            bNeedUpdateSurface = true;
        } else if (cameraMode_ == camera_FreeView) {
            cameraMode_ = camera_PoseView;
        }
        bNeedUpdate = true;
        switch (cameraMode_){
            case camera_PoseView:
                DEBUG("cameraMode: camera_PoseView\n");
                break;
            case camera_FreeView:
                DEBUG("cameraMode: camera_FreeView\n");
                break;
        }
    });

/// J Save Map
    registerKeyFunciton(window, GLFW_KEY_J, [&]() {
        printf("Save map.\n");
        slam_->SaveToFile(outputPath);
    });

/// P Change check state
    registerKeyFunciton(window, GLFW_KEY_P, [&]() {
        bCheckState = !bCheckState;
        bNeedUpdateSurface=true;
        bNeedUpdate=true;
    });

/// I Save Mesh
    registerKeyFunciton(window, GLFW_KEY_I, [&]() {
        printf("Save mesh.\n");

//        slam_->saveSceneToMesh(outputPath,false,bCheckState);

        auto meshEngine = reinterpret_cast<SCFUSION::MeshEngine<ITMVoxel, ITMVoxelIndex>*>(slam_);
        if(meshEngine)
            meshEngine->saveSceneToMesh(slam_->getScene(), outputPath,false,bCheckState);
//        if(slam) {
//
//            slam->saveSceneToMesh( outputPath,false,bCheckState);
//        }
//        slam_->save
//        slam_->SaveToFile();
    });

/// K Record Image
    registerKeyFunciton(window, GLFW_KEY_K, [&]() {
        bRecordImage = !bRecordImage;
        if (bRecordImage)printf("Start Recording Images.\n");
        else printf("Stop Recording Images.\n");
    });

/// L Load Map
    registerKeyFunciton(window, GLFW_KEY_L, [&]() {
        printf("Load map.\n");
        slam_->LoadFromFile();
    });

/// Left Control
    registerKeyFunciton(window, GLFW_KEY_LEFT_CONTROL, [&]() {});

/// Lest Shift
    registerKeyFunciton(window, GLFW_KEY_LEFT_SHIFT, [&]() {

    });

/// T Follow GT Pose in FreeView Mode
    registerKeyFunciton(window, GLFW_KEY_T, [&]() {
        bFollowGTPose = !bFollowGTPose;
        if (bFollowGTPose) {
            printf("bFollowGTPose On.\n");
            bNeedUpdate = true;
        } else {
            printf("bFollowGTPose Off.\n");
//            alignToglCam(Block_freeview_pose_.get());
        }
    });
/// X Show FPS
    registerKeyFunciton(window, GLFW_KEY_X, [&]() { bShowFPS = !bShowFPS; });

}

template<class SLAMType>
void SLAMGUI<SLAMType>::run(){
    SC::GUI_base::run();
    printf("OutputFile...");
    printResultOnFile(logFile_);
    printf("Done\n");
}

template<class SLAMType>
void SLAMGUI<SLAMType>::drawGL() {
    processInput(window_->window);
    process_impl();
}

template<class SLAMType>
void SLAMGUI<SLAMType>::drawUI() {
    GUI_base::drawUI();
    DrawLeftPanel();
    if(bDrawBottomRightInfoPanel) DrawBottomRightOverlay();
    cameraUI();
    glCam->drawUI();
    bNeedUpdate |= mouseControl();
}

template<class SLAMType>
void SLAMGUI<SLAMType>::renderImgFromSLAM(SLAMType *slam) {
//    printf("Called\n");
    TICK("[SLAMGUI][renderReProjDepth]0.all");
    ORUtils::SE3Pose se3pose;
    if (bFollowGTPose){
        alignToglCam();
    }

    auto pose = glCam->getCameraPose();
    OpenGLReferenceToCameraRefence(&pose);
    se3pose = ORUtils::SE3Pose(ORUtils::Matrix4<float>(pose.data()));

    {
        int type = cameraMode_ == camera_PoseView ? displayMode_FollowView : displayMode_FreeView;
        slam->renderImage(Block_ImageBuffer_.get(), static_cast<SCFUSION::MainEngine<ITMVoxel,ITMVoxelIndex>::GetImageType>(type), &se3pose, &render_intrinsics,
                          ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE);
        slam->SyncVisualizationEngine();
        ITMUChar4Image tmpImage(*Block_ImageBuffer_);
        /// Flip Y
        auto* data_cpu = Block_ImageBuffer_->GetData(MEMORYDEVICE_CPU);
        const auto* inv_cpu  = tmpImage.GetDataConst(MEMORYDEVICE_CPU);
        for(int y=0; y < Block_ImageBuffer_->noDims.y; ++y){
            for(int x=0; x < Block_ImageBuffer_->noDims.x; ++x){
                int Y = Block_ImageBuffer_->noDims.y - 1 - y;
                data_cpu[y * Block_ImageBuffer_->noDims.x + x] = inv_cpu[Y * Block_ImageBuffer_->noDims.x + x];
            }
        }
    }
    mImageDrawer->Update((uchar *) Block_ImageBuffer_->GetData(MEMORYDEVICE_CPU),
                         Block_ImageBuffer_->noDims.width,
                         Block_ImageBuffer_->noDims.height);
    TOCK("[SLAMGUI][renderReProjDepth]0.all");
}

template<class SLAMType>
int SLAMGUI<SLAMType>::NeedUpdateSurfaceConditions(){
    switch (displayMode_mesh) {
        case DisplayMode_FreeView_Surface: {
            if(mc_thread_.valid()){
                if (mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
                    mc_thread_.get();
                    mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractGlobalMapSurface, this));
                    bNeedUpdateSurface = false;
                }
            } else {
                mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SLAMGUI::extractGlobalMapSurface, this));
            }
            break;
        }
        case DisplayMode_FreeView_SurfaceLabel: {
            if(mc_thread_.valid()){
                if (mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
                    mc_thread_.get();
                    mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractGlobalMapSurfaceLabel, this));
                    bNeedUpdateSurface = false;
                }
            } else {
                mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SLAMGUI::extractGlobalMapSurfaceLabel, this));
            }


//            if (!mc_thread_.valid()) {
//                mc_thread_ = std::async(std::launch::async,
//                                        std::bind(&SLAMGUI::extractGlobalMapSurfaceLabel, this));
//                bNeedUpdateSurface = false;
//            } else if (mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
//                mc_thread_ = std::async(std::launch::async,
//                                        std::bind(&SLAMGUI::extractGlobalMapSurfaceLabel, this));
//                bNeedUpdateSurface = false;
//            }
            break;
        }
        case DisplayMode_FreeView_ColoredLabel: {
            if(mc_thread_.valid()){
                if (mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
                    mc_thread_.get();
                    mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractPoints, this));
                    bNeedUpdateSurface = false;
                }
            } else {
                mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SLAMGUI::extractPoints, this));
            }
//            if (!mc_thread_.valid()) {
//                mc_thread_ = std::async(std::launch::async, std::bind(&SLAMGUI::extractPoints, this));
//                bNeedUpdateSurface = false;
//            } else if (mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
//                mc_thread_ = std::async(std::launch::async, std::bind(&SLAMGUI::extractPoints, this));
//                bNeedUpdateSurface = false;
//            }
            break;
        }
        case DisplayMode_FreeView_Rendered_Height:
            break;
    }
    return 1;
}

template<class SLAMType>
inline bool SLAMGUI<SLAMType>::process_slam(){
    switch (processmode_) {
        case STOP:
            return false;
        case STEPONCE: {
            processmode_ = STOP;
        }
        case CONTINUE:
            TICK("[SLAMGUI][process_impl]1.ProcessSLAM");
            if(slamWarpper_->processOnce() >= 0){
//                image2cvMat(slamWarpper_->getDepthImage(), "depth");
                if(slamWarpper_->getImgCounter()==1) {
//                    Block_freeview_pose_->setValues(slamWarpper_->getPose()->m);
//                    alignToglCam(Block_freeview_pose_.get());
                }

                ORUtils::Vector4<float> trans = slamWarpper_->getPose()->getColumn(3);
                add_trajectory(trans.x,trans.y,trans.z);
                bNeedUpdate = true;
            } else {
                processmode_ = STOP;
            }
            if (cameraMode_ == camera_FreeView) bNeedUpdateSurface = true;
            TOCK("[SLAMGUI][process_impl]1.ProcessSLAM");
            return true;
    }
}

template<class SLAMType>
void SLAMGUI<SLAMType>::process_impl() {
    TICK("[SLAMGUI][process_impl]0.All");
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    fps_->updateFPS();

    ///SLAM
    if(process_slam()){
        bNeedUpdateSurface = bRenderMesh;
//        bNeedUpdate=true;
    }

#if 1
    if (bNeedUpdateSurface) NeedUpdateSurfaceConditions();

/// Render Image From Volume
//OPT: combine this with bNeedUpdateSurface into different case
    if (bNeedUpdate) {
//        DEBUG("[SLAMGUI][process_impl]UpdateImage\n");
        TICK("[SLAMGUI][process_impl]2.UpdateImage");
        bNeedUpdate = !bNeedUpdate;

        if(!bRenderMesh) renderImgFromSLAM(slam_);

        if (bRecordImage) recordImg();
        TOCK("[SLAMGUI][process_impl]2.UpdateImage");
    }

    glm::vec4 lightColor(1.0, 1.0, 1.0, 1);

    glm::mat4 projection = glCam->projection_control_->projection_matrix();

    /// Draw Image
    TICK("[SLAMGUI][process_impl]3.DrawImages");
    if (!bRenderMesh) {
        mImageDrawer->Draw();
    } else {
        if(true){
            glUtil::Shader *shader = nullptr;
            if(slam_->getPointCloud() != nullptr) {
                if (slam_->getPointCloud()->noTotalPoints && displayMode_mesh == DisplayMode_FreeView_ColoredLabel) {
                    shader = glShaders.at("Points");
                    glm::mat4 modelMat = glm::mat4(1.f);
                    shader->use();
                    shader->set("lightColor", lightColor);
                    shader->set("lightPos", glCam->camera_control_->Position /*glm::vec3(0,2,0)*/);
                    shader->set("model", modelMat);
                    shader->set("projection", projection);
                    shader->set("view", glCam->camera_control_->GetViewMatrix());
                    shader->set("sign", mc_normal_sign_);
                    drawPointCloud(shader);
                }
            }
            if (slam_->getMesh() != nullptr) {
                if (slam_->getMesh()->noTotalTriangles && (displayMode_mesh == DisplayMode_FreeView_Surface ||
                        displayMode_mesh == DisplayMode_FreeView_SurfaceLabel)) {
                    shader = glShaders.at("Vertices");
                    glm::mat4 modelMat = glm::mat4(1.f);
                    shader->use();
                    shader->set("lightColor", lightColor);
                    shader->set("lightPos", glCam->camera_control_->Position /*glm::vec3(0,2,0)*/);
                    shader->set("model", modelMat);
                    shader->set("projection", projection);
                    shader->set("view", glCam->camera_control_->GetViewMatrix());
                    shader->set("sign", mc_normal_sign_);
                    drawMesh(shader);
                }
            }
        } else {
            /// Draw shadow. Not working
//            glm::vec3 lightPos(2.0f, 4.0f, 2.0f);
//            glm::mat4 lightProjection, lightView;
//            glm::mat4 lightSpaceMatrix;
//            float near_plane = 1.0f, far_plane = 7.5f;
//            lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);
//            lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
//            lightSpaceMatrix = lightProjection * lightView;
//
//            glShaders["Shadow"]->use();
//            glShaders["Shadow"]->set("lightSpaceMatrix", lightSpaceMatrix);
//            glShaders["Shadow"]->set("model", glm::mat4(1.f));
//
//            const unsigned int &depthMapFBO = glFrameBuffers["DepthMapFBO"];
//            // 1. first render to depth map
//            glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
//            glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
//            glClear(GL_DEPTH_BUFFER_BIT);
//            DrawVertices(glShaders["Shadow"]);
//            glBindFramebuffer(GL_FRAMEBUFFER, 0);
//
//            // 2. then render scene as normal with shadow mapping (using depth map)
//            glViewport(0, 0, window_->runtimeWidth, window_->runtimeHeight);
//            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//            glBindTexture(GL_TEXTURE_2D, glTextures["depthMap"]);
//
//            if (mc_vertices_size) {
//                if (cameraMode_ == camera_FreeView && displayMode_mesh != DisplayMode_FreeView_Rendered_Height) {
//                    glUtil::Shader *shader = NULL;
//                    if (displayMode_mesh == DisplayMode_FreeView_ColoredLabel)
//                        shader = glShaders["Points"];
//                    else
//                        shader = glShaders["Vertices"];
//                    glm::mat4 modelMat = glm::mat4(1.f);
//                    shader->use();
//                    shader->set("lightColor", lightColor);
//                    shader->set("lightPos", glCam->camera_control_->Position /*glm::vec3(0,2,0)*/);
//                    shader->set("model", modelMat);
//                    shader->set("projection", projection);
//                    shader->set("view", glCam->camera_control_->GetViewMatrix());
////                if (displayMode_mesh == DisplayMode_FreeView_Surface || displayMode_mesh == DisplayMode_FreeView_BufferSurface)
//                    shader->set("sign", mc_normal_sign_);
//                    drawMesh(shader);
//                }
//            }
        }
    }

    glDisable(GL_DEPTH_TEST);
    SLAMGUI<SLAMType>::renderCamera();
    glEnable(GL_DEPTH_TEST);



/// TEST SCREEN
    if (0) {
        glm::mat4 modelMat = glm::mat4(1.f);
        float scale = 0.1f;
        modelMat = glm::scale(modelMat,
                              glm::vec3(scale, scale, scale));    // it's a bit too big for our scene, so scale it down
        auto *shader = glShaders["Screen3D"];
        shader->use();
        shader->set("projection", projection);
        shader->set("view", glCam->camera_control_->GetViewMatrix());
        shader->set("model", modelMat);
        glBindVertexArray(glVertexArrays["quadVAO"]);
//            glBindTexture(GL_TEXTURE_2D, glBuffers["textureColorbuffer"]);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    TOCK("[SLAMGUI][process_impl]3.DrawImages");

    TOCK("[SLAMGUI][process_impl]0.All");
    /// Save Time Information
    for (const std::pair<std::string, double> &time : getWatch.getTimings()) {
        if (!getWatch.updated(time.first)) continue;
        getWatch.getUpdateStats()[time.first] = false;

        if (times_.find(time.first) == times_.end()) {
            if (slamWarpper_->getImgCounterMax() > 0)
                times_[time.first].reserve(slamWarpper_->getImgCounterMax());
            else
                times_[time.first].reserve(10000); // assume will have this amount of images
            times_[time.first].push_back(time.second);
        } else {
            times_[time.first].push_back(time.second);
        }
    }
#endif
    GUI3D::basicProcess();
}
template<class SLAMType>
void SLAMGUI<SLAMType>::renderCamera(){
    /// Draw Camera
    if (cameraMode_ == camera_FreeView && bShowPose) {
        glm::mat4 modelMat = glm::make_mat4(slamWarpper_->getPose()->m);
        static float scale = 0.2f;
        modelMat = glm::scale(modelMat,
                              glm::vec3(scale, scale, scale));    // it's a bit too big for our scene, so scale it down

        auto *shader = glShaders["Camera"];
        shader->use();
        shader->set("projection", glCam->projection_control_->projection_matrix());
        shader->set("view", glCam->camera_control_->GetViewMatrix());
        shader->set("model", modelMat);
        glBindVertexArray(glVertexArrays["Camera"]);
        glLineWidth(3);
        glDrawElements(GL_LINES, 28, GL_UNSIGNED_INT, 0);
        glLineWidth(1);
    }
}

template<class SLAMType>
void SLAMGUI<SLAMType>::calculateStatisticalTimes(std::ostream &os ) {
    double mean, var, stdvar;
    for (auto &time : times_) {
        mean = var = stdvar = 0;
        if (time.second.size() == 1) {
            mean = time.second[0];
            var = stdvar = 0;
        } else {
            mean = std::accumulate(time.second.begin(), time.second.end(), 0.f) / time.second.size();
            var = std::accumulate(time.second.begin(), time.second.end(), 0.f, [mean](double sum, int i) {
                auto d = i - mean;
                return sum + d * d;
            });
            var /= time.second.size() - 1;
            stdvar = std::sqrt(var);
        }

        os << "---" << time.first << "---\n"
           << "samples : " << time.second.size()
           << " Mean : " << mean
           << " Variance : " << var
           << " Standard Deviation : " << stdvar << std::endl;
    }
}

template<class SLAMType>
void SLAMGUI<SLAMType>::printResultOnFile(std::ostream &os) {
    os << "Name;Mean;Variance;Standard Deviation;Number of Measurements" << ";\n";
    double mean, var, stdvar;
    for (auto &time : times_) {
        mean = var = stdvar = 0;
        if (time.second.size() == 1) {
            mean = time.second[0];
            var = stdvar = 0;
        } else {
            mean = std::accumulate(time.second.begin(), time.second.end(), 0.f) / time.second.size();
            var = std::accumulate(time.second.begin(), time.second.end(), 0.f, [mean](double sum, int i) {
                auto d = i - mean;
                return sum + d * d;
            });
            var /= time.second.size() - 1;
            stdvar = std::sqrt(var);
        }

        os << time.first << ";" << std::to_string(mean) << ";" << std::to_string(var) << ";" << std::to_string(stdvar)
           << ";" << std::to_string(time.second.size()) << "\n";
    }
}

template<class SLAMType>
void SLAMGUI<SLAMType>::recordImg() {
//    std::cout << window_->runtimeWidth << ", " << window_->runtimeHeight << std::endl;
    glfwGetFramebufferSize(window_->window, &window_->runtimeWidth, &window_->runtimeHeight);
//    std::cout << window_->runtimeWidth << ", " << window_->runtimeHeight << std::endl;

//        if(cvColorImage.cols != window_->runtimeWidth || cvColorImage.rows != window_->runtimeHeight || cvColorImage.empty()){
//            cvColorImage.release();
//
//        }
    cvColorImage.create(window_->runtimeHeight, window_->runtimeWidth, CV_8UC4);
    glReadBuffer( GL_FRONT );
    glReadPixels(0, 0, window_->runtimeWidth, window_->runtimeHeight, GL_RGBA, GL_UNSIGNED_BYTE,
                 cvColorImage.data);
    cv::flip(cvColorImage, cvColorImage, 0);
//    std::cout << cvColorImage << "\n";
    cv::cvtColor(cvColorImage, cvColorImage, cv::COLOR_RGBA2BGRA);

    char name[pth_to_image_folder.length() + 100];
    sprintf(name, (pth_to_image_folder + "/color%04d.png").c_str(), iterSave);
    cv::imwrite(name, cvColorImage);
//    printf("Image saved to %s\n", name);
    iterSave++;
}
#if 0
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}


cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        bank = 0;
        attitude = CV_PI/2;
        heading = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        bank = 0;
        attitude = -CV_PI/2;
        heading = atan2(m02,m22);
    }
    else
    {
        bank = atan2(-m12,m11);
        attitude = asin(m10);
        heading = atan2(-m20,m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}
// Converts a given Euler angles to Rotation Matrix
// Convention used is Y-Z-X Tait-Bryan angles
// Reference:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
cv::Mat euler2rot(const cv::Mat & euler)
{
    cv::Mat rotationMatrix(3,3,CV_64F);

    double bank = euler.at<double>(0);
    double attitude = euler.at<double>(1);
    double heading = euler.at<double>(2);

    // Assuming the angles are in radians.
    double ch = cos(heading);
    double sh = sin(heading);
    double ca = cos(attitude);
    double sa = sin(attitude);
    double cb = cos(bank);
    double sb = sin(bank);

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh*sb - ch*sa*cb;
    m02 = ch*sa*sb + sh*cb;
    m10 = sa;
    m11 = ca*cb;
    m12 = -ca*sb;
    m20 = -sh*ca;
    m21 = sh*sa*cb + ch*sb;
    m22 = -sh*sa*sb + ch*cb;

    rotationMatrix.at<double>(0,0) = m00;
    rotationMatrix.at<double>(0,1) = m01;
    rotationMatrix.at<double>(0,2) = m02;
    rotationMatrix.at<double>(1,0) = m10;
    rotationMatrix.at<double>(1,1) = m11;
    rotationMatrix.at<double>(1,2) = m12;
    rotationMatrix.at<double>(2,0) = m20;
    rotationMatrix.at<double>(2,1) = m21;
    rotationMatrix.at<double>(2,2) = m22;

    return rotationMatrix;
}

void fromCV2GLM(const cv::Mat& cvmat, glm::mat4* glmmat) {
    if (cvmat.cols != 4 || cvmat.rows != 4 || cvmat.type() != CV_32FC1) {
        std::cout << "Matrix conversion error!" << std::endl;
        return;
    }
    memcpy(glm::value_ptr(*glmmat), cvmat.data, 16 * sizeof(float));
}

void fromGLM2CV(const glm::mat4& glmmat, cv::Mat* cvmat) {
    if (cvmat->cols != 4 || cvmat->rows != 4) {
        (*cvmat) = cv::Mat(4, 4, CV_32F);
    }
    memcpy(cvmat->data, glm::value_ptr(glmmat), 16 * sizeof(float));
}
#endif

template<class SLAMType>
void SLAMGUI<SLAMType>::alignToglCam() {
    auto modelPose = Eigen::Matrix4f(slamWarpper_->getPose()->m);
    Eigen::Vector3f modelPosition = modelPose.topRightCorner<3,1>();
    CameraRefenceToOpenGLReference(&modelPose);
    Eigen::Vector3f euler_ori = modelPose.topLeftCorner<3, 3>().eulerAngles(0, 1, 2);
    if(euler_ori(0) < EIGEN_PI/2) {
        if (euler_ori(1) > 0)
            euler_ori(1) = EIGEN_PI - euler_ori(1);
        else
            euler_ori(1) = -EIGEN_PI - euler_ori(1);
    } else {
        euler_ori(0) = -(EIGEN_PI - euler_ori(0));
    }

    euler_ori *= 180.f / EIGEN_PI;
    glCam->camera_control_->setCamPose(euler_ori[0], euler_ori[1], euler_ori[2],
                                       modelPosition(0), modelPosition(1),modelPosition(2));
}

template<class SLAMType>
void SLAMGUI<SLAMType>::extractGlobalMapSurface() {
    if(!slam_->getITMSetting()->createMeshingEngine){
        printf("Did not enable meshing!\n");
        return;
    }
    TICK("[SLAMGUI][extractSurface]0.all");
    slam_->computeMesh(false, bCheckState);
    TOCK_P("[SLAMGUI][extractSurface]0.all");
    DEBUG("mesh->noTotalTriangles: %d\n", slam_->getMesh()->noTotalTriangles);
    TICK("[SLAMGUI][extractGlobalMapSurface]1.copy");
    if (slam_->getMesh()->noTotalTriangles){
        copyMeshToGUI(slam_);
    }
    TOCK_P("[SLAMGUI][extractGlobalMapSurface]1.copy");
}

template<class SLAMType>
void SLAMGUI<SLAMType>::extractGlobalMapSurfaceLabel() {
    if(!slam_->getITMSetting()->createMeshingEngine){
        printf("Did not enable meshing!\n");
        return;
    }
    TICK("[SLAMGUI][extractGlobalMapSurfaceLabel]0.all");
    slam_->computeMesh(true, false);
    TOCK_P("[SLAMGUI][extractGlobalMapSurfaceLabel]0.all");
    if (slam_->getMesh()->noTotalTriangles) {
        copyMeshToGUI(slam_);
    }
    DEBUG("mesh->noTotalTriangles: %d\n", slam_->getMesh()->noTotalTriangles);
}

template<class SLAMType>
void SLAMGUI<SLAMType>::extractPoints() {
    if(!slam_->getITMSetting()->createPointExtractionEngine){
        printf("Did not enable point cloud engine!\n");
        return;
    }
    TICK("[SLAMGUI][extractPointCloud]0.all");
    slam_->computePointCloud();

    TICK("[SLAMGUI][process_impl]3.1 Draw PointCloud");
    if (slam_->getPointCloud()->noTotalPoints) {
        points_location->SetFrom(slam_->getPointCloud()->points.get(), ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CPU,
                                 cuda_gui_streams_.getStream("MC_triangle"));
        points_color->SetFrom(slam_->getPointCloud()->colors.get(), ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CPU,
                              cuda_gui_streams_.getStream("MC_color"));
    }
    TOCK_IP("[SLAMGUI][process_impl]3.1 Draw PointCloud", 1);
    TOCK_P("[SLAMGUI][extractPointCloud]0.all");
}

template<class SLAMType>
void SLAMGUI<SLAMType>::copyMeshToGUI(SLAMType *slam){
    auto mesh = slam->getMesh();
    std::unique_lock<std::mutex> lock(mutex_mc_);
    if(slam->getITMSetting()->GetMemoryType() == MEMORYDEVICE_CUDA) {
        mc_triangles->SetFrom(mesh->triangles, ORUtils::MemoryBlock<ITMLib::ITMMesh::Triangle>::CUDA_TO_CPU,
                              cuda_gui_streams_.getStream("MC_triangle"));
        if (mesh->hasColor)
            mc_colors->SetFrom(mesh->colors, ORUtils::MemoryBlock<ITMLib::ITMMesh::Color>::CUDA_TO_CPU,
                               cuda_gui_streams_.getStream("MC_color"));
        if (mesh->hasNormal)
            mc_normals->SetFrom(mesh->normals, ORUtils::MemoryBlock<ITMLib::ITMMesh::Normal>::CUDA_TO_CPU,
                                cuda_gui_streams_.getStream("MC_normal"));
    } else {
        mc_triangles->SetFrom(mesh->triangles, ORUtils::MemoryBlock<ITMLib::ITMMesh::Triangle>::CPU_TO_CPU,
                              cuda_gui_streams_.getStream("MC_triangle"));
        if (mesh->hasColor)
            mc_colors->SetFrom(mesh->colors, ORUtils::MemoryBlock<ITMLib::ITMMesh::Color>::CPU_TO_CPU,
                               cuda_gui_streams_.getStream("MC_color"));
        if (mesh->hasNormal)
            mc_normals->SetFrom(mesh->normals, ORUtils::MemoryBlock<ITMLib::ITMMesh::Normal>::CPU_TO_CPU,
                                cuda_gui_streams_.getStream("MC_normal"));
    }
}

template<class SLAMType>
void SLAMGUI<SLAMType>::OpenGLReferenceToCameraRefence(ORUtils::Matrix4<float> *modelPose) {
    auto pose = getEigen4f(modelPose->m);
    OpenGLReferenceToCameraRefence(&pose);
}

template<class SLAMType>
void SLAMGUI<SLAMType>::OpenGLReferenceToCameraRefence(Eigen::Matrix4f *modelPose){
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(1 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    *modelPose =  transferMat * *modelPose;
}

template<class SLAMType>
void SLAMGUI<SLAMType>::OpenGLReferenceToCameraRefence(Eigen::Map<Eigen::Matrix4f> *modelPose){
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(1 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    *modelPose =  transferMat * *modelPose;
}

template<class SLAMType>
void SLAMGUI<SLAMType>::CameraRefenceToOpenGLReference(Eigen::Matrix4f *modelPose) {
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(1 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    *modelPose =  transferMat * *modelPose;

}


template<class SLAMType>
void SLAMGUI<SLAMType>::DrawLeftPanel(){
    auto& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300,0.8 * io.DisplaySize.y), ImGuiCond_Always);
    ImGui::Begin("Control Panel",nullptr, /*ImGuiWindowFlags_NoTitleBar | */ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar);
    // menu bar
    if(ImGui::BeginMenuBar()){
        if(ImGui::BeginMenu("System")){
            ImGui::EndMenu();
        };
        if(ImGui::BeginMenu("UI")) {
            ImGui::MenuItem("Show Demo UI", nullptr, &bDrawDemoUI);
            ImGui::MenuItem("Show Camera UI", nullptr, &glCam->bShowUI);
            ImGui::MenuItem("Show Info Panel", nullptr, &bDrawBottomRightInfoPanel);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
    if(ImGui::Button("Stop")) processmode_ = ProcessMode::STOP;
    ImGui::SameLine();
    if(ImGui::Button("Step")) processmode_ = ProcessMode::STEPONCE;
    ImGui::SameLine();
    if(ImGui::Button("Continue")) processmode_ = ProcessMode::CONTINUE;
    ImGui::Separator();

    {
        static int renderMode = 0;
        const char* desc[] =
                {
                        "Follow camera",
                        "Free camera",
                };
        bool updated = ImGui::Combo("Render Mode", &renderMode, desc, IM_ARRAYSIZE(desc));
        if(updated) {
            if(renderMode == 0) cameraMode_ = camera_PoseView;
            if(renderMode == 1) cameraMode_ = camera_FreeView;
            bNeedUpdate=true;
        }
    }
    {
        const char* followModes[] ={
                "RGB",
                "Depth",
                "Scene raycast",
                "Color from volume",
                "Normal",
                "Confidence",
                "Label from image normal",
                "Label from volume",
                };
        static int followCameraMode=0;
        bool updated = ImGui::Combo("Follow camera mode", &followCameraMode, followModes, IM_ARRAYSIZE(followModes));
        if(updated) {
            if (followCameraMode == 0) displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
            if (followCameraMode == 1) displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
            if (followCameraMode == 2) displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
            if (followCameraMode == 3)
                displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME;
            if (followCameraMode == 4)
                displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL;
            if (followCameraMode == 5)
                displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE;
            if (followCameraMode == 6)
                displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL;
            if (followCameraMode == 7)
                displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_LABEL_FROM_NORMAL;
            bNeedUpdate = true;
        }
    }
    {
        const char* freeModes[] = {
            "Shaded",
            "Color from volume",
            "Normal",
            "Confidence",
            "Label from image normal",
            "Label from normal"
        };
        static int freeCameraMode=0;
        bool updated = ImGui::Combo("Free camera mode", &freeCameraMode, freeModes, IM_ARRAYSIZE(freeModes));
        if(updated) {
            if (freeCameraMode == 0) displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
            if (freeCameraMode == 1)
                displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME;
            if (freeCameraMode == 2)
                displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL;
            if (freeCameraMode == 3)
                displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE;
            if (freeCameraMode == 4)
                displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_IMAGENORMAL;
            if (freeCameraMode == 5)
                displayMode_FreeView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL;
            bNeedUpdate = true;
        }
    }

    bNeedUpdateSurface |= ImGui::Checkbox("Render Mesh", &bRenderMesh);
    {
        const char *meshModes[] = {
                "Surface",
                "Label",
                "Pointcloud",
        };
        static int meshMode = 0;
        bool updated = ImGui::Combo("Render Mesh mode", &meshMode, meshModes, IM_ARRAYSIZE(meshModes));
        if(updated){
            if(meshMode == 0) displayMode_mesh = DisplayMode::DisplayMode_FreeView_Surface;
            if(meshMode == 1) displayMode_mesh = DisplayMode::DisplayMode_FreeView_SurfaceLabel;
            if(meshMode == 2) displayMode_mesh = DisplayMode::DisplayMode_FreeView_ColoredLabel;
            bNeedUpdateSurface = true;
        }
    }

    ImGui::Separator();
    if(ImGui::TreeNode("Draw Options")){
        ImGui::Checkbox("Grid", &bShowGrid);
        ImGui::Checkbox("Camera", &bShowPose);
        ImGui::Checkbox("Trajectory", &bPlotTrajectory);
        ImGui::Checkbox("FPS", &bShowFPS);

        ImGui::TreePop();
    }

    ImGui::End();
}

template<class SLAMType>
void SLAMGUI<SLAMType>::DrawMiddlePanel(){

}

template<class SLAMType>
void SLAMGUI<SLAMType>::DrawTopRightPanel(){

}

template<class SLAMType>
void SLAMGUI<SLAMType>::DrawBottomRightPanel(){

}

template<class SLAMType>
void SLAMGUI<SLAMType>::DrawBottomRightOverlay(){
    const float DISTANCE = 10.0f;
    bool p_open = true;
    static int corner = 3;
    ImGuiIO& io = ImGui::GetIO();
    if (corner != -1)
    {
        ImVec2 window_pos = ImVec2((corner & 1) ? io.DisplaySize.x - DISTANCE : DISTANCE, (corner & 2) ? io.DisplaySize.y - DISTANCE : DISTANCE);
        ImVec2 window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
    }
    ImGui::SetNextWindowBgAlpha(0.35f); // Transparent background
    if (ImGui::Begin("System Info", &p_open, (corner != -1 ? ImGuiWindowFlags_NoMove : 0) | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav))
    {
        ImGui::Text("System Info");
        ImGui::Separator();
        if (ImGui::IsMousePosValid()) {
            ImGui::Text("Mouse Position: (%.1f,%.1f)", io.MousePos.x, io.MousePos.y);

            unsigned char pixelColor[4];
            glReadPixels(io.MousePos.x, io.DisplaySize.y - io.MousePos.y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, &pixelColor);
            ImGui::Text("Color: %d %d %d %d", (int)pixelColor[0],(int)pixelColor[1],(int)pixelColor[2],(int)pixelColor[3]);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        } else
            ImGui::Text("Mouse Position: <invalid>");

        if (ImGui::BeginPopupContextWindow())
        {
            if (ImGui::MenuItem("Custom",       NULL, corner == -1)) corner = -1;
            if (ImGui::MenuItem("Top-left",     NULL, corner == 0)) corner = 0;
            if (ImGui::MenuItem("Top-right",    NULL, corner == 1)) corner = 1;
            if (ImGui::MenuItem("Bottom-left",  NULL, corner == 2)) corner = 2;
            if (ImGui::MenuItem("Bottom-right", NULL, corner == 3)) corner = 3;
            if (p_open && ImGui::MenuItem("Close")) p_open = false;
            ImGui::EndPopup();
        }
    }
    ImGui::End();
}

template<class SLAMType>
void SLAMGUI<SLAMType>::drawMesh(glUtil::Shader *shader) {
//    ResetMCBuffer();
    auto mesh = slam_->getMesh();
    uint totalTriangles = mesh->noTotalTriangles * 3;
    updateVertexBuffer(totalTriangles);

    const std::string name = "Vertices";
    //OPT: add texture
    glBindVertexArray(glVertexArrays[name]);
    glBindBuffer(GL_ARRAY_BUFFER, glBuffers[name]);
    cuda_gui_streams_.syncStream("MC_triangle");
    cuda_gui_streams_.syncStream("MC_color");
    cuda_gui_streams_.syncStream("MC_normal");

    {
        std::unique_lock<std::mutex> lock(mutex_mc_);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        totalTriangles * 4 * sizeof(GLfloat), mc_triangles->GetData(MEMORYDEVICE_CPU));
        glBufferSubData(GL_ARRAY_BUFFER, mc_total_size * 4 * sizeof(GLfloat),
                        totalTriangles * 4 * sizeof(GLfloat), mc_normals->GetData(MEMORYDEVICE_CPU));
        if (ITMVoxel::hasLabelInformation)
            glBufferSubData(GL_ARRAY_BUFFER, mc_total_size * 8 * sizeof(GLfloat),
                            totalTriangles * 4 * sizeof(GLfloat), mc_colors->GetData(MEMORYDEVICE_CPU));

        if (displayMode_mesh != DisplayMode_FreeView_ColoredLabel)
            glDrawArrays(GL_TRIANGLES, 0, totalTriangles);
        else if (displayMode_mesh == DisplayMode_FreeView_ColoredLabel)
            glDrawArrays(GL_POINTS, 0, totalTriangles);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

template <class SLAMType>
void SLAMGUI<SLAMType>::drawPointCloud(glUtil::Shader *shader) {
//    ResetMCBuffer();
    auto pointcloud = slam_->getPointCloud();
    uint totalPoints = pointcloud->noTotalPoints;
    updateVertexBuffer(totalPoints);

    const std::string name = "Vertices";
    //OPT: add texture
    glBindVertexArray(glVertexArrays[name]);
    glBindBuffer(GL_ARRAY_BUFFER, glBuffers[name]);
    cuda_gui_streams_.syncStream("MC_triangle");
    cuda_gui_streams_.syncStream("MC_color");

    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    totalPoints * 4 * sizeof(GLfloat), points_location->GetData(MEMORYDEVICE_CPU));
    glBufferSubData(GL_ARRAY_BUFFER, mc_total_size * 4 * sizeof(GLfloat),
                    totalPoints * 4 * sizeof(GLfloat), points_color->GetData(MEMORYDEVICE_CPU));

    glDrawArrays(GL_POINTS, 0, totalPoints);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

template <class SLAMType>
void SLAMGUI<SLAMType>::ResetMCBuffer() {
    if (slam_->getITMSetting()->createMeshingEngine) {
        slam_->getMesh()->noTotalTriangles = 0;
        slam_->getMesh()->triangles->Clear(0);
        if (slam_->getMesh()->hasNormal)slam_->getMesh()->normals->Clear(0);
        if (slam_->getMesh()->hasColor)slam_->getMesh()->colors->Clear(0);
    }
    if (slam_->getITMSetting()->createPointExtractionEngine) {
        slam_->getPointCloud()->noTotalPoints = 0;
        slam_->getPointCloud()->points->Clear(0);
        slam_->getPointCloud()->colors->Clear(0);
    }
}
#include "GUI_SurfelSLAM.hpp"
#include <CxxTools/PathTool.hpp>
#include "../../Utilities/include/Utilities/EigenHelper.h"
using namespace SCFUSION;


template<typename SLAMType>
SurfelSLAMGUI<SLAMType>::SurfelSLAMGUI(SLAMWrapper<SLAMType> *slamWarpper, std::string outputPath):
GUI3D("SLAM", 1080, 720), iterSave(0), slam_getImageType(0), outputPath_(std::move(outputPath)), slamWarpper_(slamWarpper) {
    bNeedUpdate = bRecordImage = false;
    bShowPose = true;
    processmode_ = STOP;
    slam_getImageType = 0;
    slam_ = slamWarpper_->getSLAM();
    camDepthParam = slamWarpper_->getCalibParam()->intrinsics_d;
    camColorParam = slamWarpper_->getCalibParam()->intrinsics_rgb;
    render_intrinsics.SetFrom(camDepthParam.imgSize.width, camDepthParam.imgSize.height, camDepthParam.projectionParamsSimple.fx, camDepthParam.projectionParamsSimple.fy,
                              camDepthParam.projectionParamsSimple.px, camDepthParam.projectionParamsSimple.py);
    glImageRenderer.reset(new glUtil::GLImageRenderer(nullptr,camDepthParam.imgSize.width, camDepthParam.imgSize.height));
    Block_depth2RGB_.reset(
            new ITMUChar4Image(ORUtils::Vector2<int>(camDepthParam.imgSize.width, camDepthParam.imgSize.height), true, true));
    /// Create Folder to save images
    {
        /// Create Folder to save images
        pth_to_image_folder = outputPath_ + "_images";
        tools::PathTool::check_and_delete_folder(pth_to_image_folder);
        tools::PathTool::check_and_create_folder(pth_to_image_folder);
    }
}

template<typename SLAMType>
SurfelSLAMGUI<SLAMType>::~SurfelSLAMGUI() {

}

template<typename SLAMType>
int SurfelSLAMGUI<SLAMType>::initialization() {
    registerKeys(window_);
    return 0;
}


template <class SLAMType>
void SurfelSLAMGUI<SLAMType>::registerKeys(SC::GLFWWindowContainer *window) {
    registerKeyFunciton(window, GLFW_KEY_1, [&]() {
        slam_getImageType++;
        bNeedUpdate = true;

        if (//slam_getImageType < SCFusion::SLAM::InfiniTAM_IMAGE_FREECAMERA_SHADED ||
                slam_getImageType == ITMLib::ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN + 1)
            slam_getImageType = 0;//SCFusion::SLAM::InfiniTAM_IMAGE_FREECAMERA_SHADED;
        printf("displayMode_FreeView: %s\n",
               ITMLib::ITMMainEngine::getGetImageTypeString(slam_getImageType).c_str());
    });

/// A step once
    registerKeyFunciton(window, GLFW_KEY_A, [&]() {
        processmode_ = STEPONCE;
    });
/// S continue
    registerKeyFunciton(window, GLFW_KEY_S, [&]() {
        processmode_ = CONTINUE;
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

/// J Save Map
    registerKeyFunciton(window, GLFW_KEY_J, [&]() {
        printf("Save map.\n");
        slam_->SaveToFile(outputPath);
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

/// X Show FPS
    registerKeyFunciton(window, GLFW_KEY_X, [&]() { bShowFPS = !bShowFPS; });

}


template<typename SLAMType>
void SurfelSLAMGUI<SLAMType>::drawUI() {
    GUI_base::drawUI();
    cameraUI();
    glCam->drawUI();
    bNeedUpdate |= mouseControl();
}

template<typename SLAMType>
void SurfelSLAMGUI<SLAMType>::drawGL() {
    processInput(window_->window);
    process_impl();
}


template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::process_impl() {
    TICK("[SLAMGUI][process_impl]0.All");
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    fps_->updateFPS();

    ///SLAM
    process_slam();

#if 1
/// Render Image From Volume
//OPT: combine this with bNeedUpdateSurface into different case
    if (bNeedUpdate) {
//        DEBUG("[SLAMGUI][process_impl]UpdateImage\n");
        TICK("[SLAMGUI][process_impl]2.UpdateImage");
        bNeedUpdate = !bNeedUpdate;

        renderImgFromSLAM(slam_);
        if (bRecordImage)
            recordImg();
        TOCK("[SLAMGUI][process_impl]2.UpdateImage");
    }

// make sure we clear the framebuffer's content
//glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glm::vec4 lightColor(1.0, 1.0, 1.0, 1);

    glm::mat4 projection = glCam->projection_control_->projection_matrix();

    /// Draw Image
    TICK("[SLAMGUI][process_impl]3.DrawImages");
    glImageRenderer->draw(glShaders["Screen"]);
    TOCK("[SLAMGUI][process_impl]3.DrawImages");

    glDisable(GL_DEPTH_TEST);
    renderCamera();
    glEnable(GL_DEPTH_TEST);

    TOCK("[SLAMGUI][process_impl]0.All");
    /// Save Time Information
    for (const auto &time : getWatch.getTimings()) {
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
inline void SurfelSLAMGUI<SLAMType>::process_slam(){
    switch (processmode_) {
        case STOP:
            break;
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
                add_trajectory(trans.x,trans.y,trans.z,0.001);
                bNeedUpdate = true;
            } else {
                processmode_ = STOP;
            }
            TOCK("[SLAMGUI][process_impl]1.ProcessSLAM");
            break;
    }
}

template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::recordImg() {
    glfwGetFramebufferSize(window_->window, &window_->runtimeWidth, &window_->runtimeHeight);
    cvColorImage.create(window_->runtimeHeight, window_->runtimeWidth, CV_8UC3);
    glReadBuffer( GL_FRONT );
    glReadPixels(0, 0, window_->runtimeWidth, window_->runtimeHeight, GL_RGB, GL_UNSIGNED_BYTE,
                 cvColorImage.data);
    cv::flip(cvColorImage, cvColorImage, 0);
    cv::cvtColor(cvColorImage, cvColorImage, cv::COLOR_RGBA2BGRA);

    char name[pth_to_image_folder.length() + 100];
    sprintf(name, (pth_to_image_folder + "/color%04d.png").c_str(), iterSave);
    cv::imwrite(name, cvColorImage);
    iterSave++;
}

template<typename SLAMType>
void SurfelSLAMGUI<SLAMType>::renderImgFromSLAM(SLAMType *slam) {
    TICK("[SLAMGUI][renderReProjDepth]0.all");
    ORUtils::SE3Pose se3pose;
    auto pose = glCam->getCameraPose();
    OpenGLReferenceToCameraRefence(&pose);
    se3pose = ORUtils::SE3Pose(ORUtils::Matrix4<float>(pose.data()));
    {
        slam->renderImage(Block_depth2RGB_.get(), static_cast<ITMLib::ITMMainEngine::GetImageType>(slam_getImageType), &se3pose, &render_intrinsics);
        cudaDeviceSynchronize();
        ITMUChar4Image tmpImage(*Block_depth2RGB_);
        /// Flip Y
        auto* data_cpu = Block_depth2RGB_->GetData(MEMORYDEVICE_CPU);
        auto* inv_cpu  = tmpImage.GetData(MEMORYDEVICE_CPU);
        for(int y=0;y<Block_depth2RGB_->noDims.y;++y){
            for(int x=0;x<Block_depth2RGB_->noDims.x;++x){
                int Y = Block_depth2RGB_->noDims.y-1-y;
                data_cpu[y*Block_depth2RGB_->noDims.x+x] = inv_cpu[Y*Block_depth2RGB_->noDims.x+x];
            }
        }
    }

/// Save to Cloud
//        saveImage2PointCloud(Block_vertexMap_.get(), "TMPCLOUD.ply");
    glImageRenderer->update((uchar *) Block_depth2RGB_->GetData(MEMORYDEVICE_CPU),
                            Block_depth2RGB_->noDims.x/* slamWarpper_->getCalibParam()->intrinsics_rgb.imgSize.width*/,
                            Block_depth2RGB_->noDims.y /*slamWarpper_->getCalibParam()->intrinsics_rgb.imgSize.height*/);
    TOCK("[SLAMGUI][renderReProjDepth]0.all");
}


template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::OpenGLReferenceToCameraRefence(ORUtils::Matrix4<float> *modelPose) {
    auto pose = getEigen4f(modelPose->m);
    OpenGLReferenceToCameraRefence(&pose);
}

template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::OpenGLReferenceToCameraRefence(Eigen::Matrix4f *modelPose){
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(1 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    *modelPose =  transferMat * *modelPose;
}

template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::OpenGLReferenceToCameraRefence(Eigen::Map<Eigen::Matrix4f> *modelPose){
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(1 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    *modelPose =  transferMat * *modelPose;
}

template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::CameraRefenceToOpenGLReference(Eigen::Matrix4f *modelPose) {
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(1 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    *modelPose =  transferMat * *modelPose;

}

template<class SLAMType>
void SurfelSLAMGUI<SLAMType>::renderCamera(){
    /// Draw Camera
    if (bShowPose) {
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
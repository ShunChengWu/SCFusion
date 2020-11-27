#include <fstream>
#include "GUI_SCSLAM.hpp"
#include "gui_kernel.hpp"
//#include <Utilities/EigenHelper.h>
#include "../../../ORUtils/EigenHelper.h"
#include <utility>
//#include <Utilities/utils.hpp>
#include "../../../ORUtils/Logging.h"
#include "../../Files/Label_ScanNet.h"
#include "../../Files//Label_NYU13.h"
#include "../../Files/Label_SunCG11.h"
using namespace SCFUSION;

template<class SLAMType>
int SCSLAMGUI<SLAMType>::initialization() {
    TICK("[SCSLAMGUI][init]0.All");
    SLAMGUI<SLAMType>::initMC();
    SLAMGUI<SLAMType>::loadShaders();
    this->SCShader();
    SLAMGUI<SLAMType>::registerKeys(window_);
    registerSCKeys();
    TOCK("[SCSLAMGUI][init]0.All");
    return 1;
}

template<class SLAMType>
void SCSLAMGUI<SLAMType>::registerSCKeys() {
    registerKeyFunciton(window_, GLFW_KEY_5, [&]() {
        if (SLAMGUI<SLAMType>::slam_->getITMSetting()->useSC) {
            SLAMGUI<SLAMType>::displayMode_FreeView = DisplayMode_FreeView_OBufferSurface;
            SLAMGUI<SLAMType>::bNeedUpdateSurface = true;
            printf("displayMode_FreeView: DisplayMode_FreeView_OBufferSurface\n");
        } else printf("Did not compile with scene completion!\n");
    });
    registerKeyFunciton(window_, GLFW_KEY_6, [&]() {
        if (SLAMGUI<SLAMType>::slam_->getITMSetting()->useSC) {
            SLAMGUI<SLAMType>::displayMode_FreeView = DisplayMode_FreeView_IBufferSurface;
            SLAMGUI<SLAMType>::bNeedUpdateSurface = true;
            printf("displayMode_FreeView: DisplayMode_FreeView_IBufferSurface\n");
        } else printf("Did not compile with scene completion!\n");
    });
    registerKeyFunciton(window_, GLFW_KEY_7, [&]() {
        if (SLAMGUI<SLAMType>::slam_->getITMSetting()->useSC) {
            SLAMGUI<SLAMType>::displayMode_FreeView = DisplayMode_FreeView_Surface_pure_sc;
            SLAMGUI<SLAMType>::bNeedUpdateSurface = true;
            printf("displayMode_FreeVIew: DisplayMode_FreeView_Surface_pure_sc\n");
        } else {
            SCLOG(WARNING) << "Unable to display in buffer sufface mode!. Did not run this programe with useSC!!\n";
        }
    });
    /// Z Perform Scene Completion
    registerKeyFunciton(window_, GLFW_KEY_Z, [&]() {
        if (slam_->getITMSetting()->useSC) printf("Run SceneCompletion Once!!\n");
        else printf("Cannot perform Scene Completion!. Please run the system with --useSC command.\n");
        bRunSceneCompletion = true;
    });



    /// KeyPad1 Rotate along row
    registerKeyFunciton(window_, GLFW_KEY_KP_1, [&](){
        rotateAlongYaw += 1;
        if(rotateAlongYaw>2)rotateAlongYaw=0;
    });
    registerKeyFunciton(window_, GLFW_KEY_KP_ADD, [&](){
        rotateSpeed+=5;
    });
    registerKeyFunciton(window_, GLFW_KEY_KP_SUBTRACT, [&](){
        rotateSpeed-=5;
    });

    /// Save Pose
    registerKeyFunciton(window_, GLFW_KEY_KP_MULTIPLY, [&](){
        glCam->camera_control_->getCamPose(savedPose_.row,savedPose_.yaw,savedPose_.pitch,savedPose_.x,savedPose_.y,savedPose_.z);
        std::fstream file("./pose_saved.txt", std::ios::out);
        if(file.is_open())
            file << savedPose_.row << " " << savedPose_.yaw << " " << savedPose_.pitch << " " <<
                savedPose_.x << " " << savedPose_.y << " " << savedPose_.z;
        bNeedUpdate=true;
        printf("Save pose!\n");
    });
    /// Load Pose
    registerKeyFunciton(window_, GLFW_KEY_KP_DIVIDE, [&](){
        bNeedUpdate=true;
        printf("Load pose!\n");
        std::fstream file("./pose_saved.txt", std::ios::in);
        if(file.is_open())
            file >> savedPose_.row  >> savedPose_.yaw >> savedPose_.pitch >>
                savedPose_.x >> savedPose_.y >> savedPose_.z;
        glCam->camera_control_->setCamPose(savedPose_.row,savedPose_.yaw, savedPose_.pitch,savedPose_.x,savedPose_.y,savedPose_.z);
//        Block_freeview_pose_->m03 = glCam->camera_control_->Position.x;
//        Block_freeview_pose_->m13 = glCam->camera_control_->Position.y;
//        Block_freeview_pose_->m23 = glCam->camera_control_->Position.z;
//        Eigen::Matrix4f eigenMat4f = getCameraPose();
//        Eigen::Matrix3f transferMat;
//        transferMat.setIdentity();
//        transferMat(1, 1) = -1;
//        transferMat(2, 2) = -1;
//        eigenMat4f.topLeftCorner<3, 3>() = transferMat * eigenMat4f.topLeftCorner<3, 3>();
//        eigenMat4f.topLeftCorner<3, 3>().transposeInPlace();
//        Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> mat1 = getEigenRowMajor<float, 4>(Block_freeview_pose_->m);
//        mat1.topLeftCorner<3, 3>() = eigenMat4f.topLeftCorner<3, 3>();
    });



    /// V Show Pose
    registerKeyFunciton(window_, GLFW_KEY_B,
                        [&]() {
                            if (!bShowSCGrid) {
                                printf("Show SC Grid On\n");
                                bShowSCGrid = true;
                            } else {
                                printf("Show SC Grid Off\n");
                                bShowSCGrid = false;
                            }
                        });
}

template<class SLAMType>
int SCSLAMGUI<SLAMType>::NeedUpdateSurfaceConditions(){
    switch (displayMode_mesh) {
        case Foo_DisplayMode_FreeView_Surface: {
            if(SLAMGUI<SLAMType>::mc_thread_.valid()){
                if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
                    SLAMGUI<SLAMType>::mc_thread_.get();
                    SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI<SLAMType>::extractGlobalMapSurface, this));
                    bNeedUpdateSurface = false;
                }
            } else {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SLAMGUI<SLAMType>::extractGlobalMapSurface, this));
            }
//            SLAMGUI<SLAMType>::extractGlobalMapSurface();
            break;
        }
        case Foo_DisplayMode_FreeView_SurfaceLabel: {
            if (!SLAMGUI<SLAMType>::mc_thread_.valid()) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SLAMGUI<SLAMType>::extractGlobalMapSurfaceLabel, this));
                bNeedUpdateSurface = false;
            } else if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SLAMGUI<SLAMType>::extractGlobalMapSurfaceLabel, this));
                bNeedUpdateSurface = false;
            }
            break;
        }
        case Foo_DisplayMode_FreeView_ColoredLabel: {
            if (!SLAMGUI<SLAMType>::mc_thread_.valid()) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async, std::bind(&SCSLAMGUI<SLAMType>::extractPoints, this));
                bNeedUpdateSurface = false;
            } else if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async, std::bind(&SCSLAMGUI<SLAMType>::extractPoints, this));
                bNeedUpdateSurface = false;
            }
            break;
        }

        case DisplayMode_FreeView_OBufferSurface: {
            if (!SLAMGUI<SLAMType>::mc_thread_.valid()) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SCSLAMGUI<SLAMType>::extractSCOBufferSurface, this));
                bNeedUpdateSurface = false;
            } else if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SCSLAMGUI<SLAMType>::extractSCOBufferSurface, this));
                bNeedUpdateSurface = false;
            }
            break;
        }
        case DisplayMode_FreeView_IBufferSurface: {
            if (!SLAMGUI<SLAMType>::mc_thread_.valid()) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SCSLAMGUI<SLAMType>::extractSCIBufferSurface, this));
                bNeedUpdateSurface = false;
            } else if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                        std::bind(&SCSLAMGUI<SLAMType>::extractSCIBufferSurface, this));
                bNeedUpdateSurface = false;
            }
            break;
        };
        case DisplayMode_FreeView_Surface_wo: {
#ifdef COMPARE_VOLUME
            if(SLAMGUI<SLAMType>::bMCComputed) break;
                if (!SLAMGUI<SLAMType>::mc_thread_.valid()) {
                    SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractSurface, this, slam_->getVolume_wo(),
                                                      volume_copy_.get(), label_copy_.get(),
                                                      cuda_gui_streams_.getStream("MC")));
                    bNeedUpdateSurface = false;
                } else if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractSurface, this, slam_->getVolume_wo(),
                                                      volume_copy_.get(), label_copy_.get(),
                                                      cuda_gui_streams_.getStream("MC")));
                    bNeedUpdateSurface = false;
                }
#endif
            break;
        }
        case DisplayMode_FreeView_Surface_pure_sc: {
#ifdef COMPARE_VOLUME
            if(SLAMGUI<SLAMType>::bMCComputed) break;
                if (!SLAMGUI<SLAMType>::mc_thread_.valid()) {
                    SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractSurface, this, slam_->getVolume_pure_sc(),
                                                      volume_copy_.get(), label_copy_.get(),
                                                      cuda_gui_streams_.getStream("MC")));
                    bNeedUpdateSurface = false;
                } else if (SLAMGUI<SLAMType>::mc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    SLAMGUI<SLAMType>::mc_thread_ = std::async(std::launch::async,
                                            std::bind(&SLAMGUI::extractSurface, this, slam_->getVolume_pure_sc(),
                                                      volume_copy_.get(), label_copy_.get(),
                                                      cuda_gui_streams_.getStream("MC")));
                    bNeedUpdateSurface = false;
                }
#endif
            break;
        }
    }
}

template<class SLAMType>
void SCSLAMGUI<SLAMType>::process_impl() {
    TICK("[SCSLAMGUI][process_impl]0.All");
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    SLAMGUI<SLAMType>::fps_->updateFPS();

    ///SLAM
    bool bSLAMProceed = SLAMGUI<SLAMType>::process_slam();
    if(bSLAMProceed){
        bNeedUpdateSurface = SLAMGUI<SLAMType>::bRenderMesh;
        bNeedUpdate=true;
    }

    if (bRunSceneCompletion) {
        slam_->triggerBackEnd();
        bRunSceneCompletion = false;
        bNeedUpdate = true;
        bNeedUpdateSurface = true;
    }

#if 1
    if (bNeedUpdateSurface) {
        NeedUpdateSurfaceConditions();
        // prevent missing surfel when recording video.
        if(SLAMGUI<SLAMType>::mc_thread_.valid()) {
            SLAMGUI<SLAMType>::mc_thread_.get();
            bNeedUpdateSurface=false;
        }
    }

/// Render Image From Volume
    //OPT: combine this with bNeedUpdateSurface into different case
    if (bNeedUpdate) {
//        DEBUG("[SLAMGUI][process_impl]UpdateImage\n");
        TICK("[SLAMGUI][process_impl]2.UpdateImage");
        bNeedUpdate = !bNeedUpdate;

        if(!SLAMGUI<SLAMType>::bRenderMesh) renderImgFromSLAM(slam_);

        if (SLAMGUI<SLAMType>::bRecordImage) SLAMGUI<SLAMType>::recordImg();
        TOCK("[SLAMGUI][process_impl]2.UpdateImage");
    }

    glm::vec4 lightColor(1.0, 1.0, 1.0, 1);

    glm::mat4 projection = glCam->projection_control_->projection_matrix();

    if (!SLAMGUI<SLAMType>::bRenderMesh) {
        SLAMGUI<SLAMType>::mImageDrawer->Draw();
    } else {
        glUtil::Shader *shader = nullptr;
        if (slam_->getPointCloud() != nullptr) {
            if (slam_->getPointCloud()->noTotalPoints && displayMode_mesh == DisplayMode_FreeView_ColoredLabel) {
                shader = glShaders["Points"];
                glm::mat4 modelMat = glm::mat4(1.f);
                shader->use();
                shader->set("lightColor", lightColor);
                shader->set("lightPos", glCam->camera_control_->Position /*glm::vec3(0,2,0)*/);
                shader->set("model", modelMat);
                shader->set("projection", projection);
                shader->set("view", glCam->camera_control_->GetViewMatrix());
                shader->set("sign", SLAMGUI<SLAMType>::mc_normal_sign_);
                SLAMGUI<SLAMType>::drawPointCloud(shader);
            }
        }
        if (slam_->getMesh() != nullptr) {
            if (slam_->getMesh()->noTotalTriangles && (displayMode_mesh != DisplayMode_FreeView_ColoredLabel)) {
                shader = glShaders["Vertices"];
                glm::mat4 modelMat = glm::mat4(1.f);
                shader->use();
                shader->set("lightColor", lightColor);
                shader->set("lightPos", glCam->camera_control_->Position /*glm::vec3(0,2,0)*/);
                shader->set("model", modelMat);
                shader->set("projection", projection);
                shader->set("view", glCam->camera_control_->GetViewMatrix());
                shader->set("sign", SLAMGUI<SLAMType>::mc_normal_sign_);
                SLAMGUI<SLAMType>::drawMesh(shader);
            }
        }
    }
    /// Draw Camera
    glDisable(GL_DEPTH_TEST);
    SLAMGUI<SLAMType>::renderCamera();
    glEnable(GL_DEPTH_TEST);


    /// Draw Box
//    if(0)
    if (cameraMode_ == SLAMGUI<SLAMType>::camera_FreeView && bShowSCGrid &&
    displayMode_FreeView != DisplayMode_FreeView_OBufferSurface && displayMode_FreeView != DisplayMode_FreeView_IBufferSurface) {
        auto scslam = reinterpret_cast<SCFUSION::SLAM*>(slam_);

        Eigen::Matrix4f ori_mat = Eigen::Matrix4f::Identity();
        float scale;
        {
            std::unique_lock<std::mutex> lock(scslam->getSCBuffer().mutex);
            ori_mat(0,3) = scslam->getSCBuffer()->volumeBase_[0];
            ori_mat(1,3) = scslam->getSCBuffer()->volumeBase_[1];
            ori_mat(2,3) = scslam->getSCBuffer()->volumeBase_[2];
            scale = scslam->getITMSetting()->sceneParams.voxelSize*scslam->getSCBuffer()->volumeDims_[0];
        }
        glm::mat4 modelMat = glm::make_mat4(ori_mat.data());

//        std::cout << "scale: " << scale << "\n";
        modelMat = glm::scale(modelMat,
                              glm::vec3(scale, scale, scale));    // it's a bit too big for our scene, so scale it down

        auto *shader = SLAMGUI<SLAMType>::glShaders["SCBox"];
        shader->use();
        shader->set("projection", projection);
        shader->set("view", SLAMGUI<SLAMType>::glCam->camera_control_->GetViewMatrix());
        shader->set("model", modelMat);
        glBindVertexArray(SLAMGUI<SLAMType>::glVertexArrays["SCBox"]);
//        glEnable(GL_LINE_WIDTH);
//        glDisable(GL_LINE_SMOOTH);
//        GLfloat lineWidthRange[2] = {0.0f,5.0f};
//        glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lineWidthRange);
        glLineWidth(5);
        glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);
        glLineWidth(1);

    }

    /// Draw first EC
    if(0)
    {
        auto *shader = SLAMGUI<SLAMType>::glShaders["SCBox"];
        shader->use();
        shader->set("projection", projection);
        shader->set("view", SLAMGUI<SLAMType>::glCam->camera_control_->GetViewMatrix());

        glBindVertexArray(SLAMGUI<SLAMType>::glVertexArrays["SCBox"]);

        auto scslam = reinterpret_cast<SCFUSION::SLAM*>(slam_);
        if(scslam->GetSceneCompletion()){
            auto extractionCenters = scslam->GetSceneCompletion()->GetExtractionCenters();
            for(const auto &ec :extractionCenters) {
                Eigen::Matrix4f ori_mat = Eigen::Matrix4f::Identity();
                float scale;
                {
                    ori_mat(0,3) = ec.extraction_base[0];
                    ori_mat(1,3) = ec.extraction_base[1];
                    ori_mat(2,3) = ec.extraction_base[2];
                    scale = scslam->getITMSetting()->sceneParams.voxelSize*scslam->getSCBuffer()->volumeDims_[0];
                }
                glm::mat4 modelMat = glm::make_mat4(ori_mat.data());
                modelMat = glm::scale(modelMat,
                                      glm::vec3(scale, scale, scale));    // it's a bit too big for our scene, so scale it down
                shader->set("model", modelMat);
                glBindVertexArray(SLAMGUI<SLAMType>::glVertexArrays["SCBox"]);
                glEnable(GL_LINE_SMOOTH);
                glEnable(GL_LINE_WIDTH);
                glLineWidth(10);
                glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);
                glLineWidth(1);
                glDisable(GL_LINE_SMOOTH);
                break; //
            }
        }
    }

    /// Draw extraction centers
    if(0)
    {
        auto *shader = SLAMGUI<SLAMType>::glShaders["SCBox"];
        shader->use();
        shader->set("projection", projection);
        shader->set("view", SLAMGUI<SLAMType>::glCam->camera_control_->GetViewMatrix());

        glBindVertexArray(SLAMGUI<SLAMType>::glVertexArrays["SCBox"]);

        auto scslam = reinterpret_cast<SCFUSION::SLAM*>(slam_);
        if(scslam->GetSceneCompletion()){
            auto extractionCenters = scslam->GetSceneCompletion()->GetExtractionCenters();
            for(const auto &ec :extractionCenters) {
                Eigen::Matrix4f ori_mat = Eigen::Matrix4f::Identity();
                float scale;
                {
                    ori_mat(0,3) = ec.extraction_base[0];
                    ori_mat(1,3) = ec.extraction_base[1];
                    ori_mat(2,3) = ec.extraction_base[2];
                    scale = scslam->getITMSetting()->sceneParams.voxelSize*scslam->getSCBuffer()->volumeDims_[0];
                }
                glm::mat4 modelMat = glm::make_mat4(ori_mat.data());
                modelMat = glm::scale(modelMat,
                                      glm::vec3(scale, scale, scale));    // it's a bit too big for our scene, so scale it down
                shader->set("model", modelMat);
                glBindVertexArray(SLAMGUI<SLAMType>::glVertexArrays["SCBox"]);
                glLineWidth(3);
                glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);
                glLineWidth(1);
            }
        }
    }

    if(0)
    {
        const std::string name = "ViewFrustumBBox";
        std::pair<Vector3f, Vector3f> boundaries;
        auto slam = static_cast<SCFUSION::SLAM*>(slam_);
        bool state = slam->GetViewFrustumBBox(boundaries);
        if(state) {
            glm::mat4 modelMat = glm::mat4(1.f);
            auto *shader = glShaders[name];
            shader->use();
            shader->set("projection", projection);
            shader->set("view", SLAMGUI<SLAMType>::glCam->camera_control_->GetViewMatrix());
            shader->set("model", modelMat);
            float minmax[12] = {
                    boundaries.first.x, boundaries.first.y, boundaries.first.z,
                    boundaries.second.x, boundaries.first.y, boundaries.first.z,
                    boundaries.first.x, boundaries.first.y, boundaries.second.z,
                    boundaries.second.x, boundaries.first.y, boundaries.second.z,
            };
            glBindVertexArray(Base::glVertexArrays[name]);
            glBindBuffer(GL_ARRAY_BUFFER, Base::glBuffers[name]);

            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            sizeof(minmax), minmax);
            glBindVertexArray(Base::glVertexArrays[name]);
            glLineWidth(3);
            glDrawElements(GL_LINES, 8, GL_UNSIGNED_INT, 0);
            glLineWidth(1);
        }
    }
    if(0)
    {
        const std::string name = "PointCloud";
        Vector3f min,max;
        auto slam = static_cast<SCFUSION::SLAM*>(slam_);
        if(slam->GetView()){
            Vector2i imgDim =slam->GetView()->depth->noDims;
            Vector4f projParams_d = slam->getCalib()->intrinsics_d.projectionParamsSimple.all;
            ORUtils::MemoryBlock<Vector3f> points(640*480, true, false);
            auto point_data = points.GetData(MEMORYDEVICE_CPU);
            slam->GetView()->depth->UpdateHostFromDevice();
            auto depth_data = slam->GetView()->depth->GetDataConst(MEMORYDEVICE_CPU);
            auto CameraPose = slam->GetTrackingState()->pose_d->GetInvM();
            //FIXME: The boundary calculation here is correct, but in scengine seems to be wrong.
            // Need to calculate 8 points (frustum) or 4 points(flat)
            for(int x=0; x < imgDim.x; ++x){
                for(int y=0; y < imgDim.y; ++y){
                    auto idx = y * imgDim.x + x;
                    const Vector4f pt_camera = {
                            (x - projParams_d.z) / projParams_d.x * depth_data[idx],
                            (y - projParams_d.w) / projParams_d.y * depth_data[idx],
                            depth_data[idx], 1.f
                    };
                    const Vector4f pt_world = CameraPose * pt_camera;
                    point_data[idx] = Vector3f(pt_world);

                    if(pt_world.x < min.x) min.x = pt_world.x;
                    if(pt_world.y < min.y) min.y = pt_world.y;
                    if(pt_world.z < min.z) min.z = pt_world.z;
                    if(pt_world.x > max.x) max.x = pt_world.x;
                    if(pt_world.y > max.y) max.y = pt_world.y;
                    if(pt_world.z > max.z) max.z = pt_world.z;
                }
            }

            glm::mat4 modelMat = glm::mat4(1.f);
            auto *shader = glShaders[name];
            shader->use();
            shader->set("projection", projection);
            shader->set("view", SLAMGUI<SLAMType>::glCam->camera_control_->GetViewMatrix());
            shader->set("model", modelMat);
            glBindVertexArray(Base::glVertexArrays[name]);
            glBindBuffer(GL_ARRAY_BUFFER, Base::glBuffers[name]);

            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            points.dataSize*sizeof(float)*3, point_data);
            glBindVertexArray(Base::glVertexArrays[name]);
            glDrawArrays(GL_POINTS, 0, points.dataSize);


            {
                const std::string name = "ViewFrustumBBox";
                glm::mat4 modelMat = glm::mat4(1.f);
                auto *shader = glShaders[name];
                shader->use();
                shader->set("projection", projection);
                shader->set("view", SLAMGUI<SLAMType>::glCam->camera_control_->GetViewMatrix());
                shader->set("model", modelMat);
                float minmax[12] = {
                        min.x, min.y, min.z,
                        max.x, min.y, min.z,
                        max.x, min.y, max.z,
                        min.x, min.y, max.z,
                };
                glBindVertexArray(Base::glVertexArrays[name]);
                glBindBuffer(GL_ARRAY_BUFFER, Base::glBuffers[name]);

                glBufferSubData(GL_ARRAY_BUFFER, 0,
                                sizeof(minmax), minmax);
                glBindVertexArray(Base::glVertexArrays[name]);
                glLineWidth(3);
                glDrawElements(GL_LINES, 8, GL_UNSIGNED_INT, 0);
                glLineWidth(1);
            }
        }
    }


    TOCK("[SCSLAMGUI][process_impl]0.All");
    /// Save Time Information
    for (const std::pair<std::string, double> &time : getWatch.getTimings()) {
        if (!getWatch.updated(time.first)) continue;
        getWatch.getUpdateStats()[time.first] = false;

        if (SLAMGUI<SLAMType>::times_.find(time.first) == SLAMGUI<SLAMType>::times_.end()) {
            if (slamWarpper_->getImgCounterMax() > 0)
                SLAMGUI<SLAMType>::times_[time.first].reserve(slamWarpper_->getImgCounterMax());
            else
                SLAMGUI<SLAMType>::times_[time.first].reserve(10000); // assume will have this amount of images
            SLAMGUI<SLAMType>::times_[time.first].push_back(time.second);
        } else {
            SLAMGUI<SLAMType>::times_[time.first].push_back(time.second);
        }
    }
#endif
    SC::GUI3D::basicProcess();
}

template<class SLAMType>
void SCSLAMGUI<SLAMType>::extractSCIBufferSurface() {
    if(!slam_->getITMSetting()->createMeshingEngine){
        printf("Did not enable meshing!\n");
        return;
    }


    slam_->getMesh()->noTotalTriangles = 0;
    slam_->getMesh()->triangles->Clear(0);
    if (slam_->getMesh()->hasNormal)slam_->getMesh()->normals->Clear(0);
    if (slam_->getMesh()->hasColor)slam_->getMesh()->colors->Clear(0);


    TICK("[GUI][extractSurfaceSCBuffer]0.all");
    auto scslam = reinterpret_cast<SCFUSION::SLAM*>(slam_);
    auto& scBuffer = scslam->getSCBuffer();
    {
        std::unique_lock<std::mutex> lock(scBuffer.mutex);
        scBuffer->iBuffer->UpdateDeviceFromHost();
        slam_->computeMesh(scBuffer->iBuffer.get(), {0.f,0.f,0.f}, scBuffer->volumeDims_,
                           scBuffer->scparams_->voxelSize, LOGODD_SURFACE);
//        slam_->computeMesh(scBuffer->iBuffer.get(), scBuffer->volumeBase_, scBuffer->volumeDims_,
//                           scBuffer->scparams_->voxelSize, LOGODD_SURFACE);
    }

    TOCK_P("[GUI][extractSurfaceSCBuffer]0.all");
    DEBUG("mesh->noTotalTriangles: %d\n", slam_->getMesh()->noTotalTriangles);

    if (slam_->getMesh()->noTotalTriangles) {
        SLAMGUI<SLAMType>::copyMeshToGUI(slam_);
        auto color_data = SLAMGUI<SLAMType>::mc_colors->GetData(MEMORYDEVICE_CPU,false);
        for(size_t i=0;i<scslam->getMesh()->noTotalTriangles;++i){
            color_data[i].c0 = {1.f,1.f,1.f,1.f};
            color_data[i].c1 = {1.f,1.f,1.f,1.f};
            color_data[i].c2 = {1.f,1.f,1.f,1.f};
        }
    }
}

template<class SLAMType>
void SCSLAMGUI<SLAMType>::extractSCOBufferSurface() {
    if(!slam_->getITMSetting()->createMeshingEngine){
        printf("Did not enable meshing!\n");
        return;
    }

    if (slam_->getITMSetting()->createMeshingEngine) {
        slam_->getMesh()->noTotalTriangles = 0;
        slam_->getMesh()->triangles->Clear(0);
        if (slam_->getMesh()->hasNormal)slam_->getMesh()->normals->Clear(0);
        if (slam_->getMesh()->hasColor)slam_->getMesh()->colors->Clear(0);
    }

    TICK("[GUI][extractSurfaceSCBuffer]0.all");
    auto scslam = reinterpret_cast<SCFUSION::SLAM*>(slam_);
    auto& scBuffer = scslam->getSCBuffer();
    {
        std::unique_lock<std::mutex> lock(scBuffer.mutex);
        slam_->computeMesh(scBuffer->oBuffer.get(), {0.f,0.f,0.f}, scBuffer->volumeDims_,
                           scBuffer->scparams_->voxelSize, /*LOGODD_SURFACE*/0.9);
//        slam_->computeMesh(scBuffer->oBuffer.get(), scBuffer->volumeBase_, scBuffer->volumeDims_,
//                           scBuffer->scparams_->voxelSize, 0.9);
    }

    slam_->getCUDAStream().syncStream(SCFUSION::SLAM::STREAM_VISUALIZATION);
    TOCK_P("[GUI][extractSurfaceSCBuffer]0.all");
    DEBUG("mesh->noTotalTriangles: %d\n", slam_->getMesh()->noTotalTriangles);

    if (slam_->getMesh()->noTotalTriangles) {
        SLAMGUI<SLAMType>::copyMeshToGUI(slam_);
    }
}


template<class SLAMType>
void SCSLAMGUI<SLAMType>::SCShader() {
    /// Cube
    {
        //float cam_width = 0.3, cam_height = 0.2, cam_front = 1.0, cam_back = 0.f, ratio = 3;
        float width=1;
        float camera_points[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
                0,0,0,
                width,0,0,
                0,0,width,
                width,0,width,
                0,width,0,
                width,width,0,
                0,width,width,
                width,width,width,
        };
        unsigned int indices_line[] = {
                0, 1,
                2, 0,
                3, 2,
                3, 1,
                4, 5,
                4, 6,
                7, 5,
                7, 6,
                0, 4,
                1, 5,
                2, 6,
                3, 7,
        };

        std::string name = "SCBox";
        if(glShaders.find(name) != glShaders.end())throw std::runtime_error("target shader name has been used!\n");
        const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
        glShaders[name] = new glUtil::Shader(shaderPath + "camera_shader.vs",
                                                 shaderPath + "camera_shader.fs");
        glShaders[name]->use();
        glShaders[name]->set("color", glm::vec4(1, 0.0549019608, 0, 1));
        unsigned int &VBO = glBuffers[name], &VAO = glVertexArrays[name], &EBO = glBuffers[name + "_ebo"];
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(camera_points), &camera_points, GL_STATIC_DRAW);
        // Lines
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_line), indices_line, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
        glBindVertexArray(0);
    }


    /// View Frustum region
    {
        const std::string name = "ViewFrustumBBox";
        if(glShaders.find(name) != glShaders.end())throw std::runtime_error("target shader name has been used!\n");
        const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
        glShaders[name] = new glUtil::Shader(shaderPath + "camera_shader.vs",
                                             shaderPath + "camera_shader.fs");
        glShaders[name]->use();
        glShaders[name]->set("color", glm::vec4(1, 0.0549019608, 0, 1));

        unsigned int &VBO = glBuffers[name], &VAO = glVertexArrays[name], &EBO = glBuffers[name + "_ebo"];
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 3, nullptr, GL_DYNAMIC_DRAW);

        // Lines
        unsigned int indices_line[] = {
                0, 1,
                1, 3,
                3, 2,
                2, 0
        };
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_line), indices_line, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
        glBindVertexArray(0);
    }
    {
        const std::string name = "PointCloud";
        if(glShaders.find(name) != glShaders.end())throw std::runtime_error("target shader name has been used!\n");
        const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
        glShaders[name] = new glUtil::Shader(shaderPath + "camera_shader.vs",
                                             shaderPath + "camera_shader.fs");
        glShaders[name]->use();
        glShaders[name]->set("color", glm::vec4(0, 0.0549019608, 1, 1));

        unsigned int &VBO = glBuffers[name], &VAO = glVertexArrays[name];
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 640*480 * 3, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
        glBindVertexArray(0);
    }
}

template<class SLAMType>
void SCSLAMGUI<SLAMType>::DrawLeftPanel() {
    auto& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300,0.8 * io.DisplaySize.y), ImGuiCond_Always);
    ImGui::Begin("Control panel",nullptr, /*ImGuiWindowFlags_NoTitleBar | */ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar);
    // menu bar
    if(ImGui::BeginMenuBar()){
        if(ImGui::BeginMenu("System")){
            ImGui::EndMenu();
        }
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
        ImGui::PushItemWidth(150);
        bool updated = ImGui::Combo("Render Mode", &renderMode, desc, IM_ARRAYSIZE(desc));
        if(updated) {
            if(renderMode == 0) cameraMode_ = SLAMGUI<SLAMType>::camera_PoseView;
            if(renderMode == 1) cameraMode_ = SLAMGUI<SLAMType>::camera_FreeView;
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
            if (followCameraMode == 0) SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
            if (followCameraMode == 1) SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
            if (followCameraMode == 2) SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
            if (followCameraMode == 3)
                SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME;
            if (followCameraMode == 4)
                SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL;
            if (followCameraMode == 5)
                SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE;
            if (followCameraMode == 6)
                SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL;
            if (followCameraMode == 7)
                SLAMGUI<SLAMType>::displayMode_FollowView = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_LABEL_FROM_NORMAL;
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

//    bNeedUpdateSurface |= ImGui::Checkbox("Render Mesh", &bRenderMesh);
    ImGui::Checkbox("Render Mesh", &bRenderMesh);
    if(ImGui::Button("Reset Mesh Buffer")) {
        SLAMGUI<SLAMType>::ResetMCBuffer();
        bNeedUpdateSurface = true;
    }
    {
        const char *meshModes[] = {
                "Surface",
                "Label",
                "Pointcloud",
                "OBuffer",
                "IBuffer",
                "Surface_wo",
                "Surface_pure_sc",
        };
        static int meshMode = 0;
        bool updated = ImGui::Combo("Render Mesh mode", &meshMode, meshModes, IM_ARRAYSIZE(meshModes));
        if(updated){
            if(meshMode == 0) displayMode_mesh = Foo_DisplayMode_FreeView_Surface;
            if(meshMode == 1) displayMode_mesh = Foo_DisplayMode_FreeView_SurfaceLabel;
            if(meshMode == 2) displayMode_mesh = Foo_DisplayMode_FreeView_ColoredLabel;
            if(meshMode == 3) displayMode_mesh = DisplayMode_FreeView_OBufferSurface;
            if(meshMode == 4) displayMode_mesh = DisplayMode_FreeView_IBufferSurface;
            if(meshMode == 5) displayMode_mesh = DisplayMode_FreeView_Surface_wo;
            if(meshMode == 6) displayMode_mesh = DisplayMode_FreeView_Surface_pure_sc;
            bNeedUpdateSurface = true;
        }
    }

    if(ImGui::TreeNode("Show Label Color")){
        // Color buttons, demonstrate using PushID() to add unique identifier in the ID stack, and changing style.
        int labelNum = slam_->getITMSetting()->scParams.labelNum;
        for (int i = 0; i < labelNum; i++)
        {
            if (i % 3 != 0)
                ImGui::SameLine();
            const Vector4f* color = slam_->GetLabelColorList()->GetDataConst(MEMORYDEVICE_CPU,false);
            ImGui::PushID(i);
            ImGui::PushStyleColor(ImGuiCol_Button,(ImVec4)ImColor(color[i].x,color[i].y,color[i].z, color[i].w));
            static int imguiLabelButtonSizeWidth = 75;
            static int imguiLabelButtonSizeHeight = 25;
            if(labelNum == 12) {
                auto name = SunCG11[i];
                if(name.length() > 20) name = name.substr(0, 20);
                ImGui::Button(name.c_str(), ImVec2(imguiLabelButtonSizeWidth,imguiLabelButtonSizeHeight));
            } else if (labelNum == 14) {
                auto name = NYU13[i];
                if(name.length() > 20) name = name.substr(0, 20);
                ImGui::Button(name.c_str(), ImVec2(imguiLabelButtonSizeWidth,imguiLabelButtonSizeHeight));
            } else {
                ImGui::Button(std::to_string(i).c_str(), ImVec2(imguiLabelButtonSizeWidth,imguiLabelButtonSizeHeight));
            }
            ImGui::PopStyleColor(1);
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    ImGui::Separator();
    if(ImGui::TreeNode("Draw Options")){
        ImGui::Checkbox("Grid", &bShowGrid);
        ImGui::Checkbox("Camera", &bShowPose);
        ImGui::Checkbox("Trajectory", &bPlotTrajectory);
        ImGui::Checkbox("SC grid", &bShowSCGrid);
        ImGui::Checkbox("FPS", &bShowFPS);

        ImGui::TreePop();
    }

    ImGui::End();
}

//template<class SLAMType>
//void SCSLAMGUI<SLAMType>::DrawBottomRightOverlay(){
//    const float DISTANCE = 10.0f;
//    bool p_open = true;
//    static int corner = 3;
//    ImGuiIO& io = ImGui::GetIO();
//    if (corner != -1)
//    {
//        ImVec2 window_pos = ImVec2((corner & 1) ? io.DisplaySize.x - DISTANCE : DISTANCE, (corner & 2) ? io.DisplaySize.y - DISTANCE : DISTANCE);
//        ImVec2 window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
//        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
//    }
//    ImGui::SetNextWindowBgAlpha(0.35f); // Transparent background
//    if (ImGui::Begin("System Info", &p_open, (corner != -1 ? ImGuiWindowFlags_NoMove : 0) |
//                                             ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
//                                             ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav))
//    {
//        ImGui::Text("System Info");
//        ImGui::Separator();
//        if (ImGui::IsMousePosValid()) {
//            ImGui::Text("Mouse Position: (%.1f,%.1f)", io.MousePos.x, io.MousePos.y);
//
//            unsigned char pixelColor[4];
//            glReadPixels(io.MousePos.x, io.DisplaySize.y/* window_->height*/ - io.MousePos.y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, &pixelColor);
//            ImGui::Text("Color: %d %d %d %d", (int)pixelColor[0],(int)pixelColor[1],(int)pixelColor[2],(int)pixelColor[3]);
//            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
//
//            auto labelColorList = slam_->GetLabelColorList();
//            if(labelColorList){
//                Vector4u color (pixelColor[0],pixelColor[1],pixelColor[2],pixelColor[3]);
//                ushort label = 0;
//                auto dataptr = labelColorList->GetDataConst(MEMORYDEVICE_CPU);
//                auto size = slam_->GetModelLabelNum()>0? slam_->GetModelLabelNum() : 14;
//                if(size > labelColorList->dataSize) size = labelColorList->dataSize;
//                for(size_t i=0;i<size;++i){
//                    auto targetColor = (dataptr[i]*255).toUChar();
////                        std::cout<< "[" << i << "]" << dataptr[i] << "\n";
//                    if( targetColor == color){
//                        label = i;
//                        break;
//                    }
//                }
//                ImGui::Text("Label: %d", label);
//            }
//        } else
//            ImGui::Text("Mouse Position: <invalid>");
//
//        if (ImGui::BeginPopupContextWindow())
//        {
//            if (ImGui::MenuItem("Custom",       NULL, corner == -1)) corner = -1;
//            if (ImGui::MenuItem("Top-left",     NULL, corner == 0)) corner = 0;
//            if (ImGui::MenuItem("Top-right",    NULL, corner == 1)) corner = 1;
//            if (ImGui::MenuItem("Bottom-left",  NULL, corner == 2)) corner = 2;
//            if (ImGui::MenuItem("Bottom-right", NULL, corner == 3)) corner = 3;
//            if (p_open && ImGui::MenuItem("Close")) p_open = false;
//            ImGui::EndPopup();
//        }
//    }
//    ImGui::End();
//}
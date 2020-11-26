#pragma once

#include <memory>
#include <CxxTools/LogUtil.hpp>
#include "ImageLoader/ImageLoadFactory.h"
#include "../../../MainLib/Core/ITMMainEngine.h"
#include "../../../MainLib/ITMLibDefines.h"
#ifdef COMPILE_WITH_PCL
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#endif

template<class T>
class SLAMWrapper {
public:
    explicit SLAMWrapper(const ITMLib::ITMLibSettings *setting, SCFUSION::IO::ImageLoader *imgloader, ITMLib::ITMRGBDCalib* calib = nullptr):
            itmSettings_(setting), imgloader_(imgloader), needDeleteCalib(false){
        if(calib == nullptr) {
            calib_ = new ITMLib::ITMRGBDCalib;
            needDeleteCalib = true;
        } else {
            calib_ = calib;
        }
        img_counter_ = img_counter_max_ = 0;

        if(imgloader_ != nullptr) {
            imgloader_->Init();
            DEBUG("\n There are %d images in total.\n", imgloader_->NumberOfImages());

            img_counter_ = 0;
            img_counter_max_ = imgloader_->NumberOfImages();
            calib_->intrinsics_rgb = imgloader_->getRGBCameraParams();
            calib_->intrinsics_d   = imgloader_->getDepthCameraParams();
        } else if (calib == nullptr) {
            DEBUG("\n No image loader provided!\n");
            calib_->intrinsics_rgb.SetFrom(640,480,500,500,320,240);
            calib_->intrinsics_d.SetFrom(640,480,500,500,320,240);
        }


        Block_depthImg_.reset(new ITMFloatImage(ORUtils::Vector2<int>(calib_->intrinsics_d.imgSize.width, calib_->intrinsics_d.imgSize.height), true, true) );
        Block_colorImg_.reset(new ITMUChar4Image (ORUtils::Vector2<int>(calib_->intrinsics_rgb.imgSize.width, calib_->intrinsics_rgb.imgSize.height), true, true) );
        Block_pose_.reset(new  ORUtils::Matrix4<float>());
        
        slam_.reset(new T(itmSettings_, calib_));

        timeStampLast_ = std::chrono::system_clock::now();
    }
    ~SLAMWrapper(){
        if(needDeleteCalib) delete calib_;
    }

    int processOnce(){
        TICK("[SLAMWrapper][processOnce]1.LoadFrame");
        if(imgloader_ == nullptr) {
            DEBUG("[%s][%s][%d] No image loader provided!\n", __FILE__, __FUNCTION__, __LINE__);
            return -1;
        }
        auto idx_img = imgloader_->Next();

        if(idx_img<0) {
            SCLOG(INFO) << "Cannot read next image. break";
            return -1;
        }
        imgloader_->getDepth(idx_img,Block_depthImg_.get());
        imgloader_->getColor(idx_img, Block_colorImg_.get());
        bool hasPose = imgloader_->getPose(idx_img, Block_pose_.get()) > 0;
        TOCK("[SLAMWrapper][processOnce]1.LoadFrame");

        if(Block_labelImg_){
            Block_labelImg_->ChangeDims(Block_depthImg_->noDims);
        }

        // skip frame
        if(itmSettings_->useSkipFrame >0)
            if(idx_img % itmSettings_->useSkipFrame != 0) {
                img_counter_++;
                return 1;
            }

        /// First pose align to given pose
//        hasPose = false;
        if(img_counter_ == 0 ){
            if(!hasPose) {
                Block_pose_->setIdentity();
                flooralignment();
                // align to openGL renderer
//                Block_pose_->at(0, 0) = -1;
//                Block_pose_->at(1, 1) = -1;

//                Block_pose_->at(2, 2) = -1;
//                Block_pose_->at(3, 3) = -1;


                hasPose=true;
            }

//            if(hasPose){ // Align first frame
//                ORUtils::Matrix4<float> pose_colMajor;
//                for (size_t i = 0; i < 4; ++i)
//                    for (size_t j = 0; j < 4; ++j)
//                        pose_colMajor.at(i, j) = Block_pose_->at(j, i);
//                slam_->setPose(pose_colMajor.inv().m);
//            }
        }
        SCLOG(DEBUG)<< "Block_pose_\n:" << *Block_pose_;


#ifdef UseFrameGrabber
        slam.processFrame(Block_depthImg, NULL, NULL);
//        slam.ProcessFrame(NULL, NULL, NULL);
#else

        TICK("[SLAMWrapper][processOnce]2.waitInverval");
        if(timeStampInterval_ > 0) {
            while (true) {
                std::chrono::duration<double> diff = std::chrono::system_clock::now() - timeStampLast_;
                if (diff.count() > timeStampInterval_) {
                    timeStampLast_ = std::chrono::system_clock::now();
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        TOCK("[SLAMWrapper][processOnce]2.waitInverval");

        TICK("[SLAMWrapper][processOnce]3.slam_->ProcessFrame");
        int trackingResult;
        if (hasPose){
            trackingResult = slam_->ProcessFrame(Block_depthImg_.get(), Block_colorImg_.get(), img_counter_,
                                                 Block_pose_.get(), nullptr, Block_labelImg_.get());
        } else {
            trackingResult = slam_->ProcessFrame(Block_depthImg_.get(), Block_colorImg_.get(), img_counter_, nullptr,
                                                 nullptr,
                                                 Block_labelImg_.get());
            *Block_pose_ = slam_->GetTrackingState()->pose_d->GetM().inv();
        }

#endif
        TOCK("[SLAMWrapper][processOnce]3.slam_->ProcessFrame");
        img_counter_++;
        return trackingResult;
    }
    /// 0:GTPOSEMODE_IGNORE; 1:GTPOSEMODE_ASSIST; 2:GTPOSEMODE_TACKOVER
    void useGTPose(int c=0){slam_->setGTPoseMode(static_cast<ITMLib::ITMMainEngine::GTPoseMODE >(c));}
    T* getSLAM() {return static_cast<T*>(slam_.get());}
    ITMLib::ITMRGBDCalib* getCalibParam(){return calib_;}
    void setImgLoader(SCFUSION::IO::ImageLoader *imgloader){
        imgloader_ = imgloader;

        if(imgloader == nullptr) return;
        imgloader_->Init();
        DEBUG("\n There are %d images in total.\n", imgloader_->NumberOfImages());

        img_counter_ = 0;
        img_counter_max_ = imgloader_->NumberOfImages();
        calib_->intrinsics_rgb = imgloader_->getRGBCameraParams();
        calib_->intrinsics_d   = imgloader_->getDepthCameraParams();
    }

    void UseLabelImage(bool option){
        if(option){
            Block_labelImg_.reset(new ITMUShortImage({1,1},true,true));
        } else {
            Block_labelImg_.reset();
        }
    }

    void flooralignment(){
#ifdef COMPILE_WITH_PCL
        Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();

        // build cloud from depth
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        {
            auto depthParam = calib_->intrinsics_d.projectionParamsSimple;
            auto depth = Block_depthImg_->GetDataConst(MEMORYDEVICE_CPU);
            for(size_t col=0; col < Block_depthImg_->noDims.width; col+=5){
                for(size_t row=0; row < Block_depthImg_->noDims.height; row+=5){
                    const float depth_value = depth[row*Block_depthImg_->noDims.width+col];
                    const Eigen::Vector2f xy((col - depthParam.px) / depthParam.fx, (row - depthParam.py) / depthParam.fy);
                    const Eigen::Vector4f point(xy.x() * depth_value, xy.y() * depth_value, depth_value, 1.0f);
                    pcl::PointXYZ p;
                    p.x = point.x();
                    p.y = point.y();
                    p.z = point.z();
                    cloud->push_back(p);
                }
            }
        }

        /// Plane segmentation
        {
            // normal
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  //法线估计对象
            ne.setInputCloud (cloud);
            ne.setKSearch (50);
            ne.compute (*cloud_normals);

            // sac
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac;
            sac.setInputCloud (cloud);
            sac.setInputNormals (cloud_normals);
            sac.setMethodType(pcl::SAC_RANSAC);
            sac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//            sac.setNormalDistanceWeight (0.5);
            sac.setDistanceThreshold(0.05);
//            sac.setMaxIterations(100);
//            sac.setProbability(0.95);
            sac.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            }

//            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                      << coefficients->values[1] << " "
//                      << coefficients->values[2] << " "
//                      << coefficients->values[3] << std::endl;
//            std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

//            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
//            for (int idx1 : inliers->indices) {
//                const pcl::PointXYZ &p = cloud->points[idx1];
//                cloud_plane->push_back(p);
//            }
//            viewer.addPointCloud(cloud_plane,"plane");
//            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"plane");
//            viewer.spin();

            Eigen::Vector4f plan_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
            float value = 1;
            if (plan_normal(2) > 0) plan_normal *= -1;
            float z_plane_centor = ((-plan_normal(3)-plan_normal(0)*value - plan_normal(1)*value)/ plan_normal(2));

            auto getRotmatrixfromAtoB=[](const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2 = {0,0,1})->Eigen::Matrix4f{
                auto Safeacos = [] (float x) -> float{
                    if (x < -1.0) x = -1.0 ;
                    else if (x > 1.0) x = 1.0 ;
                    return acos (x) ;
                };
                Eigen::Vector3f crossProduct = vec1.cross(vec2);
                Eigen::Vector3f vector_X = (crossProduct / crossProduct.norm());
                float thetaAngleRad = Safeacos(vec1.dot(vec2)/(vec1.norm()*vec2.norm()));

                Eigen::AngleAxis<float> aa(thetaAngleRad, vector_X);
                Eigen::Matrix4f matrix_R;
                matrix_R.setIdentity();
                matrix_R.block(0,0,3,3) = aa.toRotationMatrix();
                return matrix_R;
            };
            Eigen::Matrix4f rotmat = getRotmatrixfromAtoB(plan_normal.head(3), Eigen::Vector3f(0, 1, 0));

            Eigen::Vector4f rotated_normal = rotmat * plan_normal;
//            std::cerr << "rotated_normal: " << rotated_normal.transpose();
            pose = rotmat * pose;
            pose.topRightCorner<3,1>() += rotated_normal.head<3>();
//            std::cerr << "pose: " << pose;
//            m_pose(0, 3) += INSEG_CONFIG.volumeSize[0] / 2 * INSEG_CONFIG.voxelScale;
//            m_pose(1, 3) += INSEG_CONFIG.volumeSize[1] / 2 * INSEG_CONFIG.voxelScale;
//            m_pose(2, 3) += INSEG_CONFIG.volumeSize[2] * 0.8   * INSEG_CONFIG.voxelScale;
//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZRGB>());
//            for(size_t col=0; col < depth.cols; col+=5){
//                for(size_t row=0; row < depth.rows; row+=5){
//                    const float depth_value = depth.at<unsigned short>(row, col) * scale;
//                    const cv::Vec3b color_value = rgb.at<cv::Vec3b>(row, col);
//
//                    const Eigen::Vector2f xy((col - depthParam.px) / depthParam.fx, (row - depthParam.py) / depthParam.fy);
//                    const Eigen::Vector4f point(xy.x() * depth_value, xy.y() * depth_value, depth_value, 1.0f);
//                    const Eigen::Vector4f point_transformed = pose_ * point + rotated_normal;
//
////                    std::cout << point_transformed.transpose() << " : " << color_value << "\n";
//                    pcl::PointXYZRGB p;
//                    p.x = point_transformed.x();
//                    p.y = point_transformed.y();
//                    p.z = point_transformed.z();
//                    p.r = color_value[0];
//                    p.g = color_value[1];
//                    p.b = color_value[2];
//                    cloud_rotated->push_back(p);
//                }
//            }
//            viewer.addPointCloud(cloud_rotated,"cloud_t");
//            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"cloud_t");
//            viewer.spin();
        }
        memcpy(Block_pose_->m, pose.data(),sizeof(float)*16);
#else
        SCLOG(WARNING) << "Compile without PCL. Skip the floor alignment process.";
#endif
    }

    SCFUSION::IO::ImageLoader* getImgLoader(){return imgloader_;}
    /// Get Camera Pose (World)
    ORUtils::Matrix4<float>* getPose(){return Block_pose_.get();}
    size_t getImgCounter(){return img_counter_;}
    size_t getImgCounterMax(){return img_counter_max_;}
    ITMFloatImage *getDepthImage(){return Block_depthImg_.get();}
    ITMUChar4Image *getColorImage(){return Block_colorImg_.get();}
    void setInputFrameRate(double hz){timeStampInterval_=hz*1e-3;}
private:
    std::unique_ptr<ITMLib::ITMMainEngine> slam_;
    const ITMLib::ITMLibSettings* itmSettings_;
    SCFUSION::IO::ImageLoader* imgloader_;
    ITMLib::ITMRGBDCalib* calib_;
    bool needDeleteCalib;

    double timeStampInterval_=0;
    std::chrono::time_point<std::chrono::system_clock> timeStampLast_;

    size_t img_counter_,img_counter_max_;
    // palceholder
    std::unique_ptr<ITMFloatImage> Block_depthImg_;
    std::unique_ptr<ITMUChar4Image> Block_colorImg_;
    std::unique_ptr<ITMUShortImage> Block_labelImg_;
    std::unique_ptr<ORUtils::Matrix4<float>> Block_pose_;
};
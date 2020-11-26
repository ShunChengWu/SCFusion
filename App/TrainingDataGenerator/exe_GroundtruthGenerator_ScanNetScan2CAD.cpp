#include <ImageLoader/ImageLoader.hpp>
#include <SLAMTools/Parser.hpp>
#include "TrainingDataGenerator.h"
#include "../../MeshVoxelizer/MeshVoxelizer.h"
#include <CxxTools/PathTool.hpp>
#include <SLAM_base/SLAM.h>
#include <SLAMTools/SLAMWrapper.h>
#include <random>
#include "SLAMGUI/GUI_SLAM.tpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include "../../ConnectedComponent/ConnectedComponent/CPU/ConnectedComponent_CPU.h"
#include <CxxTools/DataWorker.h>
#include "ScanNetScan2CADLoader.h"
#include "../../Files/Label_SunCG11.h"

class MapUtil : public SCFUSION::MainEngine<ITMVoxel, ITMVoxelIndex>,
                public SCFUSION::PointCloudEngine<ITMVoxel, ITMVoxelIndex>,
                public SCFUSION::MeshEngine<ITMVoxel, ITMVoxelIndex> {
public:
    float mfRenderFilteringThreshold = 0.4;
    explicit MapUtil(const ITMLib::ITMLibSettings *itmSetting, const ITMLib::ITMRGBDCalib *calib)
            :MainEngine(itmSetting, calib),PointCloudEngine(itmSetting), MeshEngine(itmSetting){
        if(itmSetting->createMeshingEngine)
            itmMeshingEngine->setLabelColorListPtr(LabelColorList_->GetData(MEMORYDEVICE_CUDA));
        slam_gt = nullptr;
    }

    void setGTSLAM(MapUtil *slam){slam_gt = slam;}
    void computePointCloud() { PointCloudEngine::computePointCloud(itmScene.get()); }
    void computeMesh(bool labelOnly=false, bool checkState=false){ MeshEngine::computeMesh(itmScene.get(),labelOnly,checkState); }
    ITMLib::ITMTrackingState::TrackingResult ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                          ORUtils::Matrix4<float> *customPose,
                                                          ITMLib::ITMIMUMeasurement *imuMeasurement,
                                                          ITMUShortImage *imgLabel) override {
        //TODO: make this function can be an option
        if(mfRenderFilteringThreshold>0)
        {
            SCLOG(DEBUG) << "use render filtering with threshold " << mfRenderFilteringThreshold;
            ORUtils::SE3Pose tmp_pose;
            tmp_pose.SetM(customPose->inv());
            ITMUChar4Image tmp_color(imgDepth->dataSize,true,false);
            auto tmp_intrinsics = calib_->intrinsics_d;
            renderImage(&tmp_color, MainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED, &tmp_pose,&tmp_intrinsics,ITMLib::IITMVisualisationEngine::RENDER_PERSPECTIVE);
            cudaDeviceSynchronize();
            size_t num_nonEmptyPixels = 0;
            const auto *data = tmp_color.GetData(MEMORYDEVICE_CPU);
            for(size_t i=0;i<tmp_color.dataSize;++i){
                if(data[i].x>0 && data[i].y>0 && data[i].z>0 && data[i].a > 0)num_nonEmptyPixels++;
            }
            auto p = float(num_nonEmptyPixels)/float(tmp_color.dataSize);
//            SCLOG(DEBUG) << "Percentage: " << p;
            if(p>mfRenderFilteringThreshold) {
//                SCLOG(DEBUG) << "P > 0.5. Skip";
                return ITMLib::ITMTrackingState::TRACKING_GOOD;
            }
        }

        if(slam_gt == nullptr ){
            return MainEngine::ProcessFrame(imgDepth, imgColor, img_counter, customPose, imuMeasurement, imgLabel);
        } else {
            ORUtils::SE3Pose se3pose(customPose->inv());
            if(imgLabel) {
                imgLabel->ChangeDims(imgDepth->noDims);
                slam_gt->getLabelImage(imgLabel, &se3pose, &calib_->intrinsics_d);
                imgLabel->UpdateHostFromDevice();

//                ORUtils::Image2CVShow::show(imgLabel);
            }
            slam_gt->getDepthImage(imgDepth,&se3pose,&calib_->intrinsics_d);
            imgDepth->UpdateHostFromDevice();
            cudaDeviceSynchronize();

            auto result = MainEngine::ProcessFrame(imgDepth, imgColor, img_counter, customPose, imuMeasurement, imgLabel);
//            cudaDeviceSynchronize();
//            this->getLabelImage(imgLabel, &se3pose, &calib_->intrinsics_d);
//            cudaDeviceSynchronize();
//            imgLabel->UpdateHostFromDevice();
//            ORUtils::Image2CVShow::show(imgLabel, "2");
            return result;
        }
    }

    void resetAll() override{
        SCFUSION::MainEngine<ITMVoxel,ITMVoxelIndex>::resetAll();
        if (itmSettings->createMeshingEngine) {
            MeshEngine<ITMVoxel, ITMVoxelIndex>::reset();

        }
        if (itmSettings->createPointExtractionEngine) {
            PointCloudEngine<ITMVoxel,ITMVoxelIndex>::reset();
        }
        slam_gt=nullptr;
    }
private:
    MapUtil *slam_gt;
};

typedef pcl::PointXYZRGB PointT;
typedef MapUtil SLAMType;
template class SCFUSION::SLAMGUI<SLAMType>;
//#include "GUI_two.hpp"

struct Params{
    std::string pth_scan, pth_shapenet, pth_annotations;
    bool gui = false;
    bool verbose = false;
    int target_scene = -1;
    bool fill=true;
    bool occupancyOnly=false;
    Params()=default;
};

int ParseCommandLine(int argc, char** argv, Params *params){
    tools::Parser parser(argc,argv);
    parser.addOption(pkgcname("pth_scan", &params->pth_scan), "The path to the folder contains ScanNet sequences.", true);
    parser.addOption(pkgcname("pth_shapenet", &params->pth_shapenet), "The path to the ShapeNet folder.", true);
    parser.addOption(pkgcname("pth_annotations", &params->pth_annotations), "The path to the Scan2Cad annotation file.", true);
    parser.addOption(pkgcname("gui", &params->gui), "Use GUI.", false);
    parser.addOption(pkgcname("verbose", &params->verbose), ".", false);
    parser.addOption(pkgcname("target_scene", &params->target_scene), "target_scene.\n",false);
    parser.addOption(pkgcname("occupancyOnly", &params->occupancyOnly), "occupancyOnly.\n",false);
    parser.addOption(pkgcname("fill", &params->fill), "fill.\n",false);
    if(parser.showMsg(params->verbose)<0)
        exit(EXIT_FAILURE);
#ifndef NDEBUG
    parser.outputLog(std::cout);
#endif
    return 1;
}

std::shared_ptr<std::vector<Vector3f>> randomPointGenerator(size_t  num, Vector3f min, Vector3f max){
    std::default_random_engine rd(2019);
    std::mt19937 gen(rd());
    std::shared_ptr<std::vector<Vector3f>> points (new std::vector<Vector3f>);
    for(size_t i=0;i<num;++i){
        std::uniform_real_distribution<> distri_x(min.x, max.x);
        std::uniform_real_distribution<> distri_y(min.y, max.y);
        std::uniform_real_distribution<> distri_z(min.z, max.z);
        float x = distri_x(gen);
        float y = distri_x(gen);
        float z = distri_x(gen);

        bool repeat=false;
        for(auto & point: *points){
            if(point.x == x && point.y == y, point.z == z)
                repeat = true;
        }
        if(!repeat)
            points->emplace_back(Vector3f{x,y,z});

        if(points->size() == num) break;
    }
    return points;
}

int main (int argc, char ** argv) {
#ifndef NDEBUG
    SCLOG_ON(VERBOSE);
#else
    SCLOG_ON(INFO);
#endif

    auto settings = ITMLib::ITMLibSettings();
    auto calib = ITMLib::ITMRGBDCalib();
    settings.deviceType = ITMLib::ITMLibSettings::DEVICE_CUDA;
    InputParams inputParams;
    std::unique_ptr<SLAMWrapper<SLAMType>> slam_reconstruction, slam_groundtruth;

    std::unique_ptr<SCFUSION::IO::ImageLoader> imgloader_;
    if(argc < 2) {
        SCLOG(ERROR) << "Need to pass a path to configuration file.\n";
    } else {
        SCFUSIONIO scfusionIO(&inputParams, &settings);
        scfusionIO.loadParams(argv[1]);
    }

    Params params;
    ParseCommandLine(argc, argv, &params);
    parserCommandLine(argc, argv, inputParams, &settings,params.verbose);///Use Parser

    if(params.verbose){
        SCLOG_ON(VERBOSE);
    }

    if(settings.scParams.labelNum != 12)
        SCLOG(ERROR) << "Label number must be 12";

    if(params.gui)
        settings.createMeshingEngine=true;

    /// define and check output folder exist

    pcl::visualization::PCLVisualizer::Ptr viewer;
    int vp0,vp1;
    if(params.gui) {
        viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
        viewer->createViewPort(0,0.0,0.5,1,vp0);
        viewer->createViewPort(0.5,0.0,1,1,vp1);
    }

    SCFUSION::TrainingDataGenerator<OFu_Voxel_f_2label,OFu_Voxel_f_2label> trainingDataGenerator(params.verbose);
    ScanNetScan2CadMesh_Loader loader(params.pth_scan,params.pth_shapenet,params.pth_annotations,params.occupancyOnly,params.fill,
                                      settings.sceneParams.voxelSize);
    std::unique_ptr<tools::MultiDataWorker<DataBuffer>> dataWorker = nullptr;
    //FIXME: multithreading doesn't work
//    auto is_digits=[](const std::string &str)->bool
//    {
//        return std::all_of(str.begin(), str.end(), ::isdigit); // C++11
//    };
//    if(params.target_scene < 0)
//        dataWorker.reset(new tools::MultiDataWorker<DataBuffer> (&loader, 1));

    // find the number of target scene
//    auto folders = tools::PathTool::get_files_in_folder(params.pth_scan);
//    std::string targetScene = "scene0610_01";
//    for(size_t i=0;i<folders.size();++i){
//        if(folders[i] == targetScene){
//            params.target_scene = i;
//            break;
//        }
//    }



    while(true){
        size_t target_scene=params.target_scene>=0? params.target_scene : loader.next();
        SCLOG(DEBUG) << "target_scene: " << target_scene;
//        SCLOG(DEBUG) << "target_scene: " << target_scene;
        if(target_scene>=loader.dataSize())break;
        auto dataholder = loader.get_item(target_scene);

        if(dataholder){
            auto points = dataholder->points.get();
            auto labels = dataholder->labels.get();
            auto subfolder = dataholder->subFolder;
            const auto &objPoints = dataholder->objPoints;
            const auto &objLabels = dataholder->objLabels;
            const auto &objInstantces = dataholder->objInstances;

            if(scene_to_skip.find(subfolder) != scene_to_skip.end()){
                SCLOG(INFO) << "skip scene: " << subfolder;
                if(params.target_scene>=0)break;
                else continue;
            }
            SCLOG(DEBUG) << "subfolder: " << subfolder;
//            SCLOG(VERBOSE) << "subFolder: " << subfolder;
            if (params.gui) {
                int vp2;
                viewer->removeAllPointClouds(vp0);
                viewer->removeAllPointClouds(vp1);
                viewer->createViewPort(0,0.0,0.33,1,vp0);
                viewer->createViewPort(0.33,0.0,0.66,1,vp1);
                viewer->createViewPort(0.66,0.0,1,1,vp2);

                // calculate total point size
                size_t totalSize = 0;
                for(size_t i=0;i< dataholder->objPoints.size();++i){
                    totalSize += dataholder->objPoints[i]->dataSize;
                }

                pcl::PointCloud<PointT>::Ptr scan_cloud_out(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr obj_cloud_out(new pcl::PointCloud<PointT>());

                scan_cloud_out->reserve(points->dataSize);
                obj_cloud_out->reserve(totalSize);

                // Scan
//                if(0)
                {
                    auto points_ = points->GetDataConst(MEMORYDEVICE_CPU);
                    auto labels_ = labels->GetDataConst(MEMORYDEVICE_CPU);
                    for (size_t i = 0; i < points->dataSize; ++i) {
                        PointT point;
                        point.x = points_[i].x;
                        point.y = points_[i].y;
                        point.z = points_[i].z;
//                    if (params.showInstance) { // show label
//                        point.r = labelColorList.GetData(MEMORYDEVICE_CPU)[instances_[i]].r;
//                        point.g = labelColorList.GetData(MEMORYDEVICE_CPU)[instances_[i]].g;
//                        point.b = labelColorList.GetData(MEMORYDEVICE_CPU)[instances_[i]].b;
//                    } else { // show instances
                        point.r = SunCG11ColorLabel.at(labels_[i]).r;
                        point.g = SunCG11ColorLabel.at(labels_[i]).g;
                        point.b = SunCG11ColorLabel.at(labels_[i]).b;
//                    }
                        scan_cloud_out->push_back(point);
                    }
                }

                // Objects
//                if(0)
                for(size_t i=0;i< objPoints.size();++i){
                    auto points_ = objPoints[i]->GetDataConst(MEMORYDEVICE_CPU);
                    auto labels_ = objLabels[i]->GetDataConst(MEMORYDEVICE_CPU);
                    for (size_t j = 0; j < objPoints[i]->dataSize; ++j) {
                        PointT point;
                        point.x = points_[j].x;
                        point.y = points_[j].y;
                        point.z = points_[j].z;
                        point.r = SunCG11ColorLabel.at(labels_[j]).r;
                        point.g = SunCG11ColorLabel.at(labels_[j]).g;
                        point.b = SunCG11ColorLabel.at(labels_[j]).b;
                        obj_cloud_out->push_back(point);
                    }
                }

                viewer->removeAllPointClouds(vp0);
                viewer->removeAllPointClouds(vp1);
                viewer->removeAllPointClouds(vp2);
                viewer->addPointCloud(obj_cloud_out, "objcloud_vp0", vp0);
//                viewer->addPointCloud(obj_cloud_out, "objcloud_vp1", vp1);
                viewer->addPointCloud(scan_cloud_out, "scancloud_vp1", vp1);
                viewer->addPointCloud(obj_cloud_out, "objcloud_vp2", vp2);
                viewer->addPointCloud(scan_cloud_out, "scancloud_vp2", vp2);
                viewer->spin();
                viewer->removeAllPointClouds(vp0);
                viewer->removeAllPointClouds(vp1);
                viewer->removeAllPointClouds(vp2);
                viewer->createViewPort(0,0.0,0.5,1,vp0);
                viewer->createViewPort(0.5,0.0,1,1,vp1);
                viewer->addPointCloud(obj_cloud_out, "objcloud_vp2", vp1);
                viewer->addPointCloud(scan_cloud_out, "scancloud_vp2", vp1);
            }


            /// Check and create folders
            SCLOG(VERBOSE) << "Check and create folders";
            tools::PathTool::check_and_create_folder(inputParams.pth_out);


            /// Reconstruct Scene
            SCLOG(VERBOSE) << "Fuse Points to map";
            slam_groundtruth.reset(new SLAMWrapper<SLAMType>(&settings, nullptr, &calib));
            slam_groundtruth->getSLAM()->setLibSetting(&settings);
            slam_groundtruth->getSLAM()->setRGBDCalib(&calib);
            slam_groundtruth->getSLAM()->resetAll();
            slam_groundtruth->setImgLoader(nullptr);
            // scan
            slam_groundtruth->getSLAM()->getMapper()->getSceneRecoEngine()->IntegrateIntoScene(
                    slam_groundtruth->getSLAM()->getScene(),slam_groundtruth->getSLAM()->getRenderStateLive(),
                    points, labels,nullptr
            );
//            trainingDataGenerator.FuseToScene(slam_groundtruth->getSLAM()->getMapper(),
//                                              slam_groundtruth->getSLAM()->getScene(),
//                                              slam_groundtruth->getSLAM()->getRenderStateLive(),
//                                              points, labels, params.occupancyOnly);
            // objects
            for(size_t i=0;i<objPoints.size();++i){
//                trainingDataGenerator.FuseToScene(slam_groundtruth->getSLAM()->getMapper(),
//                                                  slam_groundtruth->getSLAM()->getScene(),
//                                                  slam_groundtruth->getSLAM()->getRenderStateLive(),
//                                                  objPoints[i].get(), objLabels[i].get(), params.occupancyOnly);
                slam_groundtruth->getSLAM()->getMapper()->getSceneRecoEngine()->IntegrateIntoScene(
                        slam_groundtruth->getSLAM()->getScene(),slam_groundtruth->getSLAM()->getRenderStateLive(),
                        objPoints[i].get(), objLabels[i].get(),nullptr
                );
            }

            /// Find Boundaries x, z
            SCLOG(VERBOSE) << "Find boundaries";
            Vector3i pos_min(0, 0, 0), pos_max(0, 0, 0);
            auto localVBA = slam_groundtruth->getSLAM()->getScene()->localVBA.get();
            auto index = slam_groundtruth->getSLAM()->getScene()->index.get();
            auto voxelSize = slam_groundtruth->getSLAM()->getITMSetting()->sceneParams.voxelSize;
            Vector3f pos_min_ (0, 0, 0), pos_max_(0, 0, 0);
            {
                auto points_data = points->GetDataConst(MEMORYDEVICE_CPU);
                for(size_t i=0;i<points->dataSize;++i){
                    auto p = points_data[i];
                    for(size_t j=0;j<3;++j){
//                        printf("%f",p[j]);
                        if(p[j] < pos_min_[j])pos_min_[j] = p[j];
                        if(p[j] > pos_max_[j])pos_max_[j] = p[j];
                    }
//                    printf("\n");
                }
                for(size_t i=0;i<3;++i)
                    pos_min[i] = (int)std::round(pos_min_[i] / settings.sceneParams.voxelSize);
                for(size_t i=0;i<3;++i)
                    pos_max[i] = (int)std::round(pos_max_[i] / settings.sceneParams.voxelSize);
                SCLOG(VERBOSE) << "Float_min: " << pos_min_ << "; Float_max: " << pos_max_;
                SCLOG(VERBOSE) << "min: " << pos_min << "; max: " << pos_max;
            }

            /// modify entries to fit the evaluation on ScanComplete
            Vector3s dims;
            if(1)
            {
                Vector3s center;
                for (size_t i = 0; i < 3; ++i) {
                    dims[i] = pos_max[i] - pos_min[i];
//                    // make dims dividable
//                    dims[i] = std::floor(dims[i] / 2) * 2;
//                    center[i] = pos_min[i] + dims[i] / 2;
                }
                DEBUG("Dims ori: %d %d %d\n", dims.x, dims.y, dims.z);
                DEBUG("Center: %d %d %d\n", center.x, center.y, center.z);

//                if (params.height > 0) {
//                    pos_max.y = pos_min.y + params.height;
//                    //pos_max.y=params.height;
//                    dims.y = params.height;
//                }
                DEBUG("\t pos_min %d %d %d\n", pos_min.x, pos_min.y, pos_min.z);
                DEBUG("\t pos_max %d %d %d\n", pos_max.x, pos_max.y, pos_max.z);
                DEBUG("\t Dims: %d %d %d\n", dims.x, dims.y, dims.z);
                DEBUG("\t Center: %d %d %d\n", center.x, center.y, center.z);
            }


            /// Uniform Distribution
            SCLOG(DEBUG) << "Volume Extraction Dims: " << dims;
            ORUtils::MemoryBlock<unsigned short> DataGT(dims.x * dims.y * dims.z, true, true);
            trainingDataGenerator.ExtractToUniMap(&DataGT, pos_min, dims,
                                                  slam_groundtruth->getSLAM()->getScene());
            cudaDeviceSynchronize();
            DataGT.UpdateHostFromDevice();

            // modify label to prevent out-of-range labels
            {
                auto labels_ = DataGT.GetData(MEMORYDEVICE_CPU);
                for(size_t i=0;i<DataGT.dataSize;++i){
                    if(labels_[i] >= settings.scParams.labelNum) labels_[i] = 0;
                }
            }

            if (params.gui) {
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//                        pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());

                uint overlapCounter = 0;
                for (int x_ = 0; x_ < dims.x; ++x_) {
                    for (int y_ = 0; y_ < dims.y; ++y_) {
                        for (int z_ = 0; z_ < dims.z; ++z_) {
                            int idx = (z_ * dims.y + y_) * dims.x + x_;
                            bool hasRC = false, hasGT = false;
                            float label = DataGT.GetData(MEMORYDEVICE_CPU)[idx];
                            PointT point;
                            point.x = pos_min.x + x_ * voxelSize;
                            point.y = pos_min.y + y_ * voxelSize;
                            point.z = pos_min.z + z_ * voxelSize;

                            if (label > 0) {
                                point.r = SunCG11ColorLabel[label].r;
                                point.g = SunCG11ColorLabel[label].g;
                                point.b = SunCG11ColorLabel[label].b;
                                hasGT = hasRC = true;
                            }

                            if (hasRC || hasGT) {
                                cloud->push_back(point);
                            }
                        }
                    }
                }
                printf("num of overlapping: %d\n", overlapCounter);

                viewer->addPointCloud(cloud, "cloud1", vp0);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20,
                                                         "cloud1", vp0);
                viewer->spin();
                viewer->removeAllPointClouds(vp0);
            }

            auto tmp = subfolder;
            auto save_path = inputParams.pth_out+"/"+tmp+".ply";
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
            cloud->reserve(dims.x*dims.y*dims.z);
            for (int x_ = 0; x_ < dims.x; ++x_) {
                for (int y_ = 0; y_ < dims.y; ++y_) {
                    for (int z_ = 0; z_ < dims.z; ++z_) {
                        int idx = (z_ * dims.y + y_) * dims.x + x_;
                        float label = DataGT.GetData(MEMORYDEVICE_CPU)[idx];

                        if (label > 0) {
                            pcl::PointXYZRGBL point;
                            point.x = pos_min.x + x_ * voxelSize;
                            point.y = pos_min.y + y_ * voxelSize;
                            point.z = pos_min.z + z_ * voxelSize;
                            point.label = label;
                            point.r = SunCG11ColorLabel[label].r;
                            point.g = SunCG11ColorLabel[label].g;
                            point.b = SunCG11ColorLabel[label].b;
                            cloud->push_back(point);
                        }
                    }
                }
            }
            pcl::io::savePLYFile(save_path,*cloud,true);
        }// else break;

        if(params.target_scene>=0) break;
    }
    return 0;
}
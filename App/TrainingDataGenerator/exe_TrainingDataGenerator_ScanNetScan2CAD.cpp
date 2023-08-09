#include <ImageLoader/ImageLoader.hpp>
#include <SLAMTools/Parser.hpp>
#include <ImageLoader/ImageLoadFactory.h>
#include "TrainingDataGenerator.h"
#include "../../MeshVoxelizer/MeshVoxelizer.h"
#include <CxxTools/PathTool.hpp>
#include <SLAM_base/SLAM.h>
#include <SLAMTools/SLAMWrapper.h>
#include <random>
#include "../../cnpy/cnpy.h"
//#include <cnpy/cnpy.h>
#include "SLAMGUI/GUI_SLAM.tpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <ORUtils/LabelColorUtils.h>

#include "../../ConnectedComponent/ConnectedComponent/CPU/ConnectedComponent_CPU.h"
#include "loadCloud2Array.hpp"
#include <CxxTools/DataWorker.h>
//#include <Utilities/BinvoxIO.hpp>
#include <ORUtils/Image2cvMat.h>

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
    std::string pth_ply, pth_scan, pth_shapenet, pth_annotations;
    bool checkState=true;
    bool occupancyOnly=false;
    bool gui = false;
    bool verbose = false;
    bool NYU40=false;
    bool fromGT=false;
    bool convertToISDF=false;
    bool outputVoxelDistance=false;
    bool makeInitValueInf=false;
    bool ofuNorm = true;
    bool gtFromRC = false;
    int minLabelNum=3;
    int target_scene = -1;
    bool fill=true;
    bool showInstance= false;
    int height=64;
    std::string loadDims = "";
    float baseVoxelSize=0.047;
    std::string pth_location="";
    int3 volumeDims = make_int3(64,64,64);
    int sampleNum = 1e2;
    float threshold_occupancy=0.98, threshold_highest=0.7, threshold_occupancy_rc=0.02;
    bool dryrun=false;
    float renderFilterThreshold=0.4;
    Params()=default;
};

int ParseCommandLine(int argc, char** argv, Params *params){
    tools::Parser parser(argc,argv);

    parser.addOption(pkgcname("pth_ply", &params->pth_ply), "The path to the folder contains ScanNet ply files.", true);
    parser.addOption(pkgcname("pth_scan", &params->pth_scan), "The path to the folder contains ScanNet poses.", true);
    parser.addOption(pkgcname("pth_shapenet", &params->pth_shapenet), "The path to the ShapeNet folder.", true);
    parser.addOption(pkgcname("pth_annotations", &params->pth_annotations), "The path to the Scan2Cad annotation file.", true);

    parser.addOption(pkgcname("height", &params->height), "truncated height.\n", true);
    parser.addOption(pkgcname("threshold_o", &params->threshold_occupancy), "At least the percentage of the empty voxels should be LOWER this value.\n", false);
    parser.addOption(pkgcname("threshold_occupancy_rc", &params->threshold_occupancy_rc), "The percentage of occupied voxel in scene reconstruction should exceed this value.\n", false);
    parser.addOption(pkgcname("threshold_h", &params->threshold_highest), "The maximum percentage of one label should be LOWER than this value.\n", false);
    parser.addOption(pkgcname("sampleNum", &params->sampleNum), "The number of normal distribution over x-z plan. .", false);
    parser.addOption(pkgcname("minLabelNum", &params->minLabelNum), "minimum label num should be observed in the local volume.", false);
    parser.addOption(pkgcname("occupancy", &params->occupancyOnly), "occupancy Only", false);
    parser.addOption(pkgcname("gui", &params->gui), "Use GUI.", false);
    parser.addOption(pkgcname("verbose", &params->verbose), ".", false);
    parser.addOption(pkgcname("NYU40", &params->NYU40), ".", false);
    parser.addOption(pkgcname("fromGT", &params->fromGT), ".", false);
    parser.addOption(pkgcname("pth_location", &params->pth_location), "path to load location file.", false);
    parser.addOption(pkgcname("dryrun", &params->dryrun), "only generate location file without generating scene", false);
    parser.addOption(pkgcname("checkState", &params->checkState), "Check voxel state", false);
    parser.addOption(pkgcname("volumeDims", &params->volumeDims.x), 3, "extracted volume dimension.", false);
    parser.addOption(pkgcname("ITSDF", &params->convertToISDF), "Convert to Inverted TSDF", false);
    parser.addOption(pkgcname("ofuNorm", &params->ofuNorm),"Normalize Ofusion: value*weight/max_weight.", false);
    parser.addOption(pkgcname("gtFromRC", &params->gtFromRC),"Use reconstructed scene as GT.", false);
    parser.addOption(pkgcname("outputVoxelDistance", &params->outputVoxelDistance), "voxel value will be multiply to truncation margin.", false);
    parser.addOption(pkgcname("makeInitValueInf", &params->makeInitValueInf), "makeInitValueInf.", false);
    parser.addOption(pkgcname("showInstance", &params->showInstance), "If on, show instances in visualization, otherwise show label.\n");
    parser.addOption(pkgcname("target_scene", &params->target_scene), "target_scene.\n",false);
    parser.addOption(pkgcname("renderFilterThreshold", &params->renderFilterThreshold),
            "use the percentage of the rendered depth with the current pose to decide whether skip current frame. set <=0 to skip this\n",false);

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

//    if(params.gui)
//        settings.createMeshingEngine=true;

    /// define and check output folder exist
    bool bLoadLocationFile = !params.pth_location.empty() && params.pth_location != "-1";

    pcl::visualization::PCLVisualizer::Ptr viewer;
    int vp0,vp1;
    if(params.gui) {
        viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
        viewer->createViewPort(0,0.0,0.5,1,vp0);
        viewer->createViewPort(0.5,0.0,1,1,vp1);
    }

    SCFUSION::TrainingDataGenerator<OFu_Voxel_f_2label,OFu_Voxel_f_2label> trainingDataGenerator(params.verbose);
    ScanNetScan2CadMesh_Loader loader(params.pth_ply,params.pth_shapenet,params.pth_annotations,params.occupancyOnly,params.fill,
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
                break;
//                continue;
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

            /// Location File
            SCLOG(VERBOSE) << "Create/Load Location File";
            const std::string locatName = bLoadLocationFile?
                                          tools::PathTool::CheckEnd(params.pth_location) + subfolder + "_location.txt" :
                                          tools::PathTool::CheckEnd(inputParams.pth_out) + subfolder + "_location.txt";
            tools::PathTool::check_and_create_folder(inputParams.pth_out);
            std::fstream locatFile;
            std::map<std::string, std::vector<Vector3f>> locationMap;
            std::set<std::string> locateSet;
            if(bLoadLocationFile){
                SCLOG(VERBOSE) << "Try to load location file";
                if(!tools::PathTool::isFolder(params.pth_location))
                    throw std::runtime_error("pth_location file must be a path to a folder, not a file.\n");

                params.checkState = false;
                locatFile.open(locatName, std::ios::in);
                if(!locatFile.is_open())throw std::runtime_error("Cannot open location file!\n");
                std::string name, x, y, z;
                while(locatFile >> name >> x >> y >> z) {
                    locationMap[name].emplace_back(std::stof(x),std::stof(y),std::stof(z));
                    locateSet.insert(name);
                }
            } else {
                SCLOG(VERBOSE) << "No location file provided. Try to write a location file.";
                locatFile.open(locatName, std::ios::out);
            }

            if(!locatFile.is_open())
                throw std::runtime_error("unable to open file at location.\n");

            /// Check and create folders
            SCLOG(VERBOSE) << "Check and create folders";
            const std::string pth_training = tools::PathTool::CheckEnd(inputParams.pth_out) + "train/"+subfolder;
            const std::string pth_gt = tools::PathTool::CheckEnd(inputParams.pth_out) + "gt/"+subfolder;
            const std::string pth_mask = tools::PathTool::CheckEnd(inputParams.pth_out) + "mask/"+subfolder;
            tools::PathTool::check_and_create_folder(pth_training);
            tools::PathTool::check_and_create_folder(pth_gt);
            tools::PathTool::check_and_create_folder(pth_mask);


            /// Reconstruct Scene
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

            /// Reconstruct again using filtered ground truth
            imgloader_.reset(SCFUSION::ImageLoaderFactory::MakeImageLoader(SCFUSION::IO::InputDateType::INPUTTYPE_SCANNET_POSE,
                                                                         params.pth_scan + "/" + subfolder));
            imgloader_->Init();
            slam_reconstruction.reset(new SLAMWrapper<SLAMType>(&settings, imgloader_.get()));
            slam_reconstruction->getSLAM()->mfRenderFilteringThreshold = params.renderFilterThreshold;
            slam_reconstruction->getSLAM()->setLibSetting(&settings);
            slam_reconstruction->getSLAM()->setRGBDCalib(&calib);
            slam_reconstruction->setImgLoader(imgloader_.get());
            slam_reconstruction->getSLAM()->resetAll();
            slam_reconstruction->useGTPose(2);
            slam_reconstruction->UseLabelImage(true);
            if (params.fromGT)
                slam_reconstruction->getSLAM()->setGTSLAM(slam_groundtruth->getSLAM());
            size_t counter_slam=0;//TODO: remove me
            while (slam_reconstruction->processOnce() > 0){
//                if(counter_slam++>50)break;//TODO: remove me
            }

            /// Visualize
             if (params.gui) {
                 // gt
                 {
                     SCFUSION::SLAMGUI<SLAMType> slamgui(slam_groundtruth.get(), "./");
                     slamgui.initialization();
                     slamgui.run();
                 }
                 // rc
                 {
                     SCFUSION::SLAMGUI<SLAMType> slamgui(slam_reconstruction.get(), "./");
                     slamgui.initialization();
                     slamgui.run();
                 }

//                GUI_Two guiTwo(slam_reconstruction.get(), slam_groundtruth.get(), "");
//                guiTwo.initialization();
//                guiTwo.run();
             }


            /// Find Boundaries x, z
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
            if(1)
            {
                Vector3s dims, center;
                for (size_t i = 0; i < 3; ++i) {
                    dims[i] = pos_max[i] - pos_min[i];
                    // make dims dividable
                    dims[i] = std::floor(dims[i] / 2) * 2;
                    center[i] = pos_min[i] + dims[i] / 2;
                }
                DEBUG("Dims ori: %d %d %d\n", dims.x, dims.y, dims.z);
                DEBUG("Center: %d %d %d\n", center.x, center.y, center.z);

                if (params.height > 0) {
                    pos_max.y = pos_min.y + params.height;
                    //pos_max.y=params.height;
                    dims.y = params.height;
                }
                DEBUG("\t pos_min %d %d %d\n", pos_min.x, pos_min.y, pos_min.z);
                DEBUG("\t pos_max %d %d %d\n", pos_max.x, pos_max.y, pos_max.z);
                DEBUG("\t Dims: %d %d %d\n", dims.x, dims.y, dims.z);
                DEBUG("\t Center: %d %d %d\n", center.x, center.y, center.z);
            }


            /// Uniform Distribution
            Vector3s dims (params.volumeDims.x,params.volumeDims.y,params.volumeDims.z);
            SCLOG(DEBUG) << "Volume Extraction Dims: " << dims;
            ORUtils::MemoryBlock<float> DataScan(dims.x*dims.y*dims.z,true,true);
            ORUtils::MemoryBlock<float> DataGT(dims.x * dims.y * dims.z, true, true);
            ORUtils::MemoryBlock<bool>   DataMask(dims.x*dims.y*dims.z,true,true);

            SCLOG(VERBOSE) << "Start sampling sub-volume.\n";
            size_t counter=0;
            char outputfilename[300];
            auto Loop=[&](const Vector3f &anchor){
                Vector3i origin;
                for(size_t i=0;i<3;++i) origin[i] = std::round(anchor[i]/settings.sceneParams.voxelSize);
                SCLOG(VERBOSE) << "origin at " << origin;

                bool state;
                if(params.checkState) {
                    try {
                        state = trainingDataGenerator.CheckLabelandDistribution(origin, dims,
                                                                                slam_reconstruction->getSLAM()->getScene(),
                                                                                slam_groundtruth->getSLAM()->getScene(),
                                                                                settings.scParams.labelNum, params.threshold_occupancy,
                                                                                params.minLabelNum,
                                                                                params.threshold_highest,
                                                                                params.threshold_occupancy_rc);
                    } catch (std::runtime_error &e){
                        std::stringstream s;
                        s << __FILE__ << ":" << __LINE__ << "[CheckLabelandDistribution] ->" << e.what();
                        throw std::runtime_error(s.str());
                    }
                } else
                    state = true;

                if(!state) return;
                DataScan.Clear(0);
                DataGT.Clear(0);
                DataMask.Clear(0);

                if(!params.dryrun) {
                    DEBUG("\tExtract to contiguous map.\n");
                    cudaDeviceSynchronize();
                    trainingDataGenerator.ExtractToUniMap(&DataScan, &DataGT, &DataMask,
                                                          origin, dims, slam_reconstruction->getSLAM()->getScene(),
                                                          slam_groundtruth->getSLAM()->getScene(),
                                                          params.convertToISDF, params.outputVoxelDistance, params.makeInitValueInf, params.ofuNorm, params.gtFromRC);
                    cudaDeviceSynchronize();
                    DataScan.UpdateHostFromDevice();
                    DataGT.UpdateHostFromDevice();
                    DataMask.UpdateHostFromDevice();

                    if (params.verbose) {
                        SCFUSION::volumeChecker("extracted_Scan", DataScan.GetData(MEMORYDEVICE_CPU),
                                                DataScan.dataSize);
                        SCFUSION::volumeChecker("extracted_MASK", DataMask.GetData(MEMORYDEVICE_CPU),
                                                DataMask.dataSize);
                    }

                    if (params.gui) {
                        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                        pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());

                        uint overlapCounter = 0;
                        for (int x_ = 0; x_ < dims.x; ++x_) {
                            for (int y_ = 0; y_ < dims.y; ++y_) {
                                for (int z_ = 0; z_ < dims.z; ++z_) {
                                    int idx = (z_ * dims.y + y_) * dims.x + x_;
                                    bool hasRC = false, hasGT = false;
                                    bool mask = DataMask.GetData(MEMORYDEVICE_CPU)[idx];
                                    float data = DataScan.GetData(MEMORYDEVICE_CPU)[idx];
                                    float label = DataGT.GetData(MEMORYDEVICE_CPU)[idx];
                                    PointT point;
                                    point.x = origin.x + x_ * voxelSize;
                                    point.y = origin.y + y_ * voxelSize;
                                    point.z = origin.z + z_ * voxelSize;

                                    if(ITMVoxel::integrateType == SCFUSION::IntegrateType_TSDF) {
                                        if( std::abs(data) < 0.5) hasRC = true;
                                    } else {
                                        if (params.convertToISDF) {
                                            if (data > 0.8) hasRC = true;
                                        } else if (data >= 0 && mask == 0) hasRC = true;
                                    }

                                    if (label > 0) {
                                        point.r = SunCG11ColorLabel.at(label).r * 255;
                                        point.g = SunCG11ColorLabel.at(label).g * 255;
                                        point.b = SunCG11ColorLabel.at(label).b * 255;
                                        hasGT = true;
                                    }

//                                    if(hasRC){
//                                        point.r = SunCG11ColorLabel.at(label).r * 255;
//                                        point.g = 0;
//                                        point.b = 0;
//                                    }

                                    if(hasRC && hasGT) {
                                        overlapCounter++;
                                    }

//                                    if (hasRC || hasGT) {
//                                        cloud->push_back(point);
//                                    }
                                    if(hasRC)
                                        cloud->push_back(point);
                                    if(hasGT)
                                        cloud2->push_back(point);
                                }
                            }
                        }
                        printf("num of overlapping: %d\n", overlapCounter);
                        viewer->removeAllPointClouds(0);
                        viewer->removeAllPointClouds(1);
                        viewer->addPointCloud(cloud, "cloud1", vp0);
                        viewer->addPointCloud(cloud2, "cloud2", vp1);
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20,
                                                                 "cloud1", vp0);
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20,
                                                                 "cloud2", vp1);
//                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "cloud1",vp0);
//                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud2",vp1);
                        viewer->spin();
                        viewer->removeAllPointClouds(vp0);
//                        viewer->removeAllPointClouds(vp1);
                    }

                    sprintf(outputfilename, "%s/%08zu.npy", pth_training.c_str(), counter);
                    cnpy::npy_save(outputfilename, DataScan.GetData(MEMORYDEVICE_CPU),
                                   {static_cast<unsigned long>(dims.z), static_cast<unsigned long>(dims.y),
                                    static_cast<unsigned long>(dims.x)}, "w");

                    sprintf(outputfilename, "%s/%08zu.npy", pth_gt.c_str(), counter);
                    cnpy::npy_save(outputfilename, DataGT.GetData(MEMORYDEVICE_CPU),
                                   {static_cast<unsigned long>(dims.z), static_cast<unsigned long>(dims.y),
                                    static_cast<unsigned long>(dims.x)}, "w");

                    sprintf(outputfilename, "%s/%08zu.npy", pth_mask.c_str(), counter);
                    cnpy::npy_save(outputfilename, DataMask.GetData(MEMORYDEVICE_CPU),
                                   {static_cast<unsigned long>(dims.z), static_cast<unsigned long>(dims.y),
                                    static_cast<unsigned long>(dims.x)}, "w");

                }

                /// Save location to file
                if(!bLoadLocationFile) {
                    if(!locatFile.is_open())
                        locatFile.open(locatName, std::ios::out | std::ios::app);
                    locatFile << subfolder << "\t" << anchor.x << "\t"
                              << anchor.y << "\t" << anchor.z << "\n";
                    locatFile.close();
                }

                if(params.verbose)
                    printf("file saved with name %08zu.npy \n", counter);
                counter++;
            };


            if(bLoadLocationFile) {
                for(const auto &iter : locationMap.at(subfolder) )
                    Loop(iter);
            } else {
                auto anchors = randomPointGenerator(params.sampleNum,pos_min_, pos_max_);

                for(auto anchor : *anchors){
                    anchor.y = pos_min_.y;
                    Loop(anchor);
//                    return 0;
                }
            }
        }// else break;

        if(params.target_scene>=0) break;
    }
    return 0;
}
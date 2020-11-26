#undef __AVX2__

#include <Utils/LibSettingsIO.h>
#include <SCFusion/SLAM.h>
#include <ImageLoader/ImageLoadFactory.h>
#include <SLAMTools/SLAMWrapper.h>
#include <SLAMTools/Parser.hpp>

#include <SLAMGUI/GUI_SLAM.tpp>
#include <SLAMGUI/GUI_SCSLAM.tpp>
typedef SCFUSION::SLAM SLAMType;
template class SCFUSION::SLAMGUI<SLAMType>;
template class SCFUSION::SCSLAMGUI<SLAMType>;

#if 0
void test_plainVolume(ITMLib::ITMLibSettings *itmLibSettings){
    ITMLib::ITMScene<ITMVoxel, SCFUSION::SCPlainVoxelArray> scene (&itmLibSettings->sceneParams, false, itmLibSettings->GetMemoryType());

    auto itmMesh = new ITMLib::ITMMesh(itmLibSettings->GetMemoryType());
    ITMLib::ITMMeshingEngine<ITMVoxel, SCFUSION::SCPlainVoxelArray> *meshingEngine;
    meshingEngine = (ITMLib::ITMMeshingEngineFactory::MakeMeshingEngine<ITMVoxel, SCFUSION::SCPlainVoxelArray>(
            itmLibSettings->deviceType, ITMVoxel::integrateType, ITMVoxel::integrateType==SCFUSION::IntegrateType_TSDF? 0 : LOGODD_SURFACE));

    meshingEngine->MeshScene(itmMesh, &scene);

}
#endif

int main (int argc, char ** argv){// try {
#ifndef NDEBUG
    SCLOG_ON(VERBOSE);
#else
    //SCLOG_ON(ERROR);
    SCLOG_ON(VERBOSE);
#endif

    InputParams inputParams;
    std::unique_ptr<SCFUSION::IO::ImageLoader> imgloader_;
//    std::unique_ptr<SLAMType> slam;
    std::unique_ptr<SCFUSION::SCSLAMGUI<SLAMType>> gui;

    auto calib = ITMLib::ITMRGBDCalib();
    auto settings = ITMLib::ITMLibSettings();
    settings.deviceType = ITMLib::ITMLibSettings::DEVICE_CUDA;

    if(argc < 2) {
//        SCLOG(INFO) << "Use default library setup";
        SCLOG(ERROR) << "Need to pass a path to configuration file.\n";
    } else {
        SCFUSIONIO scfusionIO(&inputParams, &settings);
        scfusionIO.loadParams(argv[1]);
    }

    parserCommandLine(argc, argv, inputParams, &settings);///Use Parser

    /// Init SLAM
    printf("Trying to initalize image loader...\n");
    imgloader_.reset(SCFUSION::ImageLoaderFactory::MakeImageLoader(
            inputParams.inputdatatype, inputParams.folder, inputParams.cam_K, inputParams.gt,inputParams.depth_images,
            inputParams.color_images) );
//    imgloader_->Init();
//    printf("Done.\n");
//    printf("\n There are %d images in total.\n", imgloader_->NumberOfImages());
//    calib.intrinsics_rgb = imgloader_->getRGBCameraParams();
//    calib.intrinsics_d   = imgloader_->getDepthCameraParams();

    SLAMWrapper<SLAMType> slamWarpper(&settings, imgloader_.get());
    slamWarpper.useGTPose(inputParams.useGTPose);
//    slamWarpper.setInputFrameRate(33);
//    slam.reset(new SLAMType(&settings, calib));
    gui.reset(new SCFUSION::SCSLAMGUI<SLAMType>(&slamWarpper, "./volume"));
    gui->setPlotTracjectory(true);
    gui->initialization();
    gui->run();

    if(inputParams.saveMesh)
        slamWarpper.getSLAM()->saveSceneToMesh(inputParams.pth_out,false,true);
    return 0;
}
/*catch (const std::exception &exc) {
    std::cout << exc.what() << std::endl;
    return -1;
}*/


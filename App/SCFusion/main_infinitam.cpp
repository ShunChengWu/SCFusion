#undef __AVX2__
#include "../../SLAMWrapper/include/SLAMTools/EvaluationHelper.h"
#include "../../SLAMWrapper/include/SLAMTools/SLAMWrapper.h"
#include "../../SLAMWrapper/include/SLAMTools/Parser.hpp"
#include <CxxTools/PathTool.hpp>
#include <CxxTools/progressBar.hpp>
//#include <Utilities/utils.hpp>
//#include <Utilities/LogUtil.hpp>
#include <ImageLoader/ImageLoader.hpp>
#include <SCFusion/SLAM.h>
#include <ImageLoader/ImageLoadFactory.h>

using namespace SCFUSION;
typedef SCFUSION::SLAM SLAMType;


void save(SLAM *slam_, const std::string &pth_out, const InputParams &params){
    // Save Vertex Map
    slam_->saveSceneToMesh(pth_out, false, false);
    slam_->SaveToFile(pth_out);
}

int main (int argc, char ** argv) {//ry {
    InputParams inputParams;
    std::unique_ptr<SCFUSION::IO::ImageLoader> imgloader_;
    std::unique_ptr<SLAMWrapper<SLAMType>> slam_;

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

    cudaSetDevice(inputParams.device);

    /// Init SLAM
    printf("Trying to initalize image loader...\n");
    imgloader_.reset(SCFUSION::ImageLoaderFactory::MakeImageLoader(inputParams.inputdatatype, inputParams.folder, inputParams.cam_K, inputParams.gt));
    imgloader_->Init();
    printf("Done.\n");
    printf("\n There are %d images in total.\n", imgloader_->NumberOfImages());


    slam_.reset(new SLAMWrapper<SCFUSION::SLAM>(&settings, imgloader_.get()));
    slam_->useGTPose(inputParams.useGTPose);

    
    
    /// Create Log File
    RPRINTF("Create Log File\n");
    SCFUSION::EVALUATION::Logging::saveRuntimeParams(inputParams.pth_out, &settings);

    /// RUN
    std::map<std::string, std::vector<double>> times_;
    int img_counter_max_ = imgloader_->NumberOfImages();
    size_t img_counter_ = 0;
    bool terminate = false;
    printf("Running...\n");
    while(true){
        if(slam_->processOnce()>=0){

        } else {
            printf("Cannot read next image. break\n");
            break;
        }
        //if(img_counter_++ > 10) break;

        /// Save Time Information
        SCFUSION::EVALUATION::Logging::saveTimeInfo(times_);
    }
    printf("\n");
    printf("Done!\n");


    SCFUSION::EVALUATION::Logging::printResultToFile(times_, inputParams.pth_out);
    printf("log file saved!\n");
    save(slam_->getSLAM(), inputParams.pth_out, inputParams);

    return 0;
}
/*catch (const std::exception &exc) {
    std::cout << exc.what() << std::endl;
    return -1;
}*/


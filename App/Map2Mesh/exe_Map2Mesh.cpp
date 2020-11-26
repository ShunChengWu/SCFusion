#include <CxxTools/parser.hpp>
#include <CxxTools/PathTool.hpp>
#include <SLAM_base/SLAM.h>

typedef SCFUSION::SLAMBase SLAMType;
struct Params{
    std::string pth_in, pth_out, labelColorPath="/home/sc/research/scslam/Files/LabelColorLists_SceneNetRGBD.txt";
    bool labelOnly=false, checkState=false;
    Params()=default;
};

int ParseCommandLine(int argc, char** argv, Params *params){
    tools::Parser parser(argc,argv);
    parser.addOption(pkgcname("pth_in", &params->pth_in), "Path to ground truth.", true);
    parser.addOption(pkgcname("labelColorPath", &params->labelColorPath), "label color path.", false);
    parser.addOption(pkgcname("pth_out", &params->pth_out), "Path to output folder.", true);
    parser.addOption(pkgcname("labelOnly", &params->labelOnly), "labelOnly");
    parser.addOption(pkgcname("checkState", &params->checkState), "checkState");
    if(parser.showMsg()<0)
        exit(EXIT_FAILURE);
    return 0;
}

int main (int argc, char ** argv) try {
    Params params;
    ParseCommandLine(argc, argv, &params);
    auto calib = ITMLib::ITMRGBDCalib();
    auto settings = ITMLib::ITMLibSettings();
    settings.deviceType = ITMLib::ITMLibSettings::DEVICE_CUDA;
    settings.createPointExtractionEngine = false;
    settings.createMeshingEngine = true;
    settings.labelColorPath=params.labelColorPath;
    calib.intrinsics_rgb.SetFrom(640,480,525,525, 320,240);
    calib.intrinsics_d.SetFrom(640,480,525,525, 320,240);
    SLAMType slam(&settings, &calib);
    slam.LoadFromFile(params.pth_in);
    tools::PathTool::create_folder(params.pth_out);
    slam.saveSceneToMesh(params.pth_out,params.labelOnly,params.checkState);
} catch (const std::exception &exc) {
    std::cout << exc.what() << std::endl;
    return -1;
}



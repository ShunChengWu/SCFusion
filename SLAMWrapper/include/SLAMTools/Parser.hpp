#pragma once

#include <CxxTools/parser.hpp>
//#include <Utilities/Exception.hpp>
#include <ORUtils/FileIO.h>
#include <ORUtils/Logging.h>
#include <ImageLoader/ImageLoader.hpp>
#include "../../../MainLib/Utils/LibSettingsIO.h"


struct InputParams{
    SCFUSION::IO::InputDateType::INPUTDATATYPE inputdatatype;
    std::string folder, cam_K, depth_images, color_images, gt, pth_out;
    bool uniMap;
    int useGTPose;
    bool saveMesh;
    Vector3f origin;
    Vector3i dims;
    float depthscale;
    int device=0;
    InputParams(): inputdatatype(SCFUSION::IO::InputDateType::INPUTTYPE_SCANNET), folder(""), cam_K(""),
                   depth_images(""), color_images(""), gt(""), pth_out("./volume"), uniMap(false), useGTPose(0), saveMesh(false){
        origin = Vector3f(0,0,0);
        dims = Vector3i (80,48,80);
        depthscale = 1.f;
    }
};
namespace SCFUSION{
    namespace IO{
        template <> void FileIO::assignValue(SCFUSION::IO::InputDateType::INPUTDATATYPE *policy, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one integrate policy but got " << map.size() << "\n";
            *policy = SCFUSION::IO::InputDateType::ToInputDateType(std::stoi(map[0]));
        }
    }
}

class SCFUSIONIO : public SCFUSION::IO::FileIO {
    InputParams *inputParams;
    SCFUSION::IO::LibSettingsIO libSettingsIo;
public:
    explicit SCFUSIONIO(InputParams *inputParams_, ITMLib::ITMLibSettings *libSettings_) : inputParams(inputParams_), libSettingsIo(libSettings_){}

    void saveParams(const std::string &path){
        std::fstream file(path, std::ios::out);
        libSettingsIo.saveParams(&file);


        file << "\n\n#\n# InputParams # \n#\n";
        file << SCFUSION::IO::InputDateType::GetAllTypesInString() + "\n";
        saveParam(packageName(&inputParams->inputdatatype, 1), file);
        saveParam(packageName(&inputParams->folder, 1), file);
        saveParam(packageName(&inputParams->cam_K, 1), file);
        saveParam(packageName(&inputParams->depth_images, 1), file);
        saveParam(packageName(&inputParams->color_images, 1), file);
        saveParam(packageName(&inputParams->gt, 1), file);
        file.close();
    }

    void loadParams(const std::string &path){
        libSettingsIo.loadParams(path);
        auto inputData = readFileToMap(path);
        loadParam(packageName(&inputParams->inputdatatype, 1), inputData);
        loadParam(packageName(&inputParams->folder, 1), inputData);
        loadParam(packageName(&inputParams->cam_K, 1), inputData);
        loadParam(packageName(&inputParams->depth_images, 1), inputData);
        loadParam(packageName(&inputParams->color_images, 1), inputData);
        loadParam(packageName(&inputParams->gt, 1), inputData);
    }
};

int parserCommandLine(int argc, char** argv, InputParams &inputParams, ITMLib::ITMLibSettings *config, bool showMsg=false){
    tools::Parser par(argc,argv);

    par.addOption(pkgcname("skipFrame", &config->useSkipFrame), "Larger than 1 to skip frames.\n", false);
    par.addOption(pkgcname("mu", &config->sceneParams.mu), "Truncation margin.\n", false);
    par.addOption(pkgcname("mesh", &config->createMeshingEngine), "Create Meshing Engine.\n", false);
    par.addOption(pkgcname("point", &config->createPointExtractionEngine), "Create Point Extraction Engine.\n", false);
    par.addOption(pkgcname("useSC", &config->useSC), "Enable Scene Completion Model\n", false);
    par.addOption(pkgcname("labelColorPath", &config->labelColorPath), "labelColorPath\n", false);



    float voxelSize = config->sceneParams.voxelSize;
    par.addOption(pkgcname("voxelSize", &voxelSize), "Enable Scene Completion Model\n", false);
    config->sceneParams.voxelSize = voxelSize;
    config->scParams.voxelSize = voxelSize;

    int inDataType = inputParams.inputdatatype;
    par.addOption(pkgcname("inputdatatype", &inDataType),
                  SCFUSION::IO::InputDateType::GetAllTypesInString() + "\n", false);
    inputParams.inputdatatype = SCFUSION::IO::InputDateType::ToInputDateType(inDataType);
    par.addOption(pkgcname("device", &inputParams.device), "cuda device\n", false);
    par.addOption(pkgcname("folder", &inputParams.folder), "path to a folder which contains data (for ScanNet)\n", false);
    par.addOption(pkgcname("camK", &inputParams.cam_K), "path to a folder which contains camera intrinsics (for INPUTTYPE_IMAGESEQUENCE)\n", false);
    par.addOption(pkgcname("depth_images", &inputParams.depth_images), "path to a folder which contains depth iamges (for INPUTTYPE_IMAGESEQUENCE)\n", false);
    par.addOption(pkgcname("color_images", &inputParams.color_images), "path to a folder which contains color images (for INPUTTYPE_IMAGESEQUENCE)\n", false);
    par.addOption(pkgcname("gt", &inputParams.gt), "path to a ground truth pose file. \n", false);
    par.addOption(pkgcname("pthOut", &inputParams.pth_out), "path to the output directory. \n", false);
    par.addOption(pkgcname("origin", &inputParams.origin[0]), 3, "Origin of the uniMap. \n", false);
    par.addOption(pkgcname("dims", &inputParams.dims[0]), 3, "Dims of the uniMap. \n", false);
    par.addOption(pkgcname("saveUniMap", &inputParams.uniMap), "Save UniMap.\n", false);
    par.addOption(pkgcname("depthscale", &inputParams.depthscale), "input depth image scale\n");
    par.addOption(pkgcname("useGTPose", &inputParams.useGTPose), "0: ignore. 1: assist. 2:take over.\n", false);
    par.addOption(pkgcname("saveMesh", &inputParams.saveMesh), "output result to mesh.\n", false);
    int scmethod = config->scParams.sceneCompletionMethod;
    par.addOption(pkgcname("scMethod", &scmethod), "back-end semantic scene completion method.\n");
    config->scParams.sceneCompletionMethod = static_cast<SCFUSION::SceneCompletionMethod>(scmethod);
    par.addOption(pkgcname("pb", &config->scParams.pth_to_pb), "path to the Tensorflow pb model.. \n", false);
    par.addOption(pkgcname("meta", &config->scParams.pth_to_meta), "path to the meta file.. \n", false);
    par.addOption(pkgcname("ckpt", &config->scParams.pth_to_ckpt), "path to the checkpoint.. \n", false);
    par.addOption(pkgcname("inTensorName", &config->scParams.inputTensorName), "the input tensor name \n", false);
    par.addOption(pkgcname("outTensorName", &config->scParams.outputTensorName), "the output tensor name \n", false);
    par.addOption(pkgcname("labelNum", &config->scParams.labelNum), "Number of labels.\n", false);

    if(par.showMsg(showMsg) < 0) {
        exit(EXIT_FAILURE);
    }

    if(showMsg){
        par.outputLog(std::cout);
    }
    return 1;
}
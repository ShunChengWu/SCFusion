#include "LibSettingsIO.h"
#include "../ORUtils/Logging.h"
#include <string>
#include <sstream>


namespace SCFUSION {
    namespace IO {
        void LibSettingsIO::saveParams(const std::string &path) {
            std::fstream file(path, std::ios::out);
            saveParams(&file);
            file.close();
        }

        void LibSettingsIO::saveParams(std::fstream *f) {
            if (!f->is_open()) throw std::runtime_error("unable to open file.\n");
            auto &file = (*f);

            file << "#\n# Scene Params\n#\n";
            saveParam(packageName(&libSettings->sceneParams.voxelSize, 1), file);
            saveParam(packageName(&libSettings->sceneParams.maxW, 1), file);
            file << "# Integrate Policy: 0:Integrate_WEIGHTED, 1:Integrate_DIRECT\n";
            saveParam(packageName(&libSettings->sceneParams.integratePolicy, 1), file);
            saveParam(packageName(&libSettings->sceneParams.mu, 1), file);
            saveParam(packageName(&libSettings->sceneParams.stopIntegratingAtMaxW, 1), file);
            saveParam(packageName(&libSettings->sceneParams.viewFrustum_max, 1), file);
            saveParam(packageName(&libSettings->sceneParams.viewFrustum_min, 1), file);
            saveParam(packageName(&libSettings->sceneParams.useInverseSensorModel, 1), file);


            file << "#\n# Scene Completion Parameters\n#\n";
            saveParam(packageName(&libSettings->scParams.labelNum, 1), file);
            saveParam(packageName(&libSettings->scParams.useCRF, 1), file);
            saveParam(packageName(&libSettings->scParams.useThread, 1), file);
            saveParam(packageName(&libSettings->scParams.thUpdate, 1), file);
            saveParam(packageName(&libSettings->scParams.thTime, 1), file);
            saveParam(packageName(&libSettings->scParams.voxelSize, 1), file);
            saveParam(packageName(&libSettings->scParams.base_y_value, 1), file);
            saveParam(packageName(&libSettings->scParams.center_distance, 1), file);
            saveParam(packageName(&libSettings->scParams.max_sc_distance, 1), file);
            saveParam(packageName(&libSettings->scParams.min_sc_distance, 1), file);
            saveParam(packageName(&libSettings->scParams.deviceNum, 1), file);
            saveParam(packageName(&libSettings->scParams.gpu_fraction, 1), file);
            saveParam(packageName(libSettings->scParams.inputDims.data(), libSettings->scParams.inputDims.size()), file);
            saveParam(packageName(libSettings->scParams.outputDims.data(), libSettings->scParams.outputDims.size()), file);
            saveParam(packageName(&libSettings->scParams.inputTensorName, 1), file);
            saveParam(packageName(&libSettings->scParams.outputTensorName, 1), file);
            saveParam(packageName(&libSettings->scParams.pth_to_pb, 1), file);
            saveParam(packageName(&libSettings->scParams.pth_to_meta, 1), file);
            saveParam(packageName(&libSettings->scParams.pth_to_ckpt, 1), file);
            file << "#\n# SceneCompletionMethod: 0:SceneCompletionMethod_ForkNet, 1:SceneCompletionMethod_SceneInpainting\n#\n";
            saveParam(packageName(&libSettings->scParams.sceneCompletionMethod,1), file);
            file << "#\n# Fusion policy: 0:FuseTwo_OCCUPIED 1:FuseTwo_ALL_CONFIDENCE 2:FuseTwo_UNKNOWN 3: FuseTwo_UNKNOWN_CONFIDENCE 4:FuseTwo_ALL_OCCUPANCY 5:FuseTwo_UNKNOWN_OCCUPANCY"
                    "6:FuseTwo_ALL_UNWEIGHT 7:FuseTwo_UNKNOWN_UNWEIGHT\n#\n";
            saveParam(packageName(&libSettings->scParams.scfusionPolicy, 1), file);


            //TODO: add surfel params
            file << "#\n# Surfel Parameters\n#\n";
            //saveParam(packageName(&libSettings->surfelSceneParams, 1), file);


            file << "#\n# Input Paths\n#\n";
            saveParam(packageName(&libSettings->useSC, 1), file);
            file << "#\n# 0:DEVICE_CPU 1:DEVICE_CUDA 2:DEVICE_METAL\n#\n";
            saveParam(packageName(&libSettings->deviceType, 1), file);
            saveParam(packageName(&libSettings->createMeshingEngine, 1), file);
            file << "#\n#\n# 0:FAILUREMODE_RELOCALISE 1:FAILUREMODE_IGNORE 2:FAILUREMODE_STOP_INTEGRATION\n#\n";
            saveParam(packageName(&libSettings->behaviourOnFailure, 1), file);
            saveParam(packageName(&libSettings->useBilateralFilter, 1), file);
            file << "#\n# 0: SWAPPINGMODE_DISABLED 1:SWAPPINGMODE_ENABLED 2:SWAPPINGMODE_DELETE\n#\n";
            saveParam(packageName(&libSettings->swappingMode, 1), file);
            file << "#\n# 0: LIBMODE_BASIC 1:LIBMODE_BASIC_SURFEL 2:LIBMODE_LOOPCLOSURE\n#\n";
            saveParam(packageName(&libSettings->libMode, 1), file);
            saveParam(packageName(&libSettings->skipPoints, 1), file);
            saveParam(packageName(&libSettings->useSkipFrame, 1), file);
            saveParam(packageName(&libSettings->useApproximateRaycast, 1), file);
            saveParam(packageName(&libSettings->labelColorPath, 1), file);
            saveParam(packageName(&libSettings->trackerConfig, 1), file);
        }

        void LibSettingsIO::loadParams(const std::string &path) {
            auto file = readFileToMap(path);

//            file << "#\n# Scene Params\n#\n";
            loadParam(packageName(&libSettings->sceneParams.voxelSize, 1), file);
            loadParam(packageName(&libSettings->sceneParams.maxW, 1), file);
            loadParam(packageName(&libSettings->sceneParams.integratePolicy, 1), file);
            loadParam(packageName(&libSettings->sceneParams.mu, 1), file);
            loadParam(packageName(&libSettings->sceneParams.stopIntegratingAtMaxW, 1), file);
            loadParam(packageName(&libSettings->sceneParams.viewFrustum_max, 1), file);
            loadParam(packageName(&libSettings->sceneParams.viewFrustum_min, 1), file);
            loadParam(packageName(&libSettings->sceneParams.useInverseSensorModel, 1), file);

//            inputData << "#\n# Scene Completion Parameters\n#\n";
            loadParam(packageName(&libSettings->scParams.labelNum, 1), file);
            loadParam(packageName(&libSettings->scParams.useCRF, 1), file);
            loadParam(packageName(&libSettings->scParams.useThread, 1), file);
            loadParam(packageName(&libSettings->scParams.thUpdate, 1), file);
            loadParam(packageName(&libSettings->scParams.thTime, 1), file);
            loadParam(packageName(&libSettings->scParams.voxelSize, 1), file);
            loadParam(packageName(&libSettings->scParams.base_y_value, 1), file);
            loadParam(packageName(&libSettings->scParams.center_distance, 1), file);
            loadParam(packageName(&libSettings->scParams.max_sc_distance, 1), file);
            loadParam(packageName(&libSettings->scParams.min_sc_distance, 1), file);
            loadParam(packageName(&libSettings->scParams.deviceNum, 1), file);
            loadParam(packageName(&libSettings->scParams.gpu_fraction, 1), file);
            loadParam(packageName(libSettings->scParams.inputDims.data(), libSettings->scParams.inputDims.size()), file);
            loadParam(packageName(libSettings->scParams.outputDims.data(), libSettings->scParams.outputDims.size()), file);
            loadParam(packageName(&libSettings->scParams.inputTensorName, 1), file);
            loadParam(packageName(&libSettings->scParams.outputTensorName, 1), file);
            loadParam(packageName(&libSettings->scParams.pth_to_pb, 1), file);
            loadParam(packageName(&libSettings->scParams.pth_to_meta, 1), file);
            loadParam(packageName(&libSettings->scParams.pth_to_ckpt, 1), file);
            loadParam(packageName(&libSettings->scParams.sceneCompletionMethod,1), file);
            loadParam(packageName(&libSettings->scParams.scfusionPolicy, 1), file);


            //TODO: add surfel params
//            inputData << "#\n# Surfel Parameters\n#\n";
            //loadParam(packageName(&libSettings->surfelSceneParams, 1), inputData);

            loadParam(packageName(&libSettings->useSC, 1), file);
            loadParam(packageName(&libSettings->deviceType, 1), file);
            loadParam(packageName(&libSettings->createMeshingEngine, 1), file);
            loadParam(packageName(&libSettings->behaviourOnFailure, 1), file);
            loadParam(packageName(&libSettings->useBilateralFilter, 1), file);
            loadParam(packageName(&libSettings->swappingMode, 1), file);
            loadParam(packageName(&libSettings->libMode, 1), file);
            loadParam(packageName(&libSettings->skipPoints, 1), file);
            loadParam(packageName(&libSettings->useSkipFrame, 1), file);
            loadParam(packageName(&libSettings->useApproximateRaycast, 1), file);
            loadParam(packageName(&libSettings->labelColorPath, 1), file);
            loadParam(packageName(&libSettings->trackerConfig, 1), file);

            printf("Param loaded from %s\n", path.c_str());
        }

        template <> void FileIO::assignValue(SCFUSION::Policy::Integrate *policy, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one integrate policy but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "Integrate_WEIGHTED") {
                *policy = SCFUSION::Policy::Integrate_WEIGHTED;
            } else if (map[0] == "1" || map[0] == "Integrate_DIRECT") {
                *policy = SCFUSION::Policy::Integrate_DIRECT;
            } else throw std::runtime_error("Unknown mode.\n");
        }

        template <> void FileIO::assignValue(SCFUSION::Policy::FuseTwo *policy, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one fusetwo policy but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "FuseTwo_OCCUPIED") {
                *policy = SCFUSION::Policy::FuseTwo_OCCUPIED;
            } else if (map[0] == "1" || map[0] == "FuseTwo_ALL_CONFIDENCE") {
                *policy = SCFUSION::Policy::FuseTwo_ALL_CONFIDENCE;
            }  else if (map[0] == "2" || map[0] == "FuseTwo_UNKNOWN") {
                *policy = SCFUSION::Policy::FuseTwo_UNKNOWN;
            } else if (map[0] == "3" || map[0] == "FuseTwo_UNKNOWN_CONFIDENCE") {
                *policy = SCFUSION::Policy::FuseTwo_UNKNOWN_CONFIDENCE;
            } else if (map[0] == "4" || map[0] == "FuseTwo_ALL_OCCUPANCY") {
                *policy = SCFUSION::Policy::FuseTwo_ALL_OCCUPANCY;
            } else if (map[0] == "5" || map[0] == "FuseTwo_UNKNOWN_OCCUPANCY") {
                *policy = SCFUSION::Policy::FuseTwo_UNKNOWN_OCCUPANCY;
            } else if (map[0] == "6" || map[0] == "FuseTwo_ALL_UNWEIGHT") {
                *policy = SCFUSION::Policy::FuseTwo_ALL_UNWEIGHT;
            } else if (map[0] == "7" || map[0] == "FuseTwo_UNKNOWN_UNWEIGHT") {
                *policy = SCFUSION::Policy::FuseTwo_UNKNOWN_UNWEIGHT;
            } else throw std::runtime_error("Unknown mode.\n");
        }

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::DeviceType *type, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one device type but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "DEVICE_CPU") {
                *type = ITMLib::ITMLibSettings::DeviceType::DEVICE_CPU;
            } else if (map[0] == "1" || map[0] == "DEVICE_CUDA") {
                *type = ITMLib::ITMLibSettings::DeviceType::DEVICE_CUDA;
            } else if (map[0] == "2" || map[0] == "DEVICE_METAL") {
                *type = ITMLib::ITMLibSettings::DeviceType::DEVICE_METAL;
            } else throw std::runtime_error("Unknown mode.\n");
        }

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::FailureMode *mode, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one faulure mode but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "FAILUREMODE_RELOCALISE") {
                *mode = ITMLib::ITMLibSettings::FailureMode::FAILUREMODE_RELOCALISE;
            } else if (map[0] == "1" || map[0] == "FAILUREMODE_IGNORE") {
                *mode = ITMLib::ITMLibSettings::FailureMode::FAILUREMODE_IGNORE;
            } else if (map[0] == "2" || map[0] == "FAILUREMODE_STOP_INTEGRATION") {
                *mode = ITMLib::ITMLibSettings::FailureMode::FAILUREMODE_STOP_INTEGRATION;
            } else throw std::runtime_error("Unknown mode.\n");
        }

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::SwappingMode *mode, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one swapping mode but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "SWAPPINGMODE_DISABLED") {
                *mode = ITMLib::ITMLibSettings::SwappingMode::SWAPPINGMODE_DISABLED;
            } else if (map[0] == "1" || map[0] == "SWAPPINGMODE_ENABLED") {
                *mode = ITMLib::ITMLibSettings::SwappingMode::SWAPPINGMODE_ENABLED;
            } else if (map[0] == "2" || map[0] == "SWAPPINGMODE_DELETE") {
                *mode = ITMLib::ITMLibSettings::SwappingMode::SWAPPINGMODE_DELETE;
            } else throw std::runtime_error("Unknown mode.\n");
        }

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::LibMode *mode, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one lib mode but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "LIBMODE_BASIC") {
                *mode = ITMLib::ITMLibSettings::LibMode::LIBMODE_BASIC;
            } else if (map[0] == "1" || map[0] == "LIBMODE_BASIC_SURFELS") {
                *mode = ITMLib::ITMLibSettings::LibMode::LIBMODE_BASIC_SURFELS;
            } else if (map[0] == "2" || map[0] == "LIBMODE_LOOPCLOSURE") {
                *mode = ITMLib::ITMLibSettings::LibMode::LIBMODE_LOOPCLOSURE;
            } else throw std::runtime_error("Unknown mode.\n");
        }

        template <> void FileIO::assignValue(SCFUSION::SceneCompletionMethod *method, const std::vector<std::string> &map){
            if(map.size() > 1) SCLOG(ERROR) << "Should provide only one lib mode but got " << map.size() << "\n";
            if(map[0] == "0" || map[0] == "SceneCompletionMethod_ForkNet") {
                *method = SCFUSION::SceneCompletionMethod ::SceneCompletionMethod_ForkNet;
            } else if (map[0] == "1" || map[0] == "SceneCompletionMethod_SceneInpainting") {
                *method = SCFUSION::SceneCompletionMethod ::SceneCompletionMethod_SceneInpainting;
            }  else throw std::runtime_error("Unknown mode.\n");
        }
    }
}
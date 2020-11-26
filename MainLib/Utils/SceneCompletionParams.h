#pragma once
#include <string>
#include <utility>
#include <vector>
#include "../Engines/SceneCompletion/FusionPolicies.h"

namespace SCFUSION {
    struct SceneCompletionParams {
        std::string pth_to_pb;
        std::string pth_to_meta;
        std::string pth_to_ckpt;
        std::string inputTensorName;
        std::string outputTensorName;
        std::vector<int> inputDims;
        std::vector<int> outputDims;
        int deviceNum;
        float gpu_fraction;
        float voxelSize;
        SCFUSION::Policy::FuseTwo scfusionPolicy;
        SceneCompletionMethod sceneCompletionMethod;
        int labelNum;
        bool useCRF;
        /// run SC on a different thread
        bool useThread = true;
        /// update threshold. The percentage of updated voxels should exeed this.
        float thUpdate = 0.05;
        float thTime = 10;

        float min_sc_distance;
        float max_sc_distance;
        float center_distance;
        float base_y_value; //TODO: make this an plane alignment


        explicit SceneCompletionParams(std::string pth_to_pb_, std::string pth_to_meta_, std::string pth_to_ckpt_,
                                       std::string inputTensorName_, std::string outputTensorName_, int labelNum_,
                                       bool useCRF_, bool useThread_, float thUpdate_, std::vector<int> inputDims_,
                                       std::vector<int> outputDims_, int deviceNum_, float gpu_fraction_, float voxelSize_,
                                       SCFUSION::Policy::FuseTwo scfusionPolicy_, SceneCompletionMethod  sceneCompletionMethod_,
                                       float min_sc_distance_, float max_sc_distance_, float center_distance_, float base_y_value_):
                pth_to_pb(std::move(pth_to_pb_)),
                pth_to_meta(std::move(pth_to_meta_)),
                pth_to_ckpt(std::move(pth_to_ckpt_)),
                inputTensorName(std::move(inputTensorName_)),
                outputTensorName(std::move(outputTensorName_)),
                inputDims(std::move(inputDims_)),
                outputDims(std::move(outputDims_)),
                deviceNum(deviceNum_),
                gpu_fraction(gpu_fraction_),
                voxelSize(voxelSize_),
                scfusionPolicy(scfusionPolicy_),
                sceneCompletionMethod(sceneCompletionMethod_),
                labelNum(labelNum_),
                useCRF(useCRF_),
                useThread(useThread_),
                thUpdate(thUpdate_),
                min_sc_distance(min_sc_distance_),
                max_sc_distance(max_sc_distance_),
                center_distance(center_distance_),
                base_y_value(base_y_value_)
                {}
    };
}
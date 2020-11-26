#pragma once
#ifdef COMPILE_WITH_TENSORFLOW
#include "../../../../TFCpp/include/TFCpp/TFCppTools.h"
#include "SceneCompletionEngine.h"
namespace SCFUSION {

    class ForkNet : public NetworkBackEnd  {
    public:
        ForkNet(std::string path_pb, std::string inputTensorName, std::string outputTensorName, std::vector<int> inputDims, std::vector<int> outputDims, float gpu_fraction);
        ForkNet(std::string pth_to_meda, std::string pth_to_ckpt, std::string inputTensorName, std::string outputTensorName, std::vector<int> inputDims, std::vector<int> outputDims, float gpu_fraction);
        ~ForkNet();

        void process(float * data, size_t size);
        void process(float * data, float * confidense, size_t size);

        void compute(float *data_in, unsigned int size);
        void getResult(float *data_out, float *confidense_out, size_t size);
//        void getResult(float *data_out, LogOddLabels *confidense_out, size_t size);
        void getResult(float *data_out, size_t size);
    protected:
        std::string inputTensorName_, outputTensorName_;
        std::vector<int> inputDims_, outputDims_;
        std::unique_ptr<tensorflow::TFCpp> tfcpp_;
        size_t outputSize_, inputSize_;
//        float *data_buffer_, *conf_buffer_;
        void init();
    };
}


#endif
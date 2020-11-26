#ifdef COMPILE_WITH_TENSORFLOW
#include "ForkNet.h"


namespace SCFUSION {

    ForkNet::ForkNet(std::string path_pb, std::string inputTensorName, std::string outputTensorName,
            std::vector<int> inputDims, std::vector<int> outputDims, float gpu_fraction): inputTensorName_(inputTensorName), outputTensorName_(outputTensorName),
            inputDims_(inputDims),outputDims_(outputDims)
            {
        tfcpp_.reset(new tensorflow::TFCpp());
        tfcpp_->setGPUFraction(gpu_fraction);
        tfcpp_->init(path_pb);
        init();
    }
    
    ForkNet::ForkNet(std::string pth_to_meda, std::string pth_to_ckpt, std::string inputTensorName, std::string outputTensorName, std::vector<int> inputDims, std::vector<int> outputDims, float gpu_fraction ):
    inputTensorName_(inputTensorName), outputTensorName_(outputTensorName), inputDims_(inputDims),outputDims_(outputDims){
        tfcpp_.reset(new tensorflow::TFCpp());
        tfcpp_->setGPUFraction(gpu_fraction);
        tfcpp_->init(pth_to_meda , pth_to_ckpt );
        init();
    }

    
    ForkNet::~ForkNet(){
    }

    
    void ForkNet::init(){
        tfcpp_->addInput(inputTensorName_, inputDims_, tensorflow::DT_FLOAT);
        tfcpp_->addOutput(outputTensorName_);
        
        outputSize_ = inputSize_ = 1;
        for (auto s: outputDims_)
            outputSize_ *= s;
        for (auto s: inputDims_)
            inputSize_ *= s;
    }

    
    void ForkNet::process(float * data, size_t size){
        compute(data, size);
        getResult(data, size);
    }

    
    void ForkNet::process(float * data, float * confidence, size_t size){
        compute(data, size);
        getResult(data, confidence, size);
    }

    void ForkNet::compute(float *data_in, unsigned int size){
        tfcpp_->setInput(inputTensorName_, data_in, size);
        tfcpp_->compute();
    }
    void ForkNet::getResult(float *data_out, float *confidense_out, size_t size){
        tfcpp_->copyOutput(outputTensorName_, data_out, confidense_out, size);
    }
//    void ForkNet::getResult(float *data_out, LogOddLabels *confidense_out, size_t size){
//        tfcpp_->copyOutput(outputTensorName_, data_out, confidense_out, size);
//    }
    void ForkNet::getResult(float *data_out, size_t size){
        tfcpp_->copyOutput(outputTensorName_, data_out, size);
    }
}


#endif
#ifdef COMPILE_WITH_PYTORCH
#include "SceneInpainting.h"
#include "../../../../ORUtils/LogUtil.h"

namespace SCFUSION {

    SceneInpainting::SceneInpainting(const std::string &path_pb, const std::vector<int> &inputDims, const std::vector<int> &inputStride, int device) {
        pytorchcpp_.reset(new torch::PytorchCpp(path_pb,device,true,true));
        for(auto d : inputDims) inputDims_.push_back(d);
        for(auto d : inputStride) inputStride_.push_back(d);
    }
    SceneInpainting::SceneInpainting(const std::string &path_pb, const std::vector<int64_t> &inputDims, const std::vector<int64_t> &inputStride, int device) {
        pytorchcpp_.reset(new torch::PytorchCpp(path_pb,device,true,true));
        inputDims_ = inputDims;
        inputStride_ = inputStride;
    }

    void SceneInpainting::compute(float * data, bool * mask){
        TICK("[SceneInpainting][compute]1.resetInput");
        pytorchcpp_->resetInputs();
        TOCK("[SceneInpainting][compute]1.resetInput");
        TICK("[SceneInpainting][compute]2.addInputs");
        pytorchcpp_->addInput(data,inputDims_);
        pytorchcpp_->addInput(mask,inputDims_);
        TOCK("[SceneInpainting][compute]2.addInputs");
        TICK("[SceneInpainting][compute]3.process");
        pytorchcpp_->process();
        TOCK("[SceneInpainting][compute]3.process");
    }

    void SceneInpainting::getResult(float *data_out, size_t size) {
        pytorchcpp_->copyOutput(data_out,size);
    }
    void SceneInpainting::getResult(float *data_out, float *confidense_out, size_t size) {
        pytorchcpp_->copyOutput(data_out, confidense_out, size);
    }
}

#endif
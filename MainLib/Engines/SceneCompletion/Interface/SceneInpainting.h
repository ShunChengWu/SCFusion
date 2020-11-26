#pragma once
#ifdef COMPILE_WITH_PYTORCH
#include "../../../../PytorchCpp/include/PytorchCpp/PytorchCpp.h"
//#include "../../../../Utilities/include/Utilities/LogUtil.hpp"
#include "SceneCompletionEngine.h"
namespace SCFUSION {

    class SceneInpainting : public NetworkBackEnd {
    public:
        /**
         *
         * @param path_pb
         * @param inputDims
         * @param inputStride
         * @param device -1 for auto detection
         */
        SceneInpainting(const std::string &path_pb, const std::vector<int> &inputDims, const std::vector<int> &inputStride, int device);

        /**
         *
         * @param path_pb
         * @param inputDims
         * @param inputStride
         * @param device -1 for auto detection
         */
        SceneInpainting(const std::string &path_pb, const std::vector<int64_t> &inputDims, const std::vector<int64_t> &inputStride, int device);

        ~SceneInpainting() = default;

        void compute(float * data, bool * mask);
        void getResult(float *data_out, float *confidense_out, size_t size);
        void getResult(float *data_out, size_t size);

    protected:
        std::vector<int64_t> inputDims_, inputStride_;
        std::unique_ptr<torch::PytorchCpp> pytorchcpp_;
    };
}
#endif
#include <torch/script.h> // One-stop header.
#include <c10/cuda/CUDAStream.h>
#include <c10/cuda/CUDAGuard.h>
#include <memory>

namespace torch{
    struct Exception : public std::exception
    {
        std::string message;
        Exception(const char *file, const char *function, int line, std::basic_string<char> msg){
            message = "["+std::string(file)+"]["+std::string(function)+"]["+std::to_string(line)+"]: "+std::string(std::move(msg));
        }
        Exception(const char *file, const char *function, int line, std::string &msg){
            message = "["+std::string(file)+"]["+std::string(function)+"]["+std::to_string(line)+"]: "+msg;
        }
        const char * what () const noexcept override
        {
            return message.c_str();
        }
    };
#define THROW(msg) throw Exception(__FILE__,__FUNCTION__,__LINE__,msg)

    class PytorchCpp {
    public:
        explicit PytorchCpp(const std::string &path_to_module, int device_index = -1, bool use_streams = false, bool showInfo=false): stream_(nullptr){
            bool high_stream_priority = true;
            DeviceIndex deviceIndex = torch::cuda::device_count()-1;
            if(device_index >= 0){
                if(device_index > torch::cuda::device_count()-1)
                    THROW("Given device index exceed the detected device number.");
                deviceIndex = device_index;
            } else
                deviceIndex = torch::cuda::device_count()-1;

            // Check device
            torch::DeviceType device_type_ = torch::hasCUDA()? torch::kCUDA: torch::kCPU;
            this->device = std::make_unique<torch::Device>(device_type_, deviceIndex);
            if(use_streams) stream_ = std::make_unique<c10::cuda::CUDAStream>(c10::cuda::getStreamFromPool(high_stream_priority, deviceIndex));

            try {
                module = torch::jit::load(path_to_module, *this->device);// 使用 torch::jit::load() 反序列化並讀入 TorchScript 的輸出檔案
            }
            catch (const c10::Error& e) {
                THROW("module not loaded");
            }

            if(this->device->is_cpu()) LOG(WARNING) << "CUDA not available for segmentation Pytorch";
            //this->module.to(*this->device, false);

            if(showInfo) {
                std::cout << "===[PytorchCpp]===\n";
                std::cout << "device_index: " << deviceIndex << "\n";
                std::cout << "use_streams: " << use_streams << "\n";
                if (stream_)
                    std::cout << "stream id: " << stream_->id() << "\n";
            }
        }

        template<class T>
        void addInput(T *data, const std::vector<std::int64_t> &shape, const std::vector<std::int64_t> &stride) {
            torch::ScalarType type;
            if(typeid(T) == typeid(float)) type = torch::kFloat;
            else if(typeid(T) == typeid(int)) type = torch::kInt;
            else if(typeid(T) == typeid(bool)) type = torch::kBool;
            else if(typeid(T) == typeid(short)) type = torch::kShort;
            else if(typeid(T) == typeid(double)) type = torch::kDouble;
            else if(typeid(T) == typeid(char)) type = torch::kChar;
            else if(typeid(T) == typeid(long)) type = torch::kLong;
            else throw std::runtime_error("Input scalar type not supported!\n");
            if (stream_) {
                c10::cuda::CUDAStreamGuard guard(*stream_);
                torch::Tensor input = torch::from_blob(data, shape, stride, type); // b, w, h, c
                input = input.to(*device, true);
                inputs.emplace_back(input);
            } else {
                torch::Tensor input = torch::from_blob(data, shape, stride, type); // b, w, h, c
                input = input.to(*device, true);
                inputs.emplace_back(input);
            }
        }

        template<class T>
        void addInput(T *data, const std::vector<std::int64_t> &shape) {
            torch::ScalarType type;
            if(typeid(T) == typeid(float)) type = torch::kFloat;
            else if(typeid(T) == typeid(int)) type = torch::kInt;
            else if(typeid(T) == typeid(bool)) type = torch::kBool;
            else if(typeid(T) == typeid(short)) type = torch::kShort;
            else if(typeid(T) == typeid(double)) type = torch::kDouble;
            else if(typeid(T) == typeid(char)) type = torch::kChar;
            else if(typeid(T) == typeid(long)) type = torch::kLong;
            else throw std::runtime_error("Input scalar type not supported!\n");
            if (stream_) {
                c10::cuda::CUDAStreamGuard guard(*stream_);
                torch::Tensor input = torch::from_blob(data, shape, type); // b, w, h, c
                input = input.to(*device, true);
                inputs.emplace_back(input);
            } else {
                torch::Tensor input = torch::from_blob(data, shape, type); // b, w, h, c
                input = input.to(*device, true);
                inputs.emplace_back(input);
            }
        }
        void resetInputs(){
            inputs.clear();
        }

        /// Forward once
        int process() {
            if (stream_) {
                c10::cuda::CUDAStreamGuard guard(*stream_);
                output = module.forward(inputs).toTensor();
            } else {
                output = module.forward(inputs).toTensor();
            }
            return 1;
        }


        void copyOutput(float *data_out, float *confidense_out, size_t size){
            if(stream_) stream_->synchronize();
            std::tuple<torch::Tensor,torch::Tensor> max = output.max(1);
            const torch::Tensor &values = std::get<0>(max).cpu();
            const torch::Tensor &indices = std::get<1>(max).cpu().toType(torch::ScalarType::Float);

            memcpy(confidense_out, values.data_ptr<float>(), sizeof(float)*size);
            memcpy(data_out, indices.data_ptr<float>(), sizeof(float)*size);
        }
        void copyOutput(float *data_out, size_t size) {
            if(stream_) stream_->synchronize();
            std::tuple<torch::Tensor,torch::Tensor> max = output.max(1);
            const torch::Tensor &indices = std::get<1>(max).detach().cpu().toType(torch::ScalarType::Float);
            memcpy(data_out, indices.data_ptr<float>(), sizeof(float)*size);
        }

        torch::jit::script::Module module;
        std::unique_ptr<torch::Device> device;
        std::vector<torch::jit::IValue> inputs;
        torch::Tensor output;
        std::unique_ptr<c10::cuda::CUDAStream> stream_;

    };


}
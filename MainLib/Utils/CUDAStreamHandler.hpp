//
//  LogUtil.hpp
//  DFGGUI
//
//  Created by Shun-Cheng Wu on 06/Jan/2019.
//

#ifndef CUDASTREAMHANDLER_HPP
#define CUDASTREAMHANDLER_HPP

#include <cuda_runtime.h>
#include <map>
#include <string>

//#define getCUDAStream CUDA_Stream_Handler::getInstance()
template <typename T>
class CUDA_Stream_Handler {
public:
    CUDA_Stream_Handler(){}
    ~CUDA_Stream_Handler(){
        reset();
    }
    /// nonBlock allows the curent stream to be run concurrently with the zero stream.
    void createStream(T name, bool nonBlocking)  {
        if(streams.find(name) == streams.end()) {
            if(nonBlocking)
                cudaStreamCreateWithFlags(&streams[name], cudaStreamNonBlocking);
            else
                cudaStreamCreate(&streams[name]);
        }
    }
    void syncStream(T name)  {
        cudaStreamSynchronize(streams[name]);

    }

    void detoryStream(T name) {
        if(streams.find(name) != streams.end()) {
            cudaStreamDestroy(streams.at(name));
            streams.erase(name);
        }
    }

    void reset(){
        for(auto pair : streams) {
            cudaStreamDestroy(pair.second);
        }
        streams.clear();
    }

    cudaStream_t getStream(T name) {
#ifndef NDEBUG
        if(streams.find(name) == streams.end()) {
            printf("[WARNING][CUDA_Stream_Handler] Stream was not initialized!! Return default stream.\n");
            return static_cast<cudaStream_t>(nullptr);
        }
#endif
        return streams[name];
    }
private:
    std::map<T, cudaStream_t> streams;
};

template<>
class CUDA_Stream_Handler<std::string> {
public:
    CUDA_Stream_Handler(){}
    ~CUDA_Stream_Handler(){
        for(auto pair : streams) {
            cudaStreamDestroy(pair.second);
        }
    }
    /// nonBlock allows the curent stream to be run concurrently with the zero stream.
    void createStream(std::string name, bool nonBlocking){
        if(streams.find(name) == streams.end()) {
            if(nonBlocking)
                cudaStreamCreateWithFlags(&streams[name], cudaStreamNonBlocking);
            else
                cudaStreamCreate(&streams[name]);
        }
    }
    void syncStream(std::string name){
        cudaStreamSynchronize(streams[name]);
    }

    cudaStream_t getStream(std::string name) {
#ifndef NDEBUG
        if(streams.find(name) == streams.end()) {
            printf("[WARNING][CUDA_Stream_Handler] Stream \"%s\" not initialized!! Return default stream.\n", name.c_str());
            return static_cast<cudaStream_t>(0);
        }
#endif
        return streams[name];
    }

//public:
//    static CUDA_Stream_Handler& getInstance()
//    {
//        static CUDA_Stream_Handler ConfigGUIInstance;
//        return ConfigGUIInstance;
//    }
private:
    std::map<std::string, cudaStream_t> streams;
};
#endif /* LogUtil_hpp */

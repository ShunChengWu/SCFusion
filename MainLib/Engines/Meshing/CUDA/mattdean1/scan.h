#pragma once
#include <cuda_runtime.h>
namespace ParallelScan{
//    long sequential_scan(int* output, int* input, int length);
//    float blockscan(int *output, int *input, int length, bool bcao);

//    float scan(int *output, int *input, int length, bool bcao, cudaStream_t stream = nullptr);
//    float scan(unsigned int *output, unsigned int *input, int length, bool bcao, cudaStream_t stream = nullptr);
    float scan_device(unsigned int *output, const unsigned int *input, int length, bool bcao, cudaStream_t stream = nullptr);

//    void scanLargeDeviceArray(int *output, int *input, int length, bool bcao);
//    void scanSmallDeviceArray(int *d_out, int *d_in, int length, bool bcao);
//    void scanLargeEvenDeviceArray(int *output, int *input, int length, bool bcao);
}

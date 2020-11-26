#include <stdlib.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

//#include "kernels.h"
#include "utils.h"
#include "scan.h"

#define checkCudaError(o, l) _checkCudaError(o, l, __func__)

int THREADS_PER_BLOCK = 512;
int ELEMENTS_PER_BLOCK = THREADS_PER_BLOCK * 2;

#define SHARED_MEMORY_BANKS 32
#define LOG_MEM_BANKS 5
#define CONFLICT_FREE_OFFSET(n) ((n) >> LOG_MEM_BANKS)

template<typename T>
__global__ void prescan_arbitrary(T *output, const T *input, int n, int powerOfTwo)
{
    extern __shared__ T temp[];// allocated on invocation
    int threadID = threadIdx.x;

    int ai = threadID;
    int bi = threadID + (n / 2);
    int bankOffsetA = CONFLICT_FREE_OFFSET(ai);
    int bankOffsetB = CONFLICT_FREE_OFFSET(bi);


    if (threadID < n) {
        temp[ai + bankOffsetA] = input[ai];
        temp[bi + bankOffsetB] = input[bi];
    }
    else {
        temp[ai + bankOffsetA] = 0;
        temp[bi + bankOffsetB] = 0;
    }


    int offset = 1;
    for (int d = powerOfTwo >> 1; d > 0; d >>= 1) // build sum in place up the tree
    {
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);

            temp[bi] += temp[ai];
        }
        offset *= 2;
    }

    if (threadID == 0) {
        temp[powerOfTwo - 1 + CONFLICT_FREE_OFFSET(powerOfTwo - 1)] = 0; // clear the last element
    }

    for (int d = 1; d < powerOfTwo; d *= 2) // traverse down tree & build scan
    {
        offset >>= 1;
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);

            int t = temp[ai];
            temp[ai] = temp[bi];
            temp[bi] += t;
        }
    }
    __syncthreads();

    if (threadID < n) {
        output[ai] = temp[ai + bankOffsetA];
        output[bi] = temp[bi + bankOffsetB];
    }
}
template<typename T>
__global__ void prescan_arbitrary_unoptimized(T *output, const T *input, int n, int powerOfTwo) {
    extern __shared__ T temp[];// allocated on invocation
    int threadID = threadIdx.x;

    if (threadID < n) {
        temp[2 * threadID] = input[2 * threadID]; // load input into shared memory
        temp[2 * threadID + 1] = input[2 * threadID + 1];
    }
    else {
        temp[2 * threadID] = 0;
        temp[2 * threadID + 1] = 0;
    }


    int offset = 1;
    for (int d = powerOfTwo >> 1; d > 0; d >>= 1) // build sum in place up the tree
    {
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            temp[bi] += temp[ai];
        }
        offset *= 2;
    }

    if (threadID == 0) { temp[powerOfTwo - 1] = 0; } // clear the last element

    for (int d = 1; d < powerOfTwo; d *= 2) // traverse down tree & build scan
    {
        offset >>= 1;
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            int t = temp[ai];
            temp[ai] = temp[bi];
            temp[bi] += t;
        }
    }
    __syncthreads();

    if (threadID < n) {
        output[2 * threadID] = temp[2 * threadID]; // write results to device memory
        output[2 * threadID + 1] = temp[2 * threadID + 1];
    }
}

template<typename T>
__global__ void prescan_large(T *output, const T *input, int n, T *sums) {
    extern __shared__ T temp[];

    int blockID = blockIdx.x;
    int threadID = threadIdx.x;
    int blockOffset = blockID * n;

    int ai = threadID;
    int bi = threadID + (n / 2);
    int bankOffsetA = CONFLICT_FREE_OFFSET(ai);
    int bankOffsetB = CONFLICT_FREE_OFFSET(bi);
    temp[ai + bankOffsetA] = input[blockOffset + ai];
    temp[bi + bankOffsetB] = input[blockOffset + bi];

    int offset = 1;
    for (int d = n >> 1; d > 0; d >>= 1) // build sum in place up the tree
    {
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);

            temp[bi] += temp[ai];
        }
        offset *= 2;
    }
    __syncthreads();


    if (threadID == 0) {
        sums[blockID] = temp[n - 1 + CONFLICT_FREE_OFFSET(n - 1)];
        temp[n - 1 + CONFLICT_FREE_OFFSET(n - 1)] = 0;
    }

    for (int d = 1; d < n; d *= 2) // traverse down tree & build scan
    {
        offset >>= 1;
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            ai += CONFLICT_FREE_OFFSET(ai);
            bi += CONFLICT_FREE_OFFSET(bi);

            int t = temp[ai];
            temp[ai] = temp[bi];
            temp[bi] += t;
        }
    }
    __syncthreads();

    output[blockOffset + ai] = temp[ai + bankOffsetA];
    output[blockOffset + bi] = temp[bi + bankOffsetB];
}
template<typename T>
__global__ void prescan_large_unoptimized(T *output, const T *input, int n, T *sums) {
    int blockID = blockIdx.x;
    int threadID = threadIdx.x;
    int blockOffset = blockID * n;

    extern __shared__ T temp[];
    temp[2 * threadID] = input[blockOffset + (2 * threadID)];
    temp[2 * threadID + 1] = input[blockOffset + (2 * threadID) + 1];

    int offset = 1;
    for (int d = n >> 1; d > 0; d >>= 1) // build sum in place up the tree
    {
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            temp[bi] += temp[ai];
        }
        offset *= 2;
    }
    __syncthreads();


    if (threadID == 0) {
        sums[blockID] = temp[n - 1];
        temp[n - 1] = 0;
    }

    for (int d = 1; d < n; d *= 2) // traverse down tree & build scan
    {
        offset >>= 1;
        __syncthreads();
        if (threadID < d)
        {
            int ai = offset * (2 * threadID + 1) - 1;
            int bi = offset * (2 * threadID + 2) - 1;
            int t = temp[ai];
            temp[ai] = temp[bi];
            temp[bi] += t;
        }
    }
    __syncthreads();

    output[blockOffset + (2 * threadID)] = temp[2 * threadID];
    output[blockOffset + (2 * threadID) + 1] = temp[2 * threadID + 1];
}

template<typename T>
__global__ void add(T *output, int length, T *n) {
    int blockID = blockIdx.x;
    int threadID = threadIdx.x;
    int blockOffset = blockID * length;

    output[blockOffset + threadID] += n[blockID];
}
template<typename T>
__global__ void add(T *output, int length, const T *n1, T *n2) {
    int blockID = blockIdx.x;
    int threadID = threadIdx.x;
    int blockOffset = blockID * length;

    output[blockOffset + threadID] += n1[blockID] + n2[blockID];
}

template<typename T>void scanLargeDeviceArray(T *d_out, const T *d_in, int length, bool bcao, cudaStream_t stream = nullptr);
template<typename T>void scanSmallDeviceArray(T *d_out, const T *d_in, int length, bool bcao, cudaStream_t stream = nullptr);
template<typename T>
void scanLargeEvenDeviceArray(T *d_out, const T *d_in, int length, bool bcao, cudaStream_t stream = nullptr) {
    const int blocks = length / ELEMENTS_PER_BLOCK;
    const int sharedMemArraySize = ELEMENTS_PER_BLOCK * sizeof(T);

    T *d_sums, *d_incr;
    cudaMalloc((void **)&d_sums, blocks * sizeof(T));
    cudaMalloc((void **)&d_incr, blocks * sizeof(T));

    if (bcao) {
        prescan_large<T><<<blocks, THREADS_PER_BLOCK, 2 * sharedMemArraySize, stream>>>(d_out, d_in, ELEMENTS_PER_BLOCK, d_sums);
    }
    else {
        prescan_large_unoptimized<T><<<blocks, THREADS_PER_BLOCK, 2 * sharedMemArraySize, stream>>>(d_out, d_in, ELEMENTS_PER_BLOCK, d_sums);
    }

    const int sumsArrThreadsNeeded = (blocks + 1) / 2;
    if (sumsArrThreadsNeeded > THREADS_PER_BLOCK) {
        // perform a large scan on the sums arr
        scanLargeDeviceArray<T>(d_incr, d_sums, blocks, bcao, stream);
    }
    else {
        // only need one block to scan sums arr so can use small scan
        scanSmallDeviceArray<T>(d_incr, d_sums, blocks, bcao, stream);
    }

    add<T><<<blocks, ELEMENTS_PER_BLOCK, 0 , stream>>>(d_out, ELEMENTS_PER_BLOCK, d_incr);

    cudaFree(d_sums);
    cudaFree(d_incr);
}

template<typename T>
void scanSmallDeviceArray(T *d_out, const T *d_in, int length, bool bcao, cudaStream_t stream) {
    int powerOfTwo = nextPowerOfTwo(length);

    if (bcao) {
        prescan_arbitrary<T> << <1, (length + 1) / 2, 2 * powerOfTwo * sizeof(T), stream >> >(d_out, d_in, length, powerOfTwo);
    }
    else {
        prescan_arbitrary_unoptimized<T><< <1, (length + 1) / 2, 2 * powerOfTwo * sizeof(T), stream >> >(d_out, d_in, length, powerOfTwo);
    }
}

template<typename T>
void scanLargeDeviceArray(T *d_out, const T *d_in, int length, bool bcao, cudaStream_t stream) {
    int remainder = length % (ELEMENTS_PER_BLOCK);
    if (remainder == 0) {
        scanLargeEvenDeviceArray<T>(d_out, d_in, length, bcao, stream);
    }
    else {
        // perform a large scan on a compatible multiple of elements
        int lengthMultiple = length - remainder;
        scanLargeEvenDeviceArray<T>(d_out, d_in, lengthMultiple, bcao, stream);

        // scan the remaining elements and add the (inclusive) last element of the large scan to this
        T *startOfOutputArray = &(d_out[lengthMultiple]);
        scanSmallDeviceArray<T>(startOfOutputArray, &(d_in[lengthMultiple]), remainder, bcao, stream);

        add<T><<<1, remainder, 0, stream>>>(startOfOutputArray, remainder, &(d_in[lengthMultiple - 1]), &(d_out[lengthMultiple - 1]));
    }
}

template<typename T>
long sequential_scan(T* output, T* input, int length) {
	long start_time = get_nanos();

	output[0] = 0; // since this is a prescan, not a scan
	for (int j = 1; j < length; ++j)
	{
		output[j] = input[j - 1] + output[j - 1];
	}

	long end_time = get_nanos();
	return end_time - start_time;
}

template<typename T>
float blockscan(T *output, T *input, int length, bool bcao, cudaStream_t stream = nullptr){
	T *d_out, *d_in;
	const int arraySize = length * sizeof(T);

	cudaMalloc((void **)&d_out, arraySize);
	cudaMalloc((void **)&d_in, arraySize);
	cudaMemcpy(d_out, output, arraySize, cudaMemcpyHostToDevice);
	cudaMemcpy(d_in, input, arraySize, cudaMemcpyHostToDevice);

	// start timer
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start);

	int powerOfTwo = nextPowerOfTwo(length);
	if (bcao) {
		prescan_arbitrary<<<1, (length + 1) / 2, 2 * powerOfTwo * sizeof(T), stream>>>(d_out, d_in, length, powerOfTwo);
	}
	else {
		prescan_arbitrary_unoptimized<<<1, (length + 1) / 2, 2 * powerOfTwo * sizeof(T), stream>>>(d_out, d_in, length, powerOfTwo);
	}
	
	// end timer
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float elapsedTime = 0;
	cudaEventElapsedTime(&elapsedTime, start, stop);

	cudaMemcpy(output, d_out, arraySize, cudaMemcpyDeviceToHost);

	cudaFree(d_out);
	cudaFree(d_in);
	cudaEventDestroy(start);
	cudaEventDestroy(stop);

	return elapsedTime;
}

//float ParallelScan::scan(int *output, int *input, int length, bool bcao, cudaStream_t stream) {
//	int *d_out, *d_in;
//	const int arraySize = length * sizeof(int);
//
//	cudaMalloc((void **)&d_out, arraySize);
//	cudaMalloc((void **)&d_in, arraySize);
//	cudaMemcpy(d_out, output, arraySize, cudaMemcpyHostToDevice);
//	cudaMemcpy(d_in, input, arraySize, cudaMemcpyHostToDevice);
//
//	// start timer
//	cudaEvent_t start, stop;
//	cudaEventCreate(&start);
//	cudaEventCreate(&stop);
//	cudaEventRecord(start);
//
//	if (length > ELEMENTS_PER_BLOCK) {
//		scanLargeDeviceArray<int>(d_out, d_in, length, bcao, stream);
//	}
//	else {
//		scanSmallDeviceArray<int>(d_out, d_in, length, bcao, stream);
//	}
//
//	// end timer
//	cudaEventRecord(stop);
//	cudaEventSynchronize(stop);
//	float elapsedTime = 0;
//	cudaEventElapsedTime(&elapsedTime, start, stop);
//
//	cudaMemcpy(output, d_out, arraySize, cudaMemcpyDeviceToHost);
//
//	cudaFree(d_out);
//	cudaFree(d_in);
//	cudaEventDestroy(start);
//	cudaEventDestroy(stop);
//
//	return elapsedTime;
//}
//
//float ParallelScan::scan(unsigned int *output, unsigned int *input, int length, bool bcao, cudaStream_t stream) {
//    int *d_out, *d_in;
//    const int arraySize = length * sizeof(uint);
//
//    cudaMalloc((void **)&d_out, arraySize);
//    cudaMalloc((void **)&d_in, arraySize);
//    cudaMemcpy(d_out, output, arraySize, cudaMemcpyHostToDevice);
//    cudaMemcpy(d_in, input, arraySize, cudaMemcpyHostToDevice);
//
//    // start timer
//    cudaEvent_t start, stop;
//    cudaEventCreate(&start);
//    cudaEventCreate(&stop);
//    cudaEventRecord(start);
//
//    if (length > ELEMENTS_PER_BLOCK) {
//        scanLargeDeviceArray<int>(d_out, d_in, length, bcao, stream);
//    }
//    else {
//        scanSmallDeviceArray<int>(d_out, d_in, length, bcao, stream);
//    }
//
//    // end timer
//    cudaEventRecord(stop);
//    cudaEventSynchronize(stop);
//    float elapsedTime = 0;
//    cudaEventElapsedTime(&elapsedTime, start, stop);
//
//    cudaMemcpy(output, d_out, arraySize, cudaMemcpyDeviceToHost);
//
//    cudaFree(d_out);
//    cudaFree(d_in);
//    cudaEventDestroy(start);
//    cudaEventDestroy(stop);
//
//    return elapsedTime;
//}


float ParallelScan::scan_device(unsigned int *d_out, const unsigned int *d_in, int length, bool bcao, cudaStream_t stream){
    // start timer
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);

    if (length > ELEMENTS_PER_BLOCK) {
        scanLargeDeviceArray<uint>(d_out, d_in, length, bcao, stream);
    }
    else {
        scanSmallDeviceArray<uint>(d_out, d_in, length, bcao, stream);
    }

    // end timer
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float elapsedTime = 0;
    cudaEventElapsedTime(&elapsedTime, start, stop);

    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    return elapsedTime;
}
#pragma once

#include <assert.h>
//#include <thread>

//////
/// DEBUG MESSAGE
//////
#ifdef NDEBUG
#define DEBUG(x, ...)
#else
//#define DEBUG(x) do { std::cerr << x << std::endl; } while (0)
#define DEBUG(x, ...) do { \
fprintf(stderr, x,  ##__VA_ARGS__); } while (0)
#endif
#ifdef NDEBUG
#define DEBUG_MSG(str) do { } while ( false )
#else
#define DEBUG_MSG(str) do { std::cout << str; } while( false )
#endif

namespace SCFUSION {



    template<class T, class T1, class T2, class T3>
    static inline  bool recursiveDynamicLoop(T *indexes, const T1 *lowLimits, const T2 *limits, T3 loopNumber){
        if(++indexes[loopNumber] < limits[loopNumber]) return true;
        if (loopNumber == 0) return false;
        indexes[loopNumber] = lowLimits[loopNumber];
        return recursiveDynamicLoop(indexes, lowLimits, limits, loopNumber - 1);
    }

    template <class T1, class T2>
    static inline uint calculateIdx(T1 *dims, T2 *iters, int d){
        assert(d>0);
        uint idx = iters[d-1];
        for(int i=d-2;i>=0;--i) idx = idx*dims[i]+iters[i];
        return idx;
    }

    template<class T>
    static void labelling_kernel(uint dims, const  uint *offset, const uint *blockDims, const uint *imgDims,
                                 const T *data, DisjointSet *djset){
        uint idx, idx_of;
        uint iters[dims], upLimit[dims];
        memcpy(iters, offset, sizeof(uint) * dims);
        for(size_t i=0;i<dims;++i) {
            iters[i] = offset[i];
            upLimit[i] = offset[i]+blockDims[i];
        }

        bool bContinue;
        do {
            bContinue = false;
            for(uint i=0;i<dims;++i) if(iters[i]>=imgDims[i]) {
                    bContinue=true;
                    break;
                }
            if(bContinue) continue;

            idx = calculateIdx(imgDims, iters, dims);

            for(uint i=0; i<dims;++i){
                if(iters[i] > offset[i]) {
//                    std::cout << "thread " << std::this_thread::get_id() << " sleeping...\n";
                    iters[i] -= 1;
                    idx_of = calculateIdx(imgDims, iters, dims);
                    if (data[idx_of] == data[idx]) djset->unionSet(idx, idx_of);
                    iters[i] += 1;
                }
            }
        } while (recursiveDynamicLoop(iters, offset, upLimit, dims-1));
    }

//    void boo(uint *offset, const uint* blockDims_local, const int targetDim){
//
//    }

    /// Merge one dimension. // host function
    template<typename T>
    static inline void linearOp  (uint *offset, const uint* blockDims_local, const int targetDim,
                    const uint *blockDims_global, const uint *blockNums, int dims, const uint *imgDims, const T* data, DisjointSet *djset_) {
//        DEBUG("Target Dim: %d\n", targetDim);
//        DEBUG("blockDims_local: ");
//        for (size_t i=0;i<dims;++i) DEBUG("%d ", blockDims_local[i]);
//        DEBUG("\n");

        /// device function here
        for (uint x = 1; x < blockNums[targetDim]; ++x) {
            offset[targetDim] = x * blockDims_global[targetDim] - 1;

//            DEBUG("offset_: ");
//            for (size_t i=0;i<dims;++i) DEBUG("%d ", offset[i]);
//            DEBUG("\n");

            labelling_kernel(dims, offset, blockDims_local, imgDims,
                             data, djset_);
        }
    }

    template<class Tin> // Shared Function
    static inline void linearOpWarp (uint *iters_global, uint *offset, uint* blockDims_local, int targetDim,
                       const uint *blockDims_global, const uint *blockNums, int dims, const uint *imgDims, Tin* data, DisjointSet *djset_){
//        for(int i=0;i<targetDim;++i) offset[i] = iters_global[i] * blockDims_global[i];
        linearOp(offset,blockDims_local,targetDim,blockDims_global,blockNums,dims,imgDims,data,djset_);
    }
}
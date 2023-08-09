#pragma once
#include "../Shared/disjointSet.h"
#include <memory>
#include <ORUtils/Image.h>

namespace SCFUSION {
    class ConnectedComponent {
    public:
        explicit ConnectedComponent(size_t size, MemoryDeviceType type):type_(type){
            djset.reset(new DisjointSet(size, type_));
        };

        virtual void process(ORUtils::Image<bool> *image, ORUtils::Image<uint> *output) = 0;
//        template<class Tin>
        virtual void process(bool *input, uint *output, const uint *input_dims_data, const uint *block_dims_data, uint dims) {};
    protected:
        MemoryDeviceType type_;
        std::unique_ptr<DisjointSet> djset;
    };
}
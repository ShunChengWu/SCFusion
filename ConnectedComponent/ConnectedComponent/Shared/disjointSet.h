#pragma once

#include "../../../ORUtils/MemoryBlock.h"
namespace SCFUSION {
    class DisjointSet{
    public:
        explicit DisjointSet(int size, MemoryDeviceType type):rootNum(0), type_(type),
        parents(size, type_), parents_data(nullptr){
            Reset();
        }
        ~DisjointSet()= default;

        void Reset(){
            parents.SetTo(-1);
            parents_data = parents.GetData(MEMORYDEVICE_CPU);
        }

        void Resize(size_t size){
            if(parents.dataSize != size)
                parents.Resize(size,false);
            Reset();
        }

        int find(int x) {
            if (parents_data[x] < 0)
                return x;
            else
                return parents_data[x] = find(parents_data[x]);
        }

        bool isEqual(int x, int y) {
            return find(x) == find(y);
        }

        bool unionSet(int x, int y) {
            x = find(x);
            y = find(y);
            if (x == y) {
                return false;
            }
            if (parents_data[x] > parents_data[y]) {
                std::swap(x, y);
            }
            parents_data[x] += parents_data[y];
            parents_data[y] = x;

            rootNum--;

            return true;
        }

        int getSize(int x){
            return -parents_data[find(x)];
        }
        int getTotalSize(){
            return static_cast<int>(parents.dataSize);
        }

        int getRootNum(){
            return rootNum;
        }

        const int * getParents(){return parents_data;}
    private:
        int rootNum;
        MemoryDeviceType type_;
        ORUtils::MemoryBlock<int> parents;
        int *parents_data;
    };
}

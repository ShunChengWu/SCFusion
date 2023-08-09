#pragma once

#include "../Interface/ConnectedComponent.h"
#include "../Shared/ConnectedComponent_shared.h"
#include <map>
#include <CxxTools/thread_pool.hpp>
#include <ORUtils/MemoryBlock.h>
#include <ORUtils/Image.h>

#ifdef WITH_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#endif
#include <set>
#include <random>

namespace SCFUSION{
    class ConnectedComponent_CPU : public ConnectedComponent {
    public:
        explicit ConnectedComponent_CPU(size_t size=1):ConnectedComponent(size,MEMORYDEVICE_CPU){
            /// DEBUG
            //vecColors = generateRandomColorMap();
        }

        void process(ORUtils::Image<bool> *image, ORUtils::Image<uint> *output) override{
            labelling(image);
            reLabelling(output->GetData(MEMORYDEVICE_CPU), &output->noDims.toUInt()[0], 2, 0);
//            showImg(&output->noDims.toUInt()[0], 2);
        }


        template<typename T>
        void process(T *input, uint *output, const uint *input_dims_data, const uint *block_dims_data, const uint dims) {
            labelling_block(input, input_dims_data, block_dims_data, dims);
            reLabelling(output, input_dims_data, dims, 0);
//            showImg(input_dims_data, 2);
        }

#ifdef WITH_OPENCV
        cv::Mat getColorImg(uint *tmpImg, const uint *label_dim_data, const uint dims){
            cv::Mat img(label_dim_data[1],label_dim_data[0], CV_8UC3);
            for(int h=0;h<label_dim_data[1];++h) {
                for (int w = 0; w < label_dim_data[0]; ++w) {
                    uint idx = (h*label_dim_data[0]+w);
                    if(tmpImg[idx] < vecColors.size()) {
                        img.at<cv::Vec<uchar, 3>>(idx)[0] = vecColors[tmpImg[idx]][0];
                        img.at<cv::Vec<uchar, 3>>(idx)[1] = vecColors[tmpImg[idx]][1];
                        img.at<cv::Vec<uchar, 3>>(idx)[2] = vecColors[tmpImg[idx]][2];
                    } else {
                        img.at<cv::Vec<uchar, 3>>(idx)[0] = 255;
                        img.at<cv::Vec<uchar, 3>>(idx)[1] = 255;
                        img.at<cv::Vec<uchar, 3>>(idx)[2] = 255;
                    }
                }
            }
            return img;
        }
        cv::Mat getColorImg(uint *tmpImg, const bool *mask, const uint *label_dim_data, const uint dims){
            cv::Mat img(label_dim_data[1],label_dim_data[0], CV_8UC3);
            for(int h=0;h<label_dim_data[1];++h) {
                for (int w = 0; w < label_dim_data[0]; ++w) {
                    uint idx = (h*label_dim_data[0]+w);
                    if(!mask[idx]) {
                        img.at<cv::Vec<uchar, 3>>(idx)[0] = 0;
                        img.at<cv::Vec<uchar, 3>>(idx)[1] = 0;
                        img.at<cv::Vec<uchar, 3>>(idx)[2] = 0;
                        continue;
                    }
                    if(tmpImg[idx] < vecColors.size()) {
                        img.at<cv::Vec<uchar, 3>>(idx)[0] = vecColors[tmpImg[idx]][0];
                        img.at<cv::Vec<uchar, 3>>(idx)[1] = vecColors[tmpImg[idx]][1];
                        img.at<cv::Vec<uchar, 3>>(idx)[2] = vecColors[tmpImg[idx]][2];
                    } else {
                        img.at<cv::Vec<uchar, 3>>(idx)[0] = 255;
                        img.at<cv::Vec<uchar, 3>>(idx)[1] = 255;
                        img.at<cv::Vec<uchar, 3>>(idx)[2] = 255;
                    }
                }
            }
            return img;
        }
#endif
    private:

        void labelling(const ORUtils::Image<bool> *image)
        {
            djset->Resize(image->dataSize);
            const auto &height = image->noDims.height;
            const auto &width = image->noDims.width;
            const auto *data_cpu = image->GetData(MEMORYDEVICE_CPU);

            for (int y = 0; y<height; y++){
                for (int x = 0; x<width; x++){
                    int x_ = x;
                    int y_ = y;
                    int index = y_*width + x_;

                    // check conectivity on horizontal pixel
                    x_ -= 1;
                    if(x_ >= 0) {
                        int leftIndex = y_ * width + x_;
                        if (data_cpu[leftIndex] == data_cpu[index]) {
                            djset->unionSet(index, leftIndex);
                        }
                    }

                    // check conectivity on vertical pixel
                    y_ -= 1; x_ = x;
                    if(y_ >= 0) {
                        int upIndex = y_ * width + x_;
                        if (data_cpu[upIndex] == data_cpu[index]) {
                            djset->unionSet(index, upIndex);
                        }
                    }

                }
            }
        }

        std::vector<ORUtils::Vector3<int>> vecColors;

        template<typename T>
        void labelling_block(T *img, const uint *img_dims_data, const uint *blockDims, const uint dims) {
            tools::TaskThreadPool pool(8);

            size_t imgSize = 1;
            for(size_t i=0;i<dims;++i) imgSize *= img_dims_data[i];
            djset->Resize(imgSize);

//            DEBUG("create djset with size %zu\n", imgSize);

            /// Block Ops
            uint blockNums[dims];
            for(size_t i=0;i<dims;++i) blockNums[i] = (img_dims_data[i] + blockDims[i]-1) / blockDims[i];

//            DEBUG("Blocks Nums: ");
//            for(auto bn : blockNums) DEBUG("%d ", bn);
//            DEBUG("\n");

            uint lowLimit[dims];
            uint iters[dims];
            memset(lowLimit, 0, sizeof(lowLimit));
            memset(iters, 0, sizeof(iters));

            /// each block is totally isolated
//            DEBUG("Block Labelling...\n");
            auto labelKernel=[&](uint* interLocal){
                uint offset[dims];
                for(size_t i=0;i<dims;++i) offset[i] = interLocal[i] * blockDims[i];
//                DEBUG("interLocal: ");
//                for(size_t i=0;i<dims;++i) DEBUG("%d ",interLocal[i]);
//                DEBUG("\n");
//                DEBUG("Offset: ");
//                for(auto d:offset) DEBUG("%d ",d);
//                DEBUG("\n");

                labelling_kernel(dims, offset, blockDims, img_dims_data,
                                 img, djset.get());
            };
            std::vector<uint*> iterPool;
            do {
                uint *iterLocal = new uint[dims];
                memcpy(iterLocal,iters,sizeof(iters));
                pool.runTaskWithID(std::bind(labelKernel, static_cast<uint*>(iterLocal)));
//                labelKernel(static_cast<uint*>(iterLocal));
                iterPool.push_back(iterLocal);
            } while (recursiveDynamicLoop(iters, lowLimit, blockNums, dims-1));
            pool.waitWorkComplete();
            for(uint *i:iterPool) delete []i;
//            DEBUG("Block Labelling Done\n");
            //TODO: need to find a smart way to allocate memory and let them not be deleted before the process is over

#if 1
            uint blockDims_local[dims];
            memcpy(blockDims_local, blockDims,sizeof(blockDims_local));

            int targetDim = dims;
            /// the target dim-block is sequential
//            auto kernel

            uint offset[dims];
            memset(offset,0,sizeof(offset));
            do {
                if (targetDim - 2 < 0) break;
                memcpy(iters, lowLimit, sizeof(lowLimit));

                blockDims_local[targetDim - 1] = 2;

                std::vector<uint *> offsetPool;
                std::vector<uint *> blockDimPool;
                do {
                    uint *offset_ = new uint[dims];
                    uint *blockDim_ = new uint[dims];
                    memset(offset_, 0, sizeof(uint) * dims);
                    memcpy(blockDim_, blockDims_local, sizeof(uint) * dims);
                    offsetPool.push_back(offset_);
                    blockDimPool.push_back(blockDim_);
                    for (int i = 0; i < targetDim - 1; ++i) offset_[i] = iters[i] * blockDims[i];

                    pool.runTaskWithID(std::bind(
                            linearOp<T>,offset_, blockDim_,targetDim-1,blockDims,
                            static_cast<uint*>(blockNums),dims,img_dims_data,img,djset.get()
                            ));
//                    DEBUG("iters: ");
 //                   for (size_t i = 0; i < dims; ++i) DEBUG("%d ", iters[i]);
 //                   DEBUG("\n");

   //                 linearOp(offset_, blockDim_, targetDim - 1, blockDims, blockNums, dims, img_dims_data, img, djset.get());
//                    showImg(img_dims_data, dims);
                } while (recursiveDynamicLoop(iters, lowLimit, blockNums, targetDim - 2));
                pool.waitWorkComplete();
                for(uint *i:offsetPool) delete []i;
                for(uint *i:blockDimPool) delete []i;


                blockDims_local[targetDim - 1] = img_dims_data[targetDim - 1];
//                offset[targetDim-1] = 0;

                blockDims_local[targetDim - 1] = img_dims_data[targetDim - 1];
                targetDim -= 1;
            } while (targetDim>0);

            memcpy(iters,lowLimit, sizeof(lowLimit));
            blockDims_local[0] = 2;


            linearOp<T>(offset, blockDims_local, 0, blockDims, blockNums, dims, img_dims_data, img, djset.get());

//            printf("Fianl\n");
//            showImg(img_dims_data,dims);
#else
            // horizontal
        int targetD = 0;
        ORUtils::Vector2<uint> threads_h = {block_dims_data[0], block_dims_data[1]};
        for (uint y = 0; y < noBlock_data[1]; ++y) {
            offset_data[1] = y * block_dims_data[1];
            linearOp(targetD, offset_data, block_dims_data, noBlock_data, dims, &threads_h[0], img_dims_data, img, &djset);
        }
        // vertical
        targetD = 1;
        linearOp(targetD, offset_data,block_dims_data,noBlock_data,dims,&threads_h[0], img_dims_data, img, &djset);
#endif
        }

        template<class T>
        int reLabelling(T*labelData, const uint *label_dim_data, const uint dims, int minRegionThresh)
        {
            uint iters[dims];
            uint lowLimits[dims];
            memset(iters,0, sizeof(iters));
            memset(lowLimits,0, sizeof(lowLimits));

            uint n = 0;
            std::map<uint, uint> labelContainer;
            do{
                const int index = calculateIdx(label_dim_data, iters, dims);
                const uint label = djset->find(index);
                if (djset->getSize(index) < minRegionThresh){
                    djset->unionSet(label, 0);
                    labelData[index] = 0;
                    continue;
                }
                if (labelContainer.count(label) == 0) labelContainer[label] = ++n;
                if(n == std::numeric_limits<ushort>::max()){
                    labelData[index] = 0;
                    //continue;
                    //throw std::runtime_error("too many labels. reach the numerical max of uint.\n");
                } else
                labelData[index] = labelContainer[label];
            } while(recursiveDynamicLoop(iters,lowLimits,label_dim_data, dims-1));
            return djset->getRootNum();
        }

#ifdef WITH_OPENCV
        void showImg(const uint *label_dim_data, const uint dims){
            if(dims != 2) return;
            const uint size = label_dim_data[1]*label_dim_data[0];
            uint tmpImg[size];
            reLabelling(tmpImg, label_dim_data,dims,0);
            auto img = getColorImg(tmpImg, label_dim_data,dims);
            cv::imshow("ShowImg", img);
            cv::waitKey();
        }
#endif
    };
}
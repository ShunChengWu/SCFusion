#include <cstdio>
#include <exception>
#include <iostream>

//#include "ConnectedComponent/Interface/ConnectedComponent.h"
#include "ConnectedComponent/CPU/ConnectedComponent_CPU.h"
#include "ConnectedComponent/CUDA/ConnectedComponent_CUDA.h"
//#include "ConnectedComponent.h"
#include <vector>
#include <gtest/gtest.h>
#include <random>
#include "../ORUtils/Image.h"
#include "../ORUtils/LogUtil.h"

#ifdef WITH_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#endif
template<class T>
void printImage(ORUtils::Image<T> *img){
    auto data_cpu = img->GetData(MEMORYDEVICE_CPU);
    for(int h=0;h<img->noDims.height; ++h) {
        for(int w=0;w<img->noDims.width; ++w){
            std::cout << data_cpu[h*img->noDims.width+w] << " ";
        }
        printf("\n");
    }
}


#ifdef WITH_OPENCV
template<class T>
void printDiffImage(ORUtils::Image<T> *img, ORUtils::Image<T> *img2){
    cv::Mat cvimg(img->noDims.height, img->noDims.width, CV_8UC1);
    auto data_cpu = img->GetData(MEMORYDEVICE_CPU);
    auto data_cpu2 = img2->GetData(MEMORYDEVICE_CPU);
    size_t no_diff=0;
    for(int h=0;h<img->noDims.height; ++h) {
        for(int w=0;w<img->noDims.width; ++w){
            auto idx = h*img->noDims.width+w;
            if(data_cpu[idx] != data_cpu2[idx]) {
                cvimg.at<uchar>(idx) = 255;
                no_diff++;
            } else cvimg.at<uchar>(idx) = 0;
        }
//        printf("\n");
    }
    printf("no_diff: %zu\n", no_diff);
    cv::imshow("diffImg", cvimg);
    cv::waitKey();
}
#endif

template<class T>
ORUtils::Image<bool> genDiffMask(ORUtils::Image<T> *img, ORUtils::Image<T> *img2){
    ORUtils::Image<bool> out(img->noDims,true,false);
    auto data_cpu = img->GetData(MEMORYDEVICE_CPU);
    auto data_cpu2 = img2->GetData(MEMORYDEVICE_CPU);
    auto data_out = out.GetData(MEMORYDEVICE_CPU);
    for(int h=0;h<img->noDims.height; ++h) {
        for (int w=0; w < img->noDims.width; ++w) {
            auto idx = h*img->noDims.width+w;
            if(data_cpu[idx] != data_cpu2[idx]) data_out[idx] = true;
            else data_out[idx] = false;
        }
    }
    return out;
}



#ifdef WITH_OPENCV
template<class T>
cv::Mat neighborCheckDiffImage(ORUtils::Image<T> *img, ORUtils::Image<T> *img2){
    cv::Mat cvimg(img->noDims.height, img->noDims.width, CV_8UC1);
    auto data_cpu = img->GetData(MEMORYDEVICE_CPU);
    auto data_cpu2 = img2->GetData(MEMORYDEVICE_CPU);
    size_t no_diff=0;
    for(int h=1;h<img->noDims.height; ++h) {
        for(int w=1;w<img->noDims.width; ++w){
            auto idx = h*img->noDims.width+w;
            int type1=0, type2=0;

            auto idx_f = idx-1;
            auto idx_t = idx-h;

            if(data_cpu[idx] == data_cpu[idx_f]) {
                if(data_cpu[idx] == data_cpu[idx_t])type1=1;
                else type1=2;
            } else {
                if(data_cpu[idx] == data_cpu[idx_t])type1=3;
                else type1=4;
            }

            if(data_cpu2[idx] == data_cpu2[idx_f]) {
                if(data_cpu2[idx] == data_cpu2[idx_t])type2=1;
                else type2=2;
            } else {
                if(data_cpu2[idx] == data_cpu2[idx_t])type2=3;
                else type2=4;
            }

            if(type1 != type2) {
                cvimg.at<uchar>(idx) = 255;
                no_diff++;
            } else cvimg.at<uchar>(idx) = 0;
        }
//        printf("\n");
    }
    printf("no_diff: %zu\n", no_diff);
//    cv::imshow("diffImg", cvimg);
//    cv::waitKey();
    return cvimg;
}
#endif

std::vector<bool> genRandomBianryData(int dims){
    std::vector<bool> testImage(dims);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distri_x(0,100);
    for(size_t i=0;i<testImage.size();++i)
        testImage[i] = distri_x(gen) > 50;
    return testImage;
}
void genRandomBianryData(bool *data, uint dims){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distri_x(0,100);
    for(uint i=0;i<dims;++i)
        data[i] = distri_x(gen) > 50;
}


TEST(CC, BlockHasConsistantOutput_CPU_3D) {
    uint width = 64, height = 64, depth = 128;
    float factor = 4;
    std::map<std::string, double> times;
    for (uint d = depth/factor; d <= depth; d *= 2)
    for (uint w = width/factor; w <= width; w *= 2) {
        for (uint h = height/factor; h <= height; h *= 2) {
            if(w==0 || h==0 || d == 0)continue;
            uint size = d*w*h;
            uint dims[3] = {w,h,d};
            ORUtils::MemoryBlock<bool> image(size,true,false);
            bool *i_cpu = image.GetData(MEMORYDEVICE_CPU);
            uint o_cpu_gt[size];
            uint o_cpu2[size];
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> distri_x(0,100);
                for(uint i=0;i<size;++i)
                    i_cpu[i] = distri_x(gen) > 50;
            }

            SCFUSION::ConnectedComponent_CPU cc;

//            cv::Mat gtImg;
            {
                TICK("process_direct");
                cc.process(i_cpu, o_cpu_gt, dims, dims, 3);
                TOCK("process_direct");
                //gtImg = cc.getColorImg(o_cpu_gt, &image.noDims.toUInt()[0], 2);
//                cv::imshow("gtImg", gtImg);
            }

            for(uint z= d/factor; z <=d; z*=2)
            for (uint x = w / factor; x <= w; x *= 2) {
                for (uint y = h / factor; y <= h; y *= 2) {
                    if(x==0 || y==0)continue;
                    TICK("process_block");
                   // ORUtils::Vector3<uint> block_dims = {x, y, z};
                    uint block_dims[3] = {x,y,z};
                    cc.process(i_cpu, o_cpu2, dims, block_dims, 3);
                    TOCK("process_block");

                    size_t diff = 0;
                    for(size_t i=0;i<size;++i)
                        diff += o_cpu_gt[i] != o_cpu2[i];

                    EXPECT_EQ(diff, 0) << "size[" << w << "," << h << "," << d<< "], block_size[" << x << "," << y << "," << z << "]\n";

                    printf("[%d] ImageSize[%4d, %4d, %4d], KernelSize[%4d, %4d, %4d]: [direct, block]: [%8f %8f]\n",
                            getWatch.getTimings()["process_direct"] > getWatch.getTimings()["process_block"],
                           w, h, d, x, y, z, getWatch.getTimings()["process_direct"], getWatch.getTimings()["process_block"]);
                    for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
                }
            }
        }
    }
    for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
    for (const auto &time : times) printf("[%s]: %f\n", time.first.c_str(), time.second);
}

TEST(CC, BlockHasConsistantOutput_CPU_2D) {
    uint width = 640, height = 640;
    float factor = 4;
    std::map<std::string, double> times;
    for (uint w = width/factor; w <= width; w *= 2) {
        for (uint h = height/factor; h <= height; h *= 2) {
            if(w==0 || h==0)continue;
            std::vector<bool> testImage = genRandomBianryData(w * h);
            ORUtils::Image<bool> image(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            ORUtils::Image<uint> output_gy(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            ORUtils::Image<uint> output2(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            auto i_cpu = image.GetData(MEMORYDEVICE_CPU);
            auto o_cpu_gt = output_gy.GetData(MEMORYDEVICE_CPU);
            auto o_cpu2 = output2.GetData(MEMORYDEVICE_CPU);

            for (size_t i = 0; i < testImage.size(); ++i)i_cpu[i] = testImage[i];

            SCFUSION::ConnectedComponent_CPU cc;

//            cv::Mat gtImg;
//            {
            ORUtils::Vector2<uint> block_dims = {w, h};
            cc.process(i_cpu, o_cpu_gt, &image.noDims.toUInt()[0], &block_dims[0], 2);
//                gtImg = cc.getColorImg(o_cpu_gt, &image.noDims.toUInt()[0], 2);
////                cv::imshow("gtImg", gtImg);
//            }

            for (uint x = w / factor; x <= w; x *= 2) {
                for (uint y = h / factor; y <= h; y *= 2) {
                    if(x==0 || y==0)continue;
                    TICK("process_block");
                    ORUtils::Vector2<uint> block_dims = {x, y};
                    cc.process(i_cpu, o_cpu2, &image.noDims.toUInt()[0], &block_dims[0], 2);
                    TOCK("process_block");

                    size_t diff = 0;
                    for (int h = 0; h < image.noDims.height; ++h)
                        for (int w = 0; w < image.noDims.width; ++w) {
                            int idx = h * image.noDims.width + w;
                            diff += o_cpu_gt[idx] != o_cpu2[idx];
                        }

                    EXPECT_EQ(diff, 0) << "size[" << w << "," << h << "], block_size[" << x << "," << y << "]\n";
    /*                if (diff > 0){
                        auto diff = neighborCheckDiffImage(&output2,&output_gy);
                        auto cptImg = cc.getColorImg(o_cpu2, &image.noDims.toUInt()[0], 2);
                        auto diffMask = genDiffMask(&output_gy, &output2);
                        auto gtImgMasked = cc.getColorImg(o_cpu_gt, diffMask.GetData(MEMORYDEVICE_CPU), &image.noDims.toUInt()[0], 2);
                        auto cpImgMasked = cc.getColorImg(o_cpu2, diffMask.GetData(MEMORYDEVICE_CPU), &image.noDims.toUInt()[0], 2);

                        cv::Mat hCon;
                        cv::hconcat(gtImg, cptImg, hCon);
//                        cv::hconcat(hCon, diff, hCon);
                        cv::imshow("gtImg,cptImg,diff", hCon);

                        cv::Mat hCon2, diff_mask;
                        diff_mask = gtImgMasked - cpImgMasked;
                        cv::hconcat(gtImgMasked, cpImgMasked, hCon2);
                        cv::hconcat(hCon2, diff_mask, hCon2);
                        cv::imshow("gtImgMasked,cpImgMasked", hCon2);
                        cv::waitKey();
                    }*/

                    printf("[%d] ImageSize[%4d, %4d], KernelSize[%4d, %4d]: [direct, block]: [%8f %8f]\n", getWatch.getTimings()["process_direct"] > getWatch.getTimings()["process_block"],
                           w, h, x, y, getWatch.getTimings()["process_direct"], getWatch.getTimings()["process_block"]);
                    for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
                }
            }
        }
    }
    for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
    for (const auto &time : times) printf("[%s]: %f\n", time.first.c_str(), time.second);
}

TEST(CC, BlockHasSameOutput_CPU_2D) {
    uint width = 1200, height = 1200;
    std::map<std::string, double> times;
    float factor = 4;
    for (uint w = width/factor; w <= width; w *= 2) {
        for (uint h = height/factor; h <= height; h *= 2) {
            if(w==0 || h==0)continue;
            std::vector<bool> testImage = genRandomBianryData(w * h);
            ORUtils::Image<bool> image(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            ORUtils::Image<uint> output1(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            ORUtils::Image<uint> output2(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            auto i_cpu = image.GetData(MEMORYDEVICE_CPU);
            auto o_cpu1 = output1.GetData(MEMORYDEVICE_CPU);
            auto o_cpu2 = output2.GetData(MEMORYDEVICE_CPU);

            for (size_t i = 0; i < testImage.size(); ++i)i_cpu[i] = testImage[i];

//            printf("in\n");
//            printImage(&image);

            SCFUSION::ConnectedComponent_CPU cc;

            for (uint x = w / factor; x <= w; x *= 2) {
                for (uint y = h / factor; y <= h; y *= 2) {
                    if(x==0 || y==0)continue;
                    TICK("process_direct");
                    cc.process(&image, &output1);
                    TOCK("process_direct");

                    TICK("process_block");
                    ORUtils::Vector2<uint> block_dims = {x, y};
                    cc.process(i_cpu, o_cpu2, &image.noDims.toUInt()[0], &block_dims[0], 2);
//                    cc.process(&image, &output2,{x,y});
                    TOCK("process_block");

                    size_t diff = 0;
                    for (int h = 0; h < image.noDims.height; ++h)
                        for (int w = 0; w < image.noDims.width; ++w) {
                            int idx = h * image.noDims.width + w;
                            diff += o_cpu1[idx] != o_cpu2[idx];
                        }
                    EXPECT_EQ(diff, 0) << "size[" << w << "," << h << "], block_size[" << x << "," << y << "]\n";
//                    if (diff > 0)printDiffImage(&output1, &output2);

                    printf("[%d] ImageSize[%4d, %4d], KernelSize[%4d, %4d]: [direct, block]: [%8f %8f]\n", getWatch.getTimings()["process_direct"] > getWatch.getTimings()["process_block"],
                           w, h, x, y, getWatch.getTimings()["process_direct"], getWatch.getTimings()["process_block"]);
                    for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
                }
            }

//            printf("out\n");
//            printImage(&output1);
//
//            printf("out\n");
//            printImage(&output2);
        }
    }
    for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
    for (const auto &time : times) printf("[%s]: %f\n", time.first.c_str(), time.second);
}


TEST(CC, BlockHasSameOutput_CPU_1D){
    uint size = 1e3;
    std::map<std::string, double> times;
    int h = 1;
    for(uint w=10;w<size; w*=10){
            std::vector<bool> testImage = genRandomBianryData(w*h);
            ORUtils::Image<bool> image(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            ORUtils::Image<uint> output1(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            ORUtils::Image<uint> output2(ORUtils::Vector2<int>(w, h), MEMORYDEVICE_CPU);
            auto i_cpu = image.GetData(MEMORYDEVICE_CPU);
            auto o_cpu1 = output1.GetData(MEMORYDEVICE_CPU);
            auto o_cpu2 = output2.GetData(MEMORYDEVICE_CPU);

            for(size_t i=0;i<testImage.size();++i)i_cpu[i] = testImage[i];

//            printf("in\n");
//            printImage(&image);

            SCFUSION::ConnectedComponent_CPU cc;

            for(uint x=1;x<w;++x) {
                TICK("process_direct");
                cc.process(&image, &output1);
                TOCK("process_direct");
                TICK("process_block");
                ORUtils::Vector2<uint> block_dims = {x,1};
                cc.process(i_cpu,o_cpu2,&image.noDims.toUInt()[0], &block_dims[0],2);
                TOCK("process_block");

                size_t diff = 0;
                for (int h = 0; h < image.noDims.height; ++h)
                    for (int w = 0; w < image.noDims.width; ++w) {
                        int idx = h * image.noDims.width + w;
                        diff += o_cpu1[idx] - o_cpu2[idx];
                    }
                EXPECT_EQ(diff, 0) << "size[" << w << "], block_size[" << x << "]\n";

                printf("[%d] ImageSize[%4d, %4d], KernelSize[%4d]: [direct, block]: [%8f %8f]\n",
                        getWatch.getTimings()["process_direct"] > getWatch.getTimings()["process_block"],
                       w, h, x, getWatch.getTimings()["process_direct"], getWatch.getTimings()["process_block"]);
                for (const auto &time : getWatch.getTimings()) times[time.first] += time.second;
            }

//            printf("out\n");
//            printImage(&output1);
//
//            printf("out\n");
//            printImage(&output2);
    }
    for(const auto& time : getWatch.getTimings()) times[time.first] += time.second;
    for(const auto& time : times) printf("[%s]: %f\n", time.first.c_str(),time.second);
}


TEST(CC, BlockHasCorrectLabel_CPU_1D) {
//    const std::vector<bool> testData = { 0,1,0,0,1,1,0,1};
    bool verbal = false;
    const uint dims = 1;
    const uint inputDimsData[dims] = {8};
    bool testData[inputDimsData[0]] = { 0,1,0,0,1,1,0,1};
    uint gtLabel[inputDimsData[0]] = {1,2,3,3,4,4,5,6};
    uint output[inputDimsData[0]];

    SCFUSION::ConnectedComponent_CPU cc;

    if(verbal) {
        printf("in\n");
        for (uint i = 0; i < inputDimsData[0]; ++i) std::cout << testData[i] << " ";
        printf("\n");
        printf("gt\n");
        for (uint i = 0; i < inputDimsData[0]; ++i) std::cout << gtLabel[i] << " ";
        printf("\n");
    }

    for(uint x=1;x<inputDimsData[0]; ++x){
        cc.process(testData,output,inputDimsData,&x,dims);

        if(verbal) {
            printf("out\n");
            for (uint i = 0; i < inputDimsData[0]; ++i) std::cout << output[i] << " ";
            printf("\n");
        }

        uint diff = 0;
        for(uint i=0;i<inputDimsData[0]; ++i)diff += output[i] - gtLabel[i];
        EXPECT_EQ(diff,0)  << "size[" << inputDimsData << "], block_size[" << x << "]\n";
    }
}

TEST(CC, BlockHasCorrectLabel_CPU_2D) {
//    const std::vector<bool> testData = { 0,1,0,0,1,1,0,1};
    bool verbal = false;
    const uint dims = 2;
    const uint inputDimsData[dims] = {8,8};
    bool testData[inputDimsData[0]*inputDimsData[1]] = {
            0,1,0,0,1,1,0,1,
            0,0,1,1,0,0,0,1,
            1,0,0,0,0,1,0,1,
            0,1,1,1,0,1,0,0,
            1,1,0,0,1,1,1,0,
            0,0,1,1,0,0,1,1,
            1,0,0,1,0,1,0,1,
            0,0,1,1,1,0,1,1,
    };
    uint gtLabel[inputDimsData[0]*inputDimsData[1]] = {
            1,7, 8, 8,12,12, 1,18,
            1,1, 9, 9, 1, 1, 1,18,
            2,1, 1, 1, 1,13, 1,18,
            3,4, 4, 4, 1,13, 1, 1,
            4,4,10,10,13,13,13, 1,
            5,5,11,11,14,14,13,13,
            6,5, 5,11,14,15,17,13,
            5,5,11,11,11,16,13,13,
    };
    uint output[inputDimsData[0]*inputDimsData[1]];

    SCFUSION::ConnectedComponent_CPU cc;

    if(verbal) {
        printf("in\n");
        for (uint y = 0; y < inputDimsData[1]; ++y) {
            for (uint x = 0; x < inputDimsData[0]; ++x)
                std::cout << testData[y*inputDimsData[0]+x] << " ";
            std::cout << std::endl;
        }

        printf("\n");
        printf("gt\n");
        for (uint y = 0; y < inputDimsData[1]; ++y) {
            for (uint x = 0; x < inputDimsData[0]; ++x)
                std::cout << gtLabel[y*inputDimsData[0]+x] << " ";
            std::cout << std::endl;
        }

        printf("\n");
    }

    for(uint x=1;x<inputDimsData[0]; ++x)
        for(uint y=1;y<inputDimsData[1]; ++y){
        uint kernel[2] = {x,y};

        cc.process(testData,output,inputDimsData,kernel,dims);

        if(verbal) {
            printf("out\n");
            for (uint y = 0; y < inputDimsData[1]; ++y){
                for (uint x = 0; x < inputDimsData[0]; ++x)
                    std::cout << output[y*inputDimsData[0]+x] << " ";
                std::cout << std::endl;
            }

            printf("\n");
        }

        uint diff = 0;

        for(uint i=0;i<inputDimsData[0]*inputDimsData[1]; ++i)diff += output[i] - gtLabel[i];
        EXPECT_EQ(diff,0)  << "size[" << inputDimsData[0] << "," << inputDimsData[1]  << "], block_size[" << x << "]\n";
    }
}

TEST(CC, BlockHasCorrectLabel_CPU_3D) {
//    const std::vector<bool> testData = { 0,1,0,0,1,1,0,1};
    bool verbal = false;
    const uint dims = 3;
    const uint inputDimsData[dims] = {4,4,4};
    bool testData[inputDimsData[0]*inputDimsData[1]*inputDimsData[2]] = {
            1,0,1,0,
            1,0,0,0,
            1,0,1,0,
            0,1,1,1,

            1,1,1,1,
            1,0,1,1,
            1,1,1,1,
            0,1,0,1,

            0,1,0,0,
            1,0,1,0,
            1,1,0,0,
            0,1,0,1,

            1,1,0,0,
            0,0,1,1,
            1,1,1,1,
            0,0,0,1,
    };
    uint gtLabel[inputDimsData[0]*inputDimsData[1]*inputDimsData[2]] = {
            1,3,1,3,
            1,3,3,3,
            1,3,1,3,
            4,1,1,1,

            1,1,1,1,
            1,3,1,1,
            1,1,1,1,
            4,1,4,1,

            2,1,4,4,
            1,3,1,4,
            1,1,4,4,
            4,1,4,1,

            1,1,4,4,
            3,3,1,1,
            1,1,1,1,
            4,4,4,1,
    };

    uint output[inputDimsData[0]*inputDimsData[1]*inputDimsData[2]];

    SCFUSION::ConnectedComponent_CPU cc;

    if(verbal) {
        printf("in\n");
        for (uint z_ = 0; z_ < inputDimsData[2]; ++z_) {
            for (uint y_ = 0; y_ < inputDimsData[1]; ++y_) {
                for (uint x_ = 0; x_ < inputDimsData[0]; ++x_)
                    std::cout << testData[(z_ * inputDimsData[1] + y_) * inputDimsData[0] + x_] << " ";
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        printf("\n");
        printf("gt\n");
        for (uint z_ = 0; z_ < inputDimsData[2]; ++z_) {
            for (uint y_ = 0; y_ < inputDimsData[1]; ++y_) {
                for (uint x_ = 0; x_ < inputDimsData[0]; ++x_)
                    std::cout << gtLabel[(z_ * inputDimsData[1] + y_) * inputDimsData[0] + x_] << " ";
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        printf("\n");
    }

    for(uint x=2;x<inputDimsData[0]; ++x)
        for(uint y=1;y<inputDimsData[1]; ++y)
            for(uint z=1;z<inputDimsData[2]; ++z){
                uint kernel[3] = {x,y,z};

                cc.process(testData,output,inputDimsData,kernel,dims);

                if(verbal) {
                    printf("out\n");

                    for (uint z_ = 0; z_ < inputDimsData[2]; ++z_) {
                        for (uint y_ = 0; y_ < inputDimsData[1]; ++y_) {
                            for (uint x_ = 0; x_ < inputDimsData[0]; ++x_)
                                std::cout << output[(z_ * inputDimsData[1] + y_) * inputDimsData[0] + x_] << " ";
                            std::cout << std::endl;
                        }
                        std::cout << std::endl;
                    }

                    printf("\n");
                }

                uint diff = 0;

                for(uint i=0;i<inputDimsData[0]*inputDimsData[1]; ++i)diff += output[i] - gtLabel[i];
                EXPECT_EQ(diff,0)  << "size[" << inputDimsData[0] << "," << inputDimsData[1] << "," <<
                    inputDimsData[2] << "], block_size[" << x << "," << y << "," << z << "]\n";

            }
}




//TEST(CC, BlockHasCorrectLabel_CUDA_1D) {
//    bool verbal = false;
//    const uint dims = 1;
//    const uint inputDimsData[dims] = {8};
//    bool testData_CPU[inputDimsData[0]] = { 0,1,0,0,1,1,0,1};
//    uint gtLabel[inputDimsData[0]] = {1,2,3,3,4,4,5,6};
//    ORUtils::MemoryBlock<bool> testData(inputDimsData[0],MEMORYDEVICE_CUDA);
//    ORUtils::MemoryBlock<uint> output(inputDimsData[0],true,true);
//    auto output_cpu = output.GetData(MEMORYDEVICE_CPU);
//    cudaMemcpy(testData.GetData(MEMORYDEVICE_CUDA), testData_CPU, sizeof(bool) * 8, cudaMemcpyHostToDevice);
//
//
//    SCFUSION::ConnectedComponent_CUDA cc(inputDimsData[0]);
//
//    if(verbal) {
//        printf("in\n");
//        for (uint i = 0; i < inputDimsData[0]; ++i) std::cout << testData[i] << " ";
//        printf("\n");
//        printf("gt\n");
//        for (uint i = 0; i < inputDimsData[0]; ++i) std::cout << gtLabel[i] << " ";
//        printf("\n");
//    }
//
//    for(uint x=1;x<inputDimsData[0]; ++x){
//        cc.process(testData.GetData(MEMORYDEVICE_CUDA),output.GetData(MEMORYDEVICE_CUDA),inputDimsData,&x,dims);
//        output.UpdateHostFromDevice();
//
//        if(verbal) {
//            printf("out\n");
//            for (uint i = 0; i < inputDimsData[0]; ++i) std::cout << output[i] << " ";
//            printf("\n");
//        }
//
//        uint diff = 0;
//        for(uint i=0;i<inputDimsData[0]; ++i)diff += output_cpu[i] - gtLabel[i];
//        EXPECT_EQ(diff,0)  << "size[" << inputDimsData << "], block_size[" << x << "]\n";
//    }
//}

int main (int argc, char **argv) try {
    testing::InitGoogleTest(&argc,argv);
    RUN_ALL_TESTS();
} catch (const std::exception &exc) {
    std::cout << exc.what() << std::endl;
    return -1;
}

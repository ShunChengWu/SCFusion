#pragma once
#include "../../../ORUtils/MemoryBlock.h"
#include "../../Utils/SceneCompletionParams.h"
#include "../Scene/ITMScene.h"
#include <memory>
namespace SCFUSION{
    class SCBuffer{
    public:
        const SceneCompletionParams *scparams_;
        std::unique_ptr<ORUtils::MemoryBlock<float>> iBuffer, oBuffer, iBuffer_conf, oBuffer_conf;
        std::unique_ptr<ORUtils::MemoryBlock<bool>> iMask;
        Vector3f volumeBase_;
        Vector3s volumeDims_;
        size_t volumeSize_;
        float time;
        Matrix4f pose_;
        Vector2i imgSize_;
        Vector4f projParams_;

        explicit SCBuffer(const SceneCompletionParams *params) {
            scparams_ = params;
            size_t size = 1;
            for (size_t i=1;i<scparams_->inputDims.size();++i) size *=scparams_->inputDims[i]; // ignore batch
            iBuffer.reset(new ORUtils::MemoryBlock<float>(size, true, true));//TODO: make CUDA depends on running mode (CPU or CUDA version)
            iBuffer_conf.reset(new ORUtils::MemoryBlock<float>(size, true, true));
            iMask.reset(new ORUtils::MemoryBlock<bool>(size, true, true));
            oBuffer.reset( new ORUtils::MemoryBlock<float>(size, true, true));
            oBuffer_conf.reset(new ORUtils::MemoryBlock<float>(size,true,true));
//            oBufferConf.reset( new ORUtils::MemoryBlock<LogOddLabels>(size, true, true));

            iBuffer->Clear();
            iBuffer_conf->Clear();
            iMask->Clear();
            oBuffer->Clear();
            oBuffer->Clear();

//            oBufferConf->Clear();

            volumeDims_.x = scparams_->inputDims[1];
            volumeDims_.y = scparams_->inputDims[2];
            volumeDims_.z = scparams_->inputDims[3];
            volumeSize_ = volumeDims_.x * volumeDims_.y * volumeDims_.z;
            volumeBase_.x = volumeBase_.y = volumeBase_.z = 0;
        }

        void setCentreTo(const Vector3f &centre) {
            volumeBase_.x = centre.x - 0.5 * volumeDims_.x * scparams_->voxelSize;
            volumeBase_.y = centre.y;
            volumeBase_.z = centre.z - 0.5 * volumeDims_.z * scparams_->voxelSize;
        }

        void SetBaseTo(const Vector3f &base){
            volumeBase_ = base;
        }
        
        void setPose(const Matrix4f &pos) {
            pose_ = pos;
        }

        void setImgSize(const Vector2i &imgSize){
            imgSize_ = imgSize;
        }

        void setProjParams(const Vector4f &projParams){
            projParams_ = projParams;
        }

        ~SCBuffer()=default;
    };
}
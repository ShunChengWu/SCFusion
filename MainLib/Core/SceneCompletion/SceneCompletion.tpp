#include "SceneCompletion.h"
#include "../../Engines/SceneCompletion/SceneCompletionEngineFactory.h"
#include "../../../ORUtils/LogUtil.h"
#include <vector>
//#include "../../../ORUtils/MatrixOperation.h"

using namespace SCFUSION;

template<class TVoxel, class TIndex>
SceneCompletion<TVoxel, TIndex>::SceneCompletion(const ITMLib::ITMLibSettings *settings):stream_(nullptr){
    sceneCompletionEngine_.reset(SceneCompletionEngineFactory::MakeSceneCompletionEngine<TVoxel,TIndex>(settings->scParams.sceneCompletionMethod, settings->deviceType));
    network_ = nullptr;
    switch (settings->scParams.sceneCompletionMethod) {
        case SCFUSION::SceneCompletionMethod_ForkNet: {
#ifndef COMPILE_WITH_TENSORFLOW
            throw std::runtime_error("Select to use ForkNet as backend but the system was not compiled with Tensorflow\n");
#endif
            break;
        }
        case SCFUSION::SceneCompletionMethod_SceneInpainting: {
#ifndef COMPILE_WITH_PYTORCH
            throw std::runtime_error("Select to use SceneInpainting as backend but the system was not compiled with Pytorch\n");
#endif
            break;
        }
    }

    assert(settings->scParams.inputDims.size()==4);
    assert(settings->scParams.inputDims[0]==1);
    mDenseCRFOpt.reset(new DenseCRFOptimization(
            {settings->scParams.inputDims[1],settings->scParams.inputDims[2],settings->scParams.inputDims[3]},
            settings->scParams.labelNum));
    //TODO: make this params
    float stddev = 2;
    mDenseCRFOpt->SetStddev({stddev,stddev,stddev});
    mDenseCRFOpt->SetWeight(3);
    mDenseCRFOpt->SetInterations(5);
    mDenseCRFOpt->SetRelexation(1.0);
}

template<class TVoxel, class TIndex>
void SceneCompletion<TVoxel, TIndex>::ResetSCBuffer(SCBuffer *scBuffer){
    sceneCompletionEngine_->ResetSCBuffer(scBuffer);

    auto params = scBuffer->scparams_;
    if (params->sceneCompletionMethod == SCFUSION::SceneCompletionMethod::SceneCompletionMethod_ForkNet) {
#ifdef COMPILE_WITH_TENSORFLOW
        if(!params->pth_to_pb.empty())
            network_.reset(new ForkNet(params->pth_to_pb,params->inputTensorName, params->outputTensorName,params->inputDims,params->outputDims,params->gpu_fraction));
        else if (!params->pth_to_meta.empty() && !params->pth_to_ckpt.empty())
            network_.reset(new ForkNet(params->pth_to_meta,params->pth_to_ckpt,params->inputTensorName, params->outputTensorName,params->inputDims,params->outputDims,params->gpu_fraction));
        else
            throw std::runtime_error("Need to pass pth_to_pb or both pth_to_meta and pth_to_ckpt in order to use scene completion.\n");
#endif
    } else {
#ifdef COMPILE_WITH_PYTORCH
        assert(!params->pth_to_pb.empty());
        network_.reset(new SceneInpainting(params->pth_to_pb,params->inputDims,{1,64*64,64,1}, params->deviceNum));
#endif
    }

}

template<class TVoxel, class TIndex>
SceneCompletion<TVoxel, TIndex>::~SceneCompletion()=default;

template<class TVoxel, class TIndex>
bool SceneCompletion<TVoxel, TIndex>::CheckCriteria(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene){
    float percentage = sceneCompletionEngine_->CriteriaCheck(scBuffer,scene);
    if(percentage < scBuffer->scparams_->thUpdate) {
        SCLOG(DEBUG) << "Less than threshold " << scBuffer->scparams_->thUpdate << ". Skip.";
        return false;
    }
    return true;
}

template<class TVoxel, class TIndex>
void SceneCompletion<TVoxel, TIndex>::Complete(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene){
//        if(network_ == nullptr) throw "Must ResetSCBuffer first before calling Complete.\n";
//        sceneCompletionEngine_->ResetSCBuffer(scBuffer);

    /// Extraction
//        TICK("[SceneCompletion][Complete]1.Extraction");
    sceneCompletionEngine_->Extraction(scBuffer,scene);
//        TOCK("[SceneCompletion][Complete]1.Extraction");

    switch (scBuffer->scparams_->sceneCompletionMethod) {
        case SCFUSION::SceneCompletionMethod_ForkNet: {
#ifdef COMPILE_WITH_TENSORFLOW
            auto *network_cast = reinterpret_cast<ForkNet*>(network_.get());
            TICK("[SceneCompletion][Complete]2.Compute");
            network_cast->compute(scBuffer->iBuffer->GetData(MEMORYDEVICE_CPU), scBuffer->iBuffer->dataSize);
            TOCK("[SceneCompletion][Complete]2.Compute");

            TICK("[SceneCompletion][Complete]3.GetResult");
            network_cast->getResult(scBuffer->oBuffer->GetData(MEMORYDEVICE_CPU),
                                scBuffer->oBuffer_conf->GetData(MEMORYDEVICE_CPU), scBuffer->oBuffer->dataSize);
#endif
            break;
        }
        case SCFUSION::SceneCompletionMethod_SceneInpainting: {
#ifdef COMPILE_WITH_PYTORCH
            auto *network_cast = reinterpret_cast<SceneInpainting*>(network_.get());
            TICK("[SceneCompletion][Complete]2.Compute");
            network_cast->compute(scBuffer->iBuffer->GetData(MEMORYDEVICE_CPU), scBuffer->iMask->GetData(MEMORYDEVICE_CPU));
            TOCK("[SceneCompletion][Complete]2.Compute");

            TICK("[SceneCompletion][Complete]3.GetResult");
            network_cast->getResult(scBuffer->oBuffer->GetData(MEMORYDEVICE_CPU,false), scBuffer->oBuffer_conf->GetData(MEMORYDEVICE_CPU, false),
                                scBuffer->oBuffer->dataSize);
#endif
            break;
        }
    }

    scBuffer->oBuffer->UpdateDeviceFromHost(1, stream_);
    scBuffer->oBuffer_conf->UpdateDeviceFromHost(1, stream_);
//        TOCK("[SceneCompletion][Complete]3.GetResult");
#if 0
    std::map<int,int> mask_value;
    std::map<float,int> output_value;
    scBuffer->iBuffer->UpdateHostFromDevice();
    scBuffer->iBuffer_conf->UpdateHostFromDevice();
//        scBuffer->oBuffer->UpdateHostFromDevice();
    auto *output_ptr = scBuffer->oBuffer->GetData(MEMORYDEVICE_CPU);
    auto *conf_ptr = scBuffer->iMask->GetData(MEMORYDEVICE_CPU);
    for(size_t i=0;i<scBuffer->iBuffer->dataSize; ++i) {
        if(mask_value.find(conf_ptr[i]) == mask_value.end())
            mask_value[conf_ptr[i]] =0;
        mask_value[conf_ptr[i]]++;
        if(output_value.find(output_ptr[i]) == output_value.end())
            output_value[output_ptr[i]] =0;
        output_value[output_ptr[i]]++;
    }
    printf("MASK:\n");
    for(const auto& pair:mask_value)
        printf("[%d] %d\n", pair.first,pair.second);
    printf("OUTPUT:\n");
    for(const auto& pair:output_value)
        printf("[%f] %d\n", pair.first,pair.second);
#endif
}

template<class TVoxel, class TIndex>
void SceneCompletion<TVoxel, TIndex>::Optimization(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, ITMLib::ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine,
                  SCBuffer *scBuffer, ITMLib::ITMRenderState *renderState) {
    // Extract label and confidence to buffer
    sceneCompletionEngine_->ExtractLabelAndConfidnece(scBuffer,scene);
    mDenseCRFOpt->Optimize(scBuffer->iBuffer->GetData(MEMORYDEVICE_CPU),
            scBuffer->iBuffer_conf->GetData(MEMORYDEVICE_CPU),
            scBuffer->iMask->GetDataConst(MEMORYDEVICE_CPU));
    scBuffer->iBuffer->UpdateDeviceFromHost(1, stream_);
    scBuffer->iBuffer_conf->UpdateDeviceFromHost(1, stream_);
    sceneCompletionEngine_->FusionLabelAndConfidence(scene,scBuffer);

    // Fuse back
}

template<class TVoxel, class TIndex>
void SceneCompletion<TVoxel, TIndex>::Fuse(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, ITMLib::ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine,
          const SCBuffer *scBuffer, ITMLib::ITMRenderState *renderState){
    // allocation
    cudaStreamSynchronize(static_cast<cudaStream_t>(stream_));
    sceneRecoEngine->AllocationSceneFromVolume(scene, scBuffer->oBuffer->GetData(MEMORYDEVICE_CUDA), scBuffer->volumeBase_, scBuffer->volumeDims_, renderState);

    // integration
    sceneRecoEngine->syncStream();
    sceneCompletionEngine_->Fusion(scene,scBuffer);
}

template<class TVoxel, class TIndex>
void SceneCompletion<TVoxel, TIndex>::setVolumeLocation(const Vector3f &center, SCBuffer *scBuffer){
    scBuffer->setCentreTo(center);
}

template<class TVoxel, class TIndex>
std::pair<Vector3f, Vector3f> SceneCompletion<TVoxel, TIndex>::CalculateMinMax(const ITMFloatImage *depth,
const Matrix4f &CameraPose, const Vector4f &projParams_d, const SCBuffer *scbuffer, float floorPosition, bool dryrun){
    auto imgDim = depth->noDims;
    depth->UpdateHostFromDevice();
    auto depthData = depth->GetDataConst(MEMORYDEVICE_CPU);
    Vector3f min = {0,0,0};
    Vector3f max = {0,0,0};

    //TODO: make this cuda?
    for(int x=0; x < imgDim.x; ++x){
        for(int y=0; y < imgDim.y; ++y){
            auto idx = y * imgDim.x + x;
            float depthvalue = depthData[idx] <= 0 ? 0.01 : depthData[idx];
            depthvalue = CLAMP(depthvalue,0.01,5);
            const Vector4f pt_camera = {
                    (x - projParams_d.z) / projParams_d.x * depthvalue,
                    (y - projParams_d.w) / projParams_d.y * depthvalue,
                    depthvalue, 1.f
            };
            const Vector4f pt_world = CameraPose * pt_camera;

            if(idx == 0) {
                max = min = pt_world.toVector3();
            } else {
                if (pt_world.x < min.x) min.x = pt_world.x;
                if (pt_world.y < min.y) min.y = pt_world.y;
                if (pt_world.z < min.z) min.z = pt_world.z;
                if (pt_world.x > max.x) max.x = pt_world.x;
                if (pt_world.y > max.y) max.y = pt_world.y;
                if (pt_world.z > max.z) max.z = pt_world.z;
            }
        }
    }

    if(!dryrun) {
        // Calculate extraction center
        Vector2f stride = {scbuffer->volumeDims_.x * scbuffer->scparams_->voxelSize,
                           scbuffer->volumeDims_.z * scbuffer->scparams_->voxelSize};

        std::unique_lock<std::mutex> lock(mutex_extraction_centers_);
        size_t counter = 0;
        for (float x = min.x; x < max.x; x += stride.x) {
            for (float z = min.z; z < max.z; z += stride.y) {
                auto entry = std::make_shared<SCQueue>();
                entry->imgSize = imgDim;
                entry->depthParam = projParams_d;
                entry->pose = CameraPose.inv();
                entry->extraction_base = {x, floorPosition, z};
                extraction_centers_.push(entry);
                SCLOG(DEBUG) << "Add extraction center: " << x << ", " << floorPosition << ", " << z;
                counter++;
            }
        }
    }

    return {min,max};
}
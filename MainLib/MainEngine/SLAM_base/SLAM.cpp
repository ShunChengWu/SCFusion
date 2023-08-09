#include "SLAM.h"
#include <ORUtils/LogUtil.h>
//#include "../../../ORUtils/EigenHelper.h"

#include "../../Engines/Visualisation/ITMVisualisationEngineFactory.h"

#include <future>
#include <thread>
#include <chrono>
#include <ORUtils/MatrixOperation.h>
#include <ORUtils/FileUtils.h>
#include <ORUtils/Logging.h>


using namespace SCFUSION;

SLAMBase::SLAMBase(const ITMLib::ITMLibSettings *itmLibSettings, const ITMLib::ITMRGBDCalib *calib):
MainEngine(itmLibSettings, calib), MeshEngine(itmLibSettings), PointCloudEngine(itmLibSettings){
    performed_sc_ = false;
    CUDA_streams.createStream(STREAM_FRAME, true);
    CUDA_streams.createStream(STREAM_MAP, true);
    CUDA_streams.createStream(STREAM_SC, true);
    CUDA_streams.createStream(STREAM_VISUALIZATION, true);

    itmMapper->setStream(CUDA_streams.getStream(STREAM_MAP));
    itmViewBuilder->setStream(CUDA_streams.getStream(STREAM_FRAME));
    itmVisualisationEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    if (itmSettings->createMeshingEngine)
        itmMeshingEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    if(itmSettings->createPointExtractionEngine)
        pointCloudExtractionEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));

    setLabelColorList(LabelColorList_->GetData(MEMORYDEVICE_CUDA));
}

SLAMBase::~SLAMBase()= default;


ITMLib::ITMTrackingState::TrackingResult  SLAMBase::ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
        ORUtils::Matrix4<float> *customPose, ITMLib::ITMIMUMeasurement *imuMeasurement, ITMUShortImage *imgLabel) {
    /// Skip Frame
    if(itmSettings->useSkipFrame >0)
        if(img_counter % itmSettings->useSkipFrame != 0)
            return itmTrackingState->trackerResult;

    DEBUG("[SLAMBase][ProcessFrame]UpdateView\n");
    TICK("[SLAMBase][ProcessFrame]1.UpdateView");
    if(imuMeasurement == nullptr) itmViewBuilder->UpdateView(&itmView, imgColor, imgDepth, imgLabel, itmSettings->useBilateralFilter, ITMVoxel::hasLabelInformation);
    else SCLOG(ERROR) << "Not Implemented.";
    TOCK("[SLAMBase][ProcessFrame]1.UpdateView");
    //itmViewBuilder->UpdateView(&itmView, imgColor, imgDepth, itmSettings->useBilateralFilter, imuMeasurement);

    CUDA_streams.syncStream(STREAM_FRAME);

    /// Tracking
    DEBUG("[SLAMBase][ProcessFrame]Tracking\n");
    TICK("[SLAMBase][ProcessFrame]2.UpdatePose");
    ITMLib::ITMTrackingState::TrackingResult trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
    ORUtils::SE3Pose oldPose(*(itmTrackingState->pose_d));
    if (customPose) {
        itmTrackingState->pose_d->SetM(customPose->inv());
        trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
        DEBUG("custom pose provided!\n");
    } else {
        if (bTrackingActive) trackingController->Track(itmTrackingState.get(), itmView);
//        switch (itmTrackingState->trackerResult){
//            case ITMLib::ITMTrackingState::TRACKING_GOOD:
//                printf("[%s][%s] TRACKING_GOOD\n", __FILE__, __FUNCTION__);
//                break;
//            case ITMLib::ITMTrackingState::TRACKING_POOR:
//                printf("[%s][%s] TRACKING_POOR\n", __FILE__, __FUNCTION__);
//                break;
//            case ITMLib::ITMTrackingState::TRACKING_FAILED:
//                printf("[%s][%s] TRACKING_FAILED\n", __FILE__, __FUNCTION__);
//                break;
//        }

        if(std::isnan(itmTrackingState->pose_d->GetM().m[0])){
            throw std::runtime_error("is Nan! break\n");
        }

        switch (itmSettings->behaviourOnFailure){
            case ITMLib::ITMLibSettings::FAILUREMODE_RELOCALISE:
                trackerResult = itmTrackingState->trackerResult;
                break;
            case ITMLib::ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
                if(itmTrackingState->trackerResult != ITMLib::ITMTrackingState::TRACKING_FAILED)
                    trackerResult = itmTrackingState->trackerResult;
                else trackerResult = ITMLib::ITMTrackingState::TRACKING_POOR;
                break;
            default:
                break;
        }
    }
    TOCK("[SLAMBase][ProcessFrame]2.UpdatePose");
//    DEBUG_MSG("pose: \n" << (getEigenRowMajor<float, 4>(itmTrackingState->pose_d->GetInvM().m)) << "\n");

    DEBUG("[SLAMBase][ProcessFrame]Relocalization\n");
    int addKeyframeIdx = -1;
    if (itmSettings->behaviourOnFailure == ITMLib::ITMLibSettings::FAILUREMODE_RELOCALISE)
    {
        TICK("[SLAMBase][ProcessFrame]3.Relocalization");
        if (trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

        int NN; float distances;
        itmView->depth->UpdateHostFromDevice();

        //find and add keyframe, if necessary
        bool hasAddedKeyframe = relocaliser->ProcessFrame(itmView->depth.get(), itmTrackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

        //frame not added and tracking failed -> we need to relocalise
        if (!hasAddedKeyframe && trackerResult == ITMLib::ITMTrackingState::TRACKING_FAILED)
        {
            relocalisationCount = 10;

            // Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
            if(itmView->rgb_prev) itmView->rgb_prev->Clear();

            const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
            itmTrackingState->pose_d->SetFrom(&keyframe.pose);

            itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get(), true);
            trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
            trackingController->Track(itmTrackingState.get(), itmView);

            trackerResult = itmTrackingState->trackerResult;
        }
        TOCK("[SLAMBase][ProcessFrame]3.Relocalization");
    }

    bool didFusion = false;
     if ((trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (bFusionActive) && (relocalisationCount == 0))
    {
        DEBUG("[SLAMBase][ProcessFrame]Mapping\n");
        TICK("[SLAMBase][ProcessFrame]4.Mapping");
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            itmMapper->ProcessFrame(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get());
        }
        TOCK("[SLAMBase][ProcessFrame]4.Mapping");
        didFusion = true;
        if (framesProcessed > 50) trackingInitialised = true;
        framesProcessed++;
    }

    if (trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || trackerResult == ITMLib::ITMTrackingState::TRACKING_POOR) {
        DEBUG("[SLAMBase][ProcessFrame]5.RayCastForNextTracking\n");
        TICK("[SLAMBase][ProcessFrame]5.RayCastForNextTracking");
        if (!didFusion) itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get());

        // raycast to renderState_live for tracking and free visualisation
        trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
        if (addKeyframeIdx >= 0)
        {
            ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
                    itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

            kfRaycast->SetFrom(itmRenderState_live->raycastImage, memoryCopyDirection);
        }
        TOCK("[SLAMBase][ProcessFrame]5.RayCastForNextTracking");
    } else *itmTrackingState->pose_d = oldPose;

    return itmTrackingState->trackerResult;
}

void SLAMBase::resetAll(){
    MainEngine<ITMVoxel, ITMVoxelIndex>::resetAll();
    performed_sc_ = false;

    CUDA_streams.reset();
    CUDA_streams.createStream(STREAM_FRAME, true);
    CUDA_streams.createStream(STREAM_MAP, true);
    CUDA_streams.createStream(STREAM_SC, true);
    CUDA_streams.createStream(STREAM_VISUALIZATION, true);

    /// Objects
    itmMapper->setStream(CUDA_streams.getStream(STREAM_MAP));
    itmViewBuilder->setStream(CUDA_streams.getStream(STREAM_FRAME));
    itmVisualisationEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    if (itmSettings->createMeshingEngine)
        itmMeshingEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    if(itmSettings->createPointExtractionEngine)
        pointCloudExtractionEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    setLabelColorList(LabelColorList_->GetData(MEMORYDEVICE_CUDA));

    if (itmSettings->createMeshingEngine) {
        MeshEngine<ITMVoxel, ITMVoxelIndex>::reset();
    }
    if (itmSettings->createPointExtractionEngine) {
        PointCloudEngine<ITMVoxel,ITMVoxelIndex>::reset();
    }
}

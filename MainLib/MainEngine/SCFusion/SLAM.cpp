#include "SLAM.h"
#include "../../../ORUtils/LogUtil.h"
//#include "../../../ORUtils/EigenHelper.h"
#include "../../../ORUtils/Logging.h"
//#include <Utilities/Exception.hpp>


#include "../../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../../Engines/PointExtraction/PointCloudExtractionEngineFactory.h"
#include "../../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../../Engines/Meshing/ITMMeshingEngineFactory.h"
#include "../../Engines/SceneCompletion/SceneCompletionEngineFactory.h"
#include "../../Objects/RenderStates/ITMRenderStateFactory.h"
#include "../../Trackers/ITMTrackerFactory.h"

#include <future>
#include <thread>
#include <chrono>
#include "../../ORUtils/MatrixOperation.h"
#include "../../ORUtils/LabelColorUtils.h"
//#include <ORUtils/FileUtils.h>
#include <CxxTools/PathTool.hpp>

using namespace SCFUSION;

SLAM::SLAM(const ITMLib::ITMLibSettings *itmLibSettings, const ITMLib::ITMRGBDCalib *calib):
MainEngine(itmLibSettings, calib), MeshEngine(itmLibSettings), PointCloudEngine(itmLibSettings){
    performed_sc_ = false;
    inited = false;
    CUDA_streams.createStream(STREAM_FRAME, true);
    CUDA_streams.createStream(STREAM_MAP, true);
    CUDA_streams.createStream(STREAM_SC, true);
    CUDA_streams.createStream(STREAM_VISUALIZATION, true);

    /// Objects
    if(itmSettings->useSC) {
        scBuffer.value.reset(new SCFUSION::SCBuffer(&itmLibSettings->scParams));
        scProcessor.reset(new SCFUSION::SceneCompletion<ITMVoxel, ITMVoxelIndex>(itmSettings));
        scProcessor->ResetSCBuffer(scBuffer.value.get());
        scProcessor->setStream(CUDA_streams.getStream(STREAM_SC));
        for(size_t i=0;i<5;++i) {
            scProcessor->Complete(scBuffer.value.get(), itmScene.get()); // dry run
        }
    }

    itmMapper->setStream(CUDA_streams.getStream(STREAM_MAP));
    itmViewBuilder->setStream(CUDA_streams.getStream(STREAM_FRAME));
    itmVisualisationEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    if (itmSettings->createMeshingEngine)
        itmMeshingEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));
    if(itmSettings->createPointExtractionEngine)
        pointCloudExtractionEngine->setStream(CUDA_streams.getStream(STREAM_VISUALIZATION));

    setLabelColorList(LabelColorList_->GetData(MEMORYDEVICE_CUDA));
}
SLAM::~SLAM(){

}


ITMLib::ITMTrackingState::TrackingResult  SLAM::ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
        ORUtils::Matrix4<float> *customPose, ITMLib::ITMIMUMeasurement *imuMeasurement, ITMUShortImage *imgLabel) {
    imgNum = img_counter;
    /// Skip Frame
    if(itmSettings->useSkipFrame >0)
        if(img_counter % itmSettings->useSkipFrame != 0)
            return itmTrackingState->trackerResult;
    SCLOG(VERBOSE) << "Process frame " << img_counter;

    SCLOG(VERBOSE) << "UpdateView";
    TICK("[SLAM][ProcessFrame]1.UpdateView");
    if(imuMeasurement == nullptr) itmViewBuilder->UpdateView(&itmView, imgColor, imgDepth, imgLabel, itmSettings->useBilateralFilter, ITMVoxel::hasLabelInformation);
    else SCLOG(ERROR) << "Not Implemented.";
    TOCK("[SLAM][ProcessFrame]1.UpdateView");
    //itmViewBuilder->UpdateView(&itmView, imgColor, imgDepth, itmSettings->useBilateralFilter, imuMeasurement);

    if(gtPoseMode != GTPOSEMODE_IGNORE && !customPose) {
        //TODO: add logging system. can take cpf_sementation's logging as reference.
        SCLOG(WARNING) << "gtPoseMode is set but customPose was not provided!";
        return ITMLib::ITMTrackingState::TRACKING_FAILED;
    }
    CUDA_streams.syncStream(STREAM_FRAME);

    /// Tracking
    SCLOG(VERBOSE) << "Tracking";
    TICK("[SLAM][ProcessFrame]2.UpdatePose");
    ITMLib::ITMTrackingState::TrackingResult trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
    if( img_counter==0 && customPose) {
        itmTrackingState->pose_d->SetM(customPose->inv());
    }
    if (gtPoseMode != GTPOSEMODE_IGNORE) {
        if(std::isnan(customPose->m[3])) {
            SCLOG(VERBOSE) << "Input pose is NAN. Skip.";
            return ITMLib::ITMTrackingState::TRACKING_FAILED;
        }
        if(customPose->m[3] != 0 || customPose->m[7] != 0 || customPose->m[11] != 0) {
            std::cout << *customPose << std::endl;
            throw std::runtime_error("Given pose has wrong format. elemnt at m30, m31, m32 should be zero.\n");
        }
        itmTrackingState->pose_d->SetM(customPose->inv());
        trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
        SCLOG(VERBOSE) << "custom pose provided!";
    }
    ORUtils::SE3Pose oldPose(*(itmTrackingState->pose_d));
    if(gtPoseMode != GTPoseMODE::GTPOSEMODE_TACKOVER)
    {
        CUDA_streams.syncStream(STREAM_FRAME);
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
//            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            if (bTrackingActive) trackingController->Track(itmTrackingState.get(), itmView);
        }
//        std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
//        std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);


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
//        DEBUG_MSG(itmTrackingState->trackerResult);
    }
    TOCK("[SLAM][ProcessFrame]2.UpdatePose");

    // Relocalization
    int addKeyframeIdx = -1;
    if (!customPose && itmSettings->behaviourOnFailure == ITMLib::ITMLibSettings::FAILUREMODE_RELOCALISE)
    {
        SCLOG(VERBOSE) << "Relocalization";
        TICK("[SLAM][ProcessFrame]3.Relocalization");
        if (trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

        int NN; float distances;
        CUDA_streams.syncStream(STREAM_FRAME);
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

            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            CUDA_streams.syncStream(STREAM_SC);
            itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get(), true);
            trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
            trackingController->Track(itmTrackingState.get(), itmView);

            trackerResult = itmTrackingState->trackerResult;
        }
        TOCK("[SLAM][ProcessFrame]3.Relocalization");
    }

    bool didFusion = false;
    if ((trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (bFusionActive) && (relocalisationCount == 0))
    {
        if(false)
        {
            SCLOG(VERBOSE) << "Segmentation";
//                if (inseg != NULL)
//                    this->inseg->labeling(itmView->label, itmView->vertex, itmView->depthNormal, itmView->depthUncertainty, SegConfig.minSegmentSize);
//                if (seseg != NULL) {
//                    this->seseg->process(rgbImage, itmView->semanticlabel, SegConfig.confidence_thredshold);
//
//                    bool initLabelOnly = true;
//                    printf("[DEBUG] inseg Label Fusion\n");
//                    int modelLabelNum = inseg->labelPropagation(itmView->label, itmView->semanticlabel, initLabelOnly);
//                }
//
//                // Label Propagation
//                if (inseg != NULL && seseg == NULL) {
//                    if (framesProcessed) {
//                        static bool initLabelOnly = false;
//                        printf("[DEBUG] inseg->labelPropagation\n");
//                        int modelLabelNum = inseg->labelPropagation(itmView->label, renderState_live->raycastLabel,
//                                                                    initLabelOnly);
//                    }
//                }
        }

        SCLOG(VERBOSE) << "Mapping";
        TICK("[SLAM][ProcessFrame]4.Mapping");
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            CUDA_streams.syncStream(STREAM_SC);
            CUDA_streams.syncStream(STREAM_FRAME);
            itmMapper->ProcessFrame(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get());
        }
        TOCK("[SLAM][ProcessFrame]4.Mapping");
        didFusion = true;
        if (framesProcessed > 50) trackingInitialised = true;
        framesProcessed++;

        if(!inited) {
            inited = true;
        }


    }

    if (trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || trackerResult == ITMLib::ITMTrackingState::TRACKING_POOR) {
        SCLOG(VERBOSE) << "RayCastForNextTracking";
        TICK("[SLAM][ProcessFrame]5.RayCastForNextTracking");
        CUDA_streams.syncStream(STREAM_SC);
        CUDA_streams.syncStream(STREAM_FRAME);
        std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
        {
            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            if (!didFusion) itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get());
        }

        // raycast to renderState_live for tracking and free visualisation
        trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
        lockScene.unlock();

        if (addKeyframeIdx >= 0)
        {
            ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
                    itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

            kfRaycast->SetFrom(itmRenderState_live->raycastImage, memoryCopyDirection);
        }
        TOCK("[SLAM][ProcessFrame]5.RayCastForNextTracking");
    } else *itmTrackingState->pose_d = oldPose;

    if ((trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (bFusionActive) && (relocalisationCount == 0))
    {
        if(itmSettings->useSC) {
            triggerBackEnd();
//            scProcessor->CleanExtractionCenters();
//            scProcessor->CalculateMinMax(itmView->depth.get(), itmTrackingState->pose_d->GetInvM(),
//                                         itmView->calib.intrinsics_d.projectionParamsSimple.all, scBuffer.value.get(),
//                                         scBuffer.value->scparams_->base_y_value, false);
        }
    }

    return itmTrackingState->trackerResult;
}

void SLAM::triggerBackEnd(){
    SCLOG(VERBOSE) << "SceneCompletion";
    TICK("[SLAM][ProcessFrame]5.SceneCompletion");
    scProcessor->CleanExtractionCenters();
    scProcessor->CalculateMinMax(itmView->depth.get(), itmTrackingState->pose_d->GetInvM(),
                                 itmView->calib.intrinsics_d.projectionParamsSimple.all, scBuffer.value.get(),
                                 scBuffer.value->scparams_->base_y_value, false);
    if(itmSettings->scParams.useThread){
        if (sc_thread_.valid()) {
            if (sc_thread_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                sc_thread_.get();
                sc_thread_ = std::async(std::launch::async, std::bind(&SLAM::sceneCompletion, this, imgNum));
            }
        } else sc_thread_ = std::async(std::launch::async, std::bind(&SLAM::sceneCompletion, this, imgNum));
    } else {
        sceneCompletion(imgNum);
    }
    TOCK("[SLAM][ProcessFrame]5.SceneCompletion");
}

void SLAM::sceneCompletion(size_t t) {
    SCLOG(VERBOSE) << "Scene completion starts";
    while(true){
        auto scQueue = scProcessor->GetSCQueue();
        if(scQueue == nullptr) {
            SCLOG(VERBOSE) << "scQueue empty. return.";
            break;
        }
        std::unique_lock<std::mutex> lock (scBuffer.mutex);

        SCLOG(VERBOSE) << "moveVolume";
        TICK("[SLAM][SceneCompletion]1.moveVolume");
        {
            scBuffer->SetBaseTo(scQueue->extraction_base);
            scBuffer->setImgSize(scQueue->imgSize);
            scBuffer->setPose(scQueue->pose);
            scBuffer->setProjParams(scQueue->depthParam);
            scBuffer->time = t;
        }
        TOCK("[SLAM][SceneCompletion]1.moveVolume");

        SCLOG(VERBOSE) << "Criteria check";
        TICK("[SLAM][SceneCompletion]2.CriteriaCheck");
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            CUDA_streams.syncStream(STREAM_MAP);
            if(!scProcessor->CheckCriteria(scBuffer.value.get(), itmScene.get())) {
                SCLOG(VERBOSE) << "Criteria check failed";
                continue;
            } else {
                SCLOG(VERBOSE) << "Criteria passed";
            }
        }
        TOCK("[SLAM][SceneCompletion]2.CriteriaCheck");


        SCLOG(VERBOSE) << "SceneCompletion";
        TICK("[SLAM][SceneCompletion]2.SceneCompletion");
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            CUDA_streams.syncStream(STREAM_MAP);
            scProcessor->Complete(scBuffer.value.get(), itmScene.get());
        }
        TOCK("[SLAM][SceneCompletion]2.SceneCompletion");

        SCLOG(VERBOSE) << "Fusion";
        TICK("[SLAM][SceneCompletion]3.FuseBack");
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            CUDA_streams.syncStream(STREAM_MAP);
            scProcessor->Fuse(itmScene.get(), itmMapper->getSceneRecoEngine(), scBuffer.value.get(), itmRenderState_live.get());
        }
        TOCK("[SLAM][SceneCompletion]3.FuseBack");
//    volumeChecker("oBuffer", scBuffer->oBuffer->GetData(MEMORYDEVICE_CPU),scBuffer->oBuffer->dataSize);

        TICK("[SLAM][SceneCompletion]4.Optimization");
        if(itmSettings->scParams.useCRF)
        {
            SCLOG(VERBOSE) << "Optimization";
            //TODO: maybe not doing this every run?
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            CUDA_streams.syncStream(STREAM_MAP);
            scProcessor->Optimization(itmScene.get(), itmMapper->getSceneRecoEngine(), scBuffer.value.get(), itmRenderState_live.get());
        }
        TOCK("[SLAM][SceneCompletion]4.Optimization");
        performed_sc_ = true;

//        break;
    }
    SCLOG(VERBOSE) << "Scene completion Finished.";
}

void SLAM::resetAll(){
    MainEngine<ITMVoxel, ITMVoxelIndex>::resetAll();
    performed_sc_ = false;
    inited = false;

    CUDA_streams.reset();
    CUDA_streams.createStream(STREAM_FRAME, true);
    CUDA_streams.createStream(STREAM_MAP, true);
    CUDA_streams.createStream(STREAM_SC, true);
    CUDA_streams.createStream(STREAM_VISUALIZATION, true);

    /// Objects
    if(itmSettings->useSC) {
        scBuffer.value.reset(new SCFUSION::SCBuffer(&itmSettings->scParams));
//        scProcessor.reset(new SCFusion::SceneCompletion<ITMVoxel, ITMVoxelIndex>(itmSettings));
//        scProcessor->ResetSCBuffer(scBuffer.value.get());
        scProcessor->setStream(CUDA_streams.getStream(STREAM_SC));
        scProcessor->Complete(scBuffer.value.get(), itmScene.get()); // dry run
    }

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

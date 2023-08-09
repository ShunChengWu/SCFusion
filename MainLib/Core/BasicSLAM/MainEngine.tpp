#include "MainEngine.h"
//#include <Utilities/LogUtil.hpp>
//#include <Utilities/EigenHelper.h>
//#include <CxxTools/LogUtil.hpp>
#include <ORUtils/LogUtil.h>
#include "../../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../../Objects/RenderStates/ITMRenderStateFactory.h"
#include "../../Trackers/ITMTrackerFactory.h"

#include <future>
#include <thread>
#include <chrono>
#include <ORUtils/MatrixOperation.h>
#include <ORUtils/LabelColorUtils.h>
#include <ORUtils/FileUtils.h>
#include <ORUtils/Logging.h>
#include <CxxTools/PathTool.hpp>

using namespace SCFUSION;

template<class TVoxel, class TIndex>
MainEngine<TVoxel,TIndex>::MainEngine(const ITMLib::ITMLibSettings *itmLibSettings, const ITMLib::ITMRGBDCalib *calib):calib_(calib){
    itmSettings = itmLibSettings;
    relocalisationCount=0;
    framesProcessed=0;
    trackingInitialised=false;
    bFusionActive =true;
    bTrackingActive = true;

    auto RGBImageSize = calib->intrinsics_rgb.imgSize;
    auto DepthImageSize = calib->intrinsics_d.imgSize;

    LabelColorList_.reset(new ORUtils::MemoryBlock<Vector4f>(std::numeric_limits<ushort>::max(), true,true));
    ORUtils::LabelColorGenerator::Run(LabelColorList_.get(), LabelColorList_->dataSize, true, true, itmLibSettings->labelColorPath, true);
    /// Tracker

    imuCalibrator.reset( new ITMLib::ITMIMUCalibrator_iPad());
    itmScene.reset(new ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>(&itmLibSettings->sceneParams,
                                                                 itmLibSettings->swappingMode ==
                                                                 ITMLib::ITMLibSettings::SWAPPINGMODE_ENABLED,
                                                                 itmLibSettings->GetMemoryType()));
    lowLevelEngine.reset(ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(itmSettings->deviceType));
    tracker.reset(ITMLib::ITMTrackerFactory::Instance().Make(RGBImageSize, DepthImageSize, itmSettings, lowLevelEngine.get(),
                                                         imuCalibrator.get(), itmScene->sceneParams) );
    trackingController.reset(new ITMLib::ITMTrackingController(tracker.get(), itmSettings));
    Vector2i trackedImageSize = trackingController->GetTrackedImageSize(RGBImageSize, DepthImageSize);

    /// Objects
    itmView = nullptr;
    itmTrackingState.reset(new ITMLib::ITMTrackingState(trackedImageSize, itmLibSettings->GetMemoryType()));
    itmRenderState_live.reset(
            ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(trackedImageSize, &itmLibSettings->sceneParams,
                                                                            itmLibSettings->GetMemoryType()));
    itmRenderState_freeview.reset(
            ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(trackedImageSize, &itmLibSettings->sceneParams,
                                                                            itmLibSettings->GetMemoryType()));

    /// Cores
    itmMapper.reset(new ITMLib::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(itmLibSettings));
    itmMapper->ResetScene(itmScene.get());
    /// Engines
    itmViewBuilder.reset(ITMLib::ITMViewBuilderFactory::MakeViewBuilder(*calib, itmLibSettings->deviceType));
    itmVisualisationEngine.reset(
            ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<ITMVoxel, ITMVoxelIndex>(
                    itmSettings->deviceType));
    itmVisualisationEngine->setLabelColorListPtr(LabelColorList_->GetDataConst(MEMORYDEVICE_CUDA));

    if (itmSettings->behaviourOnFailure == itmSettings->FAILUREMODE_RELOCALISE)
        relocaliser.reset( new FernRelocLib::Relocaliser<float>(DepthImageSize, Vector2f(itmSettings->sceneParams.viewFrustum_min, itmSettings->sceneParams.viewFrustum_max), 0.2f, 500, 4));
    else relocaliser = nullptr;
    kfRaycast.reset(new ITMUChar4Image(DepthImageSize, itmLibSettings->GetMemoryType()));

    tracker->UpdateInitialPose(itmTrackingState.get());

    itmTrackingState->pose_d->SetFrom(0,0,0,0,0,0);

}

template<class TVoxel, class TIndex>
MainEngine<TVoxel,TIndex>::~MainEngine(){
    if(itmView) delete itmView;
}

template<class TVoxel, class TIndex>
ITMLib::ITMTrackingState::TrackingResult  MainEngine<TVoxel,TIndex>::ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                                                  ORUtils::Matrix4<float> *customPose, ITMLib::ITMIMUMeasurement *imuMeasurement, ITMUShortImage *imgLabel) {
    /// Skip Frame
    if(itmSettings->useSkipFrame >0)
        if(img_counter % itmSettings->useSkipFrame != 0)
            return itmTrackingState->trackerResult;

    DEBUG("[SLAM][ProcessFrame]UpdateView\n");
    TICK("[SLAM][ProcessFrame]1.UpdateView");
    if(imuMeasurement == nullptr) itmViewBuilder->UpdateView(&itmView, imgColor, imgDepth, imgLabel, itmSettings->useBilateralFilter, ITMVoxel::hasLabelInformation);
    else SCLOG(ERROR) << "Not Implemented.";
    TOCK("[SLAM][ProcessFrame]1.UpdateView");

    if(gtPoseMode != GTPOSEMODE_IGNORE && !customPose)
        throw std::runtime_error("gtPoseMode is set but customPose was not provided!\n");


    /// Tracking
    DEBUG("[SLAM][ProcessFrame]Tracking\n");
    TICK("[SLAM][ProcessFrame]2.UpdatePose");

    ITMLib::ITMTrackingState::TrackingResult trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;

    if (gtPoseMode != GTPOSEMODE_IGNORE) {
        itmTrackingState->pose_d->SetM(customPose->inv());
        trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
        DEBUG("custom pose provided!\n");
    }
    ORUtils::SE3Pose oldPose(*(itmTrackingState->pose_d));
    if(gtPoseMode != GTPoseMODE::GTPOSEMODE_TACKOVER)
    {
        if (bTrackingActive) trackingController->Track(itmTrackingState.get(), itmView);

        if(std::isnan(itmTrackingState->pose_d->GetM().m[0])){
            throw std::runtime_error("Tracker return Nan pose! break\n");
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
        switch(itmTrackingState->trackerResult){
            case ITMLib::ITMTrackingState::TrackingResult::TRACKING_GOOD:
                DEBUG("[MainEngine][ProcessFrame] Tracking result: GOOD\n");
                break;
            case ITMLib::ITMTrackingState::TrackingResult::TRACKING_POOR:
                DEBUG("[MainEngine][ProcessFrame] Tracking result: POOR\n");
                break;
            case ITMLib::ITMTrackingState::TrackingResult::TRACKING_FAILED:
                DEBUG("[MainEngine][ProcessFrame] Tracking result: FAILED\n");
                break;
        }

    }
    TOCK("[MainEngine][ProcessFrame]2.UpdatePose");
//    DEBUG_MSG("pose: \n" << (getEigenRowMajor<float, 4>(itmTrackingState->pose_d->GetInvM().m)) << "\n");

    int addKeyframeIdx = -1;
    if(gtPoseMode != GTPoseMODE::GTPOSEMODE_IGNORE && trackerResult == ITMLib::ITMTrackingState::TRACKING_FAILED) {
//        *itmTrackingState->pose_d = oldPose;
//        itmTrackingState->age_pointCloud = -1;
//        itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get(), true);
//        trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
//        trackingController->Track(itmTrackingState.get(), itmView);
//        trackerResult = itmTrackingState->trackerResult;
//        printf("SECOND\n");
//        switch(trackerResult){
//            case ITMTrackingState::TrackingResult::TRACKING_GOOD:
//                DEBUG("[MainEngine][ProcessFrame] Tracking result: GOOD\n");
//                break;
//            case ITMTrackingState::TrackingResult::TRACKING_POOR:
//                DEBUG("[MainEngine][ProcessFrame] Tracking result: POOR\n");
//                break;
//            case ITMTrackingState::TrackingResult::TRACKING_FAILED:
//                DEBUG("[MainEngine][ProcessFrame] Tracking result: FAILED\n");
//                break;
//        }
    } else
    if (itmSettings->behaviourOnFailure == ITMLib::ITMLibSettings::FAILUREMODE_RELOCALISE)
    {
        DEBUG("[MainEngine][ProcessFrame]Relocalization\n");
        TICK("[MainEngine][ProcessFrame]3.Relocalization");
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
        TOCK("[MainEngine][ProcessFrame]3.Relocalization");
    }

    bool didFusion = false;
    if ((trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (bFusionActive) && (relocalisationCount == 0))
    {
        DEBUG("[MainEngine][ProcessFrame]Segmentation\n");
        {
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

        DEBUG("[MainEngine][ProcessFrame]Mapping\n");
        TICK("[MainEngine][ProcessFrame]4.Mapping");
        {
            std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
            std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);
            itmMapper->ProcessFrame(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get());
        }
        TOCK("[MainEngine][ProcessFrame]4.Mapping");
        didFusion = true;
        if (framesProcessed > 50) trackingInitialised = true;
        framesProcessed++;
    }

    if(gtPoseMode != GTPoseMODE::GTPOSEMODE_IGNORE && trackerResult != ITMLib::ITMTrackingState::TRACKING_GOOD) {
        *itmTrackingState->pose_d = oldPose;
        itmTrackingState->age_pointCloud = -1;
        itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get(), true);
        trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
//        trackingController->Track(itmTrackingState.get(), itmView);
//        trackerResult = itmTrackingState->trackerResult;
//        printf("SECOND\n");
//        switch(trackerResult){
//            case ITMTrackingState::TrackingResult::TRACKING_GOOD:
//                DEBUG("[MainEngine][ProcessFrame] Tracking result: GOOD\n");
//                break;
//            case ITMTrackingState::TrackingResult::TRACKING_POOR:
//                DEBUG("[MainEngine][ProcessFrame] Tracking result: POOR\n");
//                break;
//            case ITMTrackingState::TrackingResult::TRACKING_FAILED:
//                DEBUG("[MainEngine][ProcessFrame] Tracking result: FAILED\n");
//                break;
//        }
    } else
    if (trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD || trackerResult == ITMLib::ITMTrackingState::TRACKING_POOR) {
        DEBUG("[MainEngine][ProcessFrame]5.RayCastForNextTracking\n");
        TICK("[MainEngine][ProcessFrame]5.RayCastForNextTracking");
        if (!didFusion) itmMapper->UpdateVisibleList(itmView, itmTrackingState.get(), itmScene.get(), itmRenderState_live.get());

        // raycast to renderState_live for tracking and free visualisation
        trackingController->Prepare(itmTrackingState.get(), itmScene.get(), itmView, itmVisualisationEngine.get(), itmRenderState_live.get());
        if (addKeyframeIdx >= 0)
        {
            ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
                    itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

            kfRaycast->SetFrom(itmRenderState_live->raycastImage, memoryCopyDirection);
        }
        TOCK("[MainEngine][ProcessFrame]5.RayCastForNextTracking");
    } else *itmTrackingState->pose_d = oldPose;

    return itmTrackingState->trackerResult;
}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::renderImage(ITMUChar4Image *image_out, GetImageType getImageType,
        ORUtils::SE3Pose *pose, ITMLib::ITMIntrinsics *intrinsics,
        ITMLib::IITMVisualisationEngine::RenderMode renderMode){
    image_out->Clear();

    switch (getImageType)
    {
        case MainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB: {
            if(itmView==nullptr) {
                SCLOG(WARNING) << "itmView did not initialized!";
                return;
            }
            image_out->ChangeDims(itmView->rgb->noDims);
            if (itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
                image_out->SetFrom(itmView->rgb.get(), ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
            else image_out->SetFrom(itmView->rgb.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        }
            break;
        case MainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
            if(itmView==nullptr) {
                SCLOG(WARNING) << "itmView did not initialized!";
                return;
            }
            image_out->ChangeDims(itmView->depth->noDims);
            if (itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA) itmView->depth->UpdateHostFromDevice();
            ITMLib::ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(image_out, itmView->depth.get());

            break;
        case MainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
        case MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
        case MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
        case MainEngine::InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL:
        case MainEngine::InfiniTAM_IMAGE_LABEL_FROM_NORMAL:
        case MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
        {
            // use current raycast or forward projection?
            ITMLib::IITMVisualisationEngine::RenderRaycastSelection raycastType;
            if (itmTrackingState->age_pointCloud <= 0) raycastType = ITMLib::IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
            else raycastType = ITMLib::IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

            // what sort of image is it?
            ITMLib::IITMVisualisationEngine::RenderImageType imageType;
            switch (getImageType) {
                case MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
                    imageType = ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
                    break;
                case MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
                    imageType = ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
                    break;
                case MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
                    imageType = ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
                    break;
                case MainEngine::InfiniTAM_IMAGE_LABEL_FROM_IMAGENORMAL:
                    imageType = ITMLib::IITMVisualisationEngine::RENDER_LABELCOLOUR_FROM_IMAGENORMALS;
                    break;
                case MainEngine::InfiniTAM_IMAGE_LABEL_FROM_NORMAL:
                    imageType = ITMLib::IITMVisualisationEngine::RENDER_LABELCOLOUR_FROM_VOLUME;
                    break;
                default:
                    imageType = ITMLib::IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
            }

            itmVisualisationEngine->RenderImage(itmScene.get(), itmTrackingState->pose_d, &calib_->intrinsics_d,
                    itmRenderState_live.get(), renderMode, itmRenderState_live->raycastImage, imageType, raycastType);

            ORUtils::Image<Vector4u> *srcImage = nullptr;
            if (relocalisationCount != 0) srcImage = kfRaycast.get();
            else srcImage = itmRenderState_live->raycastImage;

            image_out->ChangeDims(srcImage->noDims);
            if (itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
                image_out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
            else image_out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

            break;
        }
        case MainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
        case MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
        case MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
        case MainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_IMAGENORMAL:
        case MainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL:
        case MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
        {
            ITMLib::IITMVisualisationEngine::RenderImageType type;
            switch (getImageType){
                case MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
                    type = ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
                    break;
                case MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
                    type = ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
                    break;
                case MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
                    type = ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
                    break;
                case MainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_IMAGENORMAL:
                    type = ITMLib::IITMVisualisationEngine::RENDER_LABELCOLOUR_FROM_IMAGENORMALS;
                    break;
                case MainEngine::InfiniTAM_IMAGE_FREECAMERA_LABEL_FROM_NORMAL:
                    type = ITMLib::IITMVisualisationEngine::RENDER_LABELCOLOUR_FROM_VOLUME;
                    break;
                default:
                    type = ITMLib::IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
            }

            if (itmRenderState_freeview == nullptr)
            {
                itmRenderState_freeview.reset(ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(image_out->noDims, itmScene->sceneParams, itmSettings->GetMemoryType()));
            }

            itmVisualisationEngine->FindVisibleBlocks(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get());
            itmVisualisationEngine->CreateExpectedDepths(itmScene.get(), pose, intrinsics, Vector2f(itmSettings->sceneParams.viewFrustum_min,itmSettings->sceneParams.viewFrustum_max), itmRenderState_freeview.get());
            itmVisualisationEngine->RenderImage(itmScene.get(), pose, intrinsics,
                    itmRenderState_freeview.get(), renderMode, itmRenderState_freeview->raycastImage, type);

            if (itmSettings->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
                image_out->SetFrom(itmRenderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
            else image_out->SetFrom(itmRenderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
            break;
        }
        case MainEngine::InfiniTAM_IMAGE_UNKNOWN:
            break;
    };
}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::getDepthImage(ITMFloatImage *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics) {
    if (itmRenderState_freeview == nullptr)
        itmRenderState_freeview.reset(ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(image->noDims, itmScene->sceneParams, itmSettings->GetMemoryType()));

    bool hasNan=false;
    for(size_t i=0;i<6;++i){
        if(std::isnan(pose->GetParams()[i])) {
            hasNan=true;
            break;
        }
    }
    if(hasNan){
        SCLOG(WARNING) << "invalid pose. (has nan)";
        image->Clear(0);
        return;
    }

    std::unique_lock<std::mutex> lockScene(threadLocks["Scene"]);
    std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);

    itmVisualisationEngine->FindVisibleBlocks(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get()); // this pose = freeview pose
    if(itmScene->globalCache != nullptr) itmMapper->UpdateVisibleVolume(itmScene.get(), itmRenderState_freeview.get());
    itmVisualisationEngine->CreateExpectedDepths(itmScene.get(), pose, intrinsics, Vector2f(itmSettings->sceneParams.viewFrustum_min,itmSettings->sceneParams.viewFrustum_max),
            itmRenderState_freeview.get());

    itmVisualisationEngine->GetDepthImage(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get(),
            ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE, image);
//        mapper_->renderDepth(volume_.get(), image, viewPoint, cam_K, CUDA_streams.getStream("VOLUME"));
//        CUDA_streams.syncStream("VOLUME");
}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::getVertexImage(ITMFloat3Image *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics) {
    if (itmRenderState_freeview == nullptr)
        itmRenderState_freeview.reset(ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(image->noDims, itmScene->sceneParams, itmSettings->GetMemoryType()));

    std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
    std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);

    itmVisualisationEngine->FindVisibleBlocks(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get()); // this pose = freeview pose
    if(itmScene->globalCache != nullptr) itmMapper->UpdateVisibleVolume(itmScene.get(), itmRenderState_freeview.get());
    itmVisualisationEngine->CreateExpectedDepths(itmScene.get(), pose, intrinsics, Vector2f(itmSettings->sceneParams.viewFrustum_min,itmSettings->sceneParams.viewFrustum_max),
                                                 itmRenderState_freeview.get());
    itmVisualisationEngine->GetVertexImage(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get(),
            ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE, image);
}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::getLabelImage(ITMUShortImage *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics){
    if (itmRenderState_freeview == nullptr)
        itmRenderState_freeview.reset(ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(image->noDims, itmScene->sceneParams, itmSettings->GetMemoryType()));

    std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
    std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);

    itmVisualisationEngine->FindVisibleBlocks(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get()); // this pose = freeview pose
    if(itmScene->globalCache != nullptr) itmMapper->UpdateVisibleVolume(itmScene.get(), itmRenderState_freeview.get());
    itmVisualisationEngine->CreateExpectedDepths(itmScene.get(), pose, intrinsics, Vector2f(itmSettings->sceneParams.viewFrustum_min,itmSettings->sceneParams.viewFrustum_max),
                                                 itmRenderState_freeview.get());

    itmVisualisationEngine->GetLabelImage(itmScene.get(),pose,intrinsics, itmRenderState_freeview.get(), ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE,image);
}


template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::getInstanceImage(ITMUShortImage *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics){
    if (itmRenderState_freeview == nullptr)
        itmRenderState_freeview.reset(ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(image->noDims, itmScene->sceneParams, itmSettings->GetMemoryType()));

    std::unique_lock<std::mutex> lockScene (threadLocks["Scene"]);
    std::unique_lock<std::mutex> lockMapper (threadLocks["Mapper"]);

    itmVisualisationEngine->FindVisibleBlocks(itmScene.get(), pose, intrinsics, itmRenderState_freeview.get()); // this pose = freeview pose
    if(itmScene->globalCache != nullptr) itmMapper->UpdateVisibleVolume(itmScene.get(), itmRenderState_freeview.get());
    itmVisualisationEngine->CreateExpectedDepths(itmScene.get(), pose, intrinsics, Vector2f(itmSettings->sceneParams.viewFrustum_min,itmSettings->sceneParams.viewFrustum_max),
                                                 itmRenderState_freeview.get());

    itmVisualisationEngine->GetSemanticImage(itmScene.get(),pose,intrinsics, itmRenderState_freeview.get(),
            ITMLib::IITMVisualisationEngine::RenderMode::RENDER_PERSPECTIVE,image);
}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::SaveToFile(const std::string &pth_to_directory)
{
    // throws error if any of the saves fail

    std::string saveOutputDirectory = tools::PathTool::CheckEnd(pth_to_directory) + "State/";
    std::string relocaliserOutputDirectory = saveOutputDirectory + "Relocaliser/",
    sceneOutputDirectory = saveOutputDirectory + "Scene/",
    unitMapOutputDirectory = saveOutputDirectory + "UnitMap/";

    tools::PathTool::check_and_create_folder(saveOutputDirectory);
    tools::PathTool::check_and_create_folder(relocaliserOutputDirectory);
    tools::PathTool::check_and_create_folder(sceneOutputDirectory);
    tools::PathTool::check_and_create_folder(unitMapOutputDirectory);

    if (relocaliser) relocaliser->SaveToDirectory(relocaliserOutputDirectory);

    itmScene->SaveToDirectory(sceneOutputDirectory);
}
//
//template<class TVoxel, class TIndex>
//void MainEngine<TVoxel,TIndex>::SaveToUnitMap(const std::string &pth_to_directory) {
//    std::string saveOutputDirectory = tools::PathTool::CheckEnd(pth_to_directory) + "State/",
//    unitMapOutputDirectory = saveOutputDirectory + "UnitMap/";
//    tools::PathTool::check_and_create_folder(saveOutputDirectory);
//    tools::PathTool::check_and_create_folder(unitMapOutputDirectory);
//    itmScene->SaveToUnitMap("State/UnitMap");
//}
//
//template<class TVoxel, class TIndex>
//void MainEngine<TVoxel,TIndex>::SaveToUnitMap(const std::string &pth_to_directory, const Vector3f &origin, const Vector3s &dims){
//    std::string saveOutputDirectory = tools::PathTool::CheckEnd(pth_to_directory) + "State/",
//    unitMapOutputDirectory = saveOutputDirectory + "UnitMap/";
//    tools::PathTool::check_and_create_folder(saveOutputDirectory);
//    tools::PathTool::check_and_create_folder(unitMapOutputDirectory);
//    itmScene->SaveToUnitMap(unitMapOutputDirectory, origin, dims); // local
//}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::LoadFromFile(const std::string &pth_to_directory)
{
    std::string saveInputDirectory = tools::PathTool::CheckEnd(pth_to_directory) + "State/";
    std::string relocaliserInputDirectory = saveInputDirectory + "Relocaliser/", sceneInputDirectory = saveInputDirectory + "Scene/";

    ////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
    ////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

    this->resetAll();

    try // load relocaliser
    {
        if(this->relocaliser != nullptr) {
            auto *relocaliser_temp = new FernRelocLib::Relocaliser<float>(
                    relocaliser->getImgSize(),
                    Vector2f(itmSettings->sceneParams.viewFrustum_min, itmSettings->sceneParams.viewFrustum_max), 0.2f,
                    500, 4);

            relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);
            relocaliser.reset(relocaliser_temp);
        }
    }
    catch (std::runtime_error &e)
    {
//            throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
        SCLOG(WARNING) << "Could not load relocaliser: " + std::string(e.what()) + "\n";
    }

    try // load scene
    {
        itmScene->LoadFromDirectory(sceneInputDirectory);
    }
    catch (std::runtime_error &e)
    {
        itmMapper->ResetScene(itmScene.get());
        throw std::runtime_error("Could not load scene:" + std::string(e.what()));
    }
}

template<class TVoxel, class TIndex>
void MainEngine<TVoxel,TIndex>::resetAll()
{
    itmMapper->ResetScene(itmScene.get());
    itmTrackingState->Reset();
    if(itmView) delete itmView;

    relocalisationCount=0;
    framesProcessed=0;
    trackingInitialised=false;
    bFusionActive =true;
    bTrackingActive = true;

    auto RGBImageSize = calib_->intrinsics_rgb.imgSize;
    auto DepthImageSize = calib_->intrinsics_d.imgSize;

    LabelColorList_.reset(new ORUtils::MemoryBlock<Vector4f>(255, true,true));
    ORUtils::LabelColorGenerator::Run(LabelColorList_.get(), LabelColorList_->dataSize, true, true, itmSettings->labelColorPath, true);

    /// Tracker
    imuCalibrator.reset( new ITMLib::ITMIMUCalibrator_iPad());
    itmScene.reset(new ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>(&itmSettings->sceneParams,
                                                                 itmSettings->swappingMode ==
                                                                 ITMLib::ITMLibSettings::SWAPPINGMODE_ENABLED,
                                                                 itmSettings->GetMemoryType()));
    lowLevelEngine.reset(ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(itmSettings->deviceType));
    tracker.reset(ITMLib::ITMTrackerFactory::Instance().Make(RGBImageSize, DepthImageSize, itmSettings, lowLevelEngine.get(),
                                                             imuCalibrator.get(), itmScene->sceneParams) );
    trackingController.reset(new ITMLib::ITMTrackingController(tracker.get(), itmSettings));
    Vector2i trackedImageSize = trackingController->GetTrackedImageSize(RGBImageSize, DepthImageSize);

    /// Objects
    itmView = nullptr;
    itmTrackingState.reset(new ITMLib::ITMTrackingState(trackedImageSize, itmSettings->GetMemoryType()));
    itmRenderState_live.reset(
            ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(trackedImageSize, &itmSettings->sceneParams,
                                                                            itmSettings->GetMemoryType()));
    itmRenderState_freeview.reset(
            ITMLib::ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(trackedImageSize, &itmSettings->sceneParams,
                                                                            itmSettings->GetMemoryType()));

    /// Cores
    itmMapper.reset(new ITMLib::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(itmSettings));
    itmMapper->ResetScene(itmScene.get());
    /// Engines
    itmViewBuilder.reset(ITMLib::ITMViewBuilderFactory::MakeViewBuilder(*calib_, itmSettings->deviceType));
    itmVisualisationEngine.reset(
            ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<ITMVoxel, ITMVoxelIndex>(
                    itmSettings->deviceType));
    itmVisualisationEngine->setLabelColorListPtr(LabelColorList_->GetData(MEMORYDEVICE_CUDA));

    if (itmSettings->behaviourOnFailure == itmSettings->FAILUREMODE_RELOCALISE)
        relocaliser.reset( new FernRelocLib::Relocaliser<float>(DepthImageSize, Vector2f(itmSettings->sceneParams.viewFrustum_min, itmSettings->sceneParams.viewFrustum_max), 0.2f, 500, 4));
    else relocaliser = nullptr;
    kfRaycast.reset(new ITMUChar4Image(DepthImageSize, itmSettings->GetMemoryType()));

    tracker->UpdateInitialPose(itmTrackingState.get());

    itmTrackingState->pose_d->SetFrom(0,0,0,0,0,0);
}

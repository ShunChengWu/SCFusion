#pragma once
#include "../ITMMainEngine.h"
#include <ORUtils/Matrix.h>
#include "../../../FernRelocLib/include/FernRelocLib/Relocaliser.h"
#include <future>
#include <map>
//#include "../../Objects/Scene/UnitMapSaver/Interface/UnitMapSaver.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Camera/ITMRGBDCalib.h"
#include "../../Objects/Tracking/ITMTrackingState.h"
#include "../../Objects/RenderStates/ITMRenderState.h"
#include "../../Objects/Views/ITMView.h"
#include "../../Objects/Misc/ITMIMUCalibrator.h"

#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../ITMTrackingController.h"
#include "../DenseMapper/ITMDenseMapper.h"


namespace SCFUSION {
    /*
     * Basic SLAM Engine.
     * Including tracking, mapping, relocalization, meshing, pointcloud and visualization
     */
    template<class TVoxel, class TIndex>
    class MainEngine: public ITMLib::ITMMainEngine {
    public:


        MainEngine(const ITMLib::ITMLibSettings *itmLibSettings, const ITMLib::ITMRGBDCalib *calib);

        virtual ~MainEngine();

        /// ProcessFrame
        /// If customPose is provided and useGTPose is on, the custom pose will be used as pose, otherwise the custom pose will be used as initial pose for the tracking.
        /// \param imgDepth
        /// \param imgColor
        /// \param img_counter
        /// \param customPose
        /// \param imuMeasurement
        /// \param imgLabel
        /// \return
        virtual ITMLib::ITMTrackingState::TrackingResult ProcessFrame (ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                                       ORUtils::Matrix4<float> *customPose = nullptr, ITMLib::ITMIMUMeasurement *imuMeasurement = nullptr, ITMUShortImage *imgLabel = nullptr) override;

        /// Get Image
        virtual void renderImage(ITMUChar4Image *image_out, GetImageType getImageType, ORUtils::SE3Pose *pose,
                         ITMLib::ITMIntrinsics *intrinsics, ITMLib::IITMVisualisationEngine::RenderMode renderMode);

//        virtual bool computeMesh(bool labelOnly, bool checkState);
//        virtual void computeMesh(ORUtils::MemoryBlock<float> *data, const Vector3f &origin, const Vector3s &dims, float voxelSize, float isoValue);


        /// ==== SET ====
        virtual void setPose(float *pose) { itmTrackingState->pose_d->SetM(ORUtils::Matrix4<float>(pose)); }


        virtual void setLabelColorList(Vector4f *pointer) {
            itmVisualisationEngine->setLabelColorListPtr(pointer);
        }

        void SetScene(ITMLib::ITMScene<TVoxel, TIndex> *scene) {
            this->itmScene.reset(scene);
        }

        const ITMLib::ITMRGBDCalib * getCalib(){return calib_;}
        void setRGBDCalib(const ITMLib::ITMRGBDCalib *calib) { calib_ = calib;  }

        void setLibSetting(const ITMLib::ITMLibSettings *itmLibSettings) {itmSettings = itmLibSettings;}

        /// ==== GET ====
        Vector2i GetImageSize() const override {return itmRenderState_live->raycastImage->noDims;}

        ITMLib::ITMTrackingState* GetTrackingState() override {return itmTrackingState.get();}
        /// Get Model Pose
        virtual const ORUtils::Matrix4<float> *getModelPose() const { return &itmTrackingState->pose_d->GetM(); };

        virtual ITMLib::ITMView *GetView() override { return itmView; }

        virtual ITMLib::ITMScene<TVoxel, TIndex>* getScene(){ return itmScene.get(); }

        virtual ITMLib::ITMDenseMapper<TVoxel, TIndex>* getMapper() {return itmMapper.get(); }

        virtual ITMLib::ITMRenderState* getRenderStateLive() {return itmRenderState_live.get(); }

        virtual ITMLib::ITMRenderState* getRenderStateFreeView() {return itmRenderState_freeview.get(); }

        ORUtils::MemoryBlock<Vector4f>* GetLabelColorList(){return LabelColorList_.get();}

        /// Render Depth image with the given viewpoint
        void getDepthImage(ITMFloatImage *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics);

        /// Render 3D points with the given viewpoint
        void getVertexImage(ITMFloat3Image *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics);

        void getLabelImage(ITMUShortImage *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics);

        void getInstanceImage(ITMUShortImage *image, ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics);

        const ITMLib::ITMLibSettings *getITMSetting(){ return itmSettings; }

        /// ==== SAVE ====
        virtual void SaveToFile(const std::string &pth_to_directory = "./");

//        virtual void SaveToUnitMap(const std::string &pth_to_directory);

//        virtual void SaveToUnitMap(const std::string &pth_to_directory, const Vector3f &origin, const Vector3s &dims);

        virtual void LoadFromFile(const std::string &pth_to_directory = "./");

        virtual void resetAll();

        /// ==== Sync ====
        void SyncReconstructionEngine(){ itmMapper->syncStream(); }
        void SyncVisualizationEngine(){ itmVisualisationEngine->syncStream(); }
        void SyncViewBuildingEngine(){ itmViewBuilder->syncStream(); }
    protected:

#ifdef COMPARE_VOLUME
        /// This one is to saveMap the volume without scene completion.
        std::unique_ptr<Volume> volume_wo_;
        /// This one store the output of every scene completion and integrate them directly
        std::unique_ptr<Volume> volume_pure_sc_;
#endif

        const ITMLib::ITMLibSettings *itmSettings;

        bool bTrackingActive;
        bool trackingInitialised;
        bool bFusionActive;
        size_t imgNum;
        size_t framesProcessed;
        std::unique_ptr<ITMUChar4Image> kfRaycast;

        /// Cores
        std::unique_ptr<ITMLib::ITMTrackingController> trackingController;
        std::unique_ptr<ITMLib::ITMTracker> tracker;
        std::unique_ptr<ITMLib::ITMDenseMapper<TVoxel, TIndex>> itmMapper;
        /// Engines
        std::unique_ptr<ITMLib::ITMLowLevelEngine> lowLevelEngine;
        std::unique_ptr<ITMLib::ITMViewBuilder> itmViewBuilder;
        std::unique_ptr<ITMLib::ITMVisualisationEngine<TVoxel, TIndex>> itmVisualisationEngine;

        /// Objects
        const ITMLib::ITMRGBDCalib *calib_;
        ITMLib::ITMView *itmView;
        std::unique_ptr<ITMLib::ITMScene<TVoxel, TIndex>> itmScene;
        std::unique_ptr<ITMLib::ITMRenderState> itmRenderState_live, itmRenderState_freeview;
        std::unique_ptr<ITMLib::ITMTrackingState> itmTrackingState;
        std::unique_ptr<ITMLib::ITMIMUCalibrator> imuCalibrator;

        /// Relocaliser
        std::unique_ptr<FernRelocLib::Relocaliser<float>>  relocaliser;
        size_t relocalisationCount;

        std::unique_ptr<ORUtils::MemoryBlock<Vector4f>> LabelColorList_;

        std::map<std::string, std::mutex> threadLocks;
    };
}



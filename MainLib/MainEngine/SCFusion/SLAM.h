#pragma once
#undef __AVX2__
//#include "Configuration.h"
#include "../../Utils/CUDAStreamHandler.hpp"
#include "../../Utils/ThreadUnit.hpp"
#include <CxxTools/LogUtil.hpp>
#include <future>
#include <Objects/SceneCompletion/SceneCompletionBuffer.h>
#include <Core/SceneCompletion/SceneCompletion.h>
#include "../../Utils/ITMLibSettings.h"
//#include <ORUtils/Matrix.h>
//#include <FernRelocLib/Relocaliser.h>
//#include "../../ITMLibDefines.h"
//#include "../../Objects/Scene/ITMScene.h"
//#include "../../Objects/Camera/ITMRGBDCalib.h"
//#include "../../Objects/Tracking/ITMTrackingState.h"
//#include "../../Objects/RenderStates/ITMRenderState.h"
//#include "../../Objects/Views/ITMView.h"
//#include "../../Objects/Scene/UnitMapSaver/Interface/UnitMapSaver.h"
//#include "../../Objects/PointCloud/PointCloud.h"
//#include "../../Objects/Misc/ITMIMUCalibrator.h"
//#include "../../Core/ITMDenseMapper.h"
//#include "../../Core/SceneCompletion.h"
//#include "../../Engines/SceneCompletion/Interface/SceneCompletionEngine.h"
//#include "../../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
//#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
//#include "../../Engines/Meshing/Interface/ITMMeshingEngine.h"
//#include "../../Engines/PointExtraction/Interface/PointCloudExtractionEngine.h"
//#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
//#include "../../Core/ITMTrackingController.h"
#include "../../Core/BasicSLAM/MainEngine.h"
#include "../../Engines/Meshing/MeshEngine.h"
#include "../../Engines/PointExtraction/PointCloudEngine.h"
//#define COMPARE_VOLUME

namespace SCFUSION {
    template <typename T>
    struct Thread_Component {
        std::mutex mutex;
        T value;
        template <typename Task>
        void threadSafe(Task task){
            std::unique_lock<std::mutex> lock (this->mutex);
            task();
        }

        T& operator -> () noexcept {
            return value;
        }
    };

    template <typename T>
    void  volumeChecker (std::string name, T *volume, size_t size) {
        size_t counter_lz =0;
        size_t counter_sz =0;
        size_t counter_zero = 0;
        size_t counter_01 =0;
        size_t counter_1 =0;
        size_t counter_l1 =0;
        size_t counter_inf = 0;
        size_t counter_nan = 0;
        for(size_t  i=0;i<size;++i){
            if(volume[i]>0) counter_lz++;
            if(volume[i]>0&&volume[i]<1)counter_01++;
            if(volume[i]<0) counter_sz++;
            if(volume[i] == 0) counter_zero++;
            if(volume[i]==1)counter_1++;
            if(volume[i]>1) counter_l1++;
            if(isnanf(volume[i])) counter_nan++;
            if(isinff(volume[i])) counter_inf++;

        }
        printf("[%s] size:%zu, >0:%zu(%f), <0:%zu(%f), =0:%zu(%f), (0,1):%zu(%f), 1:%zu(%f), >1:%zu(%f), NAN:%zu(%f), INF:%zu(%f)\n",
               name.c_str(), size,
               counter_lz, float(counter_lz) / float(size),
               counter_sz, float(counter_sz) / float(size),
               counter_zero, float(counter_zero)/float(size),
               counter_01,float(counter_01)/float(size),
               counter_1,float(counter_1)/float(size),
               counter_l1, float(counter_l1)/float(size),
               counter_nan, float(counter_nan)/float(size),
               counter_inf, float(counter_inf)/float(size)
        );
    }

//    template<class TVoxel, class TIndex>
    class SLAM : public MainEngine<ITMVoxel, ITMVoxelIndex>,
            public MeshEngine<ITMVoxel, ITMVoxelIndex>,
            public PointCloudEngine<ITMVoxel,ITMVoxelIndex>{
    public:
        enum CUDAStreamType {
            STREAM_FRAME, STREAM_MAP, STREAM_VISUALIZATION, STREAM_SC
        };

        SLAM(const ITMLib::ITMLibSettings *itmLibSettings, const ITMLib::ITMRGBDCalib *calib);

        virtual ~SLAM();

        virtual ITMLib::ITMTrackingState::TrackingResult ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                              ORUtils::Matrix4<float> *customPose, ITMLib::ITMIMUMeasurement *imuMeasurement, ITMUShortImage *imgLabel) override;

        Thread_Component<std::unique_ptr<SCFUSION::SCBuffer>>& getSCBuffer() {return scBuffer;}

        /// Run Completion
        virtual void sceneCompletion(size_t t);


        /// This is condition purpose. if you want to run something only when sc is performed
        std::atomic_bool *getSCPerformed() { return &performed_sc_; }

        virtual void triggerBackEnd();

        void setLabelColorList(Vector4f *pointer) override {
            if(itmSettings->createMeshingEngine)
                itmMeshingEngine->setLabelColorListPtr(pointer);
            itmVisualisationEngine->setLabelColorListPtr(pointer);
        }

        virtual /// Mesh
        bool computeMesh(bool labelOnly, bool checkState){
            return MeshEngine::computeMesh(itmScene.get(), labelOnly, checkState);
        }
        using MeshEngine::computeMesh;//(ORUtils::MemoryBlock<float> *data, const Vector3f &origin, const Vector3s &dims, float voxelSize, float isoValue);
        void saveSceneToMesh(const std::string &pth_to_directory, bool labelOnly, bool checkState){
            MeshEngine::saveSceneToMesh(itmScene.get(), pth_to_directory, labelOnly, checkState);
        }

        /// PointCloud
        void computePointCloud(){
            PointCloudEngine::computePointCloud(itmScene.get());
        }

        CUDA_Stream_Handler<CUDAStreamType>& getCUDAStream(){return CUDA_streams;}

        bool GetViewFrustumBBox(std::pair<Vector3f, Vector3f> &output) {
            if(!scProcessor || !itmView) return false;
            output = scProcessor->CalculateMinMax(itmView->depth.get(), itmTrackingState->pose_d->GetInvM(),
                                                  itmView->calib.intrinsics_d.projectionParamsSimple.all,
                                                  scBuffer.value.get(), 0, true);
            return true;
        }

        SCFUSION::SceneCompletion<ITMVoxel, ITMVoxelIndex>* GetSceneCompletion(){return scProcessor.get();}



        void resetAll() override;
    protected:
        CUDA_Stream_Handler<CUDAStreamType> CUDA_streams;
        bool inited;
        std::future<void> sc_thread_;
        std::atomic_bool performed_sc_;

        /// Cores
        std::unique_ptr<SCFUSION::SceneCompletion<ITMVoxel, ITMVoxelIndex>> scProcessor;

        /// Objects
        Thread_Component<std::unique_ptr<SCFUSION::SCBuffer>> scBuffer;
    };
}



#pragma once
#undef __AVX2__
#include "../../Utils/CUDAStreamHandler.hpp"
#include "../../Utils/ThreadUnit.hpp"
#include "../../Core/BasicSLAM/MainEngine.h"
#include "../../Engines/Meshing/MeshEngine.h"
#include "../../Engines/PointExtraction/PointCloudEngine.h"
#include "../../Utils/ITMLibSettings.h"
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
    };

    template <typename T>
    void  volumeChecker (const std::string& name, T *volume, size_t size) {
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
    class SLAMBase : public MainEngine<ITMVoxel, ITMVoxelIndex>,
            public MeshEngine<ITMVoxel, ITMVoxelIndex>,
            public PointCloudEngine<ITMVoxel,ITMVoxelIndex>{
    public:
        enum CUDAStreamType {
            STREAM_FRAME, STREAM_MAP, STREAM_VISUALIZATION, STREAM_SC
        };

        SLAMBase(const ITMLib::ITMLibSettings *itmLibSettings, const ITMLib::ITMRGBDCalib *calib);

        ~SLAMBase() override;

        ITMLib::ITMTrackingState::TrackingResult ProcessFrame(ITMFloatImage *imgDepth, ITMUChar4Image *imgColor, size_t img_counter,
                                                              ORUtils::Matrix4<float> *customPose = nullptr, ITMLib::ITMIMUMeasurement *imuMeasurement = nullptr, ITMUShortImage *imgLabel = nullptr) override;

        /// This is condition purpose. if you want to run something only when sc is performed
        std::atomic_bool *getSCPerformed() { return &performed_sc_; }

        Vector3f getLastTriggeredSCPose() {
            return last_triggered_pose_.value;
        };

        void setLastTriggeredSCPose(Vector3f pose) {
            std::unique_lock<std::mutex> lock(last_volume_sc_pose_.mutex);
            last_triggered_pose_.value = pose;
        }

        void setLabelColorList(Vector4f *pointer) override {
            if(itmSettings->createMeshingEngine)
            itmMeshingEngine->setLabelColorListPtr(pointer);
            if(itmSettings->createPointExtractionEngine)
            itmVisualisationEngine->setLabelColorListPtr(pointer);
        }

        /// Mesh
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

        void resetAll() override;
    private:
        CUDA_Stream_Handler<CUDAStreamType> CUDA_streams;
        std::atomic_bool performed_sc_;
        Thread_Component<Vector3f> last_volume_sc_pose_;
        Thread_Component<Vector3f> last_triggered_pose_;
    };
}



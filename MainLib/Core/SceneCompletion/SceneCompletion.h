#pragma once

#include "../../Utils/ITMLibSettings.h"
#include "../../Engines/SceneCompletion/Interface/SceneCompletionEngine.h"
#include "../../Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../../Objects/SceneCompletion/SceneCompletionBuffer.h"
#include "DenseCRFOptimization.h"

#ifdef COMPILE_WITH_TENSORFLOW
#include "../../Engines/SceneCompletion/Interface/ForkNet.h"
#endif

#ifdef COMPILE_WITH_PYTORCH
#include "../../Engines/SceneCompletion/Interface/SceneInpainting.h"
#endif

#include <queue>

#include <ORUtils/Logging.h>

namespace SCFUSION {
    struct SCQueue{
        Vector3f extraction_base;
        Matrix4f pose;
        Vector2i imgSize;
        Vector4f depthParam;
    };



    template<class TVoxel, class TIndex>
    class SceneCompletion{
    private:
        void *stream_;
        std::unique_ptr<NetworkBackEnd> network_;
        std::unique_ptr<SCFUSION::SceneCompletionEngine<TVoxel,TIndex>> sceneCompletionEngine_;
        std::unique_ptr<DenseCRFOptimization> mDenseCRFOpt;
    public:
        explicit SceneCompletion(const ITMLib::ITMLibSettings *settings);
        ~SceneCompletion();

        void setStream(void *stream) {
            stream_ = stream;
            sceneCompletionEngine_->setStream(stream);
            // add swapping engine here if needed
        }

        void ResetSCBuffer(SCBuffer *scBuffer);

        bool CheckCriteria(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene);

        void Complete(SCBuffer *scBuffer, const ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene);

        void Fuse(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, ITMLib::ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine,
                const SCBuffer *scBuffer, ITMLib::ITMRenderState *renderState);

        void Optimization(ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash> *scene, ITMLib::ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine,
                          SCBuffer *scBuffer, ITMLib::ITMRenderState *renderState);

        void setVolumeLocation(const Vector3f &center, SCBuffer *scBuffer);

        /**
         * Given pose and a depth map. This function calculates minimum number of extraction centres that cover the
         * region observed by given depth map.
         * First a rectangle (x_min,y_min) * (x_max,y_max), that is aligned to the floor, is extracted. Then extraction
         * centers are arranged on this region.
         */
        std::pair<Vector3f, Vector3f> CalculateMinMax(const ITMFloatImage *depth, const Matrix4f &CameraPose,
                                                      const Vector4f &projParams_d, const SCBuffer *scbuffer, float floorPosition, bool dryrun);

        void AddExtractionCenter(const SCBuffer *scbuffer, const std::pair<Vector3f, Vector3f> &minMax);

        std::vector<SCQueue> GetExtractionCenters(){
            std::unique_lock<std::mutex>lock(mutex_extraction_centers_);
            std::queue<std::shared_ptr<SCQueue>> copy = extraction_centers_;

            std::vector<SCQueue> output;
            output.reserve(copy.size());
            while(!copy.empty()){
                auto front = copy.front();
                copy.pop();
                output.push_back(*front);

            }
            return output;
        }
        void CleanExtractionCenters(){
            std::unique_lock<std::mutex>lock(mutex_extraction_centers_);
            while(!extraction_centers_.empty())extraction_centers_.pop();
        }

        std::shared_ptr<SCQueue> GetSCQueue(){
            std::unique_lock<std::mutex>lock(mutex_extraction_centers_);
            if(extraction_centers_.empty()) return nullptr;
            std::shared_ptr<SCQueue> front = extraction_centers_.front();
            SCLOG(DEBUG) << "Get extraction enter at: " << front->extraction_base;
            SCLOG(DEBUG) << "Size before Pop: " << extraction_centers_.size();
            extraction_centers_.pop();
            SCLOG(DEBUG) << "Size after Pop: " << extraction_centers_.size();
            return front;
        }

        std::mutex mutex_extraction_centers_;
        std::queue<std::shared_ptr<SCQueue>> extraction_centers_;


        SCFUSION::SceneCompletionEngine<TVoxel,TIndex>* GetSCEngine(){return sceneCompletionEngine_.get();}
    };

}
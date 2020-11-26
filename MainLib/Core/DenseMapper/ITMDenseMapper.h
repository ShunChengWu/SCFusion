// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../../Engines/Swapping/Interface/ITMSwappingEngine.h"
#include "../../Utils/ITMLibSettings.h"

namespace ITMLib
{
	/** \brief
	*/
	template<class TVoxel, class TIndex>
	class ITMDenseMapper
	{
	protected:
		ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
		ITMSwappingEngine<TVoxel,TIndex> *swappingEngine;

		ITMLibSettings::SwappingMode swappingMode;

	public:
	    void setStream(void *stream) {
            sceneRecoEngine->setStream(stream);
	        // add swapping engine here if needed
	    }
        void syncStream() {
	        sceneRecoEngine->syncStream();
	    };

		void ResetScene(ITMScene<TVoxel,TIndex> *scene) const;

		/// Process a single frame
		void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState_live, bool resetVisibleList = false);

		/// Update the visible list (this can be called to update the visible list when fusion is turned off)
		void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState, bool resetVisibleList = false);

        /// Performa integration of local memory and active memory.
        void UpdateVisibleVolume(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState);

        ITMSceneReconstructionEngine<TVoxel,TIndex>* getSceneRecoEngine(){ return sceneRecoEngine; }

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		explicit ITMDenseMapper(const ITMLibSettings *settings);
		~ITMDenseMapper();
	};
}

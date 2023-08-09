// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once


#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Scene/ITMGlobalCache.h"

#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "../../Reconstruction/Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../../Utils/ITMCUDAUtils.h"
#include <ORUtils/HSVToRGB.h>

namespace ITMLib
{
	// declaration of device functions

	__global__ void buildCompleteVisibleList_device(const ITMHashEntry *hashTable, ITMHashSwapState *swapStates, bool useSwapping, int noTotalEntries,
		int *visibleEntryIDs, int *noVisibleEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize);

	__global__ void countVisibleBlocks_device(const int *visibleEntryIDs, int noVisibleEntries, const ITMHashEntry *hashTable, uint *noBlocks, int minBlockId, int maxBlockId);

	__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *visibleEntryIDs, int noVisibleEntries,
		const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
		uint *noTotalBlocks);

	__global__ void checkProjectAndSplitBlocks_device(const ITMHashEntry *hashEntries, int noHashEntries,
		const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
		uint *noTotalBlocks);

	__global__ void fillBlocks_device(uint noTotalBlocks, const RenderingBlock *renderingBlocks,
		Vector2i imgSize, Vector2f *minmaxData);

	__global__ void findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints, const Vector2f *minmaximg,
		Vector4f *forwardProjection, float *currentDepth, Vector2i imgSize);

	__global__ void forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize, Matrix4f M,
		Vector4f projParams, float voxelSize);

	template<class TVoxel, class TIndex, bool modifyVisibleEntries>
	__global__ void genericRaycast_device(Vector4f *out_ptsRay, uchar *entriesVisibleType, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f invProjParams,
		float oneOverVoxelSize, const Vector2f *minmaximg, float mu, bool render_in_perspective)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

        castRay<TVoxel, TIndex, modifyVisibleEntries>(out_ptsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2], render_in_perspective);
	}
    template<class TVoxel, class TIndex, bool modifyVisibleEntries>
    __global__ void genericRaycastMissingPoints_device(Vector4f *forwardProjection, uchar *entriesVisibleType, const TVoxel *voxelData,
                                                       const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize,
                                                       int *fwdProjMissingPoints, int noMissingPoints, const Vector2f *minmaximg, float mu, bool usePerspective)
    {
        int pointId = threadIdx.x + blockIdx.x * blockDim.x;

        if (pointId >= noMissingPoints) return;

        int locId = fwdProjMissingPoints[pointId];
        int y = locId / imgSize.x, x = locId - y*imgSize.x;
        int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

        castRay<TVoxel, TIndex, modifyVisibleEntries>(forwardProjection[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2], usePerspective);
    }

//    template<class TVoxel, class TIndex, bool modifyVisibleEntries>
//    __global__ void genericRaycast_device(Vector4f *out_ptsRay, uchar *entriesVisibleType, const TVoxel *voxelData,
//                                          const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f invProjParams,
//                                          float oneOverVoxelSize, const Vector2f *minmaximg, float mu)
//    {
//        int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);
//
//        if (x >= imgSize.x || y >= imgSize.y) return;
//
//        int locId = x + y * imgSize.x;
//        int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;
//
//        castRay<TVoxel, TIndex, modifyVisibleEntries>(out_ptsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
//    }

//	template<class TVoxel, class TIndex, bool modifyVisibleEntries>
//	__global__ void genericRaycastMissingPoints_device(Vector4f *forwardProjection, uchar *entriesVisibleType, const TVoxel *voxelData,
//		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize,
//		int *fwdProjMissingPoints, int noMissingPoints, const Vector2f *minmaximg, float mu)
//	{
//		int pointId = threadIdx.x + blockIdx.x * blockDim.x;
//
//		if (pointId >= noMissingPoints) return;
//
//		int locId = fwdProjMissingPoints[pointId];
//		int y = locId / imgSize.x, x = locId - y*imgSize.x;
//		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;
//
//		castRay<TVoxel, TIndex, modifyVisibleEntries>(forwardProjection[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
//	}

	template<bool flipNormals>
	__global__ void renderICP_device(Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *pointsRay,
		float voxelSize, Vector2i imgSize, Vector3f lightSource
//#ifndef NDEBUG
//            , uint *counter
//#endif
	)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		bool found = processPixelICP<true, flipNormals>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
//#ifndef NDEBUG
//		atomicAdd(counter, found);
//#endif
	}

	template<bool flipNormals>
	__global__ void renderGrey_ImageNormals_device(Vector4u *outRendering, const Vector4f *pointsRay, float voxelSize, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelGrey_ImageNormals<true, flipNormals>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
	}

    template<class TVoxel, class TIndex, bool flipNormals>
    __global__ void renderLabelColor_ImageNormals_device(Vector4u *outRendering, const Vector4f *pointsRay, const TVoxel *voxelData,
    const typename TIndex::IndexData *voxelIndex, const Vector4f *labelColors, float voxelSize, Vector2i imgSize, Vector3f lightSource)
    {
        int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

        if (x >= imgSize.x || y >= imgSize.y) return;

        processPixelLabel_ImageNormals<TVoxel, TIndex, true, flipNormals>(outRendering, pointsRay, voxelData, voxelIndex, labelColors, imgSize, x, y, voxelSize, lightSource);

    }

	template<bool flipNormals>
	__global__ void renderNormals_ImageNormals_device(Vector4u *outRendering, const Vector4f *ptsRay, Vector2i imgSize, float voxelSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelNormals_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
	}

	template<bool flipNormals>
	__global__ void renderConfidence_ImageNormals_device(Vector4u *outRendering, const Vector4f *ptsRay, Vector2i imgSize, float voxelSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelConfidence_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderGrey_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, float voxelSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];


//        if(TVoxel::integrateType == SCFusion::IntegrateType_TSDF)
//            processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
//        else if(TVoxel::integrateType == SCFusion::IntegrateType_OFusion)
        processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderNormal_device(Vector4f *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		if (ptRay.w <= 0) return;
		outRendering[locId] = Vector4f(computeSingleNormalFromSDF(voxelData, voxelIndex, ptRay.toVector3().normalised()),1.f);

	}

    template<class TVoxel, class TIndex>
    __global__ void renderLabelColor_device(Vector4u *outRendering, const Vector4f *ptsRay, const Vector4f *labelColors, const TVoxel *voxelData,
                                      const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, float voxelSize, Vector3f lightSource)
    {
        int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

        if (x >= imgSize.x || y >= imgSize.y) return;

        int locId = x + y * imgSize.x;

        Vector4f ptRay = ptsRay[locId];
        processLabelNormal<TVoxel,TIndex>(outRendering[locId], ptRay.toVector3(), labelColors, ptRay.w > 0, voxelData, voxelIndex, lightSource);
    }

    template<class TVoxel, class TIndex>
    __global__ void renderLabel_device(ushort *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
                                       const typename TIndex::IndexData *voxelIndex, Vector2i imgSize)
    {
        int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

        if (x >= imgSize.x || y >= imgSize.y) return;

        int locId = x + y * imgSize.x;

        Vector4f ptRay = ptsRay[locId];
        if(ptRay.w > 0) //outRendering[locId] = 1;
            processLabel<TVoxel,TIndex>(outRendering[locId],Vector3f(ptRay),voxelData,voxelIndex);
        else outRendering[locId] = 0;

    }

    template<class TVoxel, class TIndex>
    __global__ void renderSemantic_device(ushort *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
                                       const typename TIndex::IndexData *voxelIndex, Vector2i imgSize)
    {
        int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

        if (x >= imgSize.x || y >= imgSize.y) return;

        int locId = x + y * imgSize.x;

        Vector4f ptRay = ptsRay[locId];
        if(ptRay.w > 0)
            processSemantic<TVoxel,TIndex>(outRendering[locId],Vector3f(ptRay),voxelData,voxelIndex);
        else outRendering[locId] = 0;

    }

    template<class TVoxel, class TIndex>
	__global__ void renderColourFromNormal_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];
        processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderColourFromConfidence_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderPointCloud_device(/*Vector4u *outRendering, */Vector4f *locations, Vector4f *colours, uint *noTotalPoints,
		const Vector4f *ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints,
		float voxelSize, Vector2i imgSize, Vector3f lightSource)
	{
		__shared__ bool shouldPrefix;
		shouldPrefix = false;
		__syncthreads();

		bool foundPoint = false; Vector3f point(0.0f);

		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x < imgSize.x && y < imgSize.y)
		{
			int locId = x + y * imgSize.x;
			Vector3f outNormal; float angle; Vector4f pointRay;

			pointRay = ptsRay[locId];
			point = pointRay.toVector3();
			foundPoint = pointRay.w > 0;

			computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

			if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

			if (foundPoint) shouldPrefix = true;
		}

		__syncthreads();

		if (shouldPrefix)
		{
			int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);

			if (offset != -1)
			{
				Vector4f tmp;
				tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
				if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
				colours[offset] = tmp;

				Vector4f pt_ray_out;
				pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
				pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
				locations[offset] = pt_ray_out;
			}
		}
	}

	template<class TVoxel, class TIndex>
	__global__ void findLowHeight(int size, Vector4f *data, float *low, float *height){
	    CUDA_1D_LOOP(i, size) {
	        if(data[i].y > *height) atomicMax(height, data[i].y);
            if(data[i].y < *low) atomicMax(low, data[i].y);
	    }
	}

    template<class TVoxel, class TIndex>
    __global__ void float3ToHeight_device(Vector4u *outRendering, Vector4f *data, float renderLowBound, float renderHighBound, Vector2i imgSize){
        /// Inversely
        int x = blockIdx.x*blockDim.x + threadIdx.x;
        int y = blockIdx.y*blockDim.y + threadIdx.y;
        if( x >= imgSize.x || y >= imgSize.y ) return;

        int H = imgSize.y-1-y;

        int index = H * imgSize.x + x;
        int index_vert = y*imgSize.x+x;

        outRendering[index].w = 255;

        if(data[index_vert ].x==0&&data[index_vert ].y==0&&data[index_vert ].z==0){
            outRendering[index].x = 0;
            outRendering[index].y = 0;
            outRendering[index].z = 0;
            return;
        }

        outRendering[index] = (SCFUSION::ValueHSVToRGBA(data[index_vert].y, renderLowBound, renderHighBound) * 255).toUChar();
    }


    
//    template<class TVoxel, class TIndex>
//    __global__ void renderLabel_device(Vector4u *outRendering, ushort *labelRay, const Vector4f *correspondingColor, Vector2i imgSize)
//    {
//        int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);
//
//        if (x >= imgSize.x || y >= imgSize.y) return;
//
//        int locId = x + y * imgSize.x;
//
//        //Vector4u color = correspondingColor[locId];
//
//        if(correspondingColor != nullptr)
//            outRendering[locId] = (correspondingColor[labelRay[locId]]*255).toUChar();
//        else
//            outRendering[locId] = Vector4u(255,255,255,255);
//    }

	template<class TVoxel, class TIndex>
	__global__ void renderColour_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
	}
}

// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Objects/Scene/ITMVoxelTypes.h"
#include "../../../Objects/Scene/IntegratePolicies.h"

static const CONSTPTR(int) MAX_RENDERING_BLOCKS = 65536*4;
//static const int MAX_RENDERING_BLOCKS = 16384;
static const CONSTPTR(int) minmaximg_subsample = 8;

#if !(defined __METALC__)

struct RenderingBlock {
	Vector2s upperLeft;
	Vector2s lowerRight;
	Vector2f zRange;
};

#ifndef FAR_AWAY
#define FAR_AWAY 999999.9f
#endif

#ifndef VERY_CLOSE
#define VERY_CLOSE 0.3f
#endif

static const CONSTPTR(int) renderingBlockSizeX = 16;
static const CONSTPTR(int) renderingBlockSizeY = 16;

_CPU_AND_GPU_CODE_ inline Vector4f InvertProjectionParams(const THREADPTR(Vector4f)& projParams)
{
	return Vector4f(1.0f / projParams.x, 1.0f / projParams.y, -projParams.z, -projParams.w);
}

_CPU_AND_GPU_CODE_ inline Matrix3f GetInvPerspectiveProjMatrix(const THREADPTR(Vector4f)& projParams)
{
    Matrix3f projM;
    projM.setIdentity();
    projM.at(0,0) = 1/projParams.x;
    projM.at(1,1) = 1/projParams.y;
    projM.at(2,0) = -projParams.z/projParams.x;
    projM.at(2,1) = -projParams.w/projParams.y;
    return projM;
}

_CPU_AND_GPU_CODE_ inline Matrix3f GetOrthographicProjMatrix(const THREADPTR(Vector4f) & projParams)
{
    // note: not tested yet.
    // left, right, bottom, top
    Matrix3f projM;
    projM.setIdentity();
    projM.at(0,0) = 2/(projParams.y - projParams.x);
    projM.at(1,1) = 2/(projParams.w - projParams.z);
    projM.at(2,0) = -(projParams.y + projParams.x) / (projParams.y - projParams.x);
    projM.at(2,1) = -(projParams.w + projParams.z) / (projParams.w - projParams.z);
    return projM;
}

_CPU_AND_GPU_CODE_ inline bool ProjectSingleBlock(const THREADPTR(Vector3s) & blockPos, const THREADPTR(Matrix4f) & pose, const THREADPTR(Vector4f) & intrinsics, 
	const THREADPTR(Vector2i) & imgSize, float voxelSize, THREADPTR(Vector2i) & upperLeft, THREADPTR(Vector2i) & lowerRight, THREADPTR(Vector2f) & zRange)
{
	upperLeft = imgSize / minmaximg_subsample;
	lowerRight = Vector2i(-1, -1);
	zRange = Vector2f(FAR_AWAY, VERY_CLOSE);
	for (int corner = 0; corner < 8; ++corner)
	{
		// project all 8 corners down to 2D image
		Vector3s tmp = blockPos;
		tmp.x += (corner & 1) ? 1 : 0;
		tmp.y += (corner & 2) ? 1 : 0;
		tmp.z += (corner & 4) ? 1 : 0;
		Vector4f pt3d(TO_FLOAT3(tmp) * (float)SDF_BLOCK_SIZE * voxelSize, 1.0f);
		pt3d = pose * pt3d;
		if (pt3d.z < 1e-6) continue;

		Vector2f pt2d;
		pt2d.x = (intrinsics.x * pt3d.x / pt3d.z + intrinsics.z) / minmaximg_subsample;
		pt2d.y = (intrinsics.y * pt3d.y / pt3d.z + intrinsics.w) / minmaximg_subsample;

		// remember bounding box, zmin and zmax
		if (upperLeft.x > floor(pt2d.x)) upperLeft.x = (int)floor(pt2d.x);
		if (lowerRight.x < ceil(pt2d.x)) lowerRight.x = (int)ceil(pt2d.x);
		if (upperLeft.y > floor(pt2d.y)) upperLeft.y = (int)floor(pt2d.y);
		if (lowerRight.y < ceil(pt2d.y)) lowerRight.y = (int)ceil(pt2d.y);
		if (zRange.x > pt3d.z) zRange.x = pt3d.z;
		if (zRange.y < pt3d.z) zRange.y = pt3d.z;
	}

	// do some sanity checks and respect image bounds
	if (upperLeft.x < 0) upperLeft.x = 0;
	if (upperLeft.y < 0) upperLeft.y = 0;
	if (lowerRight.x >= imgSize.x) lowerRight.x = imgSize.x - 1;
	if (lowerRight.y >= imgSize.y) lowerRight.y = imgSize.y - 1;
	if (upperLeft.x > lowerRight.x) return false;
	if (upperLeft.y > lowerRight.y) return false;
	//if (zRange.y <= VERY_CLOSE) return false; never seems to happen
	if (zRange.x < VERY_CLOSE) zRange.x = VERY_CLOSE;
	if (zRange.y < VERY_CLOSE) return false;

	return true;
}

_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks(DEVICEPTR(RenderingBlock) *renderingBlockList, int offset,
	const THREADPTR(Vector2i) & upperLeft, const THREADPTR(Vector2i) & lowerRight, const THREADPTR(Vector2f) & zRange)
{
	// split bounding box into 16x16 pixel rendering blocks
	for (int by = 0; by < ceil((float)(1 + lowerRight.y - upperLeft.y) / renderingBlockSizeY); ++by) {
		for (int bx = 0; bx < ceil((float)(1 + lowerRight.x - upperLeft.x) / renderingBlockSizeX); ++bx) {
			if (offset >= MAX_RENDERING_BLOCKS) return;
			//for each rendering block: add it to the list
			DEVICEPTR(RenderingBlock) & b(renderingBlockList[offset++]);
			b.upperLeft.x = upperLeft.x + bx*renderingBlockSizeX;
			b.upperLeft.y = upperLeft.y + by*renderingBlockSizeY;
			b.lowerRight.x = upperLeft.x + (bx + 1)*renderingBlockSizeX - 1;
			b.lowerRight.y = upperLeft.y + (by + 1)*renderingBlockSizeY - 1;
			if (b.lowerRight.x>lowerRight.x) b.lowerRight.x = lowerRight.x;
			if (b.lowerRight.y>lowerRight.y) b.lowerRight.y = lowerRight.y;
			b.zRange = zRange;
		}
	}
}

#endif

template<class TVoxel, class TIndex, bool modifyVisibleEntries, SCFUSION::IntegrateType TIntegrate>struct rayLoopSC;
template<class TVoxel, class TIndex, bool modifyVisibleEntries> struct rayLoopSC<TVoxel,TIndex,modifyVisibleEntries, SCFUSION::IntegrateType_TSDF>{
    _CPU_AND_GPU_CODE_ static bool compute(float &confidence, Vector3f &pt_result, DEVICEPTR(uchar) *entriesVisibleType, float &totalLength,
                                           const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                           const Vector3f &rayDirection, const float &totalLengthMax, const float &stepScale){
        typename TIndex::IndexCache cache;
        int vmIndex;
        float sdfValue = 1.0f, stepLength;

        sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, vmIndex, cache);
        Vector3i index,indexPrev;
        float sdfPrev;
        bool pt_found = false;

        while (totalLength < totalLengthMax) {
            indexPrev = index;
            sdfPrev = sdfValue;

            index = Vector3i((int)ROUND(pt_result.x), (int)ROUND(pt_result.y), (int)ROUND(pt_result.z));
            sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, vmIndex, cache);

            if (modifyVisibleEntries) {
                if (vmIndex) entriesVisibleType[vmIndex - 1] = 1;
            }

            if (!vmIndex) {
                stepLength = SDF_BLOCK_SIZE;
            } else {
                indexPrev = index;

                if(sdfPrev*sdfValue<0.f){
                    pt_result = Vector3f(index.x,index.y,index.z);
                    return true;
                }
                stepLength = MAX(sdfValue * stepScale, 1.0f);
            }
            pt_result += stepLength * rayDirection;
            totalLength += stepLength;
        }
        return false;
    }
};

template<class TVoxel, class TIndex, bool modifyVisibleEntries, SCFUSION::IntegrateType TIntegrate>struct rayLoop;
template<class TVoxel, class TIndex, bool modifyVisibleEntries> struct rayLoop<TVoxel,TIndex,modifyVisibleEntries, SCFUSION::IntegrateType_TSDF>{
    _CPU_AND_GPU_CODE_ static bool compute(float &confidence, Vector3f &pt_result, DEVICEPTR(uchar) *entriesVisibleType, float &totalLength,
                                           const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                           const Vector3f &rayDirection, const float &totalLengthMax, const float &stepScale){
        typename TIndex::IndexCache cache;
        int vmIndex=1;
        TVoxel voxel;
        float stepLength = SDF_BLOCK_SIZE;
        float f_t, f_tt;
        Vector3f pt;
        f_t = readFromSDF_float_interpolated(voxelData, voxelIndex, pt_result, vmIndex, cache);
//        if(f_t <= 0) return false; // already inside
        f_tt = 0;
        bool found = false;
//        totalLength = 1;
        while (totalLength < totalLengthMax) {
            pt = pt_result+totalLength*rayDirection;
//            sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, vmIndex, label, cache);
            voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);;

            if (modifyVisibleEntries) {
                if (vmIndex) entriesVisibleType[vmIndex - 1] = 1;
            }



            if (!found && !vmIndex) {
                stepLength = SDF_BLOCK_SIZE;
                totalLength += stepLength;
                continue;
            }

            if(!found){
                found = true;
                totalLength -= stepLength;
                stepLength = 1;
                continue;
            }

//            f_tt = voxel.sdf;
//            if ((f_tt <= 0.5f) && (f_tt >= -0.5f)) {
            f_tt = readFromSDF_float_interpolated(voxelData, voxelIndex, pt, vmIndex, cache);
//            }
            if( f_tt * f_t < 0) break;
//                if (f_tt < 0.0f) break;
//            stepLength = MAX(f_tt * stepScale, 1.0f);
            stepLength = 1;
            totalLength += stepLength;
            f_t = f_tt;
        }

        if( f_tt * f_t < 0) {
            totalLength = totalLength - ABS( stepLength * f_tt / (f_t - f_tt) );
            pt_result = pt_result + totalLength * rayDirection;
            readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, pt_result, vmIndex, cache);
            return totalLength > 0;

//            totalLength+=f_tt*stepScale;
//            pt_result = pt_result + totalLength * rayDirection;
//            if(voxelData->hasLabelInformation)
//                f_tt = readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, pt_result, vmIndex, label, cache);
//            else
//                f_tt = readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, pt_result, vmIndex, cache);
//            totalLength+=f_tt*stepScale;
//            pt_result = pt_result + totalLength * rayDirection;
//
//            return totalLength > 0;
        } else return false;
    }
};
template<class TVoxel, class TIndex, bool modifyVisibleEntries> struct rayLoop<TVoxel,TIndex,modifyVisibleEntries, SCFUSION::IntegrateType_OFusion>{
    _CPU_AND_GPU_CODE_ static bool compute(float &confidence, Vector3f &pt_result, DEVICEPTR(uchar) *entriesVisibleType, float &totalLength,
                                           const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                           const Vector3f &rayDirection, const float &totalLengthMax, const float &stepScale){
        typename TIndex::IndexCache cache;
        int vmIndex=1;
        float stepLength=SDF_BLOCK_SIZE;
        float f_t, f_tt=0;
        TVoxel voxel;
        Vector3f pt;
        pt = pt_result+totalLength*rayDirection;
        f_t = readFromSDF_float_interpolated(voxelData, voxelIndex, pt, vmIndex, cache);
        stepLength = SDF_BLOCK_SIZE;
        totalLength += stepLength;
        bool found = false;
//        if(f_t > LOGODD_SURFACE) return false;

        while (totalLength < totalLengthMax) {
            pt = pt_result+totalLength*rayDirection;
            voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);;

            if (modifyVisibleEntries) {
                if (vmIndex) entriesVisibleType[vmIndex - 1] = 1;
            }

            if (!found && !vmIndex) {
                stepLength = SDF_BLOCK_SIZE;
                totalLength += stepLength;
                continue;
            }

            if(!found){
                found = true;
                totalLength -= stepLength;
                stepLength = stepScale;
                totalLength += stepLength;
                continue;
            }
//#define TRUNCATE_HEIGHT
#ifdef TRUNCATE_HEIGHT
            // truncating height
            if(pt.y < 2.5/0.047) {//TODO: make this an option?
                f_tt = voxel.sdf;
                f_tt = readFromSDF_float_interpolated(voxelData, voxelIndex, pt, vmIndex, cache);
                if (f_tt > LOGODD_SURFACE) break;
            } else
            {
                f_tt = -1;
            }
#else
            f_tt = readFromSDF_float_interpolated(voxelData, voxelIndex, pt, vmIndex, cache);
            if (f_tt > LOGODD_SURFACE) break;
#endif
            f_t = f_tt;
            stepLength = stepScale;
            totalLength += stepLength;
        }
        if(f_tt > LOGODD_SURFACE) {
            if(f_tt != f_t) {
                totalLength -= (stepLength * (f_tt - LOGODD_SURFACE) / (f_tt - f_t));
//                totalLength += stepLength;
                pt = pt_result + totalLength * rayDirection;
                voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);;
                if(voxel.w_depth == 0) {
                    totalLength += (stepLength * (f_tt - LOGODD_SURFACE) / (f_tt - f_t));
                }
            }
//            totalLength=50;
            if(totalLength>0) {
                pt_result = pt_result + totalLength * rayDirection;
//                pt_result = pt_result + 25 * rayDirection;
                readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, pt_result, vmIndex,
                                                             cache);
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
};
template<class TVoxel, class TIndex, bool modifyVisibleEntries> struct rayLoop<TVoxel,TIndex,modifyVisibleEntries, SCFUSION::IntegrateType_Invert_TSDF>{
    _CPU_AND_GPU_CODE_ static bool compute(float &confidence, Vector3f &pt_result, DEVICEPTR(uchar) *entriesVisibleType, float &totalLength,
                                           const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                           const Vector3f &rayDirection, const float &totalLengthMax, const float &stepScale){
        return rayLoop<TVoxel,TIndex,modifyVisibleEntries, SCFUSION::IntegrateType_OFusion>::compute(confidence, pt_result, entriesVisibleType, totalLength, voxelData, voxelIndex,
                                                                                                     rayDirection, totalLengthMax, stepScale);
    }
};

template<class TVoxel, class TIndex, bool modifyVisibleEntries>
_CPU_AND_GPU_CODE_ inline bool castRay(DEVICEPTR(Vector4f) &pt_out, DEVICEPTR(uchar) *entriesVisibleType,
	int x, int y, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
	Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize, float mu, const CONSTPTR(Vector2f) & viewFrustum_minmax,
	bool perspective)
{
	Vector4f pt_camera_f; Vector3f pt_block_s, pt_block_e, rayDirection, pt_result;
	bool pt_found;
	float confidence=0;
	float totalLength, totalLengthMax, stepScale;

	stepScale = 1;//mu * oneOverVoxelSize;

    if(!perspective){
        pt_camera_f.z = viewFrustum_minmax.y;
    } else {
        pt_camera_f.z = VERY_CLOSE;// viewFrustum_minmax.x;
    }

    pt_camera_f.x = pt_camera_f.z * ((float(x) + invProjParams.z) * invProjParams.x);
    pt_camera_f.y = pt_camera_f.z * ((float(y) + invProjParams.w) * invProjParams.y);
    if(!perspective) pt_camera_f.z = VERY_CLOSE;// viewFrustum_minmax.x;
    pt_camera_f.w = 1.0f;
    totalLength = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;
    pt_block_s = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;

    pt_camera_f.z = viewFrustum_minmax.y;
    pt_camera_f.x = pt_camera_f.z * ((float(x) + invProjParams.z) * invProjParams.x);
    pt_camera_f.y = pt_camera_f.z * ((float(y) + invProjParams.w) * invProjParams.y);
    pt_camera_f.w = 1.0f;
    totalLengthMax = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;
    pt_block_e = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;


	rayDirection = pt_block_e - pt_block_s;
	float direction_norm = 1.0f / sqrt(rayDirection.x * rayDirection.x + rayDirection.y * rayDirection.y + rayDirection.z * rayDirection.z);
	rayDirection *= direction_norm;

	pt_result = pt_block_s;

	typename TIndex::IndexCache cache;

    pt_found = rayLoop<TVoxel,TIndex,modifyVisibleEntries, TVoxel::integrateType>::compute
            (confidence, pt_result,entriesVisibleType,totalLength,voxelData,voxelIndex,rayDirection,totalLengthMax,stepScale);

	if (pt_found) {
        pt_out.x = pt_result.x; pt_out.y = pt_result.y; pt_out.z = pt_result.z;
	    pt_out.w = confidence + 1.0f;
	} else {
        pt_out.x = 0; pt_out.y=0; pt_out.z=0; pt_out.w = 0.0f;
	}
	return pt_found;
}

_CPU_AND_GPU_CODE_ inline int forwardProjectPixel(Vector4f pixel, const CONSTPTR(Matrix4f) &M, const CONSTPTR(Vector4f) &projParams,
	const THREADPTR(Vector2i) &imgSize)
{
	pixel.w = 1;
	pixel = M * pixel;

	Vector2f pt_image;
	pt_image.x = projParams.x * pixel.x / pixel.z + projParams.z;
	pt_image.y = projParams.y * pixel.y / pixel.z + projParams.w;

	if ((pt_image.x < 0) || (pt_image.x > imgSize.x - 1) || (pt_image.y < 0) || (pt_image.y > imgSize.y - 1)) return -1;

	return (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool) & foundPoint, const THREADPTR(Vector3f) & point,
                                                     const CONSTPTR(TVoxel) *voxelBlockData, const CONSTPTR(typename TIndex::IndexData) *indexData,
                                                     const THREADPTR(Vector3f) & lightSource, THREADPTR(Vector3f) & outNormal, THREADPTR(float) & angle)
{
	if (!foundPoint) return;
	outNormal = computeSingleNormalFromSDF(voxelBlockData, indexData, point).normalised();
    Vector3f viewDir = -(point - lightSource).normalised();

    angle = outNormal.x * viewDir.x + outNormal.y * viewDir.y + outNormal.z * viewDir.z;
    angle = ABS(angle);//TODO: this is for showing front and back side. maybe make this an option?
//	if (angle <= 0.0) foundPoint = false;

}

template <bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool) & foundPoint, const THREADPTR(int) &x, const THREADPTR(int) &y,
	const CONSTPTR(Vector4f) *pointsRay, const THREADPTR(Vector3f) & lightSource, const THREADPTR(float) &voxelSize,
	const THREADPTR(Vector2i) &imgSize, THREADPTR(Vector3f) & outNormal, THREADPTR(float) & angle)
{
	if (!foundPoint) return;

	Vector4f xp1_y, xm1_y, x_yp1, x_ym1;

	if (useSmoothing)
	{
		if (y <= 2 || y >= imgSize.y - 3 || x <= 2 || x >= imgSize.x - 3) { foundPoint = false; return; }

		xp1_y = pointsRay[(x + 2) + y * imgSize.x], x_yp1 = pointsRay[x + (y + 2) * imgSize.x];
		xm1_y = pointsRay[(x - 2) + y * imgSize.x], x_ym1 = pointsRay[x + (y - 2) * imgSize.x];
	}
	else
	{
		if (y <= 1 || y >= imgSize.y - 2 || x <= 1 || x >= imgSize.x - 2) { foundPoint = false; return; }

		xp1_y = pointsRay[(x + 1) + y * imgSize.x], x_yp1 = pointsRay[x + (y + 1) * imgSize.x];
		xm1_y = pointsRay[(x - 1) + y * imgSize.x], x_ym1 = pointsRay[x + (y - 1) * imgSize.x];
	}

	Vector4f diff_x(0.0f, 0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f, 0.0f);

	bool doPlus1 = false;
	if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0) doPlus1 = true;
	else
	{
		diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

		float length_diff = MAX(diff_x.x * diff_x.x + diff_x.y * diff_x.y + diff_x.z * diff_x.z,
			diff_y.x * diff_y.x + diff_y.y * diff_y.y + diff_y.z * diff_y.z);

		if (length_diff * voxelSize * voxelSize > (0.15f * 0.15f)) doPlus1 = true;
	}

	if (doPlus1)
	{
		if (useSmoothing)
		{
			xp1_y = pointsRay[(x + 1) + y * imgSize.x]; x_yp1 = pointsRay[x + (y + 1) * imgSize.x];
			xm1_y = pointsRay[(x - 1) + y * imgSize.x]; x_ym1 = pointsRay[x + (y - 1) * imgSize.x];
			diff_x = xp1_y - xm1_y; diff_y = x_yp1 - x_ym1;
		}

		if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0)
		{
			foundPoint = false;
			return;
		}
	}

	outNormal.x = -(diff_x.y * diff_y.z - diff_x.z*diff_y.y);
	outNormal.y = -(diff_x.z * diff_y.x - diff_x.x*diff_y.z);
	outNormal.z = -(diff_x.x * diff_y.y - diff_x.y*diff_y.x);

	if (flipNormals) outNormal = -outNormal;

	float normScale = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
    outNormal *= normScale;

    Vector3f viewDir = -(Vector3f(pointsRay[(x) + y * imgSize.x]) - lightSource).normalised();
    angle = outNormal.x * viewDir.x + outNormal.y * viewDir.y + outNormal.z * viewDir.z;
    angle = ABS(angle);//TODO: this is for showing front and back side. maybe make this an option?

//	angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
//	if (angle <= 0.0) foundPoint = false;
}

_CPU_AND_GPU_CODE_ inline void drawPixelGrey(DEVICEPTR(Vector4u) & dest, const THREADPTR(float) & angle)
{
	float outRes = (0.5f * angle + 0.5f) * 255.0f;
	dest = Vector4u((uchar)outRes);
}

_CPU_AND_GPU_CODE_ inline float interpolateCol(float val, float y0, float x0, float y1, float x1) {
	return (val - x0)*(y1 - y0) / (x1 - x0) + y0;
}

_CPU_AND_GPU_CODE_ inline float baseCol(float val) {
	if (val <= -0.75f) return 0.0f;
	else if (val <= -0.25f) return interpolateCol(val, 0.0f, -0.75f, 1.0f, -0.25f);
	else if (val <= 0.25f) return 1.0f;
	else if (val <= 0.75f) return interpolateCol(val, 1.0f, 0.25f, 0.0f, 0.75f);
	else return 0.0;
}

_CPU_AND_GPU_CODE_ inline void drawPixelConfidence(DEVICEPTR(Vector4u) & dest, const THREADPTR(float) & angle, const THREADPTR(float) & confidence)
{
	//Vector4f color_red(255, 0, 0, 255), color_green(0, 255, 0, 255);
	float confidenceNorm = CLAMP(confidence, 0, 100.f) / 100.0f;

	Vector4f color;
	color.r = (uchar)(baseCol(confidenceNorm) * 255.0f);
	color.g = (uchar)(baseCol(confidenceNorm - 0.5f) * 255.0f); 
	color.b = (uchar)(baseCol(confidenceNorm + 0.5f) * 255.0f);
	color.a = 255;

	Vector4f outRes = (0.5f * angle + 0.5f) * color;
	dest = TO_UCHAR4(outRes);
}

_CPU_AND_GPU_CODE_ inline void drawShadedColor(DEVICEPTR(Vector4u) & dest, const THREADPTR(Vector3f) & normal_obj, const THREADPTR(Vector4u) &color)
{
    dest.r = (uchar)((0.3f + (-normal_obj.r + 1.0f)*0.35f)*(float)color.x);
    dest.g = (uchar)((0.3f + (-normal_obj.g + 1.0f)*0.35f)*(float)color.y);
    dest.b = (uchar)((0.3f + (-normal_obj.b + 1.0f)*0.35f)*(float)color.z);
}

_CPU_AND_GPU_CODE_ inline void drawShadedColor(DEVICEPTR(Vector4u) & dest, const THREADPTR(float) & angle, const THREADPTR(Vector4u) &color)
{
    dest = (color.toFloat()*(0.5 * angle + 0.5f)).toUChar();
}

_CPU_AND_GPU_CODE_ inline void drawPixelNormal(DEVICEPTR(Vector4u) & dest, const THREADPTR(Vector3f) & normal_obj)
{
	dest.r = (uchar)((0.3f + (-normal_obj.r + 1.0f)*0.35f)*255.0f);
	dest.g = (uchar)((0.3f + (-normal_obj.g + 1.0f)*0.35f)*255.0f);
	dest.b = (uchar)((0.3f + (-normal_obj.b + 1.0f)*0.35f)*255.0f);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawLabelColour(DEVICEPTR(Vector4u) &dest, const CONSTPTR(Vector3f) & point, const CONSTPTR(Vector4f) *labelColors,
   const CONSTPTR(TVoxel) *voxelBlockData, const CONSTPTR(typename TIndex::IndexData) *indexData){
    ushort label = VoxelLabelReader<TVoxel::hasLabelInformation, TVoxel, TIndex>::maxSampling(voxelBlockData,indexData,point);

    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
    label = accessLabel<TVoxel>::readMaxWNeighborLabel(pos,
                                                              voxelBlockData,
                                                              indexData,
                                                              1);
    dest = (labelColors[label]*255).toUChar();
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawPixelColour(DEVICEPTR(Vector4u) & dest, const CONSTPTR(Vector3f) & point, 
	const CONSTPTR(TVoxel) *voxelBlockData, const CONSTPTR(typename TIndex::IndexData) *indexData)
{
	Vector4f clr = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelBlockData, indexData, point);

	dest.x = (uchar)(clr.x * 255.0f);
	dest.y = (uchar)(clr.y * 255.0f);
	dest.z = (uchar)(clr.z * 255.0f);
	dest.w = 255;
}


template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline bool processPixelICP(DEVICEPTR(Vector4f) &pointsMap, DEVICEPTR(Vector4f) &normalsMap,
	const THREADPTR(Vector3f) & point, bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
	float voxelSize, const THREADPTR(Vector3f) &lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint)
	{
		Vector4f outPoint4;
		outPoint4.x = point.x * voxelSize; outPoint4.y = point.y * voxelSize;
		outPoint4.z = point.z * voxelSize; outPoint4.w = 1.0f;
		pointsMap = outPoint4;

		Vector4f outNormal4;
		outNormal4.x = outNormal.x; outNormal4.y = outNormal.y; outNormal4.z = outNormal.z; outNormal4.w = 0.0f;
		normalsMap = outNormal4;
	}
	else
	{
		Vector4f out4;
		out4.x = 0.0f; out4.y = 0.0f; out4.z = 0.0f; out4.w = -1.0f;

		pointsMap = out4; normalsMap = out4;
	}
	return foundPoint;
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline bool  processPixelICP(DEVICEPTR(Vector4f) *pointsMap, DEVICEPTR(Vector4f) *normalsMap,
	const CONSTPTR(Vector4f) *pointsRay, const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize,
	const THREADPTR(Vector3f) &lightSource)
{
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;

	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint)
	{
		Vector4f outPoint4;
		outPoint4.x = point.x * voxelSize; outPoint4.y = point.y * voxelSize;
		outPoint4.z = point.z * voxelSize; outPoint4.w = point.w;//outPoint4.w = 1.0f;
		pointsMap[locId] = outPoint4;

		Vector4f outNormal4;
		outNormal4.x = outNormal.x; outNormal4.y = outNormal.y; outNormal4.z = outNormal.z; outNormal4.w = 0.0f;
		normalsMap[locId] = outNormal4;
	}
	else
	{
		Vector4f out4;
		out4.x = 0.0f; out4.y = 0.0f; out4.z = 0.0f; out4.w = -1.0f;

		pointsMap[locId] = out4; normalsMap[locId] = out4;
	}
	return foundPoint;
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelGrey_ImageNormals(DEVICEPTR(Vector4u) *outRendering, const CONSTPTR(Vector4f) *pointsRay, 
	const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize, const THREADPTR(Vector3f) &lightSource)
{
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;

	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelGrey(outRendering[locId], angle);
	else outRendering[locId] = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex, bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelLabel_ImageNormals(DEVICEPTR(Vector4u) *outRendering, const CONSTPTR(Vector4f) *pointsRay,
                                                              const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                              const CONSTPTR(Vector4f) *labelColors,
                                                             const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize, const THREADPTR(Vector3f) &lightSource)
{
    Vector3f outNormal;
    float angle;

    int locId = x + y * imgSize.x;
    Vector4f point = pointsRay[locId];

    bool foundPoint = point.w > 0.0f;

    computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

    // find label
    if (foundPoint)
    {
        Vector4u color;
        drawLabelColour<TVoxel,TIndex>(color, Vector3f(point), labelColors, voxelData, voxelIndex);
        drawShadedColor(outRendering[locId],angle, color);
    } else
        outRendering[locId] = Vector4u((uchar)0);
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelNormals_ImageNormals(DEVICEPTR(Vector4u) *outRendering, const CONSTPTR(Vector4f) *pointsRay,
	const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize, Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;
	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelNormal(outRendering[locId], outNormal);
	else outRendering[locId] = Vector4u((uchar)0);
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelConfidence_ImageNormals(DEVICEPTR(Vector4u) *outRendering, const CONSTPTR(Vector4f) *pointsRay,
	const THREADPTR(Vector2i) &imgSize, const THREADPTR(int) &x, const THREADPTR(int) &y, float voxelSize, Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;
	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelConfidence(outRendering[locId], angle, point.w - 1.0f);
	else outRendering[locId] = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelGrey(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point, 
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex, 
	Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelGrey(outRendering, angle);
	else outRendering = Vector4u(0,0,0,0);//outRendering = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processLabel(DEVICEPTR(ushort) &label, const CONSTPTR(Vector3f) &point,
                                const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex){
    label = VoxelLabelReader<TVoxel::hasLabelInformation, TVoxel, TIndex>::maxSampling(voxelData,voxelIndex,point);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processSemantic(DEVICEPTR(ushort) &label, const CONSTPTR(Vector3f) &point,
                                            const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex){
    label = VoxelSemanticLabelReader<TVoxel::hasSemanticInformation, TVoxel, TIndex>::closest(voxelData,voxelIndex,point);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processLabelNormal(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point, const CONSTPTR(Vector4f) *labelColor,
                                                bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                                Vector3f lightSource)
{
    Vector3f outNormal;
    float angle;

    computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

    // find label
    if (foundPoint)
    {
        Vector4u color;
        drawLabelColour<TVoxel,TIndex>(color, point, labelColor, voxelData, voxelIndex);
        drawShadedColor(outRendering,angle, color);
    } else
        outRendering = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelColour(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point,
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex)
{
//    outRendering = Vector4u((uchar)255);
//    return;;
	if (foundPoint) drawPixelColour<TVoxel, TIndex>(outRendering, point, voxelData, voxelIndex);
	else outRendering = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelNormal(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector3f) & point,
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
	Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);
	if(angle <= 0 || angle > 1){
        outRendering = Vector4u(0,0,0,0);
        return;
	}

	if (foundPoint) drawPixelNormal(outRendering, outNormal);
	else outRendering = Vector4u((uchar)0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelConfidence(DEVICEPTR(Vector4u) &outRendering, const CONSTPTR(Vector4f) & point, 
	bool foundPoint, const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex, 
	Vector3f lightSource)
{
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, TO_VECTOR3(point), voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelConfidence(outRendering, angle, point.w - 1.0f);
	else outRendering = Vector4u((uchar)0);
}
// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"
#include "../../../Objects/Scene/DistanceField_shared.h"


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_d,
                                                             const CONSTPTR(Vector4f) & projParams_d, float voxelSize, float mu, int maxW, const CONSTPTR(float) *depth, const CONSTPTR(Vector2i) & imgSize, SCFUSION::Policy::Integrate policy)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image
	pt_camera = M_d * pt_model;
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	// get measured depth from image
	depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];
	if (depth_measure <= 0.0f) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;

	// compute updated SDF value and reliability
	oldF = newF = TVoxel::valueToFloat(voxel.sdf); oldW = voxel.w_depth;
    newW = measureW;

    if(TVoxel::integrateType == SCFUSION::IntegrateType_TSDF) {
        if (eta < -mu) return eta;
        switch (policy) {
            case SCFUSION::Policy::Integrate_WEIGHTED:
                SCFUSION::sdf_integrade_kernel<SCFUSION::Policy::Integrate_WEIGHTED>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
            case SCFUSION::Policy::Integrate_DIRECT:
                SCFUSION::sdf_integrade_kernel<SCFUSION::Policy::Integrate_DIRECT>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
        }
    } else if (TVoxel::integrateType == SCFUSION::IntegrateType_OFusion) {
        switch (policy) {
            case SCFUSION::Policy::Integrate_WEIGHTED:
                SCFUSION::occupancy_integrade_kernel_weighted(oldF, oldW, eta, newW, maxW, mu);
                break;
            case SCFUSION::Policy::Integrate_DIRECT:
                SCFUSION::occupancy_integrade_kernel_direct(oldF, oldW, eta, newW, maxW, mu, voxelSize, pt_camera.z);
                break;
        }
        newF = oldF;
        newW = oldW;
    } else if (TVoxel::integrateType == SCFUSION::IntegrateType_Invert_TSDF) {
        switch (policy) {
            case SCFUSION::Policy::Integrate_WEIGHTED:
                SCFUSION::inverst_sdf_integrade_kernel<SCFUSION::Policy::Integrate_WEIGHTED>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
            case SCFUSION::Policy::Integrate_DIRECT:
                SCFUSION::inverst_sdf_integrade_kernel<SCFUSION::Policy::Integrate_DIRECT>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
        }
    }

//    // check update state
//    if(newF != voxel.sdf || voxel.w_depth != newW) {
////    if(ABS(newF - voxel.sdf) > 5) {
//        accessUpdateState<TVoxel>::write(voxel, true);
//    }

	// write back
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_d,
                                                             const CONSTPTR(Vector4f) & projParams_d, float voxelSize, float mu, int maxW, const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize, SCFUSION::Policy::Integrate policy)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW, locId;

	// project point into image
	pt_camera = M_d * pt_model;
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	locId = (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
	// get measured depth from image
	depth_measure = depth[locId];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;

	// compute updated SDF value and reliability
	oldF = newF = TVoxel::valueToFloat(voxel.sdf); oldW = voxel.w_depth;
    newW = measureW;
    if(TVoxel::integrateType == SCFUSION::IntegrateType_TSDF) {
        if (eta < -mu) return eta;
        switch (policy) {
            case SCFUSION::Policy::Integrate_WEIGHTED:
                SCFUSION::sdf_integrade_kernel<SCFUSION::Policy::Integrate_WEIGHTED>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
            case SCFUSION::Policy::Integrate_DIRECT:
                SCFUSION::sdf_integrade_kernel<SCFUSION::Policy::Integrate_DIRECT>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
        }
    } else if (TVoxel::integrateType == SCFUSION::IntegrateType_OFusion) {
        switch (policy) {
            case SCFUSION::Policy::Integrate_WEIGHTED:
                SCFUSION::occupancy_integrade_kernel_weighted(oldF, oldW, eta, newW, maxW, mu);
                break;
            case SCFUSION::Policy::Integrate_DIRECT:
                SCFUSION::occupancy_integrade_kernel_direct(oldF, oldW, eta, newW, maxW, mu, voxelSize, pt_camera.z);
                break;
        }
        newF = oldF;
        newW = oldW;
    }  else if (TVoxel::integrateType == SCFUSION::IntegrateType_Invert_TSDF) {
        switch (policy) {
            case SCFUSION::Policy::Integrate_WEIGHTED:
                SCFUSION::inverst_sdf_integrade_kernel<SCFUSION::Policy::Integrate_WEIGHTED>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
            case SCFUSION::Policy::Integrate_DIRECT:
                SCFUSION::inverst_sdf_integrade_kernel<SCFUSION::Policy::Integrate_DIRECT>(newF, newW, oldF, oldW, eta, mu, maxW);
                break;
        }
    }

//    if(newF != voxel.sdf || voxel.w_depth != newW) {
//        accessUpdateState<TVoxel>::write(voxel, true);
//    }

	// write back^
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;
	voxel.confidence += TVoxel::floatToValue(confidence[locId]);

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelLabelInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_rgb,
        const CONSTPTR(Vector4f) & projParams_d, float voxelSize, float mu, uchar maxW, float eta, const CONSTPTR(ushort) *label, const CONSTPTR(Vector2i) & imgSize)
{
    Vector4f pt_camera; Vector2f pt_image;
    pt_camera = M_rgb * pt_model;

    pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
    pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;

    if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

    // compute updated Label value and reliability
    ushort oldLabel, newLabel;
    int minW = 10; //TODO: make this variable
    uchar w_label_old = voxel.w_label;
    oldLabel = voxel.label;
    newLabel = label[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];

    if(newLabel == 0) return;

    if(oldLabel == 0 || w_label_old < minW) {
        // init label
        voxel.label = newLabel;
        voxel.w_label = minW;
    } else {
        if ( oldLabel == newLabel ){
            if(w_label_old < maxW)
                w_label_old = fminf(maxW, w_label_old+5);
        } else
        if(w_label_old > 0)
            w_label_old -= 5;
        voxel.w_label = w_label_old;
    }
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_rgb,
	const CONSTPTR(Vector4f) & projParams_rgb, float voxelSize, float mu, uchar maxW, float eta, const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC; Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float)voxel.w_color;

	oldC = TO_FLOAT3(buffV3u) / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = measureW;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = MIN(newW, maxW);

	voxel.clr = TO_UCHAR3(newC * 255.0f);
	voxel.w_color = (uchar)newW;
}

template<bool hasColor, bool hasConfidence, bool hasLabel, class TVoxel> struct ComputeUpdatedVoxelInfo;

/// hasColor, hasConfidence, hasLabel
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
		const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
        float voxelSize, float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(ushort) *label, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb, const SCFUSION::Policy::Integrate &policy)
	{
        computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, depth, imgSize_d, policy);
	}
};

/// hasColor, hasConfidence, hasLabel
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
		const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
		float voxelSize, float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(ushort) *label, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const THREADPTR(Vector2i) & imgSize_rgb, const SCFUSION::Policy::Integrate &policy)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, depth, imgSize_d, policy);
		if ((eta > mu) || (fabs(eta / mu) > 0.5)) return;
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, voxelSize, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
/// hasColor, hasConfidence, hasLabel
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
		const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
        float voxelSize, float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(ushort) *label, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb, const SCFUSION::Policy::Integrate &policy)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, depth, confidence, imgSize_d, policy);
	}
};
/// hasColor, hasConfidence, hasLabel
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
		const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
		const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
        float voxelSize, float mu, int maxW,
		const CONSTPTR(float) *depth, const CONSTPTR(ushort) *label, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
		const CONSTPTR(Vector4u) *rgb, const THREADPTR(Vector2i) & imgSize_rgb, const SCFUSION::Policy::Integrate &policy)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, depth, confidence, imgSize_d, policy);
		if ((eta > mu) || (fabs(eta / mu) > 0.5)) return;
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, voxelSize, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
/// hasColor, hasConfidence, hasLabel
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, true, TVoxel>{
    _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                                           const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
                                           const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
                                           float voxelSize, float mu, int maxW,
                                           const CONSTPTR(float) *depth, const CONSTPTR(ushort) *label, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
                                           const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb, const SCFUSION::Policy::Integrate &policy)
    {
        float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, depth, imgSize_d, policy);
        if ((eta > mu) || (fabs(eta / mu) > 0.5)) return;
        computeUpdatedVoxelLabelInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, eta, label, imgSize_d);
    }
};
/// hasColor, hasConfidence, hasLabel
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, true, TVoxel>{
    _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                                           const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
                                           const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
                                           float voxelSize, float mu, int maxW,
                                           const CONSTPTR(float) *depth, const CONSTPTR(ushort) *label, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,
                                           const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb, const SCFUSION::Policy::Integrate &policy)
    {
        float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, depth, imgSize_d, policy);
        if ((eta > mu) || (fabs(eta / mu) > 0.5)) return;
        computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, voxelSize, mu, maxW, eta, rgb, imgSize_rgb);
        computeUpdatedVoxelLabelInfo(voxel, pt_model, M_d, projParams_d, voxelSize, mu, maxW, eta, label, imgSize_d);
    }
};

_CPU_AND_GPU_CODE_ inline int checkHashExistPoint(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType,
                                                   DEVICEPTR(Vector4s) *blockCoords, const CONSTPTR(ITMHashEntry) *hashTable,
                                                   const Vector3f &point){
    Vector3s blockPos = TO_SHORT_FLOOR3(point);

    //compute index in hash table
    unsigned int hashIdx = hashIndex(blockPos);

    //check if hash table contains entry
    bool isFound = false;

    ITMHashEntry hashEntry = hashTable[hashIdx];

    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
    {
        //entry has been streamed out but is visible or in memory and visible
        entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

        isFound = true;
    }

    if (!isFound)
    {
        bool isExcess = false;
        if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
        {
            while (hashEntry.offset >= 1)
            {
                hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
                hashEntry = hashTable[hashIdx];

                if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
                {
                    //entry has been streamed out but is visible or in memory and visible
                    entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

                    isFound = true;
                    break;
                }
            }

            isExcess = true;
        }

        if (!isFound) //still not found
        {
            entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation
            if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible

            blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1);
        }
    }
    return isFound;
}

_CPU_AND_GPU_CODE_ inline int buildHashAllocAndVisibleTypePP(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, int x, int y,
	DEVICEPTR(Vector4s) *blockCoords, const CONSTPTR(float) *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize,
	float oneOverVoxelSize, const CONSTPTR(ITMHashEntry) *hashTable, float viewFrustum_min, float viewFrustum_max, bool useInverseSensorModel)
{
	float depth_measure; int noSteps;
	Vector4f pt_camera_f; Vector3f point_e, point, direction;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) {
	    return -1;
	}

	pt_camera_f.z = depth_measure;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	Vector4f pt_buff;

	if(!useInverseSensorModel) {
        pt_buff = pt_camera_f * (1.0f - mu / norm);
        pt_buff.w = 1.0f;
    } else {
        pt_buff = Vector4f(pt_camera_f.x, pt_camera_f.y, viewFrustum_min, 1.f); // start from minimum view Frustum
    }
	point = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	pt_buff = pt_camera_f * (1.0f + mu / norm); pt_buff.w = 1.0f;
	point_e = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	direction = point_e - point;
	norm = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	noSteps = (int)ceil(2.0f*norm);

	direction /= (float)(noSteps - 1);

	//add neighbouring blocks
	int blockFound=0;
	for (int i = 0; i < noSteps; i++)
	{
        blockFound += checkHashExistPoint(entriesAllocType,entriesVisibleType,blockCoords,hashTable, point);
		point += direction;
	}
	return blockFound;
}


template<bool hasLabelInformation, bool hasSemanticInformation, class TVoxel> struct IntegrateLabels;
template<class TVoxel> struct IntegrateLabels<false,false,TVoxel> {
    _CPU_AND_GPU_CODE_ inline static void compute(TVoxel &voxel,const unsigned short &label, const unsigned short &instance){}
};
template<class TVoxel> struct IntegrateLabels<true,false,TVoxel> {
    _CPU_AND_GPU_CODE_ inline static void compute(TVoxel &voxel,const unsigned short &label, const unsigned short &instance){
        voxel.label = label;
        voxel.w_label = 1;
    }
};
template<class TVoxel> struct IntegrateLabels<false,true,TVoxel> {
    _CPU_AND_GPU_CODE_ inline static void compute(TVoxel &voxel,const unsigned short &label, const unsigned short &instance){
        voxel.semantic = instance;
    }
};
template<class TVoxel> struct IntegrateLabels<true,true,TVoxel> {
    _CPU_AND_GPU_CODE_ inline static void compute(TVoxel &voxel,const unsigned short &label, const unsigned short &instance){
        voxel.label = label;
        voxel.w_label = 1;
        voxel.semantic = instance;
    }
};
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void integrateIntoScene_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                       float oneOverVoxelSize, const Vector3f *points, const unsigned short *labels,
                                       const unsigned short *instances){
    Vector3f pt = points[i] * oneOverVoxelSize;
    typename TIndex::IndexCache cache;
    int vmIndex;
    int idx_to = findVoxel(voxelIndex,Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z)),vmIndex, cache);
    if( !vmIndex  ) return;
    TVoxel &voxel_to = voxelData[idx_to];
//    atomicMax(&voxel_to.sdf, LOGODD_OCU);
    voxel_to.sdf = 1.2;
    voxel_to.w_depth = 1;
    unsigned short label = labels? labels[i]:0;
    unsigned short instance = instances? instances[i]:0;
    IntegrateLabels<TVoxel::hasLabelInformation,TVoxel::hasSemanticInformation,TVoxel>::compute(voxel_to, label,instance);
}

/// -------------SC-------------
_CPU_AND_GPU_CODE_ inline void buildHasAllocAndVisibleTypeSC(int x, int y, const float *data, Vector3f volume_start, float oneOverVoxelSize, Vector3s blockDims,
                                                             const CONSTPTR(ITMHashEntry) *hashTable, DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, DEVICEPTR(Vector4s) *blockCoords)
{
    for (int z=0; z< blockDims.z; ++z) {
        if(data[(z*blockDims.y+y)*blockDims.x+x] <= 0) continue;
        Vector3f point = (volume_start * oneOverVoxelSize + Vector3f(x, y, z)) / float(SDF_BLOCK_SIZE);
        checkHashExistPoint(entriesAllocType,entriesVisibleType,blockCoords,hashTable, point);
    }
}
_CPU_AND_GPU_CODE_ inline void buildHashAllocAndVisibleTypeFromPoints(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, uint p,
                                                              DEVICEPTR(Vector4s) *blockCoords, const CONSTPTR(Vector3f) *points,
                                                              float oneOverVoxelSize, const CONSTPTR(ITMHashEntry) *hashTable)
{

    for(int x=-1;x<2;++x)
        for(int y=-1;y<2;++y)
            for(int z=-1;z<2;++z) {
                Vector3f point = points[p] * oneOverVoxelSize;
                checkHashExistPoint(entriesAllocType, entriesVisibleType, blockCoords, hashTable,
                                    point + Vector3f(x, y, z));
            }
}
/// -------------

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkPointVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector4f) &pt_image, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_buff;

	pt_buff = M_d * pt_image;

	if (pt_buff.z < 1e-10f) return;

	pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
	pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;

	if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && pt_buff.y >= 0 && pt_buff.y < imgSize.y) { isVisible = true; isVisibleEnlarged = true; }
	else if (useSwapping)
	{
		Vector4i lims;
		lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
		lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

		if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w) isVisibleEnlarged = true;
	}
}

/// 投影該block的八個角，看是否在當前的視野內，若有一個在視野內則回傳true
template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkBlockVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const THREADPTR(Vector3s) &hashPos, const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(float) &voxelSize, const CONSTPTR(Vector2i) &imgSize)
{
	Vector4f pt_image;
	float factor = (float)SDF_BLOCK_SIZE * voxelSize;

	isVisible = false; isVisibleEnlarged = false;

	// 0 0 0
	pt_image.x = (float)hashPos.x * factor; pt_image.y = (float)hashPos.y * factor;
	pt_image.z = (float)hashPos.z * factor; pt_image.w = 1.0f;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 0 1
	pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 1
	pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 1
	pt_image.x += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 1 0 
	pt_image.z -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 0 
	pt_image.y -= factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 0 1 0
	pt_image.x -= factor; pt_image.y += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;

	// 1 0 1
	pt_image.x += factor; pt_image.y -= factor; pt_image.z += factor;
	checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
	if (isVisible) return;
}

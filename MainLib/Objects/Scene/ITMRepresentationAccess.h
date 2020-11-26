// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMVoxelBlockHash.h"
#include "ITMVoxelTypes.h"
#include "SCPlainVoxelArray.h"

template<typename T> _CPU_AND_GPU_CODE_ inline int hashIndex(const THREADPTR(T) & blockPos) {
	return (((uint)blockPos.x * 73856093u) ^ ((uint)blockPos.y * 19349669u) ^ ((uint)blockPos.z * 83492791u)) & (uint)SDF_HASH_MASK;
}

template<class TVoxel, bool hasUpdateInformation> struct _accessUpdateState;
template<class TVoxel> struct _accessUpdateState<TVoxel,true> {
    _CPU_AND_GPU_CODE_ static inline bool read(const TVoxel &v){
        return v.updated;
    }
    _CPU_AND_GPU_CODE_ static inline void write(TVoxel &v, bool state){
        v.updated = state;
    }
};
//template<class TVoxel> struct _accessUpdateState<TVoxel,false> {
//    _CPU_AND_GPU_CODE_ static inline bool read(const TVoxel &v){
//        return false;
//    }
//    _CPU_AND_GPU_CODE_ static inline void write(TVoxel &v, bool state){}
//};
//template<class TVoxel> struct accessUpdateState{
//    _CPU_AND_GPU_CODE_ static inline bool read(const TVoxel &v){
//        return _accessUpdateState<TVoxel, TVoxel::hasUpdateInformation>::read(v);
//    }
//    _CPU_AND_GPU_CODE_ static inline void write(TVoxel &v, bool state){
//        _accessUpdateState<TVoxel, TVoxel::hasUpdateInformation>::write(v, state);
//    }
//};
template<class TVoxel, bool hasTimeInformation> struct _accessTimeStamp;
template<class TVoxel> struct _accessTimeStamp<TVoxel,true> {
    _CPU_AND_GPU_CODE_ static inline float read(const TVoxel &v){
        return v.time;
    }
    _CPU_AND_GPU_CODE_ static inline void write(TVoxel &v, float state){
        v.time = state;
    }
};
template<class TVoxel> struct _accessTimeStamp<TVoxel,false> {
    _CPU_AND_GPU_CODE_ static inline float read(const TVoxel &v){
        return 0.f;
    }
    _CPU_AND_GPU_CODE_ static inline void write(TVoxel &v, float state){
    }
};
template<class TVoxel> struct accessTimeStamp{
    _CPU_AND_GPU_CODE_ static inline float read(const TVoxel &v){
        return _accessTimeStamp<TVoxel, TVoxel::hasTimeInformation>::read(v);
    }
    _CPU_AND_GPU_CODE_ static inline void write(TVoxel &v, float state){
        _accessTimeStamp<TVoxel, TVoxel::hasTimeInformation>::write(v, state);
    }
};


template<class TVoxel, bool hasLabelInformation> struct _accessLabel;
template<class TVoxel>
struct _accessLabel<TVoxel, true> {
    _CPU_AND_GPU_CODE_ static inline ushort readLabel(const TVoxel &v){
        return v.label;
    }
    _CPU_AND_GPU_CODE_ static inline uchar readWeight(const TVoxel &v){
        return v.w_label;
    }
    _CPU_AND_GPU_CODE_ static inline void writeLabel(TVoxel &v, ushort label){
        v.label = label;
    }
    _CPU_AND_GPU_CODE_ static inline void writeLabelWeight(TVoxel &v, uchar weight){
        v.w_label = weight;
    }

    _CPU_AND_GPU_CODE_ static inline ushort readMaxWNeighborLabel(const Vector3i &pos, const TVoxel *localVBA, const ITMHashEntry *hashTable, int range){
        int vmIndex;
        ushort label=0;
        uchar w_label=0;
        TVoxel voxel_ref = readVoxel(localVBA, hashTable, pos, vmIndex);
        if(vmIndex) {
            label = voxel_ref.label;
            w_label = voxel_ref.w_label;
        }
        size_t last_distance=0;
        for (int x=-range; x<=range; x++) {
            if(x==0)continue;
            for (int y=-range;y<=range;++y) {
                if(y==0)continue;
                for (int z=-range;z<=range;++z) {
                    if(z==0)continue;

                    TVoxel voxel = readVoxel(localVBA, hashTable, pos+Vector3i(x,y,z), vmIndex);
                    if (!vmIndex) continue;

                    if(voxel.label>0) {
                        if (label == 0) {
                            label = voxel.label;
                            continue;
                        }
                        size_t distance = x*x+y*y+z*z;
                        if (last_distance > distance ) {
                            last_distance = distance;
                            label = voxel.label;
                            w_label = voxel.w_label;
                        } else if (last_distance == distance && voxel.w_label > w_label) {
                            label = voxel.label;
                            w_label = voxel.w_label;
                        }
                    }
                }
            }
        }
        return label;
    }
};
template<class TVoxel>
struct _accessLabel<TVoxel, false> {
    _CPU_AND_GPU_CODE_ static inline ushort readLabel(const TVoxel &v){
        return 0;
    }
    _CPU_AND_GPU_CODE_ static inline uchar readWeight(const TVoxel &v){
        return 0;
    }
    _CPU_AND_GPU_CODE_ static inline uchar writeLabel(TVoxel &v, ushort label){return 0;}
    _CPU_AND_GPU_CODE_ static inline uchar writeLabelWeight(TVoxel &v, uchar weight){return 0;}
    _CPU_AND_GPU_CODE_ static inline ushort readMaxWNeighborLabel(const Vector3i &pos, const TVoxel *localVBA, const ITMHashEntry *hashTable, size_t range){
        return 0;
    }
};

template<class TVoxel> struct accessLabel{
    _CPU_AND_GPU_CODE_ static inline ushort readLabel(const TVoxel &v){
        return _accessLabel<TVoxel, TVoxel::hasLabelInformation>::readLabel(v);
    }
    _CPU_AND_GPU_CODE_ static inline uchar readWeight(const TVoxel &v){
        return _accessLabel<TVoxel, TVoxel::hasLabelInformation>::readWeight(v);
    }
    _CPU_AND_GPU_CODE_ static inline void writeLabel(TVoxel &v, ushort label){
        _accessLabel<TVoxel, TVoxel::hasLabelInformation>::writeLabel(v,label);
    }
    _CPU_AND_GPU_CODE_ static inline void writeLabelWeight(TVoxel &v, uchar weight){
        _accessLabel<TVoxel, TVoxel::hasLabelInformation>::writeLabelWeight(v,weight);
    }
    _CPU_AND_GPU_CODE_ static inline ushort readMaxWNeighborLabel(const Vector3i &pos, const TVoxel *localVBA, const ITMHashEntry *hashTable, size_t range){
        return _accessLabel<TVoxel, TVoxel::hasLabelInformation>::readMaxWNeighborLabel(pos,localVBA,hashTable,range);
    }
};

_CPU_AND_GPU_CODE_ inline float readMaxWNeighborLabel(Vector3i xyz, const float *localVBA, const Vector3s &dims, size_t range){
    float label = localVBA[(xyz.z * dims.y + xyz.y) * dims.x + xyz.x];
    for (int x=-range; x<=(int)range; x++) {
        int x_ = xyz.x + x;
        if(x_<0 || x_ >= dims.x) continue;
        for (int y=-range;y<=(int)range;++y) {
            int y_ = xyz.y + y;
            if(y_<0 || y_ >= dims.y) continue;
            for (int z=range;z<=(int)range;++z) {
                int z_ = xyz.z + z;
                if(z_<-1 || z_ >= dims.z) continue;
                uint idx_off = (z_ * dims.y + y_)*dims.x + x_;
                if(localVBA[idx_off] > label) label = localVBA[idx_off];
            }
        }
    }
    return label;
}

//template<SCFusion::IntegrateType T> _CPU_AND_GPU_CODE_ inline bool checkVoxelState(const float &value);
//template<> _CPU_AND_GPU_CODE_ inline bool checkVoxelState <SCFusion::IntegrateType::IntegrateType_OFusion>(const float &value){
//    return value != 0.f;
//}
//template<> _CPU_AND_GPU_CODE_ inline bool checkVoxelState <SCFusion::IntegrateType::IntegrateType_Invert_TSDF>(const float &value){
//    return value != 0.f;
//}
//template<> _CPU_AND_GPU_CODE_ inline bool checkVoxelState <SCFusion::IntegrateType::IntegrateType_TSDF>(const float &value){
//    return value != 1.f;
////    return 1;
//}

template<class TVoxel,SCFUSION::IntegrateType T> struct checkVoxelStateChecker;
template<class TVoxel> struct checkVoxelStateChecker<TVoxel, SCFUSION::IntegrateType::IntegrateType_OFusion>{
    _CPU_AND_GPU_CODE_ inline static bool compute(const TVoxel &voxel){
        return !(voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue());
//        return TVoxel::valueToFloat(voxel.sdf) != 0.f;
    }
};
template<class TVoxel> struct checkVoxelStateChecker<TVoxel, SCFUSION::IntegrateType::IntegrateType_Invert_TSDF>{
    _CPU_AND_GPU_CODE_ inline static bool compute(const TVoxel &voxel){
        return !(voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue());
//        return TVoxel::valueToFloat(voxel.sdf) != 0.f;
    }
};
template<class TVoxel> struct checkVoxelStateChecker<TVoxel, SCFUSION::IntegrateType::IntegrateType_TSDF>{
    _CPU_AND_GPU_CODE_ inline static bool compute(const TVoxel &voxel){
        return !(voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue());
//        return TVoxel::valueToFloat(voxel.sdf) != 1.f;
    }
};

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline bool checkVoxelState(const TVoxel &voxel) {
    return checkVoxelStateChecker<TVoxel, TVoxel::integrateType>::compute(voxel);
}


_CPU_AND_GPU_CODE_ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) & point, THREADPTR(Vector3i) &blockPos) {
	blockPos.x = ((point.x < 0) ? point.x - SDF_BLOCK_SIZE + 1 : point.x) / SDF_BLOCK_SIZE;
	blockPos.y = ((point.y < 0) ? point.y - SDF_BLOCK_SIZE + 1 : point.y) / SDF_BLOCK_SIZE;
	blockPos.z = ((point.z < 0) ? point.z - SDF_BLOCK_SIZE + 1 : point.z) / SDF_BLOCK_SIZE;

	//Vector3i locPos = point - blockPos * SDF_BLOCK_SIZE;
	//return locPos.x + locPos.y * SDF_BLOCK_SIZE + locPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	return point.x + (point.y - blockPos.x) * SDF_BLOCK_SIZE + (point.z - blockPos.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE - blockPos.z * SDF_BLOCK_SIZE3;
}

_CPU_AND_GPU_CODE_ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) & point, THREADPTR(Vector3i) &blockPos, THREADPTR(Vector3s) BlockDims) {
    blockPos.x = ((point.x < 0) ? point.x - BlockDims.x + 1 : point.x) / BlockDims.x;
    blockPos.y = ((point.y < 0) ? point.y - BlockDims.y + 1 : point.y) / BlockDims.y;
    blockPos.z = ((point.z < 0) ? point.z - BlockDims.z + 1 : point.z) / BlockDims.z;

    //Vector3i locPos = point - blockPos * SDF_BLOCK_SIZE;
    //return locPos.x + locPos.y * SDF_BLOCK_SIZE + locPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
    return point.x + (point.y - blockPos.x) * SDF_BLOCK_SIZE + (point.z - blockPos.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE - blockPos.z * SDF_BLOCK_SIZE3;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point,
	THREADPTR(int) &vmIndex, THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache)
{
	Vector3i blockPos;
	short linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		vmIndex = true;
		return cache.blockPtr + linearIdx;
	}

	int hashIdx = hashIndex(blockPos);

	while (true)
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			vmIndex = true;
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			return cache.blockPtr + linearIdx;
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	vmIndex = false;
	return -1;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex, Vector3i point, THREADPTR(int) &vmIndex)
{
	ITMLib::ITMVoxelBlockHash::IndexCache cache;
	return findVoxel(voxelIndex, point, vmIndex, cache);
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex, Vector3i point, THREADPTR(bool) &foundPoint)
{
	int vmIndex;
	ITMLib::ITMVoxelBlockHash::IndexCache cache;
	int result = findVoxel(voxelIndex, point, vmIndex, cache);
	foundPoint = vmIndex != 0;
	return result;
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(SCFUSION::SCPlainVoxelArray::IndexData) *arrayInfo, const THREADPTR(Vector3i) &point, THREADPTR(int) &vmIndex)
{
    if(point.z >= arrayInfo->size.z || point.y >=arrayInfo->size.y || point.x >= arrayInfo->size.x || point.z<0 || point.y<0 || point.x < 0 ){
        vmIndex = false;
        return TVoxel();
    }
    int linearIdx = (point.z * arrayInfo->size.y + point.y) * arrayInfo->size.x + point.x;
    vmIndex = true;
    return voxelData[linearIdx];
}


_CPU_AND_GPU_CODE_ inline float readVoxel(const CONSTPTR(float) *voxelData, const CONSTPTR(Vector3s) &dims, const THREADPTR(Vector3i) &point, THREADPTR(int) &vmIndex)
{
    if(point.z >= dims.z || point.y >=dims.y || point.x >= dims.x || point.z<0 || point.y<0 || point.x < 0 ){
        vmIndex = false;
        return -1.f;
    }
    int linearIdx = (point.z * dims.y + point.y) * dims.x + point.x;
    vmIndex = true;
    return voxelData[linearIdx];
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point, THREADPTR(int) &vmIndex, THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache)
{
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		vmIndex = true;
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = hashIndex(blockPos);

	while (true)
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound

			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	vmIndex = false;
	return TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex,
	Vector3i point, THREADPTR(int) &vmIndex)
{
	ITMLib::ITMVoxelBlockHash::IndexCache cache;
	return readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) *voxelIndex,
	Vector3i point, THREADPTR(bool) &foundPoint)
{
	int vmIndex;
	ITMLib::ITMVoxelBlockHash::IndexCache cache;
	TVoxel result = readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
	foundPoint = vmIndex != 0;
	return result;
}



template <class TVoxel>
_CPU_AND_GPU_CODE_ inline float ofu_2_tsdf (TVoxel voxel) {
    if (TVoxel::integrateType == SCFUSION::IntegrateType_OFusion) {
        if(voxel.w_depth > 0)
            return - ((1.f - 1.f / (1.f + exp(TVoxel::valueToFloat(voxel.sdf)))) * 2 - 1);
        else
            return 1;
    } else
        return TVoxel::valueToFloat(voxel.sdf);
}


template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTPTR(TVoxel) *voxelData,
     const CONSTPTR(TIndex) *voxelIndex, Vector3f point, ushort &label, THREADPTR(int) &vmIndex)
{
    TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex);
    label = accessLabel<TVoxel>::readLabel(res);
    return TVoxel::valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTPTR(TVoxel) *voxelData,
     const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ushort &label, THREADPTR(TCache) & cache)
{
    TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex, cache);
    label = accessLabel<TVoxel>::readLabel(res);
    return TVoxel::valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated_OFU(const CONSTPTR(TVoxel) *voxelData,
                                                                 const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ushort &label, THREADPTR(TCache) & cache)
{
    TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex, cache);
    label = accessLabel<TVoxel>::readLabel(res);
    return ofu_2_tsdf<TVoxel>(res);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated_OFU(const CONSTPTR(TVoxel) *voxelData,
       const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
    float res1, res2, v1, v2;
    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

    v1 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache));
    v2 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache));
    res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

    v1 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache));
    v2 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache));
    res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

    v1 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache));
    v2 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache));
    res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

    v1 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache));
    v2 = ofu_2_tsdf<TVoxel>(readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache));
    res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

    vmIndex = true;
    return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}
template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated_OFU(const CONSTPTR(TVoxel) *voxelData,
                                                               const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ushort &label, THREADPTR(TCache) & cache)
{
    float res1, res2, v1, v2;
    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
    accessLabel<TVoxel> aclabel;
    TVoxel vo1, vo2;
    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(vo1);
    v2 = ofu_2_tsdf<TVoxel>(vo2);
    res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);


    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(vo1);
    v2 = ofu_2_tsdf<TVoxel>(vo2);
    res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);

    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(vo1);
    v2 = ofu_2_tsdf<TVoxel>(vo2);
    res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);

    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(vo1);
    v2 = ofu_2_tsdf<TVoxel>(vo2);
    res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);

    vmIndex = true;
    return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
	float res1, res2, v1, v2;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache).sdf;
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache).sdf;
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache).sdf;
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache).sdf;
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	vmIndex = true;
	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
   const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ushort &label, THREADPTR(TCache) & cache)
{
    float res1, res2, v1, v2;
    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
    accessLabel<TVoxel> aclabel;
    TVoxel vo1, vo2;
    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    v1 = vo1.sdf;
    v2 = vo2.sdf;
    res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);
    
    
    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    v1 = vo1.sdf;
    v2 = vo2.sdf;
    res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);
    
    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    v1 = vo1.sdf;
    v2 = vo2.sdf;
    res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);
    
    vo1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    vo2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    v1 = vo1.sdf;
    v2 = vo2.sdf;
    res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    if (aclabel.readWeight(vo1), aclabel.readWeight(vo2)) label = aclabel.readLabel(vo1);
    else label = aclabel.readLabel(vo2);
    
    vmIndex = true;
    return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_uninterpolated(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
                                                                             const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache)
{
    TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex, cache);
    confidence = res.w_depth;
    vmIndex = true;
    return TVoxel::valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_uninterpolated(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
                                                                             const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ushort &label, THREADPTR(TCache) & cache)
{
    TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), vmIndex, cache);
    confidence = res.w_depth;
    vmIndex = true;
    label = accessLabel<TVoxel>::readLabel(res);
    return TVoxel::valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated_OFU(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
                                                                             const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache, float maxValue = 1.f)
{
    float res1, res2, v1, v2;
    float res1_c, res2_c, v1_c, v2_c;
    TVoxel voxel;

    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v1_c = voxel.w_depth;
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v2_c = voxel.w_depth;
    res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    res1_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v1_c = voxel.w_depth;
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v2_c = voxel.w_depth;
    res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    res1_c = (1.0f - coeff.y) * res1_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v1_c = voxel.w_depth;
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v2_c = voxel.w_depth;
    res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    res2_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v1_c = voxel.w_depth;
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxValue; v2_c = voxel.w_depth;
    res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    res2_c = (1.0f - coeff.y) * res2_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

    vmIndex = true;

    confidence = (1.0f - coeff.z) * res1_c + coeff.z * res2_c;

    return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated_OFU(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
                                                                             const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ushort &label, THREADPTR(TCache) & cache, float maxvalue= 1.f)
{
    ushort label1=0, label2=0;
    int w_label1 = 0, w_label2 = 0;
    float res1, res2, v1, v2;
    float res1_c, res2_c, v1_c, v2_c;
    TVoxel voxel;

    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
    accessLabel<TVoxel> aclabel;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v1_c = voxel.w_depth; label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v2_c = voxel.w_depth; label2 = aclabel.readLabel(voxel); w_label2 = aclabel.readWeight(voxel);
    res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    res1_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;
    if(w_label1 > w_label2) label = label1;
    else label = label2;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v1_c = voxel.w_depth; label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v2_c = voxel.w_depth; label2 = aclabel.readLabel(voxel); w_label2 = aclabel.readWeight(voxel);
    res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    res1_c = (1.0f - coeff.y) * res1_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);
    if(w_label1 > w_label2) label = label1;
    else label = label2;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v1_c = voxel.w_depth; label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v2_c = voxel.w_depth; label2 = aclabel.readLabel(voxel); w_label2 = aclabel.readWeight(voxel);
    res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    res2_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;
    if(w_label1 > w_label2) label = label1;
    else label = label2;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    v1 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v1_c = voxel.w_depth; label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    v2 = ofu_2_tsdf<TVoxel>(voxel) / maxvalue; v2_c = voxel.w_depth; label2 = aclabel.readLabel(voxel); w_label2 = aclabel.readWeight(voxel);
    res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    res2_c = (1.0f - coeff.y) * res2_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);
    if(w_label1 > w_label2) label = label1;
    else label = label2;

    vmIndex = true;

    confidence = (1.0f - coeff.z) * res1_c + coeff.z * res2_c;

    return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated(THREADPTR(float) &confidence, const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache, float maxValue = 1.f)
{
	float res1, res2, v1, v2;
	float res1_c, res2_c, v1_c, v2_c;
	TVoxel voxel;

	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache); v1 = voxel.sdf / maxValue; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache); v2 = voxel.sdf / maxValue; v2_c = voxel.w_depth;
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
	res1_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache); v1 = voxel.sdf / maxValue; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache); v2 = voxel.sdf / maxValue; v2_c = voxel.w_depth;
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	res1_c = (1.0f - coeff.y) * res1_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache); v1 = voxel.sdf / maxValue; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache); v2 = voxel.sdf / maxValue; v2_c = voxel.w_depth;
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
	res2_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache); v1 = voxel.sdf / maxValue; v1_c = voxel.w_depth;
	voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache); v2 = voxel.sdf / maxValue; v2_c = voxel.w_depth;
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
	res2_c = (1.0f - coeff.y) * res2_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

	vmIndex = true;

	confidence = (1.0f - coeff.z) * res1_c + coeff.z * res2_c;

	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

_CPU_AND_GPU_CODE_ inline void compareLabel(THREADPTR(ushort) &label, THREADPTR(int) &weight, ushort newLabel, int newWeight){
    if(newLabel == 0) return;

//    if(label == 0 && newLabel > 0) {
//        label = newLabel;
//        weight = newWeight;
//    }
    if(newWeight>weight) {
        label=newLabel;
        weight=newWeight;
    }
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache, int & maxW)
{
	float res1, res2, v1, v2;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
		v1 = v.sdf;
		maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	vmIndex = true;
	return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
       const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache, ushort& label, int & maxW)
{
    float res1, res2, v1, v2;
    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
    accessLabel<TVoxel> aclabel;
    int maxW_label=0;
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
        v1 = v.sdf;
        maxW = v.w_depth;
        maxW_label = aclabel.readWeight(v);
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
        v2 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
        v1 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
        v2 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
        v1 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
        v2 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
    
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
        v1 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }
    {
        const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
        v2 = v.sdf;
        if (v.w_depth > maxW) maxW = v.w_depth;
        if (aclabel.readWeight(v) > maxW_label) {
            maxW_label = aclabel.readWeight(v);
            label = aclabel.readLabel(v);
        }
    }

    res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
    
    vmIndex = true;
    return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}


template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline ushort readFromSDF_semantic_closest(const CONSTPTR(TVoxel) *voxelData,
   const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache)
{
    TVoxel voxel; int vmIndex;
    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
    ushort semantic=0;
    int weight=0;

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, -1, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, -1, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}

    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, -1, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    if (voxel.semantic > 0 && voxel.w_depth > weight) {semantic = voxel.semantic; weight = voxel.w_depth;}

    return semantic ;
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline ushort readFromSDF_label_nearestneighbor(const CONSTPTR(TVoxel) *voxelData,
    const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache)
{
    TVoxel voxel; int vmIndex;
    ushort label,label1;
    int w_label,w_label1;
    accessLabel<TVoxel> aclabel;
    Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
#if 1 // 8 voxels
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0,0,0), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1,0,0), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0,1,0), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1,1,0), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0,0,1), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1,0,1), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0,1,1), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1,1,1), vmIndex, cache);
    /*if(vmIndex)*/{label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);}
#else // 27 voxels
//    if(label1 == 0)
    {
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, -1, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);

        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, -1, 0), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);

        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, -1, 1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
        voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), vmIndex, cache);
        label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    label1 = aclabel.readLabel(voxel); w_label1 = aclabel.readWeight(voxel); compareLabel(label,w_label,label1,w_label1);
    }
#endif



    return label;
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache)
{
	TVoxel resn; Vector3f ret(0.0f); Vector4f ret4; int vmIndex;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
//	if(resn.clr.r != 0 && resn.clr.g != 0 && resn.clr.b != 0)
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
	ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	ret4.x = ret.x; ret4.y = ret.y; ret4.z = ret.z; ret4.w = 255.0f;

	return ret4 / 255.0f;
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache, int & maxW)
{
	TVoxel resn; Vector3f ret(0.0f); Vector4f ret4; int vmIndex;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
	maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
	if (resn.w_depth > maxW) maxW = resn.w_depth;
	ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	ret4.x = ret.x; ret4.y = ret.y; ret4.z = ret.z; ret4.w = 255.0f;

	return ret4 / 255.0f;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline Vector3f computeSingleNormalFromSDF(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(TIndex) *voxelIndex, const THREADPTR(Vector3f) &point)
{
	int vmIndex;

	Vector3f ret;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
	Vector3f ncoeff(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);

	// all 8 values are going to be reused several times
	Vector4f front, back;
	front.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex).sdf;
	front.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex).sdf;
	front.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex).sdf;
	front.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex).sdf;
	back.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex).sdf;
	back.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex).sdf;
	back.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex).sdf;
	back.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex).sdf;

	Vector4f tmp;
	float p1, p2, v1;
	// gradient x
	p1 = front.x * ncoeff.y * ncoeff.z +
		front.z *  coeff.y * ncoeff.z +
		back.x  * ncoeff.y *  coeff.z +
		back.z  *  coeff.y *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), vmIndex).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), vmIndex).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), vmIndex).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), vmIndex).sdf;
	p2 = tmp.x * ncoeff.y * ncoeff.z +
		tmp.y *  coeff.y * ncoeff.z +
		tmp.z * ncoeff.y *  coeff.z +
		tmp.w *  coeff.y *  coeff.z;
	v1 = p1 * coeff.x + p2 * ncoeff.x;

	p1 = front.y * ncoeff.y * ncoeff.z +
		front.w *  coeff.y * ncoeff.z +
		back.y  * ncoeff.y *  coeff.z +
		back.w  *  coeff.y *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 0), vmIndex).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 0), vmIndex).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 1), vmIndex).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 1), vmIndex).sdf;
	p2 = tmp.x * ncoeff.y * ncoeff.z +
		tmp.y *  coeff.y * ncoeff.z +
		tmp.z * ncoeff.y *  coeff.z +
		tmp.w *  coeff.y *  coeff.z;

	ret.x = TVoxel::valueToFloat(p1 * ncoeff.x + p2 * coeff.x - v1);

	// gradient y
	p1 = front.x * ncoeff.x * ncoeff.z +
		front.y *  coeff.x * ncoeff.z +
		back.x  * ncoeff.x *  coeff.z +
		back.y  *  coeff.x *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), vmIndex).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), vmIndex).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), vmIndex).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), vmIndex).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.z +
		tmp.y *  coeff.x * ncoeff.z +
		tmp.z * ncoeff.x *  coeff.z +
		tmp.w *  coeff.x *  coeff.z;
	v1 = p1 * coeff.y + p2 * ncoeff.y;

	p1 = front.z * ncoeff.x * ncoeff.z +
		front.w *  coeff.x * ncoeff.z +
		back.z  * ncoeff.x *  coeff.z +
		back.w  *  coeff.x *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 0), vmIndex).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 0), vmIndex).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 1), vmIndex).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 1), vmIndex).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.z +
		tmp.y *  coeff.x * ncoeff.z +
		tmp.z * ncoeff.x *  coeff.z +
		tmp.w *  coeff.x *  coeff.z;

	ret.y = TVoxel::valueToFloat(p1 * ncoeff.y + p2 * coeff.y - v1);

	// gradient z
	p1 = front.x * ncoeff.x * ncoeff.y +
		front.y *  coeff.x * ncoeff.y +
		front.z * ncoeff.x *  coeff.y +
		front.w *  coeff.x *  coeff.y;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), vmIndex).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), vmIndex).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), vmIndex).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), vmIndex).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.y +
		tmp.y *  coeff.x * ncoeff.y +
		tmp.z * ncoeff.x *  coeff.y +
		tmp.w *  coeff.x *  coeff.y;
	v1 = p1 * coeff.z + p2 * ncoeff.z;

	p1 = back.x * ncoeff.x * ncoeff.y +
		back.y *  coeff.x * ncoeff.y +
		back.z * ncoeff.x *  coeff.y +
		back.w *  coeff.x *  coeff.y;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 2), vmIndex).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 2), vmIndex).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 2), vmIndex).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 2), vmIndex).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.y +
		tmp.y *  coeff.x * ncoeff.y +
		tmp.z * ncoeff.x *  coeff.y +
		tmp.w *  coeff.x *  coeff.y;

	ret.z = TVoxel::valueToFloat(p1 * ncoeff.z + p2 * coeff.z - v1);

	return ret;
}

template<bool hasLabel, class TVoxel, class TIndex> struct VoxelLabelReader;
template<class TVoxel, class TIndex> struct VoxelLabelReader<false,TVoxel,TIndex>{
    _CPU_AND_GPU_CODE_ static ushort maxSampling(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                                   const THREADPTR(Vector3f) & point){
        return 0;
    }
};
template<class TVoxel, class TIndex> struct VoxelLabelReader<true,TVoxel,TIndex>{
    _CPU_AND_GPU_CODE_ static ushort maxSampling(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                                 const THREADPTR(Vector3f) & point){
        typename TIndex::IndexCache cache;
        return readFromSDF_label_nearestneighbor(voxelData,voxelIndex,point, cache);
    }
};

template<bool hasSemantic, class TVoxel, class TIndex> struct VoxelSemanticLabelReader;
template<class TVoxel, class TIndex> struct VoxelSemanticLabelReader<false,TVoxel,TIndex>{
    _CPU_AND_GPU_CODE_ static ushort closest(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                                 const THREADPTR(Vector3f) & point){
        return 0;
    }
};
template<class TVoxel, class TIndex> struct VoxelSemanticLabelReader<true,TVoxel,TIndex>{
    _CPU_AND_GPU_CODE_ static ushort closest(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
                                                 const THREADPTR(Vector3f) & point){
        typename TIndex::IndexCache cache;
        return readFromSDF_semantic_closest(voxelData,voxelIndex,point, cache);
    }
};


template<bool hasColor, class TVoxel, class TIndex> struct VoxelColorReader;
template<class TVoxel, class TIndex>
struct VoxelColorReader<false, TVoxel, TIndex> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
		const THREADPTR(Vector3f) & point)
	{
		return Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
	}
};

template<class TVoxel, class TIndex>
struct VoxelColorReader<true, TVoxel, TIndex> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(typename TIndex::IndexData) *voxelIndex,
		const THREADPTR(Vector3f) & point)
	{
		typename TIndex::IndexCache cache;
		return readFromSDF_color4u_interpolated(voxelData, voxelIndex, point, cache);
	}
};

#ifndef __METALC__

#include "ITMPlainVoxelArray.h"

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point_orig,
	THREADPTR(int) &vmIndex)
{
	Vector3i point = point_orig - voxelIndex->offset;

	if ((point.x < 0) || (point.x >= voxelIndex->size.x) ||
		(point.y < 0) || (point.y >= voxelIndex->size.y) ||
		(point.z < 0) || (point.z >= voxelIndex->size.z)) {
		vmIndex = false;
		return -1;
	}

	int linearIdx = point.x + point.y * voxelIndex->size.x + point.z * voxelIndex->size.x * voxelIndex->size.y;

	vmIndex = true;
	return linearIdx;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point_orig,
	THREADPTR(int) &vmIndex, THREADPTR(ITMLib::ITMPlainVoxelArray::IndexCache) & cache)
{
	return findVoxel(voxelIndex, point_orig, vmIndex);
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point_orig, THREADPTR(int) &vmIndex)
{
	int voxelAddress = findVoxel(voxelIndex, point_orig, vmIndex);
	return vmIndex ? voxelData[voxelAddress] : TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) *voxelData, const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point_orig, THREADPTR(int) &vmIndex, THREADPTR(ITMLib::ITMPlainVoxelArray::IndexCache) & cache)
{
	return readVoxel(voxelData, voxelIndex, point_orig, vmIndex);
}

template <bool hasLabel> struct SurfelLabelManipulator;
template<> struct SurfelLabelManipulator<true>{
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static ushort read(const TSurfel& surfel)
    {
        return surfel.label;
    }
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static float read_conf(const TSurfel &surfel)
    {
        return surfel.w_label;
    }
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static void write(TSurfel& surfel, const ushort& label)
    {
        surfel.label = label;
    }
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static void write_conf(TSurfel& surfel, const float &conf)
    {
        surfel.w_label = conf;
    }
};
template<> struct SurfelLabelManipulator<false>{
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static ushort read(const TSurfel& surfel)
    {
        return 0;
    }
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static float read_conf(const TSurfel &surfel)
    {
        return 0;
    }
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static void write(TSurfel& surfel, const ushort& label)
    {
    }
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static void write_conf(TSurfel& surfel, const float &conf)
    {
    }
};

/**
* \brief The specialisations of this struct template can be used to write/read colours to/from surfels.
*
* \tparam hasColour  Whether or not the surfel type can store colour information.
*/
template <bool hasColour> struct SurfelColourManipulator;

/**
* \brief This template specialisation can be used to write/read dummy colours to/from surfels.
*
* It is intended for use with surfel types that cannot store colour information.
*/
template <>
struct SurfelColourManipulator<false>
{
	/**
	* \brief Simulates the reading of a colour from the specified surfel.
	*
	* \param surfel  The surfel.
	* \return        A dummy colour (black).
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static Vector3u read(const TSurfel& surfel)
	{
		return Vector3u((uchar)0);
	}

	/**
	* \brief Simulates the writing of a colour to the specified surfel.
	*
	* In practice, this is just a no-op, since the surfel can't store a colour.
	*
	* \param surfel  The surfel.
	* \param colour  The colour.
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static void write(TSurfel& surfel, const Vector3u& colour)
	{
		// No-op
	}
};

/**
* \brief This template specialisation can be used to write/read actual colours to/from surfels.
*
* It is intended for use with surfel types that can store colour information.
*/
template <>
struct SurfelColourManipulator<true>
{
	/**
	* \brief Gets the colour of the specified surfel.
	*
	* \param surfel  The surfel.
	* \return        The surfel's colour.
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static Vector3u read(const TSurfel& surfel)
	{
		return surfel.colour;
	}

	/**
	* \brief Sets the colour of the specified surfel.
	*
	* \param surfel  The surfel.
	* \param colour  The surfel's new colour.
	*/
	template <typename TSurfel>
	_CPU_AND_GPU_CODE_
		static void write(TSurfel& surfel, const Vector3u& colour)
	{
		surfel.colour = colour;
	}
};

#endif

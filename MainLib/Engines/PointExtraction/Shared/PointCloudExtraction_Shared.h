#pragma once
#include <ORUtils/HSVToRGB.h>
#include "../../../Objects/Scene/IntegratePolicies.h"
#include "../../../Objects/Scene/ITMVoxelTypes.h"
#include "../../../Objects/PointCloud/PointCloud.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"


template<SCFUSION::IntegrateType integrateType, SCFUSION::Policy::Integrate TIntegratePolicy> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting (float value);
template<> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting <SCFUSION::IntegrateType_TSDF, SCFUSION::Policy::Integrate_WEIGHTED> (float value){
    return SCFUSION::ValueHSVToRGBA(value, -1.1f, 1.1f);
}
template<> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting <SCFUSION::IntegrateType_TSDF, SCFUSION::Policy::Integrate_DIRECT> (float value){
    return SCFUSION::ValueHSVToRGBA(value, -1.1f, 1.1f);
}
template<> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting <SCFUSION::IntegrateType_OFusion, SCFUSION::Policy::Integrate_DIRECT> (float value){
    return SCFUSION::ValueHSVToRGBA(value, LOGODD_MIN * 1.1, LOGODD_MAX * 1.1);
}
template<> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting <SCFUSION::IntegrateType_OFusion, SCFUSION::Policy::Integrate_WEIGHTED> (float value){
    return SCFUSION::ValueHSVToRGBA(value, LOGODD_ETY * 1.1, LOGODD_OCU * 1.1);
}
template<> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting <SCFUSION::IntegrateType_Invert_TSDF, SCFUSION::Policy::Integrate_DIRECT> (float value){
    return SCFUSION::ValueHSVToRGBA(value, LOGODD_MIN * 1.1, LOGODD_MAX * 1.1);
}
template<> _CPU_AND_GPU_CODE_  inline Vector4f DistanceFieldPainting <SCFUSION::IntegrateType_Invert_TSDF, SCFUSION::Policy::Integrate_WEIGHTED> (float value){
    return SCFUSION::ValueHSVToRGBA(value, LOGODD_ETY * 1.1, LOGODD_OCU * 1.1);
}

template<SCFUSION::IntegrateType T, class TVoxel>
_CPU_AND_GPU_CODE_  inline bool classifyVoxel_shared(uint i, uint *voxelOccupied,
                                                     const Vector4s *visibleBlockPos, const TVoxel *localVBA, const ITMHashEntry *hashTable)
{
    const Vector4s Pos_4s = visibleBlockPos[i];
    int vmIndex;
    TVoxel voxel = readVoxel(localVBA, hashTable, Vector3i(Pos_4s.x,Pos_4s.y,Pos_4s.z), vmIndex);
    if (!vmIndex) return false;
//    voxelOccupied[i] = checkVoxelState<T>(voxel.sdf);
    voxelOccupied[i] = voxel.w_depth > 0;
    return true;
}

_CPU_AND_GPU_CODE_ inline void pcCompactVoxels(int i, uint *compactedVoxelArray, uint *voxelOccupied, uint *voxelOccupiedScan, uint numVoxels)
{
    if(voxelOccupied[i])
        if (voxelOccupiedScan[i] < numVoxels)
            compactedVoxelArray[voxelOccupiedScan[i]] = i;
}



template<SCFUSION::IntegrateType integrateType, SCFUSION::Policy::Integrate TIntegratePolicy, class TVoxel>
_CPU_AND_GPU_CODE_ inline void generatePoints_shared(uint i, Vector4f *points, Vector4f *colors,
        uint *compactedVoxelArray, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const ITMHashEntry *hashTable, float factor) {
    const uint& voxel = compactedVoxelArray[i];
    const Vector4s globalPos_4s = visibleBlockGlobalPos[voxel];

    if (globalPos_4s.w == 0) return;

    Vector3i globalPos = Vector3i(globalPos_4s.x, globalPos_4s.y, globalPos_4s.z);

    int vmIndex;
    float value = TVoxel::valueToFloat(readVoxel(localVBA, hashTable, globalPos, vmIndex).sdf);

    Vector3f realPos = globalPos.toFloat() * factor;
    points[i].x = realPos.x;
    points[i].y = realPos.y;
    points[i].z = realPos.z;
    points[i].w = 1;

    if(colors != nullptr) {
//        colors[i] = Vector4f(1,0,0,1);
        colors[i] = DistanceFieldPainting<integrateType, TIntegratePolicy>(value);
    }
}
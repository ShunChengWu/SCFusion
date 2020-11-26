// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Objects/Scene/ITMPlainVoxelArray.h"
#include "Objects/Scene/SCPlainVoxelArray.h"
#include "Objects/Scene/ITMSurfelTypes.h"
#include "Objects/Scene/ITMVoxelBlockHash.h"
#include "Objects/Scene/ITMVoxelTypes.h"

#include "Objects/Scene/IntegratePolicies.h"
#include "Engines/SceneCompletion/FusionPolicies.h"

/** This chooses the information stored at each surfel. At the moment, valid
    options are ITMSurfel_grey and ITMSurfel_rgb.
*/
#ifdef SURFELTYPE
//typedef ITMLib::ITMSurfel_rgb ITMSurfelT;
typedef SURFELTYPE ITMSurfelT;
#endif

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb and ITMVoxel_f_rgb.
*/
//typedef ITMVoxel_s ITMVoxel;
//typedef ITMVoxel_s_label ITMVoxel;
//typedef ITMVoxel_f_conf ITMVoxel;
//typedef ITMVoxel_f_rgb ITMVoxel;

//typedef OFu_Voxel_f_label ITMVoxel;
//typedef TSDF_Voxel_f_label ITMVoxel;
#ifdef VOXELTYPE
typedef VOXELTYPE ITMVoxel;
#endif

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::ITMVoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::ITMPlainVoxelArray ITMVoxelIndex;




/// Below is to test warp policies along with ITMVoxel
struct Policy_Direct_FuseAll
{
    static const CONSTPTR(bool) hasSCFusionPolicy = true;
    static const SCFUSION::Policy::FuseTwo fusePolicy = SCFUSION::Policy::FuseTwo::FuseTwo_OCCUPIED;
    static const SCFUSION::Policy::Integrate integratePolicy = SCFUSION::Policy::Integrate_DIRECT;
};
typedef Policy_Direct_FuseAll MappingPolicy;




//template<class VoxelType, class MapPolicy>
//struct Voxel_traits{
//    static const MapPolicy Policy;
//    static const VoxelType Voxel;
//};
//typedef Voxel_traits<ITMVoxel, MappingPolicy> SCVoxelType;



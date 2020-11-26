#pragma once
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/MathUtils.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/ITMVoxelTypes.h"
#include "../../../Objects/Scene/IntegratePolicies.h"
#include "../../../Objects/Scene/DistanceField_shared.h"
#include "../FusionPolicies.h"

//#define Use_Weight_Filtering

namespace SCFUSION {
/// ===================================================== FUSETWO =======================================-==============
    _CPU_AND_GPU_CODE_ inline int occupancy_fusetwo_kernel(Policy::FuseTwo policy, const float &value_from, const int &weight_from, float &value_to, int &weight_to, int maxW){
        switch (policy){
            case Policy::FuseTwo_OCCUPIED:
            {
                if(value_from <= 0 || value_to < 0) return 0; // only fuse complete. ignore empty space
//                fuse2_ofu<SCFusion::Policy::Integrate_DIRECT>(value_to,weight_to, calculateLogOdd(value_from), predictscW, maxW);
                if(/*value_to >= 0 && */weight_to > 0) {
                    return 1; // only fuse label
                } else {
                    fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(value_from), predictscW, maxW);
                    return 2; // fuse both.
                }
            }
                break;
            case Policy::FuseTwo_ALL_CONFIDENCE:
            {

            }
                break;
            case Policy::FuseTwo_UNKNOWN:
            {
                if(value_from <= 0) return 0; // only fuse complete. ignore empty space
                if(value_to != 0) return 0; // ignore known area
                fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(value_from), predictscW, maxW);
            }
                break;
            case Policy::FuseTwo_UNKNOWN_CONFIDENCE:
            {
                if(value_from <= 0) return 0; // only fuse complete. ignore empty space
                if(value_to != 0) return 0; // ignore known area
                fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(value_from), weight_from * predictscW, maxW);
            }
                break;
            case Policy::FuseTwo_ALL_OCCUPANCY:
            {
                if(value_from <= 0) return 0; // only fuse complete. ignore empty space
                fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(weight_from), predictscW, maxW);
            }
                break;
            case Policy::FuseTwo_UNKNOWN_OCCUPANCY:
            {
                if(value_from <= 0) return 0; // only fuse complete. ignore empty space
                if(value_to != 0) return 0; // ignore known area
                fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(weight_from), predictscW, maxW);
            }
                break;
            case Policy::FuseTwo_ALL_UNWEIGHT:
            {
                if(value_from <= 0) return 0; // only fuse complete. ignore empty space
                fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(value_from), 0, maxW);
            }
                break;
            case Policy::FuseTwo_UNKNOWN_UNWEIGHT:
            {
                if(value_from <= 0) return 0; // only fuse complete. ignore empty space
                if(value_to != 0) return 0; // ignore known area
                fuse2_ofu<SCFUSION::Policy::Integrate_DIRECT>(value_to, weight_to, calculateLogOdd(value_from), 0, maxW);
            }
                break;
        }
        return 1;
    }

//    template<class T>
//    _CPU_AND_GPU_CODE_ inline uint getMaxConfLabel(const T &label_conf, int start){return 0;}
//    template<>_CPU_AND_GPU_CODE_ inline uint getMaxConfLabel<LogOddLabels>(const LogOddLabels &label_conf, int start){
//        unsigned int max = start;
//        for(size_t i=start+1;i<LogOddLabels::size; ++i) if(label_conf.labels[i] > label_conf.labels[max]) max = i;
//        return max;
//    }
//    template<> _CPU_AND_GPU_CODE_ inline uint getMaxConfLabel<OFu_Voxel_f_label>(const OFu_Voxel_f_label &voxel, int start){
//        return getMaxConfLabel(voxel.logodd_label, start);
////    }
//    template<class TVoxel> _CPU_AND_GPU_CODE_ inline void fuseTwoLabel(TVoxel &voxel, const LogOddLabels &label_conf, int start){
//        voxel.label = getMaxConfLabel(label_conf, start);
//    }
//    template<> _CPU_AND_GPU_CODE_ inline void fuseTwoLabel<OFu_Voxel_f_label>(OFu_Voxel_f_label &voxel, const LogOddLabels &label_conf, int start){
//        for(size_t i=start;i<LogOddLabels::size; ++i)
////            voxel.logodd_label.labels[i] = clampLogOdd(voxel.logodd_label.labels[i] + calculateLogOdd(label_conf.labels[i]));
//            voxel.logodd_label.labels[i] += clampLogOdd(label_conf.labels[i] - 0.5);
//        voxel.label = getMaxConfLabel(voxel.logodd_label, start);
//    }
//    template<> _CPU_AND_GPU_CODE_ inline void fuseTwoLabel<OFu_Voxel_f_1label>(OFu_Voxel_f_1label &voxel, const LogOddLabels &label_conf, int start){
//        uint maxLabel = getMaxConfLabel(label_conf, start);
//        float maxLabelConf = label_conf.labels[maxLabel];
//        if (voxel.label == maxLabel)
//            voxel.w_label += maxLabelConf;
//        else if (voxel.w_label <= maxLabelConf){
//            voxel.label = maxLabel;
//            voxel.w_label = maxLabelConf;
//        } else
//            voxel.w_label -= maxLabelConf;
//    }

    template<SCFUSION::IntegrateType TIntegrate> _CPU_AND_GPU_CODE_ inline int fuseTwo(const SCFUSION::Policy::FuseTwo &policyFuseTwo,
                                                                                       const float &value_from, const int &weight_from, float &value_to, int &weight_to, int maxW){
        return -1;
    }
    template<>
    _CPU_AND_GPU_CODE_  inline int fuseTwo<SCFUSION::IntegrateType_TSDF>(const SCFUSION::Policy::FuseTwo &policyFuseTwo,
                                                                         const float &value_from, const int &weight_from, float &value_to, int &weight_to, int maxW){
        if(weight_to > 0) return 0; //TODO: change if this improve the result.
        if(value_from <= 0) return 0; // only complete. ignore empty space
        sdf_integrade_kernel<SCFUSION::Policy::Integrate_WEIGHTED>(value_to, weight_to, MIN(0.f, 1.f - value_from), weight_from, value_to, 1, maxW);
        return 1;
    }
    template<>
    _CPU_AND_GPU_CODE_  inline int fuseTwo<SCFUSION::IntegrateType_OFusion>(const SCFUSION::Policy::FuseTwo &policyFuseTwo,
                                                                            const float &value_from, const int &weight_from, float &value_to, int &weight_to, int maxW){
        return occupancy_fusetwo_kernel(policyFuseTwo, value_from, weight_from,value_to,weight_to, maxW);
    }

    template<class TVoxel>
    _CPU_AND_GPU_CODE_ static inline void UpdateLabel(TVoxel &voxel, ushort label_new, float w_new){
        if(label_new==0)return;
        ushort label_old = accessLabel<TVoxel>::readLabel(voxel);
        float w_old   = accessLabel<TVoxel>::readWeight(voxel);

        if(label_old == 0 || w_old == 0){ // init
            label_old = label_new;
            w_old = w_new;
        } else if(label_old == label_new){ // labels are euqal. increase weight
            w_old += w_new;
        } else if(w_old > w_new){ // labels are different & has larger w. deduce weight
            w_old  -= w_new;
        } else { // labels are different & has smaller w. replace label
            label_old = label_new;
            w_old = w_new - w_old;
        }
        w_old = MIN(w_old,MaxLabelW);

        accessLabel<TVoxel>::writeLabel(voxel, label_old);
        accessLabel<TVoxel>::writeLabelWeight(voxel, w_old);
    }

    template<bool hasLabelInformation, class TVoxel> struct FuseTwoVoxel;
    template<class TVoxel> struct FuseTwoVoxel<true, TVoxel> {
        _CPU_AND_GPU_CODE_ static  inline bool fuse(DEVICEPTR(TVoxel) & voxel, const SCFUSION::Policy::FuseTwo &policyFuseTwo,
                const float &label_from, const float &label_conf_from, const int &weight_from, int maxW){
            float value_to = TVoxel::valueToFloat(voxel.sdf);
            int weight_to = voxel.w_depth;
            float voxel_state = label_from > 0 ? 0.51f : 0.f; //TODO: see if use 0.51 can make the correrction from reconstruction faster

//#define DEBUG_FUSE_SEMANTIC_ONLY
#ifdef DEBUG_FUSE_SEMANTIC_ONLY
            voxel.label = label_from;
            voxel.w_label = label_conf_from;
#else
            int state = fuseTwo<TVoxel::integrateType>(policyFuseTwo, voxel_state, weight_from, value_to, weight_to, maxW);
            if(state > 0){
                voxel.sdf = TVoxel::floatToValue(value_to);
                if(state == 2) voxel.w_depth = weight_to;

                int w_label_old = voxel.w_label;

                int weighted_conf = weight_to; // unknown-> lower weight, known->higher weight
                int weight_label = voxel.w_label;


                if(0) {
                    if(label_from > 0){
                        voxel.label = label_from;
                        voxel.w_label = label_conf_from;
                    }

                } else
                {
                    UpdateLabel(voxel, label_from,label_conf_from);
                }


                return true;
            }
#endif

            return false;
        }
//        _CPU_AND_GPU_CODE_ static  inline void fuse(DEVICEPTR(TVoxel) & voxel, const SCFusion::Policy::FuseTwo &policyFuseTwo,
//                const LogOddLabels &label_conf, const int &weight_from, int maxW){
//            float value_to = TVoxel::valueToFloat(voxel.sdf);
//            int weight_to = voxel.w_depth;
//            float voxel_state = getMaxConfLabel(label_conf, 0) > 0 ? 1 : 0;
//            if(fuseTwo<TVoxel::integrateType>(policyFuseTwo, voxel_state, weight_from, value_to, weight_to, maxW)){
//                voxel.sdf = TVoxel::floatToValue(value_to);
////                voxel.w_depth = weight_to;
//                fuseTwoLabel(voxel, label_conf, 0);
//            }
//        }
    };
    template<class TVoxel> struct FuseTwoVoxel<false, TVoxel> {
        _CPU_AND_GPU_CODE_ static  inline bool fuse(DEVICEPTR(TVoxel) & voxel, const SCFUSION::Policy::FuseTwo &policyFuseTwo,
                                                    const float &label_from, const float &label_conf_from, const int &weight_from, int maxW){return false;}
    };

    template<class TVoxel, class TIndex>
    _CPU_AND_GPU_CODE_ inline void fuse2_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                Vector3s dim_from, Vector3f base, float oneOverVoxelSize,
                                                SCFUSION::Policy::FuseTwo policyFuseTwo,
                                                const CONSTPTR(Vector4f) & projParams_d, const CONSTPTR(Matrix4f) & M_d,
                                                const CONSTPTR(Vector2i) & imgSize,
                                                const float *labels, const float *label_confs, const bool *masks, int maxW, float time){
        int z = i / (dim_from.x * dim_from.y);
        int y = (i % (dim_from.x * dim_from.y)) / dim_from.x;
        int x = (i % (dim_from.x * dim_from.y)) % dim_from.x;

        // Remove roof
//        if(y >= dim_from.y - 20) return;

        Vector3f pt = base * oneOverVoxelSize + Vector3f(x,y,z);

        // check view frustum here
//        if(0)
//        {
//            Vector4f pt_camera; Vector2f pt_image;
//            Vector4f pt_model;// ( pt.x,pt.y,pt.z,1);
//            pt_model.x = (base.x + x * 1/oneOverVoxelSize);
//            pt_model.y = (base.y + y * 1/oneOverVoxelSize);
//            pt_model.z = (base.z + z * 1/oneOverVoxelSize);
//            pt_model.w = 1;
//            pt_camera = M_d * pt_model;
//
//            if (pt_camera.z <= 0) return;
//            pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
//            pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
//            if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;
//        }

        typename TIndex::IndexCache cache;
        int vmIndex;

        int idx_to = findVoxel(voxelIndex,Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z)),vmIndex, cache);
        if( !vmIndex  ) return;
        TVoxel &voxel_to = voxelData[idx_to];

#if 0 // debug visialization grids and centre of grids
        voxel_to.sdf = TVoxel::floatToValue(0);
        voxel_to.w_depth = 1;
        voxel_to.label = 0;
        int offset = 8;

        Vector3i blockCoord = Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z));
        // line-grid
        if( (blockCoord.x % offset == 0 && blockCoord.y % offset == 0) || (blockCoord.y % offset == 0 && blockCoord.z % offset == 0) || (blockCoord.x % offset == 0 && blockCoord.z % offset == 0) )
        {
            TVoxel &voxel_to = voxelData[idx_to];
            voxel_to.sdf = TVoxel::floatToValue(4);
            voxel_to.w_depth = 1;
            voxel_to.label = 2;
        }
        // dot
        blockCoord.x -= offset*0.5;
        blockCoord.y -= offset*0.5;
        blockCoord.z -= offset*0.5;
        if( (blockCoord.x % offset == 0 && blockCoord.y % offset == 0 && blockCoord.z % offset == 0))
        {
            TVoxel &voxel_to = voxelData[idx_to];
            voxel_to.sdf = TVoxel::floatToValue(4);
            voxel_to.w_depth = 1;
            voxel_to.label = 2;
        }
#endif
        //auto weighted_W = masks? (masks[i] > 0 ? predictscW * 0.1 : predictscW) : predictscW; // the semantic label of unseen area is most likely to be unstable -> lower weight
        int weighted_W = predictscW;
        if(FuseTwoVoxel<TVoxel::hasLabelInformation, TVoxel>::fuse(voxel_to,policyFuseTwo,labels[i], label_confs[i], weighted_W, maxW)){

        };
//        accessUpdateState<TVoxel>::write(voxel_to,false);
        accessTimeStamp<TVoxel>::write(voxel_to,time);
    }

    template<class TVoxel, class TIndex, int LabelNum>
    _CPU_AND_GPU_CODE_ inline void labelPropagation_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                Vector3s dims, Vector3f base, float oneOverVoxelSize, int radius) {
        int z = (int) floor(double(i) / double(dims.x * dims.y));
        int y = (int) floor(double(i - z * dims.x * dims.y) / double(dims.x));
        int x = i - (z * dims.x * dims.y) - (y * dims.x);
        int vmIndex;
        typename TIndex::IndexCache cache;

        Vector3f pt, pt_origin = base * oneOverVoxelSize + Vector3f(x, y, z);
        int idx_origin = findVoxel(voxelIndex, Vector3i((int) ROUND(pt_origin.x), ROUND(pt_origin.y), ROUND(pt_origin.z)), vmIndex, cache);
        TVoxel &voxel_origin = voxelData[idx_origin];
        if(accessLabel<TVoxel>::readLabel(voxel_origin) > 0) return;


        int idx;

        uchar max_w=0;
        ushort max_label=0;
        ushort new_label;
//        uchar new_w;
        ushort counter[LabelNum];
        for(auto &c : counter) c=0;

        for(int x_=-radius; x_ <= radius; ++x_) {
            if (x_ == 0)continue;
            for (int y_ = -radius; y_ <= radius; ++y_) {
                if(y_==0)continue;
                for (int z_ = -radius; z_ <= radius; ++z_) {
                    if(z_==0)continue;
                    pt = pt_origin + Vector3f(x_, y_, z_);
                    idx = findVoxel(voxelIndex, Vector3i((int) ROUND(pt.x), ROUND(pt.y), ROUND(pt.z)), vmIndex, cache);
                    if (!vmIndex) continue;
                    const TVoxel &voxel_to = voxelData[idx];
                    new_label = accessLabel<TVoxel>::readLabel(voxel_to);
                    if(new_label == 0 ) continue;


                    if(new_label < LabelNum)counter[new_label]++;
//                    new_w = accessLabel<TVoxel>::readWeight(voxel_to);
//                    if(new_w > max_w) {
//                        max_w = new_w;
//                        max_label = new_label;
//                    }



                }
            }
        }


        for(size_t l=0; l < LabelNum; ++l){
            if(counter[l] > max_w){
                max_w = counter[l];
                max_label = l;
            }
        }
        accessLabel<TVoxel>::writeLabel(voxel_origin, max_label);
        accessLabel<TVoxel>::writeLabelWeight(voxel_origin, max_w);
    }

/// ====================================================================================================================



    /// Without Mask
    template <class TVoxel>
    _CPU_AND_GPU_CODE_ static inline void copyValueFromScene_shared(int i, const TVoxel *voxelData, const typename ITMLib::ITMVoxelBlockHash::IndexData *voxelIndex,
                                             const Vector3f &base, float oneOverVoxelSize, const Vector3s &dims, float *data){
        int z = i / (dims.x * dims.y);
        int y = (i % (dims.x * dims.y)) / dims.x;
        int x = (i % (dims.x * dims.y)) % dims.x;

        //int idx = (z * dims.y + y) * dims.x + x;

        Vector3f pt = base * oneOverVoxelSize + Vector3f(x, y, z);
        typename ITMLib::ITMVoxelBlockHash::IndexCache cache;

        int vmIndex;
        TVoxel voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);
        data[i] = TVoxel::valueToFloat(voxel.sdf);
    }

    template <class TVoxel>
    _CPU_AND_GPU_CODE_ static inline void copyUpdateStateFromScene_shared(int i, const TVoxel *voxelData, const typename ITMLib::ITMVoxelBlockHash::IndexData *voxelIndex,
                                                                    const Vector3f &base, float oneOverVoxelSize, const Vector3s &dims, float threshold, float time, bool *updateState){
        int z = i / (dims.x * dims.y);
        int y = (i % (dims.x * dims.y)) / dims.x;
        int x = (i % (dims.x * dims.y)) % dims.x;
        Vector3f pt = base * oneOverVoxelSize + Vector3f(x, y, z);
        typename ITMLib::ITMVoxelBlockHash::IndexCache cache;
        int vmIndex;

        TVoxel voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);
//        if(TVoxel::floatToValue(voxel.sdf) > LOGODD_MAX * 0.5) {
#ifdef Use_Weight_Filtering
        if(voxel.w_depth > 50) {
            updateState[i] = accessUpdateState<TVoxel>::read(voxel);
        } else {
            updateState[i] = false; // set to false to lower the change to run back-end
        }
#else
        if(!vmIndex) updateState[i] = false;
        else if (accessTimeStamp<TVoxel>::read(voxel)==0) {
            updateState[i] = true;
        } else {
            float diff = time - accessTimeStamp<TVoxel>::read(voxel);
            updateState[i] = diff > threshold;
        }

        //updateState[i] = accessUpdateState<TVoxel>::read(voxel);
#endif
    }
    /// With Mask
    template <class TVoxel>
    _CPU_AND_GPU_CODE_ static inline void copyValueFromScene_shared(int i, const TVoxel *voxelData, const typename ITMLib::ITMVoxelBlockHash::IndexData *voxelIndex,
                                                                    const Vector3f &base, float oneOverVoxelSize, const Vector3s &dims, float *data, bool *mask){
        int z = i / (dims.x * dims.y);
        int y = (i % (dims.x * dims.y)) / dims.x;
        int x = (i % (dims.x * dims.y)) % dims.x;
        Vector3f pt = base * oneOverVoxelSize + Vector3f(x, y, z);
        typename ITMLib::ITMVoxelBlockHash::IndexCache cache;
        int vmIndex;

        TVoxel voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);

//        if(TVoxel::floatToValue(voxel.sdf) > LOGODD_MAX * 0.5) {

#ifdef Use_Weight_Filtering
        if(voxel.w_depth > 50) {
            mask[i] = voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue();
            data[i] = voxel.w_depth == 0? TVoxel::SDF_initialValue() : TVoxel::valueToFloat(voxel.sdf);
        } else {
            mask[i] = true;
            data[i] = TVoxel::SDF_initialValue();
        }
#else
        // Test with filter out completed voxels ( weight==1 )
        mask[i] = voxel.w_depth < 2;
        data[i] = TVoxel::valueToFloat(voxel.sdf);
//        if(voxel.w_depth <=1 && voxel.sdf >= TVoxel::SDF_initialValue()){
//            mask[i] = true;
//            data[i] = voxel.sdf == TVoxel::SDF_initialValue();
//        } else {
//            mask[i] = false;
//            data[i] = TVoxel::valueToFloat(voxel.sdf);
//        }
//        mask[i] = voxel.w_depth == 0 && voxel.sdf == TVoxel::SDF_initialValue();
//        data[i] = voxel.w_depth == 0? TVoxel::SDF_initialValue() : TVoxel::valueToFloat(voxel.sdf);
#endif
    }

    _CPU_AND_GPU_CODE_ inline void LogOddLikelihood2Occupancy(float &value){
        value = 1.f - 1.f/(1.f+expf(value));
        // Ignore initial state (==0)
//    return value == 0.f ? 0 : 1.f - 1.f/(1.f+expf(value));
    }


    template <SCFUSION::Policy::Integrate T> _CPU_AND_GPU_CODE_ inline void LogOddP2InvertTSDF_kernel(float &value);
    template <> _CPU_AND_GPU_CODE_ inline void LogOddP2InvertTSDF_kernel<SCFUSION::Policy::Integrate_DIRECT>(float &value){
        if (value <= /*LOGODD_SURFACE*/ 0) {
            value = 0;
            return;
        }
        value = (value - LOGODD_SURFACE) / (LOGODD_MAX - LOGODD_SURFACE);
    }
    template <> _CPU_AND_GPU_CODE_ inline void LogOddP2InvertTSDF_kernel<SCFUSION::Policy::Integrate_WEIGHTED>(float &value){
        if (value <= /*LOGODD_SURFACE*/ 0) {
            value = 0;
            return;
        }
        value = (value / LOGODD_OCU);
    }

    _CPU_AND_GPU_CODE_ inline void TSDF2TDF_shared (float &value) {
        if(value == 0) return;
        if(value == 1) value = 0;
        else value = 1 - ABS(value);
    }

    template<SCFUSION::IntegrateType TIntegrateType, SCFUSION::Policy::Integrate TIntegratePolicy> struct convert2SC{
        _CPU_AND_GPU_CODE_ static inline  void compute (float &value) {
            value = -1;
        }
    };

    template<SCFUSION::Policy::Integrate TIntegratePolicy> struct convert2SC <SCFUSION::IntegrateType_TSDF, TIntegratePolicy> {
        _CPU_AND_GPU_CODE_ static inline  void compute (float &value) {
            TSDF2TDF_shared(value);
        }
    };

    template<SCFUSION::Policy::Integrate TIntegratePolicy> struct convert2SC <SCFUSION::IntegrateType_OFusion, TIntegratePolicy> {
        _CPU_AND_GPU_CODE_ static inline  void compute (float &value) {
            LogOddP2InvertTSDF_kernel<TIntegratePolicy>(value);
        }
    };


    /// Copy the labels and confidences of occupied voxels
    template <class TVoxel>
    _CPU_AND_GPU_CODE_ static inline void copyLabelAndConfidenceFromScene_shared(int i, const TVoxel *voxelData, const typename ITMLib::ITMVoxelBlockHash::IndexData *voxelIndex,
         const Vector3f &base, float oneOverVoxelSize, const Vector3s &dims, float *labels, float *confs, bool *masks){
        int z = i / (dims.x * dims.y);
        int y = (i % (dims.x * dims.y)) / dims.x;
        int x = (i % (dims.x * dims.y)) % dims.x;
        Vector3f pt = base * oneOverVoxelSize + Vector3f(x, y, z);
        typename ITMLib::ITMVoxelBlockHash::IndexCache cache;
        int vmIndex;
        //TODO: maybe extract unlabeled occupied voxels as well? treat them as unknown
        TVoxel voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(pt.x), (int)ROUND(pt.y), (int)ROUND(pt.z)), vmIndex, cache);

        if(TVoxel::valueToFloat(voxel.sdf) >= 0 && voxel.w_depth > 0) {
            labels[i] = accessLabel<TVoxel>::readLabel(voxel);
            confs[i]  = CLAMP(accessLabel<TVoxel>::readWeight(voxel) / MaxLabelW, 0.1, 0.999);
            masks[i]  = 1;
        } else {
            labels[i] = 0;
            confs[i]  = 0;
            masks[i]  = 0;
        }
    }

    template<class TVoxel, class TIndex>
    _CPU_AND_GPU_CODE_ inline void copyLabelAndConfidenceToScene_shared(int i, TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                Vector3s dim_from, Vector3f base, float oneOverVoxelSize,
                                                const float *labels, const float *label_confs, const bool *masks){
        int z = i / (dim_from.x * dim_from.y);
        int y = (i % (dim_from.x * dim_from.y)) / dim_from.x;
        int x = (i % (dim_from.x * dim_from.y)) % dim_from.x;

        Vector3f pt = base * oneOverVoxelSize + Vector3f(x,y,z);

        typename TIndex::IndexCache cache;
        int vmIndex;

        int idx_to = findVoxel(voxelIndex,Vector3i((int)ROUND(pt.x),ROUND(pt.y),ROUND(pt.z)),vmIndex, cache);
        if( !vmIndex  ) return;
        TVoxel &voxel_to = voxelData[idx_to];

        if(masks[i]/*labels[i]>0 && label_confs[i]>0*/) {
            //TODO: maybe reduce the weight from CRF?
            // right now if CRF predict 1, then it's scaled to MaxLabelW which mean the label will be replaced for sure
//            UpdateLabel(voxel_to, ushort(labels[i]), label_confs[i]*2 /*MaxLabelW*/);
            accessLabel<TVoxel>::writeLabel(voxel_to, labels[i]);
            accessLabel<TVoxel>::writeLabelWeight(voxel_to, label_confs[i]*2);
        }
//TODO: debug. overwrite everything to see the effect of optimization.
// and to see whehter the bug is in only due to optimization or with fusion.

    }
}
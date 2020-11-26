// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"


namespace SCFUSION {
    enum IntegrateType {
        IntegrateType_TSDF, IntegrateType_OFusion, IntegrateType_Invert_TSDF
    };
    inline static IntegrateType toIntegrateType(int num){
        if(num > 2) {
            printf("[ERROR] Called toIntegrateType with larger number than 3 is"
                   " not allowed.\n");
            exit(-1);
        }
        return  static_cast<IntegrateType>(num);
    }

    inline static std::string fromIntegrateType(const IntegrateType &type) {
        switch (type){
            case IntegrateType_TSDF:
                return "TSDF";
            case IntegrateType_OFusion:
                return "Occupancy";
            case IntegrateType_Invert_TSDF:
                return "InvertTSDF";
            default:
                return "UNDEFINEDTYPE!";
        }
    }
    inline static int toIntegrateType(IntegrateType &type, const std::string &num){
        if(num == "TSDF") {
            type = IntegrateType_TSDF;
            return 1;
        }
        if(num == "Occupancy") {
            type = IntegrateType_OFusion;
            return 1;
        }
        if(num == "InvertTSDF"){
            type = IntegrateType_Invert_TSDF;
            return 1;
        }
        return 0;
    }

    inline std::ostream& operator<<(std::ostream & os, IntegrateType & cat)
    {
        os << fromIntegrateType(cat);
        return os;
    }
}

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_f_rgb
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}
};

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_s_rgb
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_s_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}
};

struct ITMVoxel_s
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_s()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

struct ITMVoxel_f
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_f()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

struct ITMVoxel_f_conf
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
	}
};


struct ITMVoxel_s_label
{
    _CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
    _CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = true;
    static const CONSTPTR(bool) hasTimeInformation = true;
    
    /** Value of the truncated signed distance transformation. */
    short sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** Label information */
    ushort label;
    /** Number of fused observations that make up @p label. */
    uchar w_label;

    float time;
    /** Padding that may or may not improve performance on certain GPUs */
    //uchar pad;

    _CPU_AND_GPU_CODE_ ITMVoxel_s_label()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        label = 0;
        w_label = 0;
        time=0;
    }
};

typedef struct TSDF_Voxel_f_RGB
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
    static const CONSTPTR(bool) hasColorInformation = true;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** RGB colour information stored for this voxel. */
    Vector3u clr;
    /** Number of observations that made up @p clr. */
    uchar w_color;

    _CPU_AND_GPU_CODE_ TSDF_Voxel_f_RGB()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        clr = Vector3u((uchar)0);
        w_color = 0;
    }
} TSDF_Voxel_f_RGB;

typedef struct TSDF_Voxel_f
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;

    _CPU_AND_GPU_CODE_ TSDF_Voxel_f()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
    }
} TSDF_Voxel_f;

typedef struct OFu_Voxel_f
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_OFusion;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;

    _CPU_AND_GPU_CODE_ OFu_Voxel_f()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
    }
} OFu_Voxel_f;

typedef struct TSDF_Voxel_f_label
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = true;
    static const CONSTPTR(bool) hasTimeInformation = true;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** Label information */
    ushort label;
    /** Number of fused observations that make up @p label. */
    uchar w_label;

    float time;
    _CPU_AND_GPU_CODE_ TSDF_Voxel_f_label()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        label = 0;
        w_label = 0;
        time=0;
    }
} TSDF_Voxel_f_label;

//struct LogOddLabels {
//    static const CONSTPTR(uchar) size = 12;
//    float labels[size];
//    _CPU_AND_GPU_CODE_ LogOddLabels(){
//        for(float & i : labels) i = 0;
//    }
//};
//
//typedef struct OFu_Voxel_f_label
//{
//    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
//    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
//    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }
//
//    static const CONSTPTR(SCFusion::IntegrateType) integrateType = SCFusion::IntegrateType_OFusion;
//    static const CONSTPTR(bool) hasColorInformation = false;
//    static const CONSTPTR(bool) hasConfidenceInformation = false;
//    static const CONSTPTR(bool) hasSemanticInformation = false;
//    static const CONSTPTR(bool) hasLabelInformation = true;
//
//    /** Value of the truncated signed distance transformation. */
//    float sdf;
//    /** Number of fused observations that make up @p sdf. */
//    uchar w_depth;
//    /** Label information */
//    ushort label;
//    /** Number of fused observations that make up @p label. */
//    uchar w_label;
//
//    LogOddLabels logodd_label;
//
//    _CPU_AND_GPU_CODE_ OFu_Voxel_f_label()
//    {
//        sdf = SDF_initialValue();
//        w_depth = 0;
//        label = 0;
//        w_label = 0;
//    }
//} OFu_Voxel_f_label;

typedef struct OFu_Voxel_f_1label
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_OFusion;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = true;
    static const CONSTPTR(bool) hasTimeInformation = true;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** Label information */
    ushort label;
    /** Number of fused observations that make up @p label. */
    float w_label;

    float time;
    _CPU_AND_GPU_CODE_ OFu_Voxel_f_1label()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        label = 0;
        w_label = 0;
        time=0;
    }
} OFu_Voxel_f_1label;

//typedef struct InvertTSDF_Voxel_f_label
//{
//    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
//    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
//    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }
//
//    static const CONSTPTR(SCFusion::IntegrateType) integrateType = SCFusion::IntegrateType_Invert_TSDF;
//    static const CONSTPTR(bool) hasColorInformation = false;
//    static const CONSTPTR(bool) hasConfidenceInformation = false;
//    static const CONSTPTR(bool) hasSemanticInformation = false;
//    static const CONSTPTR(bool) hasLabelInformation = true;
//
//    /** Value of the truncated signed distance transformation. */
//    float sdf;
//    /** Number of fused observations that make up @p sdf. */
//    uchar w_depth;
//    /** Label information */
//    ushort label;
//    /** Number of fused observations that make up @p label. */
//    uchar w_label;
//
//    LogOddLabels logodd_label;
//
//    _CPU_AND_GPU_CODE_ InvertTSDF_Voxel_f_label()
//    {
//        sdf = SDF_initialValue();
//        w_depth = 0;
//        label = 0;
//        w_label = 0;
//    }
//} InvertTSDF_Voxel_f_label;

typedef struct OFu_Voxel_f_2label
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_OFusion;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = true;
    static const CONSTPTR(bool) hasLabelInformation = true;
    static const CONSTPTR(bool) hasTimeInformation = true;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** Label information */
    ushort label;
    /** Number of fused observations that make up @p label. */
    float w_label;
    /** Semantic Label */
    ushort semantic;

    float time;
    _CPU_AND_GPU_CODE_ OFu_Voxel_f_2label()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        label = 0;
        w_label = 0;
        semantic = 0;
        time=0;
    }
} OFu_Voxel_f_2label;

typedef struct TSDF_Voxel_f_2label
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_TSDF;
    static const CONSTPTR(bool) hasColorInformation = false;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = true;
    static const CONSTPTR(bool) hasLabelInformation = true;
    static const CONSTPTR(bool) hasTimeInformation = true;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** Label information */
    ushort label;
    /** Number of fused observations that make up @p label. */
    float w_label;
    /** Semantic Label */
    ushort semantic;

    float time;

    _CPU_AND_GPU_CODE_ TSDF_Voxel_f_2label()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        label = 0;
        w_label = 0;
        semantic = 0;
        time=0;
    }
} TSDF_Voxel_f_2label;

typedef struct OFu_Voxel_f_rgb
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_OFusion;
    static const CONSTPTR(bool) hasColorInformation = true;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = false;
    static const CONSTPTR(bool) hasTimeInformation = false;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** RGB colour information stored for this voxel. */
    Vector3u clr;
    /** Number of observations that made up @p clr. */
    uchar w_color;

    _CPU_AND_GPU_CODE_ OFu_Voxel_f_rgb()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        clr = Vector3u((uchar)0);
        w_color = 0;
    }
} OFu_Voxel_f_rgb;

typedef struct OFu_Voxel_f_rgb_1label
{
    _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 0.0f; }
    _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
    _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

    static const CONSTPTR(SCFUSION::IntegrateType) integrateType = SCFUSION::IntegrateType_OFusion;
    static const CONSTPTR(bool) hasColorInformation = true;
    static const CONSTPTR(bool) hasConfidenceInformation = false;
    static const CONSTPTR(bool) hasSemanticInformation = false;
    static const CONSTPTR(bool) hasLabelInformation = true;
    static const CONSTPTR(bool) hasTimeInformation = true;

    /** Value of the truncated signed distance transformation. */
    float sdf;
    /** Number of fused observations that make up @p sdf. */
    uchar w_depth;
    /** Label information */
    ushort label;
    /** Number of fused observations that make up @p label. */
    float w_label;
    /** RGB colour information stored for this voxel. */
    Vector3u clr;
    /** Number of observations that made up @p clr. */
    uchar w_color;
    
    float time;

    _CPU_AND_GPU_CODE_ OFu_Voxel_f_rgb_1label()
    {
        sdf = SDF_initialValue();
        w_depth = 0;
        label = 0;
        w_label = 0;
        clr = Vector3u((uchar)0);
        w_color = 0;
        time=0;
    }
} OFu_Voxel_f_rgb_1label;
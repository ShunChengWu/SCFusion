#pragma once
#include <ORUtils/PlatformIndependence.h>
#ifndef CUDA_VECTR_TYPES
#define CUDA_VECTOR_TYPES

typedef unsigned int uint;

#ifdef COMPILE_WITH_CUDA
#include <cuda_runtime.h>
#else

typedef unsigned char uchar;

struct float2 {
    float x,y;
};

struct float3{
    float x,y,z;
};

struct float4{
    float x,y,z,w;
};

struct int2{
    int x,y;
};
struct int3{
    int x,y,z;
};
struct int4{
    int x,y,z,w;
};

struct uint2{
    uint x,y;
};
struct uint3{
    uint x,y,z;
};
struct uint4{
    uint x,y,z,w;
};

struct uchar2{
    uchar x,y;
};
struct uchar3{
    uchar x,y,z;
};
struct uchar4{
    uchar x,y,z,w;
};

inline _CPU_AND_GPU_CODE_ int2 make_int2(int x, int y)
{
    int2 f;
    f.x = x;
    f.y = y;
    return f;
}
inline _CPU_AND_GPU_CODE_ uint2 make_uint2(uint x, uint y)
{
    uint2 f;
    f.x = x;
    f.y = y;
    return f;
}

inline _CPU_AND_GPU_CODE_ int3 make_int3(int x, int y, int z)
{
    int3 f;
    f.x = x;
    f.y = y;
    f.z = z;
    return f;
}

inline _CPU_AND_GPU_CODE_ uint3 make_uint3(uint x, uint y, uint z)
{
    uint3 f;
    f.x = x;
    f.y = y;
    f.z = z;
    return f;
}
inline _CPU_AND_GPU_CODE_ int4 make_int4(int x, int y, int z, int w)
{
    int4 f;
    f.x = x;
    f.y = y;
    f.z = z;
    f.w = w;
    return f;
}

inline _CPU_AND_GPU_CODE_ uint4 make_uint4(uint x, uint y, uint z, uint w)
{
    uint4 f;
    f.x = x;
    f.y = y;
    f.z = z;
    f.w = w;
    return f;
}

inline _CPU_AND_GPU_CODE_ float2 make_float2(float a, float b)
{
    float2 f;
    f.x = a;
    f.y = b;
    return f;
}
inline _CPU_AND_GPU_CODE_ float3 make_float3(float x,float y, float z)
{
    float3 f;
    f.x = x;
    f.y = y;
    f.z = z;
    return f;
}
inline _CPU_AND_GPU_CODE_ float4 make_float4(float x,float y, float z, float w)
{
    float4 f;
    f.x = x;
    f.y = y;
    f.z = z;
    f.w = w;
    return f;
}

#endif
#endif

#include <iostream>
inline std::ostream& operator << (std::ostream &out, const int3 &a) {
    out << a.x << ", " << a.y << ", " << a.z;
    return out;
}


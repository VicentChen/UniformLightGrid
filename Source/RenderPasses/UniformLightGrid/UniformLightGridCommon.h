#pragma once
#include "Falcor.h"

using namespace Falcor;

inline uint deinterleave_32bit(uint x)
{
    x &= 0x09249249;                  // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    x = (x ^ (x >> 2)) & 0x030c30c3;  // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x ^ (x >> 4)) & 0x0300f00f;  // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x ^ (x >> 8)) & 0xff0000ff;  // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x ^ (x >> 16)) & 0x000003ff; // x = ---- ---- ---- ---- ---- --98 7654 3210
    return x;
}

inline uint3 deinterleave_uint3(uint x)
{
    uint3 v;
    v.z = deinterleave_32bit(x);
    v.y = deinterleave_32bit(x >> 1);
    v.x = deinterleave_32bit(x >> 2);
    return v;
}

inline float3 computePosByMortonCode(const uint mortonCode, const float quantLevels, const AABB& sceneBound)
{
    float3 quantPos = float3(deinterleave_uint3(mortonCode)) / quantLevels;
    float3 pos = quantPos * sceneBound.extent() + sceneBound.minPoint;
    return pos;
}

inline AABB computeAABBByMortonCode(const uint mortonCode, const uint prefixLength, const float quantLevels, const AABB& sceneBound)
{
    uint maskMin = 0xFFFFFFFF << (30 - prefixLength);
    uint mortonCodeMin = mortonCode & (maskMin);
    uint maskMax = 1 << (30 - prefixLength);
    uint mortonCodeMax = mortonCode | (maskMax - 1);

    float3 minPos = computePosByMortonCode(mortonCodeMin, quantLevels, sceneBound);
    float3 maxPos = computePosByMortonCode(mortonCodeMax, quantLevels, sceneBound);

    return AABB(minPos, maxPos);
}

inline float3 computePosByMortonCode(const uint mortonCode, const uint prefixLength, const float quantLevels, const AABB& sceneBound)
{
    uint maskMin = 0xFFFFFFFF << (30 - prefixLength);
    uint mortonCodeMin = mortonCode & (maskMin);
    uint maskMax = 1 << (30 - prefixLength);
    uint mortonCodeMax = mortonCode | (maskMax - 1);

    float3 minPos = computePosByMortonCode(mortonCodeMin, quantLevels, sceneBound);
    float3 maxPos = computePosByMortonCode(mortonCodeMax, quantLevels, sceneBound);

    return (minPos + maxPos) * 0.5f;
}

void createAndCopyBuffer(RenderContext* pRenderContext, Buffer::SharedPtr& pBuffer, Buffer::SharedPtr& pStagingBuffer, uint elementSize, uint elementCount, void* pCpuData, const std::string& bufferName, const std::string& stagingBufferName);

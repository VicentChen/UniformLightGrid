#include "LightProxyData.slangh"
#include "BVHNodeData.slangh"
import Scene.Scene;
import Experimental.Scene.Lights.LightCollection;
import MortonCodeHelpers;

#ifndef GROUP_SIZE
    // Compile-time error if GROUP_SIZE is not defined.
    #error GROUP_SIZE is not defined. Add define in cpp file.
#endif

cbuffer PerFrameCB
{
    uint emissiveTriangleCount;
}

RWStructuredBuffer<LightProxy> gProxys;

[numthreads(GROUP_SIZE, 1, 1)]
void genLightProxys(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint threadId = dispatchThreadId.x;
    if (threadId >= emissiveTriangleCount) return;

    // prepare triangle
    const EmissiveTriangle tri = gScene.lightCollection.getTriangle(threadId);
    float3 barycentrics = float3(1.0f / 3);
    float3 triangleCenter = tri.getPosition(barycentrics);
    float2 uv = tri.getTexCoord(barycentrics);
    float3 emissive = gScene.getEmissive(tri.materialID, uv, gScene.materialResources[tri.materialID]);
    float area = tri.area;

    uint mortonCode = computeMortonCodeByPos(triangleCenter);

    LightProxy proxy = {};
    proxy.pos = triangleCenter;
    proxy.mortonCode = mortonCode;
    proxy.triangleIdx = threadId;
    proxy.intensity = emissive * area;
    proxy.averageRadiance = gScene.lightCollection.fluxData[threadId].averageRadiance;
    proxy.flux = gScene.lightCollection.fluxData[threadId].flux;
    proxy.paddingAndDebug = float3(0, 0, 0);

    gProxys[threadId] = proxy;
}
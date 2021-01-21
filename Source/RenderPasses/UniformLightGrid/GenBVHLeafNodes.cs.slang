#include "BVHNodeData.slangh"
import Experimental.Scene.Lights.LightCollection;
import Utils.Math.BitTricks;
import Utils.Math.AABB;

#ifndef GROUP_SIZE
    // Compile-time error if GROUP_SIZE is not defined.
    #error GROUP_SIZE is not defined. Add define in cpp file.
#endif

cbuffer PerFrameCB
{
    LightCollection gLights;
    uint emissiveTriangleCount;
    uint quantLevels; // morton code dimension
    AABB sceneBound; // reference: LightBVHRefits.cs.slang
}

RWStructuredBuffer<LeafNode> gLeafNodes;

[numthreads(GROUP_SIZE, 1, 1)]
void genBVHLeafNodes(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint threadId = dispatchThreadId.x;
    if (threadId >= emissiveTriangleCount) return;

    float3 triangleCenter = gLights.getTriangle(threadId).getPosition(float3(1.0f/3));

    //normalize position to [0,1]
    float3 normPos = (triangleCenter - sceneBound.minPoint) / sceneBound.extent();
    uint3 quantPos = min(max(0, uint3(normPos * quantLevels)), quantLevels - 1);
    uint mortonCode = interleave_uint3(quantPos);

    LeafNode node = {};
    node.debug = triangleCenter;
    node.mortonCode = mortonCode;
    node.triangleIdx = threadId;

    gLeafNodes[threadId] = node;
}
#include "BVHNodeData.slangh"
import Scene.Scene;
import Experimental.Scene.Lights.LightCollection;
import Utils.Math.AABB;
import Utils.Math.BitTricks;

#ifndef GROUP_SIZE
    // Compile-time error if GROUP_SIZE is not defined.
    #error GROUP_SIZE is not defined. Add define in cpp file.
#endif

cbuffer PerFrameCB
{
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

    // prepare triangle
    const EmissiveTriangle tri = gScene.lightCollection.getTriangle(threadId);
    float3 barycentrics = float3(1.0f/3);
    float3 triangleCenter = tri.getPosition(barycentrics);
    float2 uv = tri.getTexCoord(barycentrics);
    float3 emissive = gScene.getEmissive(tri.materialID, uv, gScene.materialResources[tri.materialID]);
    float area = tri.area;

    //normalize position to [0,1]
    float3 normPos = (triangleCenter - sceneBound.minPoint) / sceneBound.extent();
    uint3 quantPos = min(max(0, uint3(normPos * quantLevels)), quantLevels - 1);
    uint mortonCode = interleave_uint3(quantPos);

    LeafNode node = {};
    node.mortonCode = mortonCode;
    node.triangleIdx = threadId;
    node.intensity = emissive * area;
    node.debug = float4(triangleCenter, area);

    gLeafNodes[threadId] = node;
}
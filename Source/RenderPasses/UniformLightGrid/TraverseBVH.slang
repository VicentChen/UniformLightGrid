#include "LightProxyData.slangh"
#include "BVHNodeData.slangh"
#include "GridData.slangh"
import Scene.ShadingData;
__exported import Experimental.Scene.Material.MaterialShading;
__exported import Utils.Sampling.SampleGenerator;
__exported import MortonCodeHelpers;
__exported import ULGStaticParams;

StructuredBuffer<LightProxy> gProxys;
StructuredBuffer<BVHInternalNode> gInternalNodes;
StructuredBuffer<UniformGrid> gGrids;

cbuffer PerFrameBVHCB
{
    uint gridMortonCodePrefixLength;
}

// This function uses SampleGenerator so we place it here.
float3 computeRandomPosByMortonCode<S : ISampleGenerator>(uint mortonCode, uint prefixLength, S sg)
{
    uint maskMin = 0xFFFFFFFF << (30 - prefixLength);
    uint mortonCodeMin = mortonCode & (maskMin);
    uint maskMax = 1 << (30 - prefixLength);
    uint mortonCodeMax = mortonCode | (maskMax - 1);

    float3 quantPosMin = float3(deinterleave_uint3(mortonCodeMin)) / quantLevels;
    float3 quantPosMax = float3(deinterleave_uint3(mortonCodeMax)) / quantLevels;

    float3 minPos = quantPosMin * sceneBound.extent() + sceneBound.minPoint;
    float3 maxPos = quantPosMax * sceneBound.extent() + sceneBound.minPoint;

    float3 random = sampleNext3D(sg);
    return minPos * random + maxPos * (float3(1.0f) - random);
}

UniformGrid traverseTreeForGird<S : ISampleGenerator>(const ShadingData sd, float3 dir, out float pdf, inout S sg)
{
    float3 pos = sd.posW;

    UniformGrid grid = {};
    uint currNodeId = 0;
    BVHInternalNode currNode = gInternalNodes[currNodeId];
    pdf = 1.f;
    uint depth = 0;
    while (true)
    {
        uint lChild = currNode.lChildId;
        uint2 lChildRange = currNode.range;
        bool isLChildLeaf = currNode.isLChildLeaf;
        float3 lChildIntensity = 0;
        float lChildDis = 0;
        float3 lChildPos = float3(0, 0, 0);
        float3 lChildDir = float3(0, 0, 0);
        if (isLChildLeaf)
        {
            lChildIntensity = gGrids[lChild].intensity;
            lChildPos = computePosByMortonCode(gGrids[lChild].mortonCode, gridMortonCodePrefixLength);
            // lChildPos = computeRandomPosByMortonCode(gGrids[lChild].mortonCode, gridMortonCodePrefixLength, sg);
            lChildDir = normalize(lChildPos - pos);
            lChildDis = length(lChildPos - pos);
        }
        else
        {
            lChildIntensity = gInternalNodes[lChild].intensity;
            lChildPos = computePosByMortonCode(gInternalNodes[lChild].splitMortonCode, gInternalNodes[lChild].commonPrefixLength);
            // lChildPos = computeRandomPosByMortonCode(gInternalNodes[lChild].splitMortonCode, gInternalNodes[lChild].commonPrefixLength, sg);
            lChildDir = normalize(lChildPos - pos);
            lChildDis = length(lChildPos - pos);
        }

        uint rChild = currNode.rChildId;
        uint2 rChildRange = currNode.range;
        bool isRChildLeaf = currNode.isRChildLeaf;
        float3 rChildIntensity = 0;
        float rChildDis = 0;
        float3 rChildPos = float3(0, 0, 0);
        float3 rChildDir = float3(0, 0, 0);
        if (isRChildLeaf)
        {
            rChildIntensity = gGrids[rChild].intensity;
            rChildPos = computePosByMortonCode(gGrids[rChild].mortonCode, gridMortonCodePrefixLength);
            // rChildPos = computeRandomPosByMortonCode(gGrids[rChild].mortonCode, gridMortonCodePrefixLength, sg);
            rChildDir = normalize(lChildPos - pos);
            rChildDis = length(lChildPos - pos);
        }
        else
        {
            rChildIntensity = gInternalNodes[rChild].intensity;
            rChildPos = computePosByMortonCode(gInternalNodes[rChild].splitMortonCode, gInternalNodes[rChild].commonPrefixLength);
            // rChildPos = computeRandomPosByMortonCode(gInternalNodes[rChild].splitMortonCode, gInternalNodes[rChild].commonPrefixLength, sg);
            rChildDir = normalize(rChildPos - pos);
            rChildDis = length(lChildPos - pos);
        }

        // choose one child
        bool isLeaf = false;
        float pL, pR;
        if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DistanceIntensity)
        {
            if (depth < 5)
            {
                pL = length(lChildIntensity) / lChildDis / lChildDis;
                pR = length(rChildIntensity) / rChildDis / rChildDis;
            }
            else
            {
                pL = 100.f / lChildDis / lChildDis;
                pR = 100.f / rChildDis / rChildDis;
            }
        }
        else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DirectionDistanceIntensity)
        {
            pL = length(lChildIntensity) * clamp(dot(dir, lChildDir), 0.0f, 1.f) / lChildDis / lChildDis;
            pR = length(rChildIntensity) * clamp(dot(dir, rChildDir), 0.0f, 1.f) / rChildDis / rChildDis;
        }
        else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::BRDFShading)
        {
            pL = length(evalBSDFCosine(sd, lChildDir) * lChildIntensity) / lChildDis / lChildDis;
            pR = length(evalBSDFCosine(sd, rChildDir) * rChildIntensity) / rChildDis / rChildDis;
        }
        else
        {
            pL = pR = 0.5f;
        }
        float p = max(pL + pR, 0.00001f);
        if (sampleNext1D(sg) < pL / p)
        {
            // Left child
            currNodeId = lChild;
            isLeaf = isLChildLeaf;
            pdf *= pL / p;
        }
        else
        {
            // Right child
            currNodeId = rChild;
            isLeaf = isRChildLeaf;
            pdf *= pR / p;
        }

        // stop if child is leaf
        if (isLeaf)
        {
            grid = gGrids[currNodeId];
            break;
        }
        else
        {
            currNode = gInternalNodes[currNodeId];
        }

        depth++;
    }

    return grid;
}

// ---------------- Octree Code ----------------- //

// BUG: If we use scene "AnimatedCubes", attribute "isLeaf" in root node will be true
//      but every thing works fine in other scenes.
StructuredBuffer<OctreeNode> gOctree;

cbuffer PerFrameOctreeCB
{
	uint octreeRootIndex;
}

void buildGridByOctreeNode(in OctreeNode octreeNode, out UniformGrid grid)
{
    grid.pos = octreeNode.pos;
    grid.intensity = octreeNode.intensity;
    grid.range = octreeNode.triangleRange;
    grid.gridIndex = octreeNode.nodeIdx;
}

/** Compute the near and far intersections of the cube (stored in the x and y components) using the slab method.
    No intersection means vec.x > vec.y (really tNear > tFar).
    https://gist.github.com/DomNomNom/46bb1ce47f68d255fd5d
    \param rayOrigin
    \param rayDir
    \param aabb
    \return A float2 with (tNear, tFar)
*/
float2 intersectWithBound(const float3 rayOrigin, const float3 rayDir, const AABB aabb)
{
    float3 tMin = (aabb.minPoint - rayOrigin) / rayDir;
    float3 tMax = (aabb.maxPoint - rayOrigin) / rayDir;
    float3 t1 = min(tMin, tMax);
    float3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);
    return float2(tNear, tFar);
}

float computeOctreeNodeWeightForTreeTraversal(const ShadingData sd, const OctreeNode node)
{
    float weight = 0.f;
    float3 dir = normalize(node.pos - sd.posW);
    float  distance = length(node.pos - sd.posW);
    if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DistanceIntensity)
    {
        weight = length(node.intensity) / distance / distance;
    }
    else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DirectionDistanceIntensity)
    {
        float cosine = dot(sd.N, normalize(node.pos - sd.posW));
        weight = length(node.intensity) * max(cosine, 0) / distance / distance;
    }
    else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::BRDFShading)
    {
        weight = length(evalBSDFCosine(sd, dir) * node.intensity) / distance / distance;
    }
    return weight;
}

OctreeNode traverseOctree<S : ISampleGenerator>(const ShadingData sd, const uint rootIndex, const uint leafCount, out float pdf, inout S sg)
{
    // initialize return values
    pdf = 1.f;

    uint currNodeIdx = rootIndex;
    while (true)
    {
        OctreeNode currNode = gOctree[currNodeIdx];

        if (currNode.isLeaf || currNode.leafRange.y - currNode.leafRange.x + 1 <= leafCount)
            return currNode;

        uint nextNodeIdx = currNodeIdx; // next child grid to traverse
        float nextNodePdf = 0.00001f;
        float weightSum = 0.00001f;
        for (int i = currNode.childRange.x; i <= currNode.childRange.y; i++)
        {
            OctreeNode child = gOctree[i];

            float childWeight = computeOctreeNodeWeightForTreeTraversal(sd, child);
            weightSum += childWeight;
            if (sampleNext1D(sg) < childWeight / weightSum)
            {
                nextNodeIdx = i;
                nextNodePdf = childWeight;
            }
        }
        nextNodePdf = nextNodePdf / weightSum;

        // can't find a child to traverse
        if (nextNodeIdx == currNodeIdx)
        {
            return currNode;
        }
        else
        {
            pdf *= nextNodePdf;
            currNodeIdx = nextNodeIdx;
        }
    }

    OctreeNode bugNode = gOctree[rootIndex];
    return bugNode; // return only if bug happens
}

OctreeNode findContainingGrid(AABB aabb)
{
    uint currNodeIdx = octreeRootIndex;
    for (int level = 0; level < 10; level++)
    {
        OctreeNode currNode = gOctree[currNodeIdx];

        if (currNode.isLeaf) return currNode;

        uint nextNodeIdx = currNodeIdx; // next child grid to traverse
        for (int i = currNode.childRange.x; i <= currNode.childRange.y; i++)
        {
            OctreeNode child = gOctree[i];
            AABB childAABB = computeAABBByMortonCode(child.mortonCode, child.prefixLength);
            // if childAABB contains aabb
            if (all(aabb.minPoint >= childAABB.minPoint) && all(aabb.maxPoint <= childAABB.maxPoint))
            {
                nextNodeIdx = i;
                break;
            }
        }

        // can't find a child containg this morton code
        if (nextNodeIdx == currNodeIdx) return currNode;
        else currNodeIdx = nextNodeIdx;
    }

    OctreeNode ret = {};
    return ret; // return root only if bug happens
}

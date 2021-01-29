#include "BVHNodeData.slangh"
#include "GridData.slangh"
import Scene.ShadingData;
import Experimental.Scene.Material.MaterialShading;
import Utils.Math.AABB;
import Utils.Math.BitTricks;
import Utils.Sampling.SampleGenerator; // TODO: remove this import
import ULGStaticParams; // TODO: remove this import

StructuredBuffer<BVHLeafNode> gLeafNodes;
StructuredBuffer<BVHInternalNode> gInternalNodes;

// TODO: better way to solve cbuffer name conflict
cbuffer PerFrameBVHCB
{
    uint gridMortonCodePrefixLength;
    uint quantLevels; // morton code dimension
    AABB sceneBound;
}

float3 computePosByMortonCode(uint mortonCode)
{
    float3 quantPos = float3(deinterleave_uint3(mortonCode)) / quantLevels;
    float3 pos = quantPos * sceneBound.extent() + sceneBound.minPoint;
    return pos;
}

float3 computePosByMortonCode(uint mortonCode, uint prefixLength)
{
    uint maskMin = 0xFFFFFFFF << (30 - prefixLength);
    uint mortonCodeMin = mortonCode & (maskMin);
    uint maskMax = 1 << (30 - prefixLength);
    uint mortonCodeMax = mortonCode | (maskMax - 1);

    float3 quantPosMin = float3(deinterleave_uint3(mortonCodeMin)) / quantLevels;
    float3 quantPosMax = float3(deinterleave_uint3(mortonCodeMax)) / quantLevels;

    float3 minPos = quantPosMin * sceneBound.extent() + sceneBound.minPoint;
    float3 maxPos = quantPosMax * sceneBound.extent() + sceneBound.minPoint;

    return (minPos + maxPos) * 0.5f;
}

void buildGridByInternalNode(in uint rootNode, out UniformGrid grid)
{
    grid.rootNode = rootNode;
    grid.isLeafNode = false;

    BVHInternalNode node = gInternalNodes[rootNode];
    grid.pos = computePosByMortonCode(node.splitMortonCode, gridMortonCodePrefixLength);
    grid.intensity = node.intensity;
    grid.range = node.range;
}

void buildGridByLeafNode(in uint rootNode, out UniformGrid grid)
{
    grid.rootNode = rootNode;
    grid.isLeafNode = true;

    BVHLeafNode node = gLeafNodes[rootNode];
    grid.pos = computePosByMortonCode(node.mortonCode, gridMortonCodePrefixLength);
    grid.intensity = node.intensity;
    grid.range = uint2(rootNode, rootNode);
}

/** Choose a nearest grid from BVH
    \param[in] morton code of current grid
    \return nearest grid

    TODO: remove this
*/
UniformGrid traverseTreeForGird<S : ISampleGenerator>(uint mortonCode, out float pdf, inout S sg)
{
    UniformGrid grid = {};
    uint currNodeId = 0;
    BVHInternalNode currNode = gInternalNodes[currNodeId];
    float3 pos = computePosByMortonCode(mortonCode);

    while (true)
    {
        // stop if current node size > grid size
        // we have 32-bit integer and 30-bit morton code
        if (currNode.commonPrefixLength > gridMortonCodePrefixLength + 2)
        {
            buildGridByInternalNode(currNodeId, grid);
            break;
        }

        // prepare l child
        uint lChild = currNode.lChildId;
        bool isLChildLeaf = currNode.isLChildLeaf;
        float lIntensity = 0.0f;
        float3 lChildPos = 0;
        if (isLChildLeaf)
        {
            lIntensity = length(gLeafNodes[lChild].intensity);
            lChildPos = computePosByMortonCode(gLeafNodes[lChild].mortonCode);
        }
        else
        {
            lIntensity = length(gInternalNodes[lChild].intensity);
            lChildPos = computePosByMortonCode(gInternalNodes[lChild].splitMortonCode, gInternalNodes[lChild].commonPrefixLength);
        }

        // prepare r child
        uint rChild = currNode.rChildId;
        bool isRChildLeaf = currNode.isRChildLeaf;
        float rIntensity = 0.0f;
        float3 rChildPos = 0;
        if (isRChildLeaf)
        {
            rIntensity = length(gLeafNodes[rChild].intensity);
            rChildPos = computePosByMortonCode(gLeafNodes[rChild].mortonCode);
        }
        else
        {
            rIntensity = length(gInternalNodes[rChild].intensity);
            rChildPos = computePosByMortonCode(gInternalNodes[rChild].splitMortonCode, gInternalNodes[rChild].commonPrefixLength);
        }

        // choose one child
        bool isLeaf = false;

        if (mortonCode <= currNode.splitMortonCode)
        {
            // Left child
            currNodeId = lChild;
            isLeaf = isLChildLeaf;
        }
        else
        {
            // Right child
            currNodeId = rChild;
            isLeaf = isRChildLeaf;
        }

        // stop if child is leaf
        if (isLeaf)
        {
            buildGridByLeafNode(currNodeId, grid);
            break;
        }
        else
        {
            currNode = gInternalNodes[currNodeId];
        }
    }
    pdf = 1.f;
    return grid;
}

UniformGrid traverseTreeForGird<S : ISampleGenerator>(const ShadingData sd, float3 dir, out float pdf, inout S sg)
{
    float3 pos = sd.posW;

    UniformGrid grid = {};
    uint currNodeId = 0;
    BVHInternalNode currNode = gInternalNodes[currNodeId];
    pdf = 1.f;
    while (true)
    {
        // stop if current node size > grid size
        // we have 32-bit integer and 30-bit morton code
        if (currNode.commonPrefixLength > gridMortonCodePrefixLength + 2)
        {
            buildGridByInternalNode(currNodeId, grid);
            break;
        }

        // TODO: add struct for childs
        // TODO: add function for childs
        uint lChild = currNode.lChildId;
        bool isLChildLeaf = currNode.isLChildLeaf;
        float3 lChildIntensity = 0;
        float lChildDis = 0;
        float3 lChildPos = float3(0, 0, 0);
        float3 lChildDir = float3(0, 0, 0);
        if (isLChildLeaf)
        {
            lChildIntensity = gLeafNodes[lChild].intensity;
            lChildPos = computePosByMortonCode(gLeafNodes[lChild].mortonCode, gridMortonCodePrefixLength);
            // lChildPos = computePosByMortonCode(gLeafNodes[lChild].mortonCode);
            lChildDir = normalize(lChildPos - pos);
            lChildDis = length(lChildPos - pos);
        }
        else
        {
            lChildIntensity = gInternalNodes[lChild].intensity;
            lChildPos = computePosByMortonCode(gInternalNodes[lChild].splitMortonCode, gInternalNodes[lChild].commonPrefixLength);
            // lChildPos = computePosByMortonCode(gInternalNodes[lChild].splitMortonCode);
            lChildDir = normalize(lChildPos - pos);
            lChildDis = length(lChildPos - pos);
        }

        uint rChild = currNode.rChildId;
        bool isRChildLeaf = currNode.isRChildLeaf;
        float3 rChildIntensity = 0;
        float rChildDis = 0;
        float3 rChildPos = float3(0, 0, 0);
        float3 rChildDir = float3(0, 0, 0);
        if (isRChildLeaf)
        {
            rChildIntensity = gLeafNodes[rChild].intensity;
            rChildPos = computePosByMortonCode(gLeafNodes[rChild].mortonCode, gridMortonCodePrefixLength);
            // rChildPos = computePosByMortonCode(gLeafNodes[rChild].mortonCode);
            rChildDir = normalize(lChildPos - pos);
            rChildDis = length(lChildPos - pos);
        }
        else
        {
            rChildIntensity = gInternalNodes[rChild].intensity;
            rChildPos = computePosByMortonCode(gInternalNodes[rChild].splitMortonCode, gInternalNodes[rChild].commonPrefixLength);
            // rChildPos = computePosByMortonCode(gInternalNodes[rChild].splitMortonCode);
            rChildDir = normalize(rChildPos - pos);
            rChildDis = length(lChildPos - pos);
        }
        // choose one child
        bool isLeaf = false;
        float pL, pR;
        // TODO: make a function for weigh type select
        if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DistanceIntensity)
        {
            pL = length(lChildIntensity) / lChildDis / lChildDis;
            pR = length(rChildIntensity) / rChildDis / rChildDis;
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
            buildGridByLeafNode(currNodeId, grid);
            break;
        }
        else
        {
            currNode = gInternalNodes[currNodeId];
        }
    }

    return grid;
}

/** Choose a light from BVH
    \param[in] staring node for traversal
    \return a triangle index in subtree
*/
uint traverseTreeForLight(uint root)
{
    // TODO: we implemented it in SelectGridAndLight.cs.slang > chooseTriangle
    return 0;
}

// ---------------- Octree Code ----------------- //
// TODO: find a better place

StructuredBuffer<OctreeNode> gOctree;

// TODO: better way to solve cbuffer name conflict
cbuffer PerFrameOctreeCB
{
	uint octreeRootIndex;
}

bool checkPrefix(uint mortonCode1, uint mortonCode2, uint prefixLength)
{
    uint mask = 0xFFFFFFFF << (30 - prefixLength);
    return (mortonCode1 & mask) == (mortonCode2 & mask);
}

void buildGridByGridNode(in OctreeNode gridNode, out UniformGrid grid)
{
    grid.pos = gridNode.pos;
    grid.intensity = gridNode.intensity;
    grid.range = gridNode.triangleRange;
    grid.rootNode = 0;
    grid.isLeafNode = true;
}

// TODO: remove duplicate code with computeBoundByMortonCode
AABB computeAABBByMortonCode(uint mortonCode, uint prefixLength)
{
    uint maskMin = 0xFFFFFFFF << (30 - prefixLength);
    uint mortonCodeMin = mortonCode & (maskMin);
    uint maskMax = 1 << (30 - prefixLength);
    uint mortonCodeMax = mortonCode | (maskMax - 1);

    float3 quantPosMin = float3(deinterleave_uint3(mortonCodeMin)) / quantLevels;
    float3 quantPosMax = float3(deinterleave_uint3(mortonCodeMax)) / quantLevels;

    float3 minPos = quantPosMin * sceneBound.extent() + sceneBound.minPoint;
    float3 maxPos = quantPosMax * sceneBound.extent() + sceneBound.minPoint;

    AABB aabb;
    aabb.minPoint = minPos;
    aabb.maxPoint = maxPos;
    return aabb;
}

/** Compute the near and far intersections of the cube (stored in the x and y components) using the slab method.
    No intersection means vec.x > vec.y (really tNear > tFar).
    https://gist.github.com/DomNomNom/46bb1ce47f68d255fd5d
    TODO: combine duplicate code in SelectGridAndLight.cs.slang
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

float computeOctreeNodeWeight(const ShadingData sd, const OctreeNode node)
{
    float weight = 0.f;
    float3 dir = normalize(node.pos - sd.posW);
    float  distance = length(node.pos - sd.posW);
    if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DistanceIntensity)
    {
        weight = length(node.intensity) / distance / distance;
    }
    else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::BRDFShading)
    {
        weight = length(evalBSDFCosine(sd, dir) * node.intensity) / distance / distance;
    }
    return weight;
}

UniformGrid traverseOctree<S : ISampleGenerator>(const ShadingData sd, out float pdf, inout S sg)
{
    UniformGrid grid = {};
    pdf = 1.f;

    uint currNodeIdx = octreeRootIndex;
    while (true)
    {
        OctreeNode currNode = gOctree[currNodeIdx];

        if (currNode.isLeaf)
        {
            buildGridByGridNode(currNode, grid);
            break;
        }

        uint nextNodeIdx = currNodeIdx; // next child grid to traverse
        float nextNodePdf = 0.00001f;
        float weightSum = 0.00001f;
        for (int i = currNode.childRange.x; i <= currNode.childRange.y; i++)
        {
            OctreeNode child = gOctree[i];

            // float childDistance = max(distance(child.pos, sd.posW), 0.0001f);
            // float childWeight = length(child.intensity) / childDistance / childDistance;
            float childWeight = computeOctreeNodeWeight(sd, child);
            
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
            buildGridByGridNode(currNode, grid);
            break;
        }
        else
        {
            pdf *= nextNodePdf;
            currNodeIdx = nextNodeIdx;
        }
    }

    return grid;
}

UniformGrid traverseOctree<S : ISampleGenerator>(const ShadingData sd, float3 dir, out float pdf, inout S sg)
{
    UniformGrid grid = {};
    pdf = 1.f;


    uint currNodeIdx = octreeRootIndex;
    while (true)
    {
        OctreeNode currNode = gOctree[currNodeIdx];

        if (currNode.isLeaf)
        {
            buildGridByGridNode(currNode, grid);
            break;
        }

        uint nextNodeIdx = currNodeIdx; // next child grid to traverse
        float nextNodePdf = 0.00001f;
        float weightSum = 0.00001f;
        int   candidateCount = 0;
        for (int i = currNode.childRange.x; i <= currNode.childRange.y; i++)
        {
            OctreeNode child = gOctree[i];
            AABB childBound = computeAABBByMortonCode(child.mortonCode, child.prefixLength);
            float2 intersectT = intersectWithBound(sd.posW, dir, childBound);

            float childWeight = 0.f;
            float3 intersectPoint = sd.posW + dir * (intersectT.x + intersectT.y) * 0.5f;
            // Strategy 1: compute distance to intersection point
            // float childDistance = max(distance(child.pos, intersectPoint), 0.0001f);
            // Strategy 2: compute distance to shading point
            float childDistance = max(distance(child.pos, sd.posW), 0.0001f);
            float3 bsdf = evalBSDFCosine(sd, normalize(child.pos - sd.posW));
            if (intersectT.x <= intersectT.y) // if intersect
            {
                childWeight = length(bsdf * child.intensity) / childDistance / childDistance;
                // childWeight = length(child.intensity) / childDistance / childDistance;
                candidateCount++;
            }

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
            buildGridByGridNode(currNode, grid);
            break;
        }
        else
        {
            pdf *= nextNodePdf;
            currNodeIdx = nextNodeIdx;
        }
    }

    return grid;
}

UniformGrid findContainingGird(uint mortonCode)
{
    UniformGrid grid = {};

    uint currNodeIdx = octreeRootIndex;
    while (true)
    {
        OctreeNode currNode = gOctree[currNodeIdx];

        if (currNode.isLeaf)
        {
            buildGridByGridNode(currNode, grid);
            break;
        }

        uint nextNodeIdx = currNodeIdx; // next child grid to traverse
        for (int i = currNode.childRange.x; i <= currNode.childRange.y; i++)
        {
            OctreeNode child = gOctree[i];
            if (checkPrefix(child.mortonCode, mortonCode, child.prefixLength))
            {
                nextNodeIdx = i;
                break;
            }
        }

        // can't find a child containg this morton code
        if (nextNodeIdx == currNodeIdx)
        {
            buildGridByGridNode(currNode, grid);
            break;
        }
        else
            currNodeIdx = nextNodeIdx;
    }

    return grid;
}

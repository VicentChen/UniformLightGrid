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
*/
UniformGrid traverseTreeForGird(uint mortonCode)
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

        // >>> Stragety 1: traverse by morton code
        if (mortonCode <= currNode.splitMortonCode)
        // >>> Stragety 2: traverse by position
        // if (distance(pos, lChildPos) < distance(pos, rChildPos))
        // >>> Stragety 3: traverse by intensity & position
        // if (lIntensity / distance(pos, lChildPos) / distance(pos, lChildPos) > rIntensity / distance(pos, rChildPos) / distance(pos, rChildPos))
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
        if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::Direction)
        {
            pL = clamp(dot(dir, lChildDir), 0.0f, 1.f);
            pR = clamp(dot(dir, rChildDir), 0.0f, 1.f);
        }
        else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DistanceIntensity)
        {
            pL = length(lChildIntensity) / lChildDis / lChildDis;
            pR = length(rChildIntensity) / rChildDis / rChildDis;
        }
        else if (kTreeTraverseWeightType == (uint)TreeTraverseWeightType::DirectionDistanceIntensity)
        {
            pL = length(lChildIntensity) * clamp(dot(dir, lChildDir), 0.0f, 1.f) / lChildDis / lChildDis;
            pR = length(rChildIntensity) * clamp(dot(dir, rChildDir), 0.0f, 1.f) / rChildDis / rChildDis;
            // pL = length(lChildIntensity) * clamp(dot(sd.N, lChildDir), 0.0f, 1.f) / lChildDis / lChildDis;
            // pR = length(rChildIntensity) * clamp(dot(sd.N, rChildDir), 0.0f, 1.f) / rChildDis / rChildDis;
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
        //if (pL > pR)
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
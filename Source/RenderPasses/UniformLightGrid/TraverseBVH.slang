#include "BVHNodeData.slangh"
#include "GridData.slangh"
import Utils.Math.AABB;
import Utils.Math.BitTricks;

StructuredBuffer<LeafNode> gLeafNodes;
StructuredBuffer<InternalNode> gInternalNodes;

cbuffer PerFrameCB
{
    uint gridMortonCodePrefixLength;
    uint quantLevels; // morton code dimension
    AABB sceneBound;
}

float3 computePosByMortonCode(uint mortonCode)
{
    uint maskMin = 0xFFFFFFFF << (30 - gridMortonCodePrefixLength);
    uint mortonCodeMin = mortonCode & (maskMin);
    uint maskMax = 1 << (30 - gridMortonCodePrefixLength);
    uint mortonCodeMax = mortonCode | (maskMax - 1);

    float3 quantPosMin = float3(deinterleave_uint3(mortonCodeMin)) / quantLevels;
    float3 quantPosMax = float3(deinterleave_uint3(mortonCodeMax)) / quantLevels;

    float3 minPos = quantPosMin * sceneBound.extent() + sceneBound.minPoint;
    float3 maxPos = quantPosMax * sceneBound.extent() + sceneBound.minPoint;

    return (minPos + maxPos) * 0.5f;
}

void buildGrid(in uint rootNode, in bool isLeafNode, out Grid grid)
{
    grid.rootNode = rootNode;
    grid.isLeafNode = isLeafNode;

    if (isLeafNode)
    {
        LeafNode node = gLeafNodes[rootNode];
        grid.pos = computePosByMortonCode(node.mortonCode);
        grid.intensity = node.intensity;
        grid.range = uint2(rootNode, rootNode);
    }
    else
    {
        InternalNode node = gInternalNodes[rootNode];
        grid.pos = computePosByMortonCode(node.splitMortonCode);
        grid.intensity = node.intensity;
        grid.range = node.range;
    }
}

/** Choose a nearest grid from BVH
    \param[in] morton code of current grid
    \return nearest grid
*/
Grid traverseTreeForGird(uint mortonCode)
{
    Grid grid = {};
    uint currNodeId = 0;
    InternalNode currNode = gInternalNodes[currNodeId];
    while (true)
    {
        // stop if current node size > grid size
        // we have 32-bit integer and 30-bit morton code
        if (currNode.commonPrefixLength > gridMortonCodePrefixLength + 2)
        {
            buildGrid(currNodeId, false, grid);
            break;
        }

        // choose one child
        bool isLeaf = false;
        if (mortonCode <= currNode.splitMortonCode)
        {
            // Left child
            currNodeId = currNode.lChildId;
            isLeaf = currNode.isLChildLeaf;
        }
        else
        {
            // Right child
            currNodeId = currNode.rChildId;
            isLeaf = currNode.isRChildLeaf;
        }

        // stop if child is leaf
        if (isLeaf)
        {
            buildGrid(currNodeId, true, grid);
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
    // TODO:
    return 0;
}
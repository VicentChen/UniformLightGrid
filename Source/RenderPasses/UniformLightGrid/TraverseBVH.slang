#include "BVHNodeData.slangh"
#include "GridData.slangh"
import Utils.Math.AABB;
import Utils.Math.BitTricks;

StructuredBuffer<LeafNode> gLeafNodes;
StructuredBuffer<InternalNode> gInternalNodes;

cbuffer PerFrameBVHCB
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

void buildGridByInternalNode(in uint rootNode, out UniformGrid grid)
{
    grid.rootNode = rootNode;
    grid.isLeafNode = false;

    InternalNode node = gInternalNodes[rootNode];
    grid.pos = computePosByMortonCode(node.splitMortonCode);
    grid.intensity = node.intensity;
    grid.range = node.range;
}

void buildGridByLeafNode(in uint rootNode, out UniformGrid grid)
{
    grid.rootNode = rootNode;
    grid.isLeafNode = true;

    LeafNode node = gLeafNodes[rootNode];
    grid.pos = computePosByMortonCode(node.mortonCode);
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
    InternalNode currNode = gInternalNodes[currNodeId];

    while (true)
    {
        // stop if current node size > grid size
        // we have 32-bit integer and 30-bit morton code
        if (currNode.commonPrefixLength > gridMortonCodePrefixLength + 2)
        {
            buildGridByInternalNode(currNodeId, grid);
            break;
        }

        uint lChild = currNode.lChildId;
        bool isLChildLeaf = currNode.isLChildLeaf;
        uint rChild = currNode.rChildId;
        bool isRChildLeaf = currNode.isRChildLeaf;

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

    return grid;
}

// /** Choose a nearest grid from BVH
//     \param[in] morton code of current grid
//     \return nearest grid
// */
// UniformGrid traverseTreeForGird(uint mortonCode)
// {
//     UniformGrid grid = {};
//     uint currNodeId = 0;
//     InternalNode currNode = gInternalNodes[currNodeId];
//     while (true)
//     {
//         // stop if current node size > grid size
//         // we have 32-bit integer and 30-bit morton code
//         if (currNode.commonPrefixLength >= gridMortonCodePrefixLength + 2)
//         {
//             buildGridByInternalNode(currNodeId, grid);
//             break;
//         }

//         // choose one child
//         bool isLeaf = false;
//         if (mortonCode <= currNode.splitMortonCode)
//         {
//             // Left child
//             currNodeId = currNode.lChildId;
//             isLeaf = currNode.isLChildLeaf;
//         }
//         else
//         {
//             // Right child
//             currNodeId = currNode.rChildId;
//             isLeaf = currNode.isRChildLeaf;
//         }

//         // stop if child is leaf
//         if (isLeaf)
//         {
//             buildGridByLeafNode(currNodeId, grid);
//             break;
//         }
//         else
//         {
//             currNode = gInternalNodes[currNodeId];
//         }
//     }

//     return grid;
// }

/** Choose a light from BVH
    \param[in] staring node for traversal
    \return a triangle index in subtree
*/
uint traverseTreeForLight(uint root)
{
    // TODO:
    return 0;
}
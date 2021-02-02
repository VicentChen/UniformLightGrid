#include "Octree.h"
#include "UniformLightGridCommon.h"

void Octree::buildOctree(RenderContext* pContext, const std::vector<BVHLeafNode>& points, const Params& params)
{
    assert(points.size() > 0);
    assert(params.leafNodePrefixLength % 3 == 0);
    mParams = params;

    mOctree.clear(); // must clear this to avoid
    mergePoints(points); // generate leaf nodes
    mergeNodes(); // generate internal nodes
    
    createAndCopyBuffer(pContext, mpOctreeBuffer, mpOctreeStagingBuffer, sizeof(OctreeNode), (uint)mOctree.size(), mOctree.data(), "ULG::mpOctreeBuffer", "ULG::mpOctreeBufferHelper");
}

void Octree::visualize(RenderContext* pContext, GraphicsState* pState, GraphicsVars* pVars)
{
    PROFILE("visualizeOctree");
    // TODO: visualize octree
}

void Octree::mergePoints(const std::vector<BVHLeafNode>& points)
{
    // TODO: too many duplicate code
    for (size_t i = 0; i < points.size();)
    {
        uint mask = ~(0xFFFFFFFF << (30 - mParams.leafNodePrefixLength));
        uint bound = points[i].mortonCode | mask;

        float3 intensity = float3(0, 0, 0);

        size_t beginIdx = i;
        while (i < points.size() && points[i].mortonCode <= bound)
        {
            intensity += points[i].intensity;
            i++;
        }
        size_t endIdx = i - 1;

        OctreeNode node;
        node.pos = computePosByMortonCode(bound, mParams.leafNodePrefixLength, (float)mParams.quantLevel, mParams.sceneBound);
        node.mortonCode = bound;
        node.intensity = intensity;
        node.childRange = uint2(beginIdx, endIdx);
        node.triangleRange = uint2(beginIdx, endIdx);
        node.isLeaf = true;
        node.prefixLength = mParams.leafNodePrefixLength;
        node.paddingAndDebug = float3(mOctree.size(), 0, 0);

        mOctree.emplace_back(node);
    }
}

void Octree::mergeNodes()
{
    // TODO: too many duplicate code
    size_t prevLevelBegin = 0, prevLevelEnd = mOctree.size();
    for (uint currPrefixLength = mParams.leafNodePrefixLength - 3; currPrefixLength > 0; currPrefixLength -= 3)
    {
        if (prevLevelEnd - prevLevelBegin <= 1) break; // early end if previous level has only one grid

        for (size_t i = prevLevelBegin; i < prevLevelEnd;)
        {
            uint mask = ~(0xFFFFFFFF << (30 - currPrefixLength));
            uint bound = mOctree[i].mortonCode | mask;

            float3 intensity = float3(0, 0, 0);

            size_t beginIdx = i;
            while (i < prevLevelEnd && mOctree[i].mortonCode <= bound)
            {
                intensity += mOctree[i].intensity;
                i++;
            }
            size_t endIdx = i - 1;

            OctreeNode grid;
            grid.pos = computePosByMortonCode(bound, currPrefixLength, (float)mParams.quantLevel, mParams.sceneBound);
            grid.intensity = intensity;
            grid.mortonCode = bound;
            grid.childRange = uint2(beginIdx, endIdx);
            grid.triangleRange = uint2(mOctree[beginIdx].triangleRange.x, mOctree[endIdx].triangleRange.y); // merge two range
            grid.isLeaf = false;
            grid.prefixLength = currPrefixLength;
            grid.paddingAndDebug = float3(mOctree.size(), 0, 0);

            mOctree.emplace_back(grid);
        }
        prevLevelBegin = prevLevelEnd;
        prevLevelEnd = mOctree.size();
    }
}

// ----- Backup codes ------ //

//void UniformLightGrid::generateOctree(RenderContext* pRenderContext)
//{
//    assert(mLeafNodes.size() > 0);
//
//    Octree::Params octreeParams;
//    octreeParams.leafNodePrefixLength = mGridAndLightSelectorParams.gridMortonCodePrefixLength;
//    octreeParams.quantLevel = kQuantLevels;
//    octreeParams.sceneBound = sceneBoundHelper();
//    mOctree.buildOctree(pRenderContext, mLeafNodes, octreeParams);
//
//    //PROFILE("ULG_generateOctree");
//
//    AABB sceneBound = sceneBoundHelper();
//
//    mOctreeNodes.clear();
//    // generate leaves
//    for (size_t i = 0; i < mLeafNodes.size();)
//    {
//        uint mask = ~(0xFFFFFFFF << (30 - mGridAndLightSelectorParams.gridMortonCodePrefixLength));
//        uint bound = mLeafNodes[i].mortonCode | mask;
//
//        float3 intensity = float3(0, 0, 0);
//
//        size_t beginIdx = i;
//        while (i < mLeafNodes.size() && mLeafNodes[i].mortonCode <= bound)
//        {
//            intensity += mLeafNodes[i].intensity;
//            i++;
//        }
//        size_t endIdx = i - 1;
//
//        OctreeNode grid;
//        grid.pos = computePosByMortonCode(bound, mGridAndLightSelectorParams.gridMortonCodePrefixLength, kQuantLevels, sceneBound);
//        grid.mortonCode = bound;
//        grid.intensity = intensity;
//        grid.childRange = uint2(beginIdx, endIdx);
//        grid.triangleRange = uint2(beginIdx, endIdx);
//        grid.isLeaf = true;
//        grid.prefixLength = mGridAndLightSelectorParams.gridMortonCodePrefixLength;
//        grid.paddingAndDebug = float3(mOctreeNodes.size(), 0, 0);
//
//        mOctreeNodes.emplace_back(grid);
//    }
//
//    // generate internal nodes
//    // TODO: too many duplicate code
//    size_t prevLevelBegin = 0, prevLevelEnd = mOctreeNodes.size();
//    for (uint currPrefixLength = mGridAndLightSelectorParams.gridMortonCodePrefixLength - 3; currPrefixLength > 0; currPrefixLength -= 3)
//    {
//        if (prevLevelEnd - prevLevelBegin <= 1) break; // early end if previous level has only one grid
//
//        for (size_t i = prevLevelBegin; i < prevLevelEnd;)
//        {
//            uint mask = ~(0xFFFFFFFF << (30 - currPrefixLength));
//            uint bound = mOctreeNodes[i].mortonCode | mask;
//
//            float3 intensity = float3(0, 0, 0);
//
//            size_t beginIdx = i;
//            while (i < prevLevelEnd && mOctreeNodes[i].mortonCode <= bound)
//            {
//                intensity += mOctreeNodes[i].intensity;
//                i++;
//            }
//            size_t endIdx = i - 1;
//
//            OctreeNode grid;
//            grid.pos = computePosByMortonCode(bound, currPrefixLength, kQuantLevels, sceneBound);
//            grid.intensity = intensity;
//            grid.mortonCode = bound;
//            grid.childRange = uint2(beginIdx, endIdx);
//            grid.triangleRange = uint2(mOctreeNodes[beginIdx].triangleRange.x, mOctreeNodes[endIdx].triangleRange.y); // merge two range
//            grid.isLeaf = false;
//            grid.prefixLength = currPrefixLength;
//            grid.paddingAndDebug = float3(mOctreeNodes.size(), 0, 0);
//
//            mOctreeNodes.emplace_back(grid);
//        }
//        prevLevelBegin = prevLevelEnd;
//        prevLevelEnd = mOctreeNodes.size();
//    }
//
//    createAndCopyBuffer(pRenderContext, mpOctreeDataBuffer, mpOctreeDataStagingBuffer, sizeof(OctreeNode), (uint)mOctreeNodes.size(), mOctreeNodes.data(), "ULG::mpOctreeDataBuffer", "ULG::mpOctreeDataBufferHelper");
//}

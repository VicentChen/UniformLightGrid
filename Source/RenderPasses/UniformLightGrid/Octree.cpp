#include "Octree.h"
#include "UniformLightGridCommon.h"

void Octree::buildOctree(RenderContext* pContext, const std::vector<BVHLeafNode>& points, const Params& params)
{
    assert(points.size() > 0);
    assert(params.leafNodePrefixLength % 3 == 0);
    mParams = params;

    clearCpuBuffers();
    mergePoints(points); // generate leaf nodes
    mergeNodes(); // generate internal nodes
    updateDrawParams();
    
    createAndCopyBuffer(pContext, mpOctreeBuffer, mpOctreeStagingBuffer, sizeof(OctreeNode), (uint)mOctree.size(), mOctree.data(), "ULG::mpOctreeBuffer", "ULG::mpOctreeStagingBuffer");
    createAndCopyBuffer(pContext, mpOctreeNodeWorldMatBuffer, mpOctreeNodeWorldMatStagingBuffer, sizeof(float4x4), (uint)mOctreeNodeWorldMats.size(), mOctreeNodeWorldMats.data(), "ULG::mpOctreeNodeWorldMatBuffer", "ULG::mpOctreeNodeWorldMatStagingBuffer");
}

void Octree::visualize(RenderContext* pContext, Scene::SharedPtr pScene, const RenderData& renderData)
{
    PROFILE("visualizeOctree");

    prepareResources(pContext, renderData);
    
    Texture::SharedPtr posTexture = renderData["posW"]->asTexture();
    Texture::SharedPtr colorTexture = renderData["color"]->asTexture(); 

    Camera::SharedPtr pCamera = pScene->getCamera();
    mpProgramVars["PerFrameCB"]["camPos"] = pCamera->getPosition();
    mpProgramVars["PerFrameCB"]["viewMat"] = pCamera->getViewMatrix();
    mpProgramVars["PerFrameCB"]["projMat"] = pCamera->getProjMatrix();

    mpProgramVars["gPosW"] = posTexture;
    mpProgramVars["gColor"] = colorTexture;
    mpProgramVars["gInstanceWorldMats"] = mpOctreeNodeWorldMatBuffer;

    // TODO: hope this will not bug
    assert(mDrawParams.drawBeginLevel <= mDrawParams.drawEndLevel);
    uint instanceBeginIdx = mLevelIndex[mDrawParams.drawBeginLevel].first;
    uint instanceCount = (mDrawParams.drawEndLevel < mLevelIndex.size() ? mLevelIndex[mDrawParams.drawEndLevel].second : mLevelIndex.back().second) - instanceBeginIdx;
    assert(instanceBeginIdx >= 0 && instanceCount > 0);
    mpProgramVars["PerFrameCB"]["instanceOffset"] = instanceBeginIdx; // Seems that we can't pass this paramater in `drawInstanced(), startInstanceLocation`

    pContext->drawInstanced(mpGraphicsState.get(), mpProgramVars.get(), mpVertexBuffer->getElementCount(), instanceCount, 0u, 0);
}

void Octree::renderUI(Gui::Widgets& widget)
{
    // TODO: may be we need to choose some grid to visualize
    auto octreeWidget = widget.group("octree options", true);
    octreeWidget.text("NOTE: 0 is leaf node, 9 is root node");
    octreeWidget.text("Octree level count:" + std::to_string(mLevelIndex.size()));
    octreeWidget.var("draw begin level", mDrawParams.drawBeginLevel, 0ull, mLevelIndex.size()-1, 1ull);
    octreeWidget.var("draw end level", mDrawParams.drawEndLevel, mDrawParams.drawBeginLevel, mLevelIndex.size()-1, 1ull);
    if (mDrawParams.drawBeginLevel > mDrawParams.drawEndLevel) mDrawParams.drawBeginLevel = mDrawParams.drawEndLevel;
}

void Octree::clearCpuBuffers()
{
    mOctree.clear();
    mOctreeNodeWorldMats.clear();
    mLevelIndex.clear();
}

float4x4 Octree::computeNodeWorldMat(uint mortonCode, uint prefixLength)
{
    // TODO: make as inline
    // TODO: check if matrices correct
    AABB aabb = computeAABBByMortonCode(mortonCode, prefixLength, (float)mParams.quantLevel, mParams.sceneBound);
    float3 centerPos = aabb.center();
    float3 extent = aabb.extent();
    return glm::scale(glm::translate(glm::mat4(), centerPos), extent);
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
        mOctreeNodeWorldMats.emplace_back(computeNodeWorldMat(bound, mParams.leafNodePrefixLength));
    }
}

void Octree::mergeNodes()
{
    // TODO: too many duplicate code
    size_t prevLevelBegin = 0, prevLevelEnd = mOctree.size();
    mLevelIndex.emplace_back(std::make_pair((uint)prevLevelBegin, (uint)(prevLevelEnd)));
    // We still need to calculate the biggest node(node with prefix length == 0)
    for (uint currPrefixLength = mParams.leafNodePrefixLength - 3; currPrefixLength >= 0; currPrefixLength -= 3)
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
            mOctreeNodeWorldMats.emplace_back(computeNodeWorldMat(bound, currPrefixLength));
        }
        prevLevelBegin = prevLevelEnd;
        prevLevelEnd = mOctree.size();
        mLevelIndex.emplace_back(std::make_pair((uint)prevLevelBegin, (uint)(prevLevelEnd)));
    }
}

void Octree::prepareResources(RenderContext* pContext, const RenderData& renderData)
{
    if (!mpProgram)
    {
        // prepare programs
        mpProgram = GraphicsProgram::createFromFile("RenderPasses/UniformLightGrid/VisualizeOctree.3d.slang", "vsMain", "psMain");
        mpGraphicsState = GraphicsState::create();
        mpGraphicsState->setProgram(mpProgram);
        mpProgramVars = GraphicsVars::create(mpProgram->getReflector());

        // prepare rasterize state
        RasterizerState::Desc lineDesc;
        lineDesc.setFillMode(RasterizerState::FillMode::Solid);
        lineDesc.setCullMode(RasterizerState::CullMode::None);
        mpRasterizerState = RasterizerState::create(lineDesc);
        DepthStencilState::Desc dsDesc;
        dsDesc.setDepthEnabled(false);
        mpDepthStencilState = DepthStencilState::create(dsDesc);

        // prepare vertex buffer
        //        Y             
        //        |             
        //        3------2      float3(-0.5, -0.5, -0.5), // 0
        //       /|     /|      float3( 0.5, -0.5, -0.5), // 1
        //      / |    / |      float3( 0.5,  0.5, -0.5), // 2
        //     7--|---6  |      float3(-0.5,  0.5, -0.5), // 3
        //     |  0---|--1--X   float3(-0.5, -0.5,  0.5), // 4
        //     | /    | /       float3( 0.5, -0.5,  0.5), // 5
        //     |/     |/        float3( 0.5,  0.5,  0.5), // 6
        //     4------5         float3(-0.5,  0.5,  0.5), // 7
        //    /                 
        //   Z                  
        std::vector<float3> verteices({
            //float3(0.0, 0.0, 1.0), float3(1, 1, 1);
            float3(-0.5, -0.5, -0.5), float3( 0.5, -0.5, -0.5), // 0-1
            float3( 0.5, -0.5, -0.5), float3( 0.5,  0.5, -0.5), // 1-2
            float3( 0.5,  0.5, -0.5), float3(-0.5,  0.5, -0.5), // 2-3
            float3(-0.5,  0.5, -0.5), float3(-0.5, -0.5, -0.5), // 3-0
            float3(-0.5, -0.5,  0.5), float3( 0.5, -0.5,  0.5), // 4-5
            float3( 0.5, -0.5,  0.5), float3( 0.5,  0.5,  0.5), // 5-6
            float3( 0.5,  0.5,  0.5), float3(-0.5,  0.5,  0.5), // 6-7
            float3(-0.5,  0.5,  0.5), float3(-0.5, -0.5,  0.5), // 7-4
            float3(-0.5, -0.5, -0.5), float3(-0.5, -0.5,  0.5), // 0-4
            float3( 0.5, -0.5, -0.5), float3( 0.5, -0.5,  0.5), // 1-5
            float3( 0.5,  0.5, -0.5), float3( 0.5,  0.5,  0.5), // 2-6
            float3(-0.5,  0.5, -0.5), float3(-0.5,  0.5,  0.5), // 3-7
        });

        createAndCopyBuffer(pContext, mpVertexBuffer, mpVertexStagingBuffer, sizeof(float3), (uint)verteices.size(), verteices.data(), "ULG::mpOctreeVertexBuffer", "ULG::mpOctreeVertexStagingBuffer");

        // prepare vao (TODO: what the hell this means?)
        std::vector<Buffer::SharedPtr> bufferPtrs = { mpVertexBuffer };
        VertexBufferLayout::SharedPtr pBufferLayout = VertexBufferLayout::create();
        pBufferLayout->addElement("POSITION", 0, ResourceFormat::RGB32Float, 1, 0);
        VertexLayout::SharedPtr pLayout = VertexLayout::create();
        pLayout->addBufferLayout(0, pBufferLayout);
        mpVao = Vao::create(Vao::Topology::LineList, pLayout, bufferPtrs);
        mpGraphicsState->setVao(mpVao);

        // prepare fbo
        Texture::SharedPtr pOutput = renderData["color"]->asTexture();
        mpDepthStencil = Texture::create2D(1920, 1080, ResourceFormat::D24UnormS8, 1, 1, nullptr, ResourceBindFlags::DepthStencil | ResourceBindFlags::ShaderResource);
        mpFbo = Fbo::create({ pOutput }, mpDepthStencil);

        mpGraphicsState->setFbo(mpFbo);
    }

    mpGraphicsState->setRasterizerState(mpRasterizerState);
    mpGraphicsState->setDepthStencilState(mpDepthStencilState);
}

void Octree::updateDrawParams()
{
    if (mDrawParams.drawEndLevel >= mLevelIndex.size())
        mDrawParams.drawEndLevel = mLevelIndex.size() - 1;
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

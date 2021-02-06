#include "Octree.h"

namespace 
{
    const std::string kShaderFile = "RenderPasses/UniformLightGrid/VisualizeOctree.3d.slang";
    const std::string kVertexShader = "vsMain";
    const std::string kPixelShader = "psMain";

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
        const std::vector<float3> kWiredCubeVertices({
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
}

void Octree::buildOctree(RenderContext* pContext, const std::vector<BVHLeafNode>& points, const Params& params)
{
    assert(points.size() > 0);
    assert(params.leafNodePrefixLength % 3 == 0);
    mParams = params;

    clearCpuBuffers();
    mergePoints(points); // generate leaf nodes
    mergeNodes(); // generate internal nodes
    updateDrawParams();

    createAndCopyBuffer(mpOctreeBuffer, sizeof(OctreeNode), (uint)mOctree.size(), mOctree.data(), "ULG::mpOctreeBuffer");
    createAndCopyBuffer(mpOctreeNodeWorldMatBuffer, sizeof(float4x4), (uint)mOctreeNodeWorldMats.size(), mOctreeNodeWorldMats.data(), "ULG::mpOctreeNodeWorldMatBuffer");
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
    assert(mDrawParams.drawBeginLevel <= mDrawParams.drawEndLevel && mDrawParams.drawEndLevel < mLevelIndex.size());
    uint instanceBeginIdx = mLevelIndex[mDrawParams.drawBeginLevel].first;
    uint instanceCount = (mDrawParams.drawEndLevel < mLevelIndex.size() ? mLevelIndex[mDrawParams.drawEndLevel].second : mLevelIndex.back().second) - instanceBeginIdx;
    assert(instanceBeginIdx >= 0 && instanceCount > 0);
    mpProgramVars["PerFrameCB"]["instanceOffset"] = instanceBeginIdx; // Seems that we can't pass this paramater in `drawInstanced(), startInstanceLocation`
    mpProgramVars["PerFrameCB"]["isDrawingSingleNode"] = mDrawParams.drawSingleNode;
    mpProgramVars["PerFrameCB"]["singleNodeIdx"] = mDrawParams.singleNodeIndex;
    pContext->drawInstanced(mpGraphicsState.get(), mpProgramVars.get(), mpVertexBuffer->getElementCount(), instanceCount, 0u, 0);
}

void Octree::renderUI(Gui::Widgets& widget)
{
    // TODO: may be we need to choose some grid to visualize
    auto octreeWidget = widget.group("octree options", true);
    octreeWidget.checkbox("draw single node", mDrawParams.drawSingleNode);
    if (mDrawParams.drawSingleNode)
    {
        octreeWidget.var("node index", mDrawParams.singleNodeIndex, 0ull, mOctree.size() - 1ull, 1ull);
    }
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
        node.nodeIdx = (uint)mOctree.size();
        node.pos = computePosByMortonCode(bound, mParams.leafNodePrefixLength, (float)mParams.quantLevel, mParams.sceneBound);
        node.mortonCode = bound;
        node.intensity = intensity;
        node.childRange = uint2(beginIdx, endIdx);
        node.leafRange = uint2(mOctree.size(), mOctree.size());
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

            OctreeNode node;
            node.nodeIdx = (uint)mOctree.size();
            node.pos = computePosByMortonCode(bound, currPrefixLength, (float)mParams.quantLevel, mParams.sceneBound);
            node.intensity = intensity;
            node.mortonCode = bound;
            node.childRange = uint2(beginIdx, endIdx);
            node.leafRange = uint2(mOctree[beginIdx].leafRange.x, mOctree[endIdx].leafRange.y);
            node.triangleRange = uint2(mOctree[beginIdx].triangleRange.x, mOctree[endIdx].triangleRange.y); // merge two range
            node.isLeaf = false;
            node.prefixLength = currPrefixLength;
            node.paddingAndDebug = float3(mOctree.size(), 0, 0);

            mOctree.emplace_back(node);
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
        mpProgram = GraphicsProgram::createFromFile(kShaderFile, kVertexShader, kPixelShader);
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

        createAndCopyBuffer(mpVertexBuffer, sizeof(float3), (uint)kWiredCubeVertices.size(), kWiredCubeVertices.data(), "ULG::mpOctreeVertexBuffer");

        // prepare vao (TODO: what the hell this means?)
        std::vector<Buffer::SharedPtr> bufferPtrs = { mpVertexBuffer };
        VertexBufferLayout::SharedPtr pBufferLayout = VertexBufferLayout::create();
        pBufferLayout->addElement(VERTEX_POSITION_NAME, 0, ResourceFormat::RGB32Float, 1, 0);
        VertexLayout::SharedPtr pLayout = VertexLayout::create();
        pLayout->addBufferLayout(0, pBufferLayout);
        mpVao = Vao::create(Vao::Topology::LineList, pLayout, bufferPtrs);
        mpGraphicsState->setVao(mpVao);

        // prepare fbo
        Texture::SharedPtr pOutput = renderData["color"]->asTexture();
        mpFbo = Fbo::create({ pOutput });

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

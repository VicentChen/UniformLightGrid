#pragma once
#include "Falcor.h"
#include "BVHNodeData.slangh"
#include "GridData.slangh"

using namespace Falcor;

class Octree
{
public:
    struct Params
    {
        uint leafNodePrefixLength = 27;
        uint quantLevel = 1024;
        AABB sceneBound;
    };

    // TODO: do we need SharedPtr?

    Octree() = default;
    ~Octree() = default;

    size_t getNodeCount() const { return mOctree.size(); }
    size_t getRootIndex() const { return getNodeCount() - 1; }
    Buffer::SharedPtr getGpuBufferPtr() const { return mpOctreeBuffer; }

    // FIXME: currently we use BVH leaf nodes as input to octree
    void buildOctree(RenderContext* pContext, const std::vector<BVHLeafNode>& points, const Params& params);
    void visualize(RenderContext* pContext, Scene::SharedPtr pScene, const RenderData& renderData);
    void renderUI(Gui::Widgets& widget);

private:
    void clearCpuBuffers();
    float4x4 computeNodeWorldMat(uint mortonCode, uint prefixLength);
    void mergePoints(const std::vector<BVHLeafNode>& points);
    void mergeNodes();
    void prepareResources(RenderContext* pContext, const RenderData& renderData);
    void updateDrawParams();

    Params mParams;

    std::vector<OctreeNode> mOctree;
    std::vector<float4x4> mOctreeNodeWorldMats;
    std::vector<std::pair<uint, uint>> mLevelIndex;

    Buffer::SharedPtr mpOctreeBuffer;
    Buffer::SharedPtr mpOctreeStagingBuffer;

    Buffer::SharedPtr mpOctreeNodeWorldMatBuffer;
    Buffer::SharedPtr mpOctreeNodeWorldMatStagingBuffer;

    Buffer::SharedPtr mpVertexBuffer;
    Buffer::SharedPtr mpVertexStagingBuffer;

    Vao::SharedPtr mpVao;

    RasterizerState::SharedPtr mpRasterizerState;
    DepthStencilState::SharedPtr mpDepthStencilState;
    GraphicsState::SharedPtr mpGraphicsState;
    GraphicsProgram::SharedPtr mpProgram;
    GraphicsVars::SharedPtr mpProgramVars;
    Texture::SharedPtr mpDepthStencil;
    Fbo::SharedPtr mpFbo;

    struct 
    {
        size_t drawBeginLevel = 0;
        size_t drawEndLevel = 9;
    } mDrawParams;

};

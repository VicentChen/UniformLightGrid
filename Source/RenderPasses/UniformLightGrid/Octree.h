#pragma once
#include "Falcor.h"
#include "LightProxyData.slangh"
#include "GridData.slangh"
#include "UniformLightGridCommon.h"

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

    Octree() = default;
    ~Octree() = default;

    size_t getNodeCount() const { return mOctree.size(); }
    size_t getRootIndex() const { return getNodeCount() - 1; }
    Buffer::SharedPtr& getGpuBufferPtr() { return mpOctreeBuffer; }

    // FIXME: currently we use BVH leaf nodes as input to octree
    void buildOctree(RenderContext* pContext, const std::vector<LightProxy>& proxys, const Params& params);
    void visualize(RenderContext* pContext, Scene::SharedPtr pScene, const RenderData& renderData);
    void renderUI(Gui::Widgets& widget);

private:
    void clearCpuBuffers();
    float4x4 computeNodeWorldMat(uint mortonCode, uint prefixLength);
    void mergePoints(const std::vector<LightProxy>& proxys);
    void mergeNodes();
    void prepareResources(RenderContext* pContext, const RenderData& renderData);
    void updateDrawParams();

    Params mParams;                                 ///> Parameters for octree.

    struct
    {
        size_t drawBeginLevel = 0;
        size_t drawEndLevel = 9;
        bool drawSingleNode = false;
        size_t singleNodeIndex = 0;
    } mDrawParams;                                  ///> Parameters for visualization.

    std::vector<OctreeNode> mOctree;
    std::vector<std::pair<uint, uint>> mLevelIndex; ///> Vector to store begin and end index of each octree level.
    Buffer::SharedPtr mpOctreeBuffer;               ///> Buffer to store octree nodes.

    std::vector<float4x4> mOctreeNodeWorldMats;
    Buffer::SharedPtr mpOctreeNodeWorldMatBuffer;   ///> Buffer to store AABB transformation matrices.

    // Graphics attributes.
    Buffer::SharedPtr mpVertexBuffer;
    Vao::SharedPtr mpVao;

    Fbo::SharedPtr mpFbo;

    DepthStencilState::SharedPtr mpDepthStencilState;
    RasterizerState::SharedPtr mpRasterizerState;
    GraphicsState::SharedPtr mpGraphicsState;
    GraphicsVars::SharedPtr mpProgramVars;
    GraphicsProgram::SharedPtr mpProgram;
};

inline float4x4 Octree::computeNodeWorldMat(uint mortonCode, uint prefixLength)
{
    AABB aabb = computeAABBByMortonCode(mortonCode, prefixLength, (float)mParams.quantLevel, mParams.sceneBound);
    float3 centerPos = aabb.center();
    float3 extent = aabb.extent();
    return glm::scale(glm::translate(glm::mat4(), centerPos), extent);
}

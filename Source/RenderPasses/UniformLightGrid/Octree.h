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
    void visualize(RenderContext* pContext, GraphicsState* pState, GraphicsVars* pVars);

private:
    void mergePoints(const std::vector<BVHLeafNode>& points);
    void mergeNodes();

    Params mParams;

    std::vector<OctreeNode> mOctree;

    Buffer::SharedPtr mpOctreeBuffer;
    Buffer::SharedPtr mpOctreeStagingBuffer;

    Vao::SharedPtr mpVao;
};

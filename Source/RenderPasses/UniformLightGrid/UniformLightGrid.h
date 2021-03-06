/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#pragma once
#include "Falcor.h"
#include "RenderPasses/Shared/PathTracer/PathTracer.h"
#include "Utils/Algorithm/BitonicSort.h"
#include "Octree.h"
#include "ULGStaticParams.slang"

using namespace Falcor;

/** Forward path tracer using a megakernel in DXR 1.0.

    The path tracer has a loop over the path vertices in the raygen shader.
    The kernel terminates when all paths have terminated.

    This pass implements a forward path tracer with next-event estimation,
    Russian roulette, and multiple importance sampling (MIS) with sampling
    of BRDFs and light sources.
*/
class UniformLightGrid : public PathTracer
{
public:
    using SharedPtr = std::shared_ptr<UniformLightGrid>;

    static SharedPtr create(RenderContext* pRenderContext, const Dictionary& dict);

    virtual std::string getDesc() override { return sDesc; }
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    void renderUI(Gui::Widgets& widget) override;

    static const char* sDesc;

private:
    struct AliasTable
    {
        float weightSum;                ///< Total weight of all elements used to create the alias table
        uint32_t N;                     ///< Number of entries in the alias table (and # elements in the buffers)
        Buffer::SharedPtr fullTable;    ///< A compressed/packed merged table.  Max 2^24 (16 million) entries per table.
    };

    UniformLightGrid(const Dictionary& dict);

    AABB sceneBoundHelper();

    void generateLightProxys(RenderContext* pRenderContext);
    void sortProxys(RenderContext* pRenderContext);
    void constructBVHTree(RenderContext* pRenderContext);

    void generateUniformGrids(RenderContext* pRenderContext);
    AliasTable genGridFluxAliasTable(std::vector<float>& weight);
    void generateOctree(RenderContext* pRenderContext);

    void chooseGridsAndLights(RenderContext* pRenderContext, const RenderData& renderData);

    void recreateVars() override { mULGTracer.pVars = nullptr; }
    void prepareVars();
    void setTracerData(const RenderData& renderData);

    void setULGTracerStaticParams(Program* pProgram) const;
    void addGridAndLightSelectorStaticParams(Program::DefineList& list) const;

    // Ray tracer parameters
    struct
    {
        bool shadowRayAlwaysVisible = false;
        bool useGroundTruthShadowRay = false;
        bool useReflection = false;
    } mULGTracerParams;

    // Ray tracing program.
    struct
    {
        RtProgram::SharedPtr pProgram;
        RtProgramVars::SharedPtr pVars;
        ParameterBlock::SharedPtr pParameterBlock;      ///< ParameterBlock for all data.
    } mULGTracer;

    // light proxy data
    std::vector<LightProxy> mProxys;
    Buffer::SharedPtr mpProxyBuffer;
    ComputePass::SharedPtr mpProxyGenerator;

    // internal node data
    Buffer::SharedPtr mpBVHInternalNodesBuffer;
    ComputePass::SharedPtr mpBVHConstructor;

    // uniform grid data
    std::vector<UniformGrid> mGrids;
    Buffer::SharedPtr mpGridDataBuffer;
    std::mt19937 mAliasTableRng;
    AliasTable mTriangleTable;

    // octree data
    Octree mOctree;

    // grid and light selection data
    struct
    {
        bool needToRegenerateSelector = false;
        uint gridMortonCodePrefixLength = 27;
        uint RISSampleCount = 4;
        uint treeTraverseWeightType = (uint)(TreeTraverseWeightType::DistanceIntensity);
        uint gridSelectionStrategy = (uint)(GridSelectionStrategy::OctreeWithResampling);
        uint triangleSelectionStrategy = (uint)(TriangleSelectionStrategy::Uniform);
    } mGridAndLightSelectorParams;

    ComputePass::SharedPtr mpGridAndLightSelector;

    // TODO: add a struct for ULG parameters
    // hack output color
    float mAmplifyCofficient = 1.0f;
    bool mVisualizeOctree = false;
};

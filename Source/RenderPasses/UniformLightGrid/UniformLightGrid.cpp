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
#include "UniformLightGrid.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "Scene/HitInfo.h"
#include "BVHNodeData.slangh"
#include "UniformLightGridCommon.h"
#include <sstream>

namespace
{
    const char kGenBVHLeafNodesFile[] = "RenderPasses/UniformLightGrid/GenBVHLeafNodes.cs.slang";
    const char kConstructBVHFile[] = "RenderPasses/UniformLightGrid/ConstructBVH.cs.slang";
    const char kGridAndLightSelectorFile[] = "RenderPasses/UniformLightGrid/SelectGridAndLight.cs.slang";
    const char kULGTracerFile[] = "RenderPasses/UniformLightGrid/ULGTracer.rt.slang";

    const char kParameterBlockName[] = "gData";

    // compute shader settings
    const uint32_t kQuantLevels = 1024; // TODO: make as variable
    const uint32_t kGroupSize = 512;
    const uint32_t kChunkSize = 16;

    // Ray tracing settings that affect the traversal stack size.
    // These should be set as small as possible.
    // The payload for the scatter rays is 8-12B.
    // The payload for the shadow rays is 4B.
    const uint32_t kMaxPayloadSizeBytes = HitInfo::kMaxPackedSizeInBytes;
    const uint32_t kMaxAttributesSizeBytes = 8;
    const uint32_t kMaxRecursionDepth = 1;

    // Render pass internal channels.
    const std::string kLightIndexInternal = "lightIndex";

    // Render pass output channels.
    const std::string kColorOutput = "color";
    const std::string kAlbedoOutput = "albedo";
    const std::string kTimeOutput = "time";

    const ChannelList kOutputChannels =
    {
        { kColorOutput,     "gOutputColor",               "Output color (linear)", true /* optional */                              },
        { kAlbedoOutput,    "gOutputAlbedo",              "Surface albedo (base color) or background color", true /* optional */    },
        { kTimeOutput,      "gOutputTime",                "Per-pixel execution time", true /* optional */, ResourceFormat::R32Uint  },
    };
};

const char* UniformLightGrid::sDesc = "Uniform light grid path tracer";

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("UniformLightGrid", UniformLightGrid::sDesc, UniformLightGrid::create);
}

UniformLightGrid::SharedPtr UniformLightGrid::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new UniformLightGrid(dict));
}

UniformLightGrid::UniformLightGrid(const Dictionary& dict)
    : PathTracer(dict, kOutputChannels)
{
    { // Create BVH construction program
        Program::DefineList BVHConstructorDefines;
        BVHConstructorDefines.add("GROUP_SIZE", std::to_string(kGroupSize));
        mpBVHConstructor = ComputePass::create(kConstructBVHFile, "constructBVH", BVHConstructorDefines);
    }

    // Create ray tracing program.
    RtProgram::Desc progDesc;
    progDesc.addShaderLibrary(kULGTracerFile).setRayGen("rayGen");
    progDesc.addHitGroup(kRayTypeScatter, "scatterClosestHit", "scatterAnyHit").addMiss(kRayTypeScatter, "scatterMiss");
    progDesc.addHitGroup(kRayTypeShadow, "", "shadowAnyHit").addMiss(kRayTypeShadow, "shadowMiss");
    progDesc.addDefine("MAX_BOUNCES", std::to_string(mSharedParams.maxBounces));
    progDesc.addDefine("SAMPLES_PER_PIXEL", std::to_string(mSharedParams.samplesPerPixel));
    progDesc.setMaxTraceRecursionDepth(kMaxRecursionDepth);
    mULGTracer.pProgram = RtProgram::create(progDesc, kMaxPayloadSizeBytes, kMaxAttributesSizeBytes);
}

AABB UniformLightGrid::sceneBoundHelper()
{
    const auto& sceneBound = mpScene->getSceneBounds();
    AABB hackedSceneBound = sceneBound;
    auto extent = hackedSceneBound.extent();
    float maxExtent = std::max(extent.x, std::max(extent.y, extent.z));
    hackedSceneBound.maxPoint = hackedSceneBound.maxPoint + float3(maxExtent, maxExtent, maxExtent);
    return hackedSceneBound;
}

void UniformLightGrid::generateBVHLeafNodes(RenderContext* pRenderContext)
{
    PROFILE("ULG_generateBVHLeafNodes");

    // TODO: find a better place for program creation
    // Create leaf nodes generating program
    if (mpLeafNodeGenerator == nullptr)
    {
        assert(mpScene);
        Program::DefineList leafGeneratorDefines = mpScene->getSceneDefines();
        leafGeneratorDefines.add("GROUP_SIZE", std::to_string(kGroupSize));
        mpLeafNodeGenerator = ComputePass::create(kGenBVHLeafNodesFile, "genBVHLeafNodes", leafGeneratorDefines);
    }

    uint32_t emissiveTriangleCount = mpScene->getLightCollection(pRenderContext)->getTotalLightCount();
    if (!mpBVHLeafNodesBuffer || mpBVHLeafNodesBuffer->getElementCount() < emissiveTriangleCount)
    {
        mpBVHLeafNodesBuffer = Buffer::createStructured(mpLeafNodeGenerator->getRootVar()["gLeafNodes"], emissiveTriangleCount, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr, false);
        mpBVHLeafNodesBuffer->setName("ULG::mpBVHLeafNodesBuffer");

        mpBVHLeafNodesHelperBuffer = Buffer::createStructured(mpLeafNodeGenerator->getRootVar()["gLeafNodes"], emissiveTriangleCount, Resource::BindFlags::None, Buffer::CpuAccess::Write);
        mpBVHLeafNodesHelperBuffer->setName("ULG::mpBVHLeafNodesHelperBuffer");
    }

    // TODO: do we have better way to get scene bound?
    auto sceneBound = sceneBoundHelper();

    mpLeafNodeGenerator.getRootVar()["gScene"] = mpScene->getParameterBlock();
    auto var = mpLeafNodeGenerator->getRootVar()["PerFrameCB"];
    var["emissiveTriangleCount"] = emissiveTriangleCount;
    var["quantLevels"] = kQuantLevels;
    var["sceneBound"]["minPoint"] = sceneBound.minPoint;
    var["sceneBound"]["maxPoint"] = sceneBound.maxPoint;
    mpLeafNodeGenerator->getRootVar()["gLeafNodes"] = mpBVHLeafNodesBuffer;

    mpLeafNodeGenerator->execute(pRenderContext, emissiveTriangleCount, 1, 1);
}

void UniformLightGrid::sortLeafNodes(RenderContext* pRenderContext)
{
    PROFILE("ULG_BitonicSortLeafNodes");
    // TODO: move to gpu
    uint32_t emissiveTriangleCount = mpScene->getLightCollection(pRenderContext)->getTotalLightCount();
    size_t bufferSize = sizeof(BVHLeafNode) * emissiveTriangleCount;

    if (mLeafNodes.size() < emissiveTriangleCount)
        mLeafNodes.resize(emissiveTriangleCount);
    BVHLeafNode* pLeaves = (BVHLeafNode*)mpBVHLeafNodesBuffer->map(Buffer::MapType::Read);
    memcpy(mLeafNodes.data(), pLeaves, bufferSize);
    mpBVHLeafNodesBuffer->unmap();

    std::sort(mLeafNodes.begin(), mLeafNodes.end(), [](const BVHLeafNode& a, const BVHLeafNode& b) { return a.mortonCode < b.mortonCode; });

    pLeaves = (BVHLeafNode*)mpBVHLeafNodesHelperBuffer->map(Buffer::MapType::Write);
    memcpy(pLeaves, mLeafNodes.data(), bufferSize);
    mpBVHLeafNodesHelperBuffer->unmap();

    pRenderContext->copyBufferRegion(mpBVHLeafNodesBuffer.get(), 0, mpBVHLeafNodesHelperBuffer.get(), 0, bufferSize);
}

void UniformLightGrid::constructBVHTree(RenderContext* pRenderContext)
{
    PROFILE("ULG_constructBVHTree");

    uint32_t leafNodeCount = mpScene->getLightCollection(pRenderContext)->getTotalLightCount();
    uint32_t internalNodeCount = leafNodeCount - 1;

    if (!mpBVHInternalNodesBuffer || mpBVHInternalNodesBuffer->getElementCount() < internalNodeCount)
    {
        mpBVHInternalNodesBuffer = Buffer::createStructured(mpBVHConstructor->getRootVar()["gInternalNodes"], internalNodeCount, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr, false);
        mpBVHInternalNodesBuffer->setName("ULG::mpBVHInternalNodesBuffer");
    }

    auto var = mpBVHConstructor->getRootVar()["PerFrameCB"];
    var["numLeafNodes"] = leafNodeCount;
    mpBVHConstructor->getRootVar()["gLeafNodes"] = mpBVHLeafNodesBuffer;
    mpBVHConstructor->getRootVar()["gInternalNodes"] = mpBVHInternalNodesBuffer;

    mpBVHConstructor->execute(pRenderContext, internalNodeCount, 1, 1);
}

void UniformLightGrid::generateUniformGrids()
{
    PROFILE("ULG_generateUniformGrids");

    AABB sceneBound = sceneBoundHelper();

    mGrids.clear();
    for (size_t i = 0; i < mLeafNodes.size();)
    {
        uint mask = ~(0xFFFFFF << (30 - mGridAndLightSelectorParams.gridMortonCodePrefixLength));
        uint bound = mLeafNodes[i].mortonCode | mask;

        float3 intensity = float3(0, 0, 0);

        size_t beginIdx = i;
        while (i < mLeafNodes.size() && mLeafNodes[i].mortonCode <= bound)
        {
            intensity += mLeafNodes[i].intensity;
            i++;
        }
        size_t endIdx = i - 1;

        UniformGrid grid;
        grid.pos = computePosByMortonCode(bound, mGridAndLightSelectorParams.gridMortonCodePrefixLength, kQuantLevels, sceneBound);
        grid.intensity = intensity;
        grid.range = uint2(beginIdx, endIdx);
        grid.rootNode = 0;
        grid.isLeafNode = false;

        mGrids.emplace_back(grid);
    }
}

void UniformLightGrid::chooseGridsAndLights(RenderContext* pRenderContext, const RenderData& renderData)
{
    PROFILE("ULG_chooseGridAndLight");

    // TODO: find a better place for program creation
    // Create grid and light selector program
    if (mpGridAndLightSelector == nullptr)
    {
        assert(mpScene);
        Program::DefineList gridAndLightSelectorDefines = mpScene->getSceneDefines();
        gridAndLightSelectorDefines.add("CHUNK_SIZE", std::to_string(kChunkSize));
        gridAndLightSelectorDefines.add(getValidResourceDefines(mInputChannels, renderData)); // We need `loadShadingData`, which uses input channels
        gridAndLightSelectorDefines.add(mpSampleGenerator->getDefines()); // We need `SampleGenerator`
        addGridAndLightSelectorStaticParams(gridAndLightSelectorDefines);
        mpGridAndLightSelector = ComputePass::create(kGridAndLightSelectorFile, "selectGridAndLight", gridAndLightSelectorDefines);
        setStaticParams(mpGridAndLightSelector->getProgram().get()); // BUG: defines will not be added after compute program created
    }

    // prepare grid data
    generateUniformGrids();
    if (!mpGridDataBuffer || mpGridDataBuffer->getElementCount() < mGrids.size())
    {
        uint32_t gridSize = (uint32_t)mGrids.size();
        mpGridDataBuffer = Buffer::createStructured(mpGridAndLightSelector->getRootVar()["gGrids"], gridSize, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr, false);
        mpGridDataBuffer->setName("ULG::mpGridDataBuffer");

        mpGridDataHelperBuffer = Buffer::createStructured(mpGridAndLightSelector->getRootVar()["gGrids"], gridSize, Resource::BindFlags::None, Buffer::CpuAccess::Write);
        mpGridDataHelperBuffer->setName("ULG::mpGridDataBufferHelper");
    }

    // Copy grid data to gpu
    {
        size_t bufferSize = sizeof(UniformGrid) * mGrids.size();
        UniformGrid* pGrids = (UniformGrid*)mpGridDataHelperBuffer->map(Buffer::MapType::Write);
        memcpy(pGrids, mGrids.data(), bufferSize);
        mpGridDataHelperBuffer->unmap();

        pRenderContext->copyBufferRegion(mpGridDataBuffer.get(), 0, mpGridDataHelperBuffer.get(), 0, bufferSize);
    }

    Texture::SharedPtr pLightIndexTexture = renderData[kLightIndexInternal]->asTexture();

    // TODO: do we have better way to get scene bound?
    //const auto& sceneBound = mpScene->getSceneBounds();
    // for grid selection, we need real scene bound and uniform scene bound here
    auto sceneBound = sceneBoundHelper();
    auto realSceneBound = mpScene->getSceneBounds();

    // Add missed channels here
    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto bind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            auto pGlobalVars = mpGridAndLightSelector.getRootVar();
            pGlobalVars[desc.texname] = renderData[desc.name]->asTexture();
        }
    };
    for (auto channel : mInputChannels) bind(channel);

    mpGridAndLightSelector.getRootVar()["gLightIndex"] = pLightIndexTexture;
    mpGridAndLightSelector.getRootVar()["gScene"] = mpScene->getParameterBlock();
    mpGridAndLightSelector.getRootVar()["gLeafNodes"] = mpBVHLeafNodesBuffer;
    mpGridAndLightSelector.getRootVar()["gInternalNodes"] = mpBVHInternalNodesBuffer;
    mpGridAndLightSelector.getRootVar()["gGrids"] = mpGridDataBuffer;

    auto var = mpGridAndLightSelector.getRootVar()["PerFrameCB"];
    var["dispatchDim"] = mSharedParams.frameDim;
    var["frameCount"] = mSharedParams.frameCount;
    var["minDistance"] = mGridAndLightSelectorParams.minDistanceOfGirdSelection;
    var["samplesPerDirection"] = mGridAndLightSelectorParams.samplesPerDirection;
    // TODO: function to bind AABB
    var["realSceneBound"]["minPoint"] = realSceneBound.minPoint;
    var["realSceneBound"]["maxPoint"] = realSceneBound.maxPoint;
    var["gridCount"] = mGrids.size();
    mpGridAndLightSelector.getRootVar()["PerFrameBVHCB"]["gridMortonCodePrefixLength"] = mGridAndLightSelectorParams.gridMortonCodePrefixLength;
    mpGridAndLightSelector.getRootVar()["PerFrameBVHCB"]["quantLevels"] = kQuantLevels;
    mpGridAndLightSelector.getRootVar()["PerFrameBVHCB"]["sceneBound"]["minPoint"] = sceneBound.minPoint;
    mpGridAndLightSelector.getRootVar()["PerFrameBVHCB"]["sceneBound"]["maxPoint"] = sceneBound.maxPoint;

    mpGridAndLightSelector->execute(pRenderContext, uint3(mSharedParams.frameDim, 1));
}

void UniformLightGrid::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    PathTracer::setScene(pRenderContext, pScene);

    if (pScene)
    {
        mULGTracer.pProgram->addDefines(pScene->getSceneDefines());
    }
}

RenderPassReflection UniformLightGrid::reflect(const CompileData& compileData)
{
    auto reflector = PathTracer::reflect(compileData);

    // TODO: find a place to clear this texture
    reflector.addInternal(kLightIndexInternal, "Light index")
        .format(ResourceFormat::RGBA32Float)
        .bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);

    return reflector;
}

void UniformLightGrid::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Call shared pre-render code.
    if (!beginFrame(pRenderContext, renderData)) return;

    // ----- Uniform Light Grid Codes ----- //
    generateBVHLeafNodes(pRenderContext);
    sortLeafNodes(pRenderContext);
    constructBVHTree(pRenderContext);

    chooseGridsAndLights(pRenderContext, renderData);

    // ----- Uniform Light Grid Codes ----- //

    // Set compile-time constants.
    RtProgram::SharedPtr pProgram = mULGTracer.pProgram;
    setStaticParams(pProgram.get());
    setULGTracerStaticParams(pProgram.get());

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    pProgram->addDefines(getValidResourceDefines(mInputChannels, renderData));
    pProgram->addDefines(getValidResourceDefines(mOutputChannels, renderData));

    if (mUseEmissiveSampler)
    {
        // Specialize program for the current emissive light sampler options.
        assert(mpEmissiveSampler);
        if (pProgram->addDefines(mpEmissiveSampler->getDefines())) mULGTracer.pVars = nullptr;
    }

    // Prepare program vars. This may trigger shader compilation.
    // The program should have all necessary defines set at this point.
    if (!mULGTracer.pVars) prepareVars();
    assert(mULGTracer.pVars);

    // Set shared data into parameter block.
    setTracerData(renderData);

    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto bind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            auto pGlobalVars = mULGTracer.pVars->getRootVar();
            pGlobalVars[desc.texname] = renderData[desc.name]->asTexture();
        }
    };
    for (auto channel : mInputChannels) bind(channel);
    for (auto channel : mOutputChannels) bind(channel);

    // ----- Uniform Light Grid Codes ----- //
    mULGTracer.pVars.getRootVar()["gLightIndex"] = renderData[kLightIndexInternal]->asTexture();
    mULGTracer.pVars.getRootVar()["PerFrameCB"]["AmplifyCofficient"] = mAmplifyCofficient;
    // ----- Uniform Light Grid Codes ----- //

    // Get dimensions of ray dispatch.
    const uint2 targetDim = renderData.getDefaultTextureDims();
    assert(targetDim.x > 0 && targetDim.y > 0);

    mpPixelDebug->prepareProgram(pProgram, mULGTracer.pVars->getRootVar());
    mpPixelStats->prepareProgram(pProgram, mULGTracer.pVars->getRootVar());

    // Spawn the rays.
    {
        PROFILE("ULG_RayTrace");
        mpScene->raytrace(pRenderContext, mULGTracer.pProgram.get(), mULGTracer.pVars, uint3(targetDim, 1));
    }

    // Call shared post-render code.
    endFrame(pRenderContext, renderData);
}

void UniformLightGrid::renderUI(Gui::Widgets& widget)
{
    widget.var("output color amplifer", mAmplifyCofficient, 1.0f, 100.0f, 1.0f);

    {
        bool regenerateSelector = false;
        widget.text("Grid & Light Selector Params");
        widget.var("grid morton code prefix length", mGridAndLightSelectorParams.gridMortonCodePrefixLength, 3u, 27u, 3u);
        widget.var("min distance of grid selection", mGridAndLightSelectorParams.minDistanceOfGirdSelection, 0.1f, 100.0f, 0.1f);
        widget.var("grid samples per direction", mGridAndLightSelectorParams.samplesPerDirection, 1u, 64u, 1u);

        { // Grid selection strategy gui component
            Gui::DropdownList list;
            list.push_back({ (uint)GridSelectionStrategy::MortonCodeAndBRDF, "MortonCodeAndBRDF" });
            list.push_back({ (uint)GridSelectionStrategy::TreeTraversal, "TreeTraversal" });
            list.push_back({ (uint)GridSelectionStrategy::Resampling, "Resampling" });
            if (widget.dropdown("Grid selection strategy", list, mGridAndLightSelectorParams.gridSelectionStrategy))
                regenerateSelector = true;
        }

        // Tree traverse weight gui component
        if (mGridAndLightSelectorParams.gridSelectionStrategy == (uint)GridSelectionStrategy::TreeTraversal)
        {
            Gui::DropdownList list;
            list.push_back({ (uint)TreeTraverseWeightType::Direction, "Direction" });
            list.push_back({ (uint)TreeTraverseWeightType::DistanceIntensity, "DistanceIntensity" });
            list.push_back({ (uint)TreeTraverseWeightType::DirectionDistanceIntensity, "DirectionDistanceIntensity" });
            list.push_back({ (uint)TreeTraverseWeightType::BRDFShading, "BRDFShading" });
            if (widget.dropdown("Tree traverse weight type", list, mGridAndLightSelectorParams.treeTraverseWeightType))
                regenerateSelector = true;
        }

        // we can't add define for compute pass, so we regenerate it every time defines change
        if (regenerateSelector) mpGridAndLightSelector = nullptr;
    }

    widget.text("ULG Tracer Params");
    widget.checkbox("shadow ray always visible?", mULGTracerParams.shadowRayAlwaysVisible);
    widget.checkbox("use ground truth shadow ray?", mULGTracerParams.useGroundTruthShadowRay);
    widget.checkbox("use reflection?", mULGTracerParams.useReflection);
    widget.text("");

    PathTracer::renderUI(widget);
}

void UniformLightGrid::prepareVars()
{
    assert(mpScene);
    assert(mULGTracer.pProgram);

    // Configure program.
    mULGTracer.pProgram->addDefines(mpSampleGenerator->getDefines());

    // Create program variables for the current program/scene.
    // This may trigger shader compilation. If it fails, throw an exception to abort rendering.
    mULGTracer.pVars = RtProgramVars::create(mULGTracer.pProgram, mpScene);

    // Bind utility classes into shared data.
    auto pGlobalVars = mULGTracer.pVars->getRootVar();
    bool success = mpSampleGenerator->setShaderData(pGlobalVars);
    if (!success) throw std::exception("Failed to bind sample generator");

    // Create parameter block for shared data.
    ProgramReflection::SharedConstPtr pReflection = mULGTracer.pProgram->getReflector();
    ParameterBlockReflection::SharedConstPtr pBlockReflection = pReflection->getParameterBlock(kParameterBlockName);
    assert(pBlockReflection);
    mULGTracer.pParameterBlock = ParameterBlock::create(pBlockReflection);
    assert(mULGTracer.pParameterBlock);

    // Bind static resources to the parameter block here. No need to rebind them every frame if they don't change.
    // Bind the light probe if one is loaded.
    if (mpEnvMapSampler) mpEnvMapSampler->setShaderData(mULGTracer.pParameterBlock["envMapSampler"]);

    // Bind the parameter block to the global program variables.
    mULGTracer.pVars->setParameterBlock(kParameterBlockName, mULGTracer.pParameterBlock);
}

void UniformLightGrid::setTracerData(const RenderData& renderData)
{
    auto pBlock = mULGTracer.pParameterBlock;
    assert(pBlock);

    // Upload parameters struct.
    pBlock["params"].setBlob(mSharedParams);

    // Bind emissive light sampler.
    if (mUseEmissiveSampler)
    {
        assert(mpEmissiveSampler);
        bool success = mpEmissiveSampler->setShaderData(pBlock["emissiveSampler"]);
        if (!success) throw std::exception("Failed to bind emissive light sampler");
    }
}

void UniformLightGrid::setULGTracerStaticParams(Program* pProgram) const
{
    pProgram->addDefine("ULG_TRACER_PARAM", "1");
    pProgram->addDefine("SHADOW_RAY_ALWAYS_VISIBLE", mULGTracerParams.shadowRayAlwaysVisible ? "1" : "0");
    pProgram->addDefine("USE_GROUND_TRUTH_SHADOW_RAY", mULGTracerParams.useGroundTruthShadowRay ? "1" : "0");
    pProgram->addDefine("USE_REFLECTION", mULGTracerParams.useReflection ? "1" : "0");
}

void UniformLightGrid::addGridAndLightSelectorStaticParams(Program::DefineList& list) const
{
    list.add("ULG_GRID_AND_LIGHT_SELECTOR_PARAM", "1");
    list.add("TREE_TRAVERSE_WEIGHT_TYPE", std::to_string(mGridAndLightSelectorParams.treeTraverseWeightType));
    list.add("GRID_SELECTION_STRATEGY", std::to_string(mGridAndLightSelectorParams.gridSelectionStrategy));
}

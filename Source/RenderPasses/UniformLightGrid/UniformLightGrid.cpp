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
#include <sstream>

namespace
{
    const char kGenBVHLeafNodesFile[] = "RenderPasses/UniformLightGrid/GenBVHLeafNodes.cs.slang";
    const char kConstructBVHFile[] = "RenderPasses/UniformLightGrid/constructBVH.cs.slang";
    const char kULGTracerFile[] = "RenderPasses/UniformLightGrid/ULGTracer.rt.slang";

    const char kParameterBlockName[] = "gData";

    // compute shader settings
    const uint32_t kGroupSize = 512;

    // Ray tracing settings that affect the traversal stack size.
    // These should be set as small as possible.
    // The payload for the scatter rays is 8-12B.
    // The payload for the shadow rays is 4B.
    const uint32_t kMaxPayloadSizeBytes = HitInfo::kMaxPackedSizeInBytes;
    const uint32_t kMaxAttributesSizeBytes = 8;
    const uint32_t kMaxRecursionDepth = 1;

    // Render pass output channels.
    const std::string kColorOutput = "color";
    const std::string kAlbedoOutput = "albedo";
    const std::string kTimeOutput = "time";

    const Falcor::ChannelList kOutputChannels =
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
    { // Create leaf nodes generating program
        Program::DefineList leafGeneratorDefines;
        leafGeneratorDefines.add("GROUP_SIZE", std::to_string(kGroupSize));
        mpLeafNodeGenerator = ComputePass::create(kGenBVHLeafNodesFile, "genBVHLeafNodes", leafGeneratorDefines);
    }

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

void UniformLightGrid::generateBVHLeafNodes(RenderContext* pRenderContext)
{
    uint32_t emissiveTriangleCount = mpScene->getLightCollection(pRenderContext)->getTotalLightCount();
    if (!mpBVHLeafNodesBuffer || mpBVHLeafNodesBuffer->getElementCount() < emissiveTriangleCount)
    {
        mpBVHLeafNodesBuffer = Buffer::createStructured(mpLeafNodeGenerator->getRootVar()["gLeafNodes"], emissiveTriangleCount, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr, false);
        mpBVHLeafNodesBuffer->setName("ULG::mpBVHLeafNodesBuffer");

        mpBVHLeafNodesHelperBuffer = Buffer::createStructured(mpLeafNodeGenerator->getRootVar()["gLeafNodes"], emissiveTriangleCount, Resource::BindFlags::None, Buffer::CpuAccess::Write);
        mpBVHLeafNodesHelperBuffer->setName("ULG::mpBVHLeafNodesHelperBuffer");
    }

    const auto& sceneBound = mpScene->getSceneBounds();

    auto var = mpLeafNodeGenerator->getRootVar()["PerFrameCB"];
    mpScene->getLightCollection(pRenderContext)->setShaderData(var["gLights"]);
    var["emissiveTriangleCount"] = emissiveTriangleCount;
    var["quantLevels"] = 1024; // TODO: make as variable
    var["sceneBound"]["minPoint"] = sceneBound.minPoint;
    var["sceneBound"]["maxPoint"] = sceneBound.maxPoint;
    mpLeafNodeGenerator->getRootVar()["gLeafNodes"] = mpBVHLeafNodesBuffer;

    PROFILE("ULG_generateBVHLeafNodes");
    mpLeafNodeGenerator->execute(pRenderContext, emissiveTriangleCount, 1, 1);
}

void UniformLightGrid::sortLeafNodes(RenderContext* pRenderContext)
{
    PROFILE("ULG_BitonicSortLeafNodes");
    // TODO: move to gpu
    uint32_t emissiveTriangleCount = mpScene->getLightCollection(pRenderContext)->getTotalLightCount();
    size_t bufferSize = sizeof(::LeafNode) * emissiveTriangleCount;

    std::vector<::LeafNode> leaves(emissiveTriangleCount);
    ::LeafNode* pLeaves = (::LeafNode*)mpBVHLeafNodesBuffer->map(Buffer::MapType::Read);
    memcpy(leaves.data(), pLeaves, bufferSize);
    mpBVHLeafNodesBuffer->unmap();

    std::sort(leaves.begin(), leaves.end(), [](const ::LeafNode& a, const ::LeafNode& b) { return a.mortonCode < b.mortonCode; });

    pLeaves = (::LeafNode*)mpBVHLeafNodesHelperBuffer->map(Buffer::MapType::Write);
    memcpy(pLeaves, leaves.data(), bufferSize);
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

void UniformLightGrid::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    PathTracer::setScene(pRenderContext, pScene);

    if (pScene)
    {
        mpLeafNodeGenerator->getProgram()->addDefines(mpScene->getSceneDefines());
        mULGTracer.pProgram->addDefines(pScene->getSceneDefines());
    }
}

void UniformLightGrid::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Call shared pre-render code.
    if (!beginFrame(pRenderContext, renderData)) return;

    // ----- Uniform Light Grid Codes ----- //
    generateBVHLeafNodes(pRenderContext);
    sortLeafNodes(pRenderContext);
    constructBVHTree(pRenderContext);

    // ----- Uniform Light Grid Codes ----- //

    // Set compile-time constants.
    RtProgram::SharedPtr pProgram = mULGTracer.pProgram;
    setStaticParams(pProgram.get());

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

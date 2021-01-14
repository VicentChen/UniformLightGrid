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
#include "ReSTIR.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "Scene/HitInfo.h"
#include <sstream>
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
namespace
{
    const char kShaderFile[] = "RenderPasses/ReSTIR/PathTracer.rt.slang";
    const char kFinalShader[] = "RenderPasses/ReSTIR/ReSTIRFinal.rt.slang";
    const char kParameterBlockName[] = "gData";

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

    // Internal
    const std::string kCurrReservoir = "Current Reservoir";
    const std::string kPrevReservoir = "Previous Reservoir";
    const std::string kTemporalReservoir = "Temporal Reservoir";
    const std::string kSpatialReservoir = "Spatial Reservoir";
    const std::string kuFinalCurr = "uFinalCurr";
    const std::string kuFinalPrev = "uFInalPrev";
    const std::string kuFinalTemporal = "uFinalTemporal";
    const std::string kuFinalSpatial = "uFinalSpatial";

    // Input
    const std::string kMotionVector = "MotionVec";
    const Falcor::ChannelList kOutputChannels =
    {
        { kColorOutput,     "gOutputColor",               "Output color (linear)", true /* optional */                              },
        { kAlbedoOutput,    "gOutputAlbedo",              "Surface albedo (base color) or background color", true /* optional */    },
        { kTimeOutput,      "gOutputTime",                "Per-pixel execution time", true /* optional */, ResourceFormat::R32Uint  },
    };
};

const char* ReSTIR::sDesc = "ReSITR path tracer";

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("ReSTIR", ReSTIR::sDesc, ReSTIR::create);
}

ReSTIR::SharedPtr ReSTIR::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new ReSTIR(dict));
}

ReSTIR::ReSTIR(const Dictionary& dict)
    : PathTracer(dict, kOutputChannels)
{
    mSelectedEmissiveSampler = EmissiveLightSamplerType::Power;

    // Create ray tracing program.
    {
        RtProgram::Desc progDesc;
        progDesc.addShaderLibrary(kShaderFile).setRayGen("rayGen");
        progDesc.addHitGroup(kRayTypeScatter, "scatterClosestHit", "scatterAnyHit").addMiss(kRayTypeScatter, "scatterMiss");
        progDesc.addHitGroup(kRayTypeShadow, "", "shadowAnyHit").addMiss(kRayTypeShadow, "shadowMiss");
        progDesc.addDefine("MAX_BOUNCES", std::to_string(mSharedParams.maxBounces));
        progDesc.addDefine("SAMPLES_PER_PIXEL", std::to_string(mSharedParams.samplesPerPixel));
        progDesc.setMaxTraceRecursionDepth(kMaxRecursionDepth);
        mTracer.pProgram = RtProgram::create(progDesc, kMaxPayloadSizeBytes, kMaxAttributesSizeBytes);
    }
    {
        RtProgram::Desc progDesc;
        progDesc.addShaderLibrary(kFinalShader).setRayGen("rayGen");
        //progDesc.addDefine("MAX_BOUNCES", std::to_string(mSharedParams.maxBounces));
        //progDesc.addDefine("SAMPLES_PER_PIXEL", std::to_string(mSharedParams.samplesPerPixel));
        progDesc.setMaxTraceRecursionDepth(kMaxRecursionDepth);
        m_FinalTracer.pProgram = RtProgram::create(progDesc, kMaxPayloadSizeBytes, kMaxAttributesSizeBytes);
    }
}

void ReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    PathTracer::setScene(pRenderContext, pScene);

    if (pScene)
    {
        mTracer.pProgram->addDefines(pScene->getSceneDefines());
        m_FinalTracer.pProgram->addDefines(pScene->getSceneDefines());
    }
}

void ReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    Texture::SharedPtr pCurrReservoir = renderData[kCurrReservoir]->asTexture();
    Texture::SharedPtr pPrevReservoir = renderData[kPrevReservoir]->asTexture();
    Texture::SharedPtr pTemporalReservoir = renderData[kTemporalReservoir]->asTexture();
    Texture::SharedPtr pSpatialReservoir = renderData[kSpatialReservoir]->asTexture();
    Texture::SharedPtr puFinalCurr = renderData[kuFinalCurr]->asTexture();
    Texture::SharedPtr puFinalPrev = renderData[kuFinalPrev]->asTexture();
    Texture::SharedPtr puFinalTemporal = renderData[kuFinalTemporal]->asTexture();
    Texture::SharedPtr puFinalSpatial = renderData[kuFinalSpatial]->asTexture();
    Texture::SharedPtr pOutputColor = renderData[kColorOutput]->asTexture();
    Texture::SharedPtr pMotionVector = renderData[kMotionVector]->asTexture();
    pRenderContext->clearTexture(pOutputColor.get());
    if (m_IsClearPrev)
    {
        pRenderContext->clearTexture(pPrevReservoir.get());
        pRenderContext->clearTexture(puFinalPrev.get());
        m_IsClearPrev = false;
    }
    // Call shared pre-render code.
    if (!beginFrame(pRenderContext, renderData)) return;

    // Set compile-time constants.
    RtProgram::SharedPtr pProgram = mTracer.pProgram;
    RtProgram::SharedPtr pFinalProgram = m_FinalTracer.pProgram;
    setStaticParams(pProgram.get());
    setStaticParams(pFinalProgram.get());

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    pProgram->addDefines(getValidResourceDefines(mInputChannels, renderData));
    pProgram->addDefines(getValidResourceDefines(mOutputChannels, renderData));
    pFinalProgram->addDefines(getValidResourceDefines(mInputChannels, renderData));
    pFinalProgram->addDefines(getValidResourceDefines(mOutputChannels, renderData));

    if (mUseEmissiveSampler)
    {
        // Specialize program for the current emissive light sampler options.
        assert(mpEmissiveSampler);
        if (pProgram->addDefines(mpEmissiveSampler->getDefines()))
        {
            mTracer.pVars = nullptr;
         /*   m_FinalTracer.pVars = nullptr;*/
        }

        //if (pFinalProgram->addDefines(mpEmissiveSampler->getDefines()))
        //{
        //    m_FinalTracer.pVars = nullptr;
        //}
    }

    // Prepare program vars. This may trigger shader compilation.
    // The program should have all necessary defines set at this point.
    if (!mTracer.pVars) prepareVars();
    assert(mTracer.pVars);

    // Set shared data into parameter block.
    setTracerData(renderData);

    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto bind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            auto pGlobalVars = mTracer.pVars->getRootVar();
            pGlobalVars[desc.texname] = renderData[desc.name]->asTexture();
        }
    };

    for (auto channel : mInputChannels) bind(channel);
    for (auto channel : mOutputChannels) bind(channel);

    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto Finalbind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            auto pGlobalVars = m_FinalTracer.pVars->getRootVar();
            pGlobalVars[desc.texname] = renderData[desc.name]->asTexture();
        }
    };

    for (auto channel : mInputChannels) Finalbind(channel);
 
    // Get dimensions of ray dispatch.
    const uint2 targetDim = renderData.getDefaultTextureDims();
    assert(targetDim.x > 0 && targetDim.y > 0);

    mpPixelDebug->prepareProgram(pProgram, mTracer.pVars->getRootVar());
    mpPixelStats->prepareProgram(pProgram, mTracer.pVars->getRootVar());

    m_LastCameraMatrix = m_CurrCameraMatrix;
    m_CurrCameraMatrix = mpScene->getCamera()->getViewProjMatrix();

    static bool IsInitLight = true;
    // Spawn the rays.
    {
        PROFILE("ReSTIR::execute()_RayTrace");

        mTracer.pVars["ReservoirTemporal"] = pTemporalReservoir;
        mTracer.pVars["ReservoirCurr"] = pCurrReservoir;
        mTracer.pVars["ReservoirPrev"] = pPrevReservoir;
        mTracer.pVars["uFinalCurr"] = puFinalCurr;
        mTracer.pVars["uFinalPrev"] = puFinalPrev;
        mTracer.pVars["uFinalTemporal"] = puFinalTemporal;
        mTracer.pVars["MotionVector"] = pMotionVector;
        mTracer.pVars["PerFrameCB"]["CandidateCount"] = m_CandidateCount;
        mTracer.pVars["PerFrameCB"]["ReservoirPerPixel"] = m_ReservoirPerPixel;
        mTracer.pVars["PerFrameCB"]["IsTemporalReuse"] = m_IsTemporalReuse;
        mTracer.pVars["PerFrameCB"]["IsInitLight"] = IsInitLight;
        mTracer.pVars["PerFrameCB"]["LastCameraMatrix"] = m_LastCameraMatrix;
        mTracer.pVars["PerFrameCB"]["IsUseMotionVector"] = m_IsUseMotionVector;
        mpScene->raytrace(pRenderContext, mTracer.pProgram.get(), mTracer.pVars, uint3(targetDim, 1));
    }

    {
        PROFILE("ReSTIR::execute()_RayTrace");

        m_FinalTracer.pVars["ReservoirTemporal"] = pTemporalReservoir;
        m_FinalTracer.pVars["ReservoirCurr"] = pCurrReservoir;
        m_FinalTracer.pVars["ReservoirPrev"] = pPrevReservoir;
        m_FinalTracer.pVars["ReservoirSpatial"] = pSpatialReservoir;
        m_FinalTracer.pVars["uFinalCurr"] = puFinalCurr;
        m_FinalTracer.pVars["uFinalPrev"] = puFinalPrev;
        m_FinalTracer.pVars["uFinalTemporal"] = puFinalTemporal;
        m_FinalTracer.pVars["uFinalSpatial"] = puFinalSpatial;
        m_FinalTracer.pVars["MotionVector"] = pMotionVector;
        m_FinalTracer.pVars["PerFrameCB"]["CandidateCount"] = m_CandidateCount;
        m_FinalTracer.pVars["PerFrameCB"]["ReservoirPerPixel"] = m_ReservoirPerPixel;
        m_FinalTracer.pVars["PerFrameCB"]["IsTemporalReuse"] = m_IsTemporalReuse;
        m_FinalTracer.pVars["PerFrameCB"]["IsSpatialReuse"] = m_IsSpatialReuse;
        m_FinalTracer.pVars["PerFrameCB"]["NeighborCount"] = m_NeighborCount;
        m_FinalTracer.pVars["PerFrameCB"]["NeighborsRange"] = m_NeighborsRange;
        m_FinalTracer.pVars["PerFrameCB"]["FrameCount"] = m_FrameCount++;
        m_FinalTracer.pVars["gOutputColor"] = pOutputColor;

        mpScene->raytrace(pRenderContext, m_FinalTracer.pProgram.get(), m_FinalTracer.pVars, uint3(targetDim, 1));
    }
    IsInitLight = false;
    // Call shared post-render code.
    endFrame(pRenderContext, renderData);
}

RenderPassReflection ReSTIR::reflect(const CompileData& compileData)
{
    auto Reflector = PathTracer::reflect(compileData);
    Reflector.addInput(kMotionVector, "Motion Vector");
    Reflector.addInternal(kCurrReservoir, "Current Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kPrevReservoir, "Previous Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kTemporalReservoir, "Temporal Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kSpatialReservoir, "Spatial Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kuFinalCurr, "u Final Current").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kuFinalPrev, "u Final Previous").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kuFinalTemporal, "u Final Temporal").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kuFinalSpatial, "u Final Spatial").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    return Reflector;
}

void ReSTIR::prepareVars()
{
    assert(mpScene);
    assert(mTracer.pProgram);
    {
        // Configure program.
        mTracer.pProgram->addDefines(mpSampleGenerator->getDefines());

        // Create program variables for the current program/scene.
        // This may trigger shader compilation. If it fails, throw an exception to abort rendering.
        mTracer.pVars = RtProgramVars::create(mTracer.pProgram, mpScene);

        // Bind utility classes into shared data.
        auto pGlobalVars = mTracer.pVars->getRootVar();
        bool success = mpSampleGenerator->setShaderData(pGlobalVars);
        if (!success) throw std::exception("Failed to bind sample generator");

        // Create parameter block for shared data.
        ProgramReflection::SharedConstPtr pReflection = mTracer.pProgram->getReflector();
        ParameterBlockReflection::SharedConstPtr pBlockReflection = pReflection->getParameterBlock(kParameterBlockName);
        assert(pBlockReflection);
        mTracer.pParameterBlock = ParameterBlock::create(pBlockReflection);
        assert(mTracer.pParameterBlock);

        // Bind static resources to the parameter block here. No need to rebind them every frame if they don't change.
        // Bind the light probe if one is loaded.
        if (mpEnvMapSampler) mpEnvMapSampler->setShaderData(mTracer.pParameterBlock["envMapSampler"]);

        // Bind the parameter block to the global program variables.
        mTracer.pVars->setParameterBlock(kParameterBlockName, mTracer.pParameterBlock);
    }
    {
        m_FinalTracer.pProgram->addDefines(mpSampleGenerator->getDefines());
        m_FinalTracer.pVars = RtProgramVars::create(m_FinalTracer.pProgram, mpScene);

        auto pGlobalVars = m_FinalTracer.pVars->getRootVar();
        bool success = mpSampleGenerator->setShaderData(pGlobalVars);
        if (!success) throw std::exception("Failed to bind sample generator");

        //ProgramReflection::SharedConstPtr pReflection = m_FinalTracer.pProgram->getReflector();
        //ParameterBlockReflection::SharedConstPtr pBlockReflection = pReflection->getParameterBlock(kParameterBlockName);
        //assert(pBlockReflection);
        //m_FinalTracer.pParameterBlock = ParameterBlock::create(pBlockReflection);
        //assert(m_FinalTracer.pParameterBlock);
        //m_FinalTracer.pVars->setParameterBlock(kParameterBlockName, m_FinalTracer.pParameterBlock);
    }
}

void ReSTIR::setTracerData(const RenderData& renderData)
{
    auto pBlock = mTracer.pParameterBlock;
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

    //m_FinalTracer.pParameterBlock["params"].setBlob(mSharedParams);
}

void ReSTIR::renderUI(Gui::Widgets& widget)
{
    widget.var("Candidate Count", m_CandidateCount, 1u, 64u);
    widget.var("Reservoir Per Pixel", m_ReservoirPerPixel, 1u, 8u);
    widget.var("Neighbor Count", m_NeighborCount, 1u, 7u);
    widget.var("Neighbor Range", m_NeighborsRange, 1.f, 100.f, 1.f);
    if (widget.checkbox("Temporal reuse?", m_IsTemporalReuse)) m_IsClearPrev = true;
    widget.checkbox("Spatial reuse?", m_IsSpatialReuse);
    widget.checkbox("Use Motion Vector?", m_IsUseMotionVector);
    PathTracer::renderUI(widget);
}

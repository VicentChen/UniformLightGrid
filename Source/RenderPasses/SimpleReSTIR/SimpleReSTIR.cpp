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
#include "SimpleReSTIR.h"

const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
namespace
{
    // Shaders
    const char kInitReservoirShader[] = "RenderPasses/SimpleReSTIR/InitReservoir.rt.slang";
    const char kSpatialResueShader[] = "RenderPasses/SimpleReSTIR/Reuse.rt.slang";
    const char kFinalShader[] = "RenderPasses/SimpleReSTIR/Final.rt.slang";

    //Parameters
    const char kCandiateCount[] = "CandiateCount";
    const char kReservoirPerPixel[] = "ReservoirPerPixel";
    const char kUnbiased[] = "Unbiased";
    const char kIsSpatialReuse[] = "SpatialReuse";
    const char kIsTemporalReuse[] = "TemporalReuse";
    const char kNeighborCount[] = "NegihbotCount";
    const char kNeighborRange[] = "NeighborRange";

    // Input buffer names
    const char kInputDiffuse[] = "Diffuse";
    const char kInputSpecular[] = "Specular";
    const char kInputWorldPosition[] = "WorldPosition";
    const char kInputWorldNormal[] = "WorldNormal";

    // Output buffer names
    const char kOutput[] = "ReSTIROutput";

    // Internal buffer names
    const char kCurrReservoir[] = "Current Reservoir";
    const char kPrevReservoir[] = "Previous Reservoir";
    const char kTemporalReservoir[] = "Temporal Reservoir";
    const char kSpatialReservoir[] = "Spatial Reservoir";

}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("CSimpleReSTIR", "ReSTIR Pass", CSimpleReSTIR::create);
}

CSimpleReSTIR::SharedPtr CSimpleReSTIR::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new CSimpleReSTIR(dict));
    return pPass;
}

CSimpleReSTIR::CSimpleReSTIR(const Dictionary& vDict)
{
    for (const auto& [key, value] : vDict)
    {
        if (key == kCandiateCount) m_CandiateCount = value;
        else if (key == kReservoirPerPixel) m_ReservoirPerPixel = value;
        else if (key == kUnbiased) m_Unbiased = value;
        else if (key == kIsSpatialReuse) m_IsSpatialReuse = value;
        else if (key == kIsTemporalReuse) m_IsTemporalReuse = value;
        else if (key == kNeighborCount) m_NeighborCount = value;
        else if (key == kNeighborRange) m_NeighborRange = value;
       
        else logWarning("Unknown field '" + key + "' in ReSTIRPass dictionary");
    }

    __prepareShaders();
    m_pCurrReservoir = Texture::create2D(WINDOW_WIDTH, WINDOW_HEIGHT, ResourceFormat::RGBA32Float, 8, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    m_pCurrReservoir->setName("Current Reservior");
    m_pPrevReservoir = Texture::create2D(WINDOW_WIDTH, WINDOW_HEIGHT, ResourceFormat::RGBA32Float, 8, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    m_pPrevReservoir->setName("Previous Reservoir");
    m_pSpatailReservoir = Texture::create2D(WINDOW_WIDTH, WINDOW_HEIGHT, ResourceFormat::RGBA32Float, 8, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    m_pSpatailReservoir->setName("Spatail Reservoir");
    m_pTemporalReservoir = Texture::create2D(WINDOW_WIDTH, WINDOW_HEIGHT, ResourceFormat::RGBA32Float, 8, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    m_pTemporalReservoir->setName("Temporal Reservoir");
}

void CSimpleReSTIR::__prepareShaders()
{
    // Init and temporal reuse
    {
        RtProgram::Desc Desc;
        Desc.addShaderLibrary(kInitReservoirShader).addRayGen("rayGen");
        Desc.addHitGroup(0, "", "shadowAnyHit").addMiss(0, "shadowMiss");
        Desc.setMaxTraceRecursionDepth(1);
        m_pInitReservoirProgram = RtProgram::create(Desc, 256);
        m_pInitReservoirProgram->addDefine("SAMPLE_GENERATOR_TYPE", "SAMPLE_GENERATOR_UNIFORM");
    }

    //Spatial reuse
    {
        RtProgram::Desc Desc;
        Desc.addShaderLibrary(kSpatialResueShader).addRayGen("rayGen");
        Desc.addHitGroup(0, "", "shadowAnyHit").addMiss(0, "shadowMiss");
        Desc.setMaxTraceRecursionDepth(1);
        m_pReuseProgram = RtProgram::create(Desc, 256);
        m_pReuseProgram->addDefine("SAMPLE_GENERATOR_TYPE", "SAMPLE_GENERATOR_UNIFORM");
    }

    // Final
    {
        RtProgram::Desc Desc;
        Desc.addShaderLibrary(kFinalShader).addRayGen("rayGen");
        Desc.addHitGroup(0, "", "shadowAnyHit").addMiss(0, "shadowMiss");
        Desc.setMaxTraceRecursionDepth(1);
        m_pFinalProgram = RtProgram::create(Desc, 256);
        m_pFinalProgram->addDefine("SAMPLE_GENERATOR_TYPE", "SAMPLE_GENERATOR_UNIFORM");
    }
}

void CSimpleReSTIR::__prepareVars()
{
    assert(m_pScene);
    assert(m_pInitReservoirProgram);
    assert(m_pReuseProgram);
    assert(m_pFinalProgram);

    m_pInitReservoirVars = RtProgramVars::create(m_pInitReservoirProgram, m_pScene);
    m_pReuseVars = RtProgramVars::create(m_pReuseProgram, m_pScene);
    m_pFinalVars = RtProgramVars::create(m_pFinalProgram, m_pScene);
}

void CSimpleReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    m_pScene = pScene;

    if (pScene)
    {
        m_pInitReservoirProgram->addDefines(m_pScene->getSceneDefines());
        m_pReuseProgram->addDefines(m_pScene->getSceneDefines());
        m_pFinalProgram->addDefines(m_pScene->getSceneDefines());
    }
}

std::string CSimpleReSTIR::getDesc() { return "ReSTIR Pass"; }

Dictionary CSimpleReSTIR::getScriptingDictionary()
{
    Dictionary Dict;
    Dict[kCandiateCount] = m_CandiateCount;
    Dict[kReservoirPerPixel] = m_ReservoirPerPixel;
    Dict[kUnbiased] = m_Unbiased;
    Dict[kIsSpatialReuse] = m_IsSpatialReuse;
    Dict[kIsTemporalReuse] = m_IsTemporalReuse;
    Dict[kNeighborCount] = m_NeighborCount;
    Dict[kNeighborRange] = m_NeighborRange;
    return Dict;
}

RenderPassReflection CSimpleReSTIR::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection Reflector;
    Reflector.addInput(kInputDiffuse, "Diffuse");
    Reflector.addInput(kInputSpecular, "Specular");
    Reflector.addInput(kInputWorldPosition, "World Position");
    Reflector.addInput(kInputWorldNormal, "World Normal");

    Reflector.addInternal(kCurrReservoir, "Current Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kPrevReservoir, "Previous Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kTemporalReservoir, "Temporal Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addInternal(kSpatialReservoir, "Spatial Reservoir").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess).texture2D(WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1, 8);
    Reflector.addOutput(kOutput, "ReSTIROutput").format(ResourceFormat::RGBA32Float).bindFlags(ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess);
    return Reflector;
}

void CSimpleReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    Texture::SharedPtr pPos = renderData[kInputWorldPosition]->asTexture();
    Texture::SharedPtr pNorm = renderData[kInputWorldNormal]->asTexture();
    Texture::SharedPtr pDiff = renderData[kInputDiffuse]->asTexture();
    Texture::SharedPtr pSpec = renderData[kInputSpecular]->asTexture();
    Texture::SharedPtr pOutput = renderData[kOutput]->asTexture();
    Texture::SharedPtr pCurrReservoir = renderData[kCurrReservoir]->asTexture();
    Texture::SharedPtr pPrevReservoir = renderData[kPrevReservoir]->asTexture();
    Texture::SharedPtr pTemporalReservoir = renderData[kTemporalReservoir]->asTexture();
    Texture::SharedPtr pSpatialReservoir = renderData[kSpatialReservoir]->asTexture();

    if (m_pScene)
    {
        m_pScene->update(pRenderContext, gpFramework->getGlobalClock().getTime());

        if (!m_pInitReservoirVars || !m_pReuseVars || !m_pFinalVars) __prepareVars();
        m_LastCameraMatrix = m_CurrCameraMatrix;
        m_CurrCameraMatrix = m_pScene->getCamera()->getViewProjMatrix();

        // Init and temporal reuse
        static bool IsInit = true;
        static uint FrameCount = 0;
        m_pInitReservoirVars["PerFrameCB"]["FrameCount"] = FrameCount++;
        m_pInitReservoirVars["PerFrameCB"]["IsInitLight"] = IsInit;
        m_pInitReservoirVars["PerFrameCB"]["LastCameraMatrix"] = m_LastCameraMatrix;
        m_pInitReservoirVars["PerFrameCB"]["CandidateCount"] = m_CandiateCount;
        m_pInitReservoirVars["PerFrameCB"]["ReservoirPerPixel"] = m_ReservoirPerPixel;
        m_pInitReservoirVars["PerFrameCB"]["IsTemporalReuse"] = m_IsTemporalReuse;
        m_pInitReservoirVars["PerFrameCB"]["Unbiased"] = m_Unbiased;

        m_pInitReservoirVars["Pos"] = pPos;
        m_pInitReservoirVars["Norm"] = pNorm;
        m_pInitReservoirVars["Diffuse"] = pDiff;
        m_pInitReservoirVars["Specular"] = pSpec;
        m_pInitReservoirVars["ReservoirTemporal"] = pTemporalReservoir;;
        m_pInitReservoirVars["ReservoirCurr"] = pCurrReservoir;
        m_pInitReservoirVars["ReservoirPrev"] = pPrevReservoir;
        m_pScene->raytrace(pRenderContext, m_pInitReservoirProgram.get(), m_pInitReservoirVars, uint3(WINDOW_WIDTH, WINDOW_HEIGHT, 1));
        IsInit = false;

        // Spatial reuse
        m_pReuseVars["PerFrameCB"]["IsSpatialReuse"] = m_IsSpatialReuse;
        m_pReuseVars["PerFrameCB"]["ReservoirPerPixel"] = m_ReservoirPerPixel;
        m_pReuseVars["PerFrameCB"]["NeighborCount"] = m_NeighborCount;
        m_pReuseVars["PerFrameCB"]["NeighborsRange"] = m_NeighborRange;
        m_pReuseVars["PerFrameCB"]["FrameCount"] = FrameCount;
        m_pReuseVars["PerFrameCB"]["Resolution"] = uint2(WINDOW_WIDTH, WINDOW_HEIGHT);
        m_pReuseVars["PerFrameCB"]["Unbiased"] = m_Unbiased;

        m_pReuseVars["ReservoirTemporal"] = pTemporalReservoir;
        m_pReuseVars["ReservoirSpatial"] = pSpatialReservoir;
        m_pReuseVars["Pos"] = pPos;
        m_pReuseVars["Norm"] = pNorm;
        m_pReuseVars["Diffuse"] = pDiff;
        m_pReuseVars["Specular"] = pSpec;
        m_pScene->raytrace(pRenderContext, m_pReuseProgram.get(), m_pReuseVars, uint3(WINDOW_WIDTH, WINDOW_HEIGHT, 1));

        // Final
        pRenderContext->clearUAV(m_pPrevReservoir->getUAV().get(), float4(0.0f));
        m_pFinalVars["PerFrameCB"]["ReservoirPerPixel"] = m_ReservoirPerPixel;
        m_pFinalVars["PerFrameCB"]["Unbiased"] = m_Unbiased;
        m_pFinalVars["ReservoirSpatial"] = pSpatialReservoir;
        m_pFinalVars["ReservoirPrev"] = pPrevReservoir;
        m_pFinalVars["Pos"] = pPos;
        m_pFinalVars["Norm"] = pNorm;
        m_pFinalVars["Diffuse"] = pDiff;
        m_pFinalVars["Specular"] = pSpec;
        m_pFinalVars["Output"] = pOutput;

        m_pScene->raytrace(pRenderContext, m_pFinalProgram.get(), m_pFinalVars, uint3(WINDOW_WIDTH, WINDOW_HEIGHT, 1));
    }
}

void CSimpleReSTIR::renderUI(Gui::Widgets& widget)
{
    /*widget.checkbox("Temporal Reuse", m_IsTemporalReuse);
    widget.checkbox("Spatail Reuse", m_IsSpatialReuse);

    widget.separator();
    widget.var<uint32_t>("Candidate", m_CandiateCount, 1u, 32u, 1u);
    widget.var<uint32_t>("Reservoir per pixel", m_ReservoirPerPixel, 1u, 8u, 1u);
    widget.var<uint32_t>("NeighborCount", m_NeighborCount, 1, 10, 1);
    widget.var<float>("Neighbor Reuse Range", m_NeighborRange, 1, 100, 1);
    widget.checkbox("Unbiased", m_Unbiased);*/
}

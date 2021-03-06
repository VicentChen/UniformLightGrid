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
#include "PositionNormalFwidth.h"


namespace
{
    const char kDesc[] = "Position and normal fwidth";

    // shader file
    const char kPositionNormalFwidthFile[] = "RenderPasses/PositionNormalFwidth/PositionNormalFwidth.ps.slang";

    // Input buffer names
    const char kPositon[] = "PosW";
    const char kNormal[] = "NormW";

    // Output buffer names
    const char kPositionNormalFwidth[] = "PositionNormalFwidth";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("PositionNormalFwidth", kDesc, PositionNormalFwidth::create);
}

PositionNormalFwidth::PositionNormalFwidth()
{
    mpPositionNomralFwidth = FullScreenPass::create(kPositionNormalFwidthFile, Program::DefineList(), 0);
}


PositionNormalFwidth::SharedPtr PositionNormalFwidth::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new PositionNormalFwidth());
    return pPass;
}

std::string PositionNormalFwidth::getDesc() { return kDesc; }

Dictionary PositionNormalFwidth::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection PositionNormalFwidth::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kPositon, "World Position");
    reflector.addInput(kNormal, "World Normal");
    reflector.addOutput(kPositionNormalFwidth, "PositonNormalFwidth").format(ResourceFormat::RG32Float).
        bindFlags(ResourceBindFlags::UnorderedAccess);
    return reflector;
}

void PositionNormalFwidth::compile(RenderContext* pContext, const CompileData& compileData)
{
    Fbo::Desc desc;
    desc.setColorTarget(0, Falcor::ResourceFormat::RG32Float);
    mpFbo = Fbo::create2D(compileData.defaultTexDims.x, compileData.defaultTexDims.y, desc);
}

void PositionNormalFwidth::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData["src"]->asTexture();
    Texture::SharedPtr pPosition = renderData[kPositon]->asTexture();
    Texture::SharedPtr pNormal = renderData[kNormal]->asTexture();
    Texture::SharedPtr pPositionNormalFwidth = renderData[kPositionNormalFwidth]->asTexture();

    mpPositionNomralFwidth["gPosW"] = pPosition;
    mpPositionNomralFwidth["gNormW"] = pNormal;
    //mpPositionNomralFwidth["gPositionNormalFwidth"] = pPositionNormalFwidth;

    mpPositionNomralFwidth->execute(pRenderContext, mpFbo);
    pRenderContext->blit(mpFbo->getColorTexture(0)->getSRV(), pPositionNormalFwidth->getRTV());
}

void PositionNormalFwidth::renderUI(Gui::Widgets& widget)
{
}


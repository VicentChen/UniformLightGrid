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
#include "FalcorExperimental.h"

using namespace Falcor;

class CSimpleReSTIR : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<CSimpleReSTIR>;

    /** Create a new render pass object.
        \param[in] pRenderContext The render context.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    CSimpleReSTIR(const Dictionary& vDict);
    void __prepareShaders();
    void __prepareVars();
    void __setModelString(double vLoadTime);
    void __setCamController();

    Scene::SharedPtr m_pScene;

    /*****************
     *Init Reservoir
     *****************/
    uint32_t m_CandiateCount = 32;
    uint32_t m_ReservoirPerPixel = 1;
    bool m_Unbiased = false;

    RtProgram::SharedPtr m_pInitReservoirProgram = nullptr;
    RtProgramVars::SharedPtr m_pInitReservoirVars = nullptr;

    Texture::SharedPtr m_pCurrReservoir = nullptr;
    Texture::SharedPtr m_pTemporalReservoir = nullptr;
    Texture::SharedPtr m_pPrevReservoir = nullptr;
    Texture::SharedPtr m_pSpatailReservoir = nullptr;

    /*****************
     *Spatial and Temporal Reuse
     *****************/
    bool m_IsSpatialReuse = false;
    bool m_IsTemporalReuse = false;
    uint m_NeighborCount = 1;
    float m_NeighborRange = 30;
    float4x4 m_CurrCameraMatrix = float4x4(0.0);
    float4x4 m_LastCameraMatrix = float4x4(0.0);

    RtProgram::SharedPtr m_pReuseProgram = nullptr;
    RtProgramVars::SharedPtr m_pReuseVars = nullptr;

    /*****************
    *Final Pass
    *****************/
    RtProgram::SharedPtr m_pFinalProgram = nullptr;
    RtProgramVars::SharedPtr m_pFinalVars = nullptr;
};

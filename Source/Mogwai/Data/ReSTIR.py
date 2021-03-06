from falcor import *

def render_graph_DefaultRenderGraph():
    g = RenderGraph('DefaultRenderGraph')
    loadRenderPassLibrary('ErrorMeasurePass.dll')
    loadRenderPassLibrary('BSDFViewer.dll')
    loadRenderPassLibrary('SimpleReSTIR.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('Antialiasing.dll')
    loadRenderPassLibrary('BlitPass.dll')
    loadRenderPassLibrary('CSM.dll')
    loadRenderPassLibrary('DebugPasses.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('ForwardLightingPass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('ImageLoader.dll')
    loadRenderPassLibrary('MegakernelPathTracer.dll')
    loadRenderPassLibrary('MinimalPathTracer.dll')
    loadRenderPassLibrary('PassLibraryTemplate.dll')
    loadRenderPassLibrary('PixelInspectorPass.dll')
    loadRenderPassLibrary('SceneDebugger.dll')
    loadRenderPassLibrary('SkyBox.dll')
    loadRenderPassLibrary('SSAO.dll')
    loadRenderPassLibrary('SVGFPass.dll')
    loadRenderPassLibrary('TemporalDelayPass.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    loadRenderPassLibrary('Utils.dll')
    loadRenderPassLibrary('WhittedRayTracer.dll')
    CSimpleReSTIR = createPass('CSimpleReSTIR', {'CandiateCount': 32, 'ReservoirPerPixel': 1, 'Unbiased': False, 'SpatialReuse': False, 'TemporalReuse': False, 'NegihbotCount': 1, 'NeighborRange': 30.0})
    g.addPass(CSimpleReSTIR, 'CSimpleReSTIR')
    GBufferRT = createPass('GBufferRT', {'samplePattern': SamplePattern.Center, 'sampleCount': 16, 'disableAlphaTest': False, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack, 'texLOD': LODMode.UseMip0})
    g.addPass(GBufferRT, 'GBufferRT')
    g.addEdge('GBufferRT.posW', 'CSimpleReSTIR.WorldPosition')
    g.addEdge('GBufferRT.normW', 'CSimpleReSTIR.WorldNormal')
    g.addEdge('GBufferRT.diffuseOpacity', 'CSimpleReSTIR.Diffuse')
    g.addEdge('GBufferRT.specRough', 'CSimpleReSTIR.Specular')
    g.markOutput('CSimpleReSTIR.ReSTIROutput')
    return g

DefaultRenderGraph = render_graph_DefaultRenderGraph()
try: m.addGraph(DefaultRenderGraph)
except NameError: None

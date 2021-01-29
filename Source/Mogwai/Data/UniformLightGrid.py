from falcor import *

def render_graph_PathTracerGraph():
    g = RenderGraph('PathTracerGraph')
    loadRenderPassLibrary('ErrorMeasurePass.dll')
    loadRenderPassLibrary('BSDFViewer.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('Antialiasing.dll')
    loadRenderPassLibrary('BlitPass.dll')
    loadRenderPassLibrary('CSM.dll')
    loadRenderPassLibrary('DebugPasses.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('ForwardLightingPass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('ImageLoader.dll')
    loadRenderPassLibrary('SVGFPass.dll')
    loadRenderPassLibrary('MegakernelPathTracer.dll')
    loadRenderPassLibrary('MinimalPathTracer.dll')
    loadRenderPassLibrary('PixelInspectorPass.dll')
    loadRenderPassLibrary('PassLibraryTemplate.dll')
    loadRenderPassLibrary('PositionNormalFwidth.dll')
    loadRenderPassLibrary('ReSTIR.dll')
    loadRenderPassLibrary('SceneDebugger.dll')
    loadRenderPassLibrary('SkyBox.dll')
    loadRenderPassLibrary('SSAO.dll')
    loadRenderPassLibrary('TemporalDelayPass.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    loadRenderPassLibrary('UniformLightGrid.dll')
    loadRenderPassLibrary('Utils.dll')
    loadRenderPassLibrary('WhittedRayTracer.dll')
    AccumulatePass = createPass('AccumulatePass', {'enableAccumulation': False, 'autoReset': True, 'precisionMode': AccumulatePrecision.Single, 'subFrameCount': 0})
    g.addPass(AccumulatePass, 'AccumulatePass')
    ToneMappingPass = createPass('ToneMapper', {'exposureCompensation': 0.0, 'autoExposure': True, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Reinhard, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMappingPass, 'ToneMappingPass')
    UniformLightGrid = createPass('UniformLightGrid', {'mSharedParams': PathTracerParams(samplesPerPixel=1, lightSamplesPerVertex=1, maxNonSpecularBounces=0, maxBounces=0, adjustShadingNormals=0, useVBuffer=0, forceAlphaOne=1, useAlphaTest=1, clampSamples=0, useMIS=1, clampThreshold=10.0, useLightsInDielectricVolumes=0, specularRoughnessThreshold=0.25, useBRDFSampling=1, useNestedDielectrics=1, useNEE=1, misHeuristic=1, misPowerExponent=2.0, probabilityAbsorption=0.20000000298023224, useRussianRoulette=0, useFixedSeed=0, useLegacyBSDF=0, disableCaustics=0, rayFootprintMode=0, rayConeMode=2, rayFootprintUseRoughness=0), 'mSelectedSampleGenerator': 1, 'mSelectedEmissiveSampler': EmissiveLightSamplerType.LightBVH, 'mUniformSamplerOptions': EmissiveUniformSamplerOptions(), 'mLightBVHSamplerOptions': LightBVHSamplerOptions(useBoundingCone=True, buildOptions=LightBVHBuilderOptions(splitHeuristicSelection=SplitHeuristic.BinnedSAOH, maxTriangleCountPerLeaf=10, binCount=16, volumeEpsilon=0.0010000000474974513, useLeafCreationCost=True, createLeavesASAP=True, useLightingCones=True, splitAlongLargest=False, useVolumeOverSA=False, allowRefitting=True, usePreintegration=True), useLightingCone=True, disableNodeFlux=False, useUniformTriangleSampling=True, solidAngleBoundMethod=SolidAngleBoundMethod.Sphere)})
    g.addPass(UniformLightGrid, 'UniformLightGrid')
    SVGFPass = createPass('SVGFPass', {'Enabled': True, 'Iterations': 4, 'FeedbackTap': 1, 'VarianceEpsilon': 9.999999747378752e-05, 'PhiColor': 10.0, 'PhiNormal': 128.0, 'Alpha': 0.05000000074505806, 'MomentsAlpha': 0.20000000298023224})
    g.addPass(SVGFPass, 'SVGFPass')
    GBufferRasterWithView = createPass('GBufferRasterWithView', {'samplePattern': SamplePattern.Center, 'sampleCount': 16, 'disableAlphaTest': False, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack})
    g.addPass(GBufferRasterWithView, 'GBufferRasterWithView')
    g.addEdge('AccumulatePass.output', 'ToneMappingPass.src')
    g.addEdge('UniformLightGrid.color', 'SVGFPass.Color')
    g.addEdge('UniformLightGrid.albedo', 'SVGFPass.Albedo')
    g.addEdge('SVGFPass.Filtered image', 'AccumulatePass.input')
    g.addEdge('GBufferRasterWithView.posW', 'UniformLightGrid.posW')
    g.addEdge('GBufferRasterWithView.normW', 'UniformLightGrid.normalW')
    g.addEdge('GBufferRasterWithView.tangentW', 'UniformLightGrid.tangentW')
    g.addEdge('GBufferRasterWithView.diffuseOpacity', 'UniformLightGrid.mtlDiffOpacity')
    g.addEdge('GBufferRasterWithView.specRough', 'UniformLightGrid.mtlSpecRough')
    g.addEdge('GBufferRasterWithView.emissive', 'UniformLightGrid.mtlEmissive')
    g.addEdge('GBufferRasterWithView.matlExtra', 'UniformLightGrid.mtlParams')
    g.addEdge('GBufferRasterWithView.vbuffer', 'UniformLightGrid.vbuffer')
    g.addEdge('GBufferRasterWithView.faceNormalW', 'UniformLightGrid.faceNormalW')
    g.addEdge('GBufferRasterWithView.ViewW', 'UniformLightGrid.viewW')
    g.addEdge('GBufferRasterWithView.emissive', 'SVGFPass.Emission')
    g.addEdge('GBufferRasterWithView.posW', 'SVGFPass.WorldPosition')
    g.addEdge('GBufferRasterWithView.normW', 'SVGFPass.WorldNormal')
    g.addEdge('GBufferRasterWithView.pnFwidth', 'SVGFPass.PositionNormalFwidth')
    g.addEdge('GBufferRasterWithView.linearZ', 'SVGFPass.LinearZ')
    g.addEdge('GBufferRasterWithView.mvec', 'SVGFPass.MotionVec')
    g.markOutput('ToneMappingPass.dst')
    return g

PathTracerGraph = render_graph_PathTracerGraph()
try: m.addGraph(PathTracerGraph)
except NameError: None

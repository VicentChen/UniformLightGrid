#include "PathCommon.slangh"
import Scene.Scene;
import Scene.Shading;
import Scene.Raster;
import Scene.Raytracing;
import Scene.Lights.Lights;
import RenderPasses.Shared.PathTracer.LoadShadingData;
import Scene.HitInfo;
import Experimental.Scene.Material.MaterialShading;

RWTexture2D<float4> gOutputColor;

[shader("raygeneration")]
void rayGen()
{
	// // Get our pixel's position on the screen
	uint2 LaunchIndex = DispatchRaysIndex().xy;
	uint2 LaunchDim = DispatchRaysDimensions().xy;
    SampleGenerator Sg = SampleGenerator.create(LaunchIndex, FrameCount * DispatchRaysIndex().z + FrameCount);
    HitInfo hit;
    ShadingData sd;
    float3 ShadeColor = 0;//DiffColor.rgb;  

    if (loadShadingData(LaunchIndex, LaunchDim, gScene.camera, sd, hit))
    {
        
        // If we don't hit any geometry, our difuse material contains our background color.

        ShadeColor = float3(0.0f);
        for (int i = 0; i < ReservoirPerPixel; i++)
        {
            float4 Reservoir = ReservoirTemporal[uint3(LaunchIndex, i)];
            float2 u = uFinalTemporal[uint3(LaunchIndex, i)].xy;
            float pdf = uFinalTemporal[uint3(LaunchIndex, i)].z;
            
            uint xi = 0;

            if (IsSpatialReuse)
            {
                float4 SpatialRes = float4(0.0f);
                uint2 NeighborOffset;
                uint2 NeighborIndex;
                float4 NeighborRes = float4(0);	
                float2 Neighboru = float2(0);
                float  NeighborPdf = 0;
                // r : y
                // g : weight sum
                // b : M
                // a : W
                
                float4 Res2Combine[8];
                //float3 PixelPos[8];
                float2 uArray[8];
                float pdfArray[8];
                Res2Combine[0] = Reservoir;
                uArray[0] = u;
                pdfArray[0] = pdf;

                //PixelPos[0] = WorldPos.xyz;
                for(int k = 0; k < NeighborCount && k < 7; k++)
                {
                    NeighborOffset.x = int(sampleNext1D(Sg) * NeighborsRange * 2.f) - NeighborsRange;
                    NeighborOffset.y = int(sampleNext1D(Sg) * NeighborsRange * 2.f) - NeighborsRange;	
                    NeighborIndex.x = max(0, min(LaunchDim.x - 1, LaunchIndex.x + NeighborOffset.x));
                    NeighborIndex.y = max(0, min(LaunchDim.y - 1, LaunchIndex.y + NeighborOffset.y));	
                    NeighborRes = ReservoirTemporal[uint3(NeighborIndex,i)];	
                    Neighboru = uFinalTemporal[uint3(NeighborIndex,i)].xy;
                    NeighborPdf = uFinalTemporal[uint3(NeighborIndex,i)].z;
                    Res2Combine[k + 1] = NeighborRes;
                    uArray[k + 1] = Neighboru;
                    pdfArray[k + 1] = NeighborPdf;
                    //PixelPos[k + 1] = Pos[NeighborIndex].xyz;
                }

                // if (Unbiased)
                // {
                //     SpatialRes = CombineResservoirsUnbiased(Sd, Res2Combine, PixelPos, min(NeighborCount, 7) + 1, Sg);
                // }
                // else
                SpatialRes = CombineReservoirs(sd, Sg, Res2Combine, uArray, pdfArray, min(NeighborCount, 7) + 1, u, pdf);

                ReservoirSpatial[uint3(LaunchIndex, i)] = SpatialRes;	
                uFinalSpatial[uint3(LaunchIndex, i)] = float4(u, pdf, 1.0f);
            }
            else
            {
                ReservoirSpatial[uint3(LaunchIndex, i)] = ReservoirTemporal[uint3(LaunchIndex,i)];
                uFinalSpatial[uint3(LaunchIndex, i)] = uFinalTemporal[uint3(LaunchIndex, i)];
            }

            Reservoir = ReservoirSpatial[uint3(LaunchIndex, i)];
            u = uFinalSpatial[uint3(LaunchIndex, i)].xy;
            pdf = uFinalSpatial[uint3(LaunchIndex, i)].z;
            xi = Reservoir.r;
            ReservoirPrev[uint3(LaunchIndex, i)] = Reservoir; // Update reservoir value to be used for next pass
            uFinalPrev[uint3(LaunchIndex, i)] = float4(u, pdf, 1.0f);
            float3 Color = evalColor(xi, u, pdf, sd) * Reservoir.w;
            if (isnan(length(Color)) || isinf(length(Color))) Color = float3(0, 0, 0);
            ShadeColor += Color;
            // if (!Unbiased)
            //     ReservoirPrev[uint3(LaunchIndex, i)] = Reservoir;
        }
        ShadeColor /= (float)ReservoirPerPixel;
        ShadeColor += gMaterialEmissive[LaunchIndex].xyz;
        //ShadeColor = ShadeColor > 10.0f ? 10.0f : ShadeColor;
    }
	gOutputColor[LaunchIndex] = float4(ShadeColor, 1.f);
}
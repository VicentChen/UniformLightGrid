//#include "PathCommon.slangh"
import Scene.Scene;
import Scene.Shading;
import Scene.Raster;
import Scene.Raytracing;
import Scene.Lights.Lights;
import RenderPasses.Shared.PathTracer.LoadShadingData;
import Scene.HitInfo;
import Experimental.Scene.Material.MaterialShading;
RWTexture2DArray<float4> ReservoirCurr;
//RWTexture2DArray<float4> ReservoirSpatial;
RWTexture2DArray<float4> ReservoirPrev;
RWTexture2DArray<float4> FinalLightPos;
RWTexture2DArray<float4> FinalLightLi;

RWTexture2D<float4> gOutputColor;

cbuffer PerFrameCB
{		
	uint ReservoirPerPixel;
	bool Unbiased;
}

[shader("raygeneration")]
void rayGen()
{
	// // Get our pixel's position on the screen
	 uint2 LaunchIndex = DispatchRaysIndex().xy;
	uint2 LaunchDim = DispatchRaysDimensions().xy;

    HitInfo hit;
    ShadingData sd;
    float3 ShadeColor = 0;//DiffColor.rgb;
    if (loadShadingData(LaunchIndex, LaunchDim, gScene.camera, sd, hit))
    {
        // If we don't hit any geometry, our difuse material contains our background color.
        uint xi = 0;

        ShadeColor = float3(0.0f);
        for (int i = 0; i < ReservoirPerPixel; i++)
        {
            float4 Reservoir = ReservoirCurr[uint3(LaunchIndex, i)];
            //ReservoirPrev[uint3(LaunchIndex, i)] = Reservoir; // Update reservoir value to be used for next pass
            xi = Reservoir.r;
            float3 Li = FinalLightLi[uint3(LaunchIndex,i)].xyz;
            if (length(Li) < 0.000001f) continue;
            float3 Dir = normalize(FinalLightPos[uint3(LaunchIndex,i)].xyz - sd.posW);
            // float3 Temp = evalBSDFCosine(sd, Dir) * Li;// * Reservoir.w;
            // Temp = Temp > 10.0f ? 10.0f : Temp;
            ShadeColor += evalBSDFCosine(sd, Dir) * Li * Reservoir.w;
            // if (!Unbiased)
            //     ReservoirPrev[uint3(LaunchIndex, i)] = Reservoir;
        }
        ShadeColor /= (float)ReservoirPerPixel;
        ShadeColor += gMaterialEmissive[LaunchIndex].xyz;
        //ShadeColor = ShadeColor > 10.0f ? 10.0f : ShadeColor;
    }
	gOutputColor[LaunchIndex] = float4(ShadeColor, 1.f);
}
#include "Utils/Math/MathConstants.slangh"
#include "ReSTIRCommon.slangh"
import Scene.Scene;
import Scene.Shading;
import Scene.Raster;
import Scene.Raytracing;
import Scene.Lights.Lights;

RWTexture2DArray<float4> ReservoirSpatial;
RWTexture2DArray<float4> ReservoirPrev;

Texture2D<float4> Pos; 
Texture2D<float4> Norm;
Texture2D<float4> Diffuse;
Texture2D<float4> Specular;
Texture2D<float4> Tangent;
Texture2D<float4> Bitangent;
RWTexture2D<float4> Output;

cbuffer PerFrameCB
{
	uint ReservoirPerPixel;
	bool Unbiased;
}

[shader("raygeneration")]
void rayGen()
{
	// Get our pixel's position on the screen
	uint2 LaunchIndex = DispatchRaysIndex().xy;
	uint2 LaunchDim = DispatchRaysDimensions().xy;

	// Load g-buffer data:  world-space position, normal, and diffuse color
	float4 WorldPos = Pos[LaunchIndex];
	float4 WorldNorm = Norm[LaunchIndex];
	float4 DiffColor = Diffuse[LaunchIndex];
	float4 SpecColor = Specular[LaunchIndex];
	// float4 T = Tangent[LaunchIndex];
	// float4 B = Bitangent[LaunchIndex];
	ResVertexData VData = {};
	VData.Pos = WorldPos.xyz;
	VData.Normal = WorldNorm.xyz;
	VData.DiffColor = DiffColor.xyz;
	VData.SpecColor = SpecColor.xyz;
	VData.LinearRoughness = SpecColor.w;
	// VData.T = T.xyz;
	// VData.B = B.xyz;

	float3 ViewDir = normalize(gScene.camera.getPosition() - WorldPos.xyz);
	ShadingData Sd = prepareResShadingData(VData, ViewDir);

	// If we don't hit any geometry, our difuse material contains our background color.
	float3 ShadeColor = 0;//DiffColor.rgb;
	float3 LightIntensity = float3(0.0f);
	float3 ToLight = float3(0.0);
	float Dis2Light = 0.0f;
	float LdotN = 0.0f;
	uint xi = 0;

	// Our camera sees the background if worldPos.w is 0, only do diffuse shading elsewhere
	if (WorldPos.w != 0.0f)
	{
		ShadeColor = float3(0.0f);
		for (int i = 0; i < ReservoirPerPixel; i++)
		{
			float4 Reservoir = ReservoirSpatial[uint3(LaunchIndex, i)];
			//ReservoirPrev[uint3(LaunchIndex, i)] = Reservoir; // Update reservoir value to be used for next pass
			xi = Reservoir.r;
			
			float Shadow = checkLightHit(WorldPos.xyz, gScene.getLight(xi).posW) ? 0.0 : 1.0f;
			ShadeColor += evalMaterial(Sd, gScene.getLight(xi),1).color * Reservoir.w * 1.f * gScene.getLightCount();
			if (!Unbiased)
				ReservoirPrev[uint3(LaunchIndex, i)] = Reservoir;
		}
		ShadeColor /= (float)ReservoirPerPixel;
	}

	Output[LaunchIndex] = float4(ShadeColor, 1.f);
}
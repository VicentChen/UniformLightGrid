#include "Utils/Math/MathConstants.slangh"
#include "ReSTIRCommon.slangh"
import Scene.Scene;
import Scene.Shading;
import Scene.Raster;
import Scene.Raytracing;
import Scene.Lights.Lights;

cbuffer PerFrameCB
{
	//float MinT;        // Min distance to start a ray to avoid self-occlusion
	uint  FrameCount;  // Frame counter, used to perturb random seed each frame
	bool  IsInitLight;		// For ReSTIR - to choose an arbitrary light for this pixel after choosing 32 random light candidates
	bool IsTemporalReuse;
	float4x4 LastCameraMatrix;
	uint CandidateCount;
	uint ReservoirPerPixel;
	bool Unbiased;
}

Texture2D<float4> Pos;           // G-buffer world-space position
Texture2D<float4> Norm;          // G-buffer world-space normal
Texture2D<float4> Diffuse;   // G-buffer diffuse material (RGB) and opacity (A)
Texture2D<float4> Specular;  // G-buffer specular color (RGB) and linearRoughness (A) 
Texture2D<float4> Tangent;
Texture2D<float4> Bitangent;
Texture2D<float4> PrevPos;
RWTexture2DArray<float4> ReservoirTemporal;
RWTexture2DArray<float4> ReservoirCurr;		// For ReSTIR - need to be read-write because it is also updated in the shader as wellRWTexture2D<float4> gOutput; 
RWTexture2DArray<float4> ReservoirPrev;
//TODO: EnvMap

[shader("raygeneration")]
void rayGen()
{
	uint2 LaunchIndex = DispatchRaysIndex().xy;
	uint2 LaunchDimension = DispatchRaysDimensions().xy;

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
	SampleGenerator Sg = SampleGenerator.create(LaunchIndex, FrameCount * DispatchRaysIndex().z + FrameCount);

	if (WorldPos.w != 0.0f)
	{
		// r : y
		// g : weight sum
		// b : M
		// a : W
		float4 CurrRes = float4(0.0);
		float3 LightIntensity = float3(0.0f);
		float3 ToLight = float3(0.0);
		float Dis2Light = 0.0f;
		float LdotN = 0.0f;
		uint xi = 0;
		//Temporal reuse index
		float4 ScreenSpace = mul(WorldPos, LastCameraMatrix);
		ScreenSpace /= ScreenSpace.w;
		uint2 PrevIndex = LaunchIndex.xy;
		PrevIndex.x = ((ScreenSpace.x + 1.f) / 2.f) * (float)LaunchDimension.x;
		PrevIndex.y = ((1.f - ScreenSpace.y) / 2.f) * (float)LaunchDimension.y;	
		for (int i = 0; i < ReservoirPerPixel; i++)
		{
			//Gen candidate
			CurrRes = RIS(Sd, CandidateCount, Sg);
			// float CurrPHat = length(evalMaterial(Sd, gScene.getLight(CurrRes.r), 1).color);
			// LightSample Ls = evalLight(gScene.getLight(CurrRes.r), Sd);
			// float3 BRDF = evalSpecularBrdf(Sd, Ls);
			// ReservoirCurr[uint3(LaunchIndex, 4)] = length(Ls.specular * Ls.NdotL * BRDF);
			// ReservoirCurr[uint3(LaunchIndex, 5)] = length(evalMaterial(Sd, gScene.getLight(CurrRes.r), 1).specular);
			// ReservoirCurr[uint3(LaunchIndex, 6)] = length(Ls.diffuse * Ls.NdotL * Sd.diffuse / M_PI);

			//Reweight W
			xi = CurrRes.r;
			
			// If occlusion, set W to 0
			if (checkLightHit(WorldPos.xyz, gScene.getLight(xi).posW))
				CurrRes.w = 0;	
			ReservoirCurr[uint3(LaunchIndex, i)] = CurrRes;	
			
			if (IsTemporalReuse && !IsInitLight)
			{
				if(PrevIndex.x >= 0 && PrevIndex.x < 1920 && PrevIndex.y >= 0 && PrevIndex.y < 1080)
				{					
					float4 Res2Combine[8];
					float3 PixelPos[8];

					Res2Combine[0] = CurrRes;
					PixelPos[0] = WorldPos.xyz;
					Res2Combine[1] = ReservoirPrev[uint3(PrevIndex,i)];
					PixelPos[1] = PrevPos[PrevIndex].xyz;
					// Res2Combine[1] = ReservoirPrev[uint3(LaunchIndex,i)];
					// PixelPos[1] = PrevPos[LaunchIndex].xyz;
					ReservoirCurr[uint3(LaunchIndex, 5)] = Res2Combine[1];
					float4 Res = float4(0.0f);
					if (Unbiased)
					{
						Res = CombineResservoirsUnbiased(Sd, Res2Combine, PixelPos, 2, Sg);
						//ReservoirCurr[uint3(LaunchIndex, 2)] = float4(M,Z,0,0);
						ReservoirPrev[uint3(LaunchIndex, i)] = CurrRes;
					}
					else
						Res = CombineReservoirs(Sd, Res2Combine, 2, Sg);	

					ReservoirTemporal[uint3(LaunchIndex, i)] = Res;
				}
			}
			else
			{
				for (int i = 0; i < ReservoirPerPixel; i++)
				{
					ReservoirPrev[uint3(LaunchIndex, i)] = float4(0.0f);
					ReservoirTemporal[uint3(LaunchIndex, i)] = CurrRes;
				}
			}
			CurrRes = float4(0.0f);
		}
	}
	else
	{
			for (int i = 0; i < ReservoirPerPixel; i++)
			{
				ReservoirCurr[uint3(LaunchIndex, i)] = float4(0.0f);
			}
	}
}
#include "ReSTIRCommon.slangh"
#include "Utils/Math/MathConstants.slangh"
import Scene.Scene;
import Scene.Shading;
import Scene.Raster;
import Scene.Raytracing;
import Scene.Lights.Lights;
cbuffer PerFrameCB
{
	bool IsSpatialReuse;
	uint ReservoirPerPixel;
	uint NeighborCount;
	float NeighborsRange;
	uint FrameCount;
	uint2 Resolution;
	bool Unbiased;
}

RWTexture2DArray<float4> ReservoirTemporal;
RWTexture2DArray<float4> ReservoirSpatial;

Texture2D<float4> Pos; 
Texture2D<float4> Norm;
Texture2D<float4> Diffuse;
Texture2D<float4> Specular;
Texture2D<float4> Tangent;
Texture2D<float4> Bitangent;


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

	if(WorldPos.w == 0.0) 
	{
		for(int i = 0; i < ReservoirPerPixel; i++)
			ReservoirSpatial[uint3(LaunchIndex, i)] = ReservoirTemporal[uint3(LaunchIndex,i)];
		return;
	}	

	if (IsSpatialReuse)
	{
		float4 SpatialRes = float4(0.0f);
		float3 Color = float3(0.0f);
		float pHat = 0;	
		uint2 NeighborOffset;
		uint2 NeighborIndex;
		float4 NeighborRes;	
		for(int i = 0; i < ReservoirPerPixel; i++)
		{
			// r : y
			// g : weight sum
			// b : M
			// a : W
			
			float4 Res2Combine[8];
			float3 PixelPos[8];
			Res2Combine[0] = ReservoirTemporal[uint3(LaunchIndex,i)];
			ReservoirTemporal[uint3(LaunchIndex, 6)] = Res2Combine[0];
			PixelPos[0] = WorldPos.xyz;
			for(int k = 0; k < NeighborCount && k < 7; k++)
			{
				NeighborOffset.x = int(sampleNext1D(Sg) * NeighborsRange * 2.f) - NeighborsRange;
				NeighborOffset.y = int(sampleNext1D(Sg) * NeighborsRange * 2.f) - NeighborsRange;	
				NeighborIndex.x = max(0, min(Resolution.x - 1, LaunchIndex.x + NeighborOffset.x));
				NeighborIndex.y = max(0, min(Resolution.y - 1, LaunchIndex.y + NeighborOffset.y));	
				NeighborRes = ReservoirTemporal[uint3(NeighborIndex,i)];	
				
				Res2Combine[k + 1] = ReservoirTemporal[uint3(NeighborIndex,i)];
				ReservoirTemporal[uint3(LaunchIndex, 7)] = Res2Combine[k + 1];
				PixelPos[k + 1] = Pos[NeighborIndex].xyz;
			}

			if (Unbiased)
			{
				SpatialRes = CombineResservoirsUnbiased(Sd, Res2Combine, PixelPos, min(NeighborCount, 7) + 1, Sg);
			}
			else
				SpatialRes = CombineReservoirs(Sd, Res2Combine, min(NeighborCount, 7) + 1, Sg);

			ReservoirSpatial[uint3(LaunchIndex, i)] = SpatialRes;	
		}
	}
	else
	{
		for (int i = 0; i < ReservoirPerPixel; i++)
			ReservoirSpatial[uint3(LaunchIndex, i)] = ReservoirTemporal[uint3(LaunchIndex,i)];
	}
}

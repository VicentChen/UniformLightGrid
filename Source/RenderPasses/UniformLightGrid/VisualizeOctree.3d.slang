struct VSIn
{
    float3 posW : POSITION;
};

struct VSOut
{
    float3 posW : POSW;
    float4 posH : SV_POSITION;
};

cbuffer PerFrameCB
{
    float3   camPos;
    float4x4 viewMat;
    float4x4 projMat;
    uint     instanceOffset;
}

Texture2D<float4> gPosW;
Texture2D<float4> gColor;
StructuredBuffer<float4x4> gInstanceWorldMats;

VSOut vsMain(VSIn vsIn, uint instanceID : SV_InstanceID)
{
    VSOut vsOut;
    float4x4 worldMat = gInstanceWorldMats[instanceOffset + instanceID];
    float4x4 worldViewProj = mul(mul(worldMat, viewMat), projMat);
    float4 posW = mul(float4(vsIn.posW, 1.0f), worldMat);
    float4 posH = mul(float4(vsIn.posW, 1.0f), worldViewProj);
    vsOut.posW = posW.xyz;
    vsOut.posH = posH;
    return vsOut;
}

float4 psMain(VSOut vsOut) : SV_TARGET
{
    float3 posW = gPosW[vsOut.posH.xy].xyz;
    float4 color = gColor[vsOut.posH.xy];
    if (distance(vsOut.posW, camPos) < distance(posW, camPos)) return float4(0, 1, 0, 1);
    else return color;
}
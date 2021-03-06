struct VSIn
{
    float3 posW : POSITION;
};

struct VSOut
{
    uint   nodeIdx : NODE_IDX;
    float3 posW : POSW;
    float4 posH : SV_POSITION;
};

cbuffer PerFrameCB
{
    bool     isDrawingSingleNode;
    uint     singleNodeIdx;
    float3   camPos;
    float4x4 viewMat;
    float4x4 projMat;
    uint     instanceOffset; ///> not sure how to use "startInstanceLocation" in "DrawInstanced()"
}

// TODO: may be we should use depth?
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
    vsOut.nodeIdx = instanceOffset + instanceID;
    vsOut.posW = posW.xyz;
    vsOut.posH = posH;
    return vsOut;
}

float4 psMain(VSOut vsOut) : SV_TARGET
{
    float4 wireColor = (isDrawingSingleNode && vsOut.nodeIdx == singleNodeIdx) ? float4(0, 1, 1, 1) : float4(0, 1, 0, 1);
    float3 posW = gPosW[vsOut.posH.xy].xyz;
    float4 color = gColor[vsOut.posH.xy];
    if (distance(vsOut.posW, camPos) < distance(posW, camPos)) return wireColor;
    else return color;
}
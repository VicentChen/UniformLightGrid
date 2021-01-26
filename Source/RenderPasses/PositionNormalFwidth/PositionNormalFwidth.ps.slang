import Scene.ShadingData;
import Utils.Helpers;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;

Texture2D<float4> gPosW;
Texture2D<float4> gNormW;
//RWTexture2D<float2> gPositionNormalFwidth;

struct PositionNormalFwidth
{
    float2 PNFwidth : SV_TARGET0;
}

struct FullScreenPassVsOut
{
    float2 texC       : TEXCOORD;
#ifndef _VIEWPORT_MASK
    float4 posH       : SV_POSITION;
#else
    float4 posH       : POSITION;
#endif
};

PositionNormalFwidth main(FullScreenPassVsOut vsOut)
{
    int2 ipos = int2(vsOut.posH.xy);
    float3 posW = gPosW[ipos].xyz;
    float3 normW = gNormW[ipos].xyz;
    PositionNormalFwidth result = {};
    result.PNFwidth = float2(length(fwidth(posW)), length(fwidth(normW)));

    return result;
}
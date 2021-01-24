import Scene.Scene;
import RenderPasses.Shared.PathTracer.LoadShadingData;
import Experimental.Scene.Material.MaterialShading;
import Utils.Math.AABB;
import Utils.Math.BitTricks;
import Utils.Sampling.SampleGenerator;
import TraverseBVH;

#ifndef CHUNK_SIZE
    // Compile-time error if GROUP_SIZE is not defined.
    #error CHUNK_SIZE is not defined. Add define in cpp file.
#endif

cbuffer PerFrameCB
{
    uint2 dispatchDim;
    uint frameCount;

    float minDistance;
    uint samplesPerDirection;

    // Duplicate in TraverseBVH.slang
    // uint quantLevels;
    // AABB sceneBound;
}

// float4: triangleIndex, pdf, ??, ??
RWTexture2D<float4> gLightIndex;

/** Compute the near and far intersections of the cube (stored in the x and y components) using the slab method.
    No intersection means vec.x > vec.y (really tNear > tFar).
    https://gist.github.com/DomNomNom/46bb1ce47f68d255fd5d
*/
float intersectWithSceneBound(const float3 rayOrigin, const float3 rayDir)
{
    float3 tMin = (sceneBound.minPoint - rayOrigin) / rayDir;
    float3 tMax = (sceneBound.maxPoint - rayOrigin) / rayDir;
    float3 t1 = min(tMin, tMax);
    float3 t2 = max(tMin, tMax);
    // float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);
    return tFar;
}

void sampleDistanceUniformly<S : ISampleGenerator>(const float dmin, const float dmax, inout S sg, out float d, out float pdf)
{
    // avoid dmin == dmax
    float deltaD = dmax - dmin;
    if (deltaD < 1E-5)
    {
        d = 0;
        pdf = 1;
        return;
    }

    float x = sampleNext1D(sg);
    d = dmin + x * deltaD;
    pdf = 1 / deltaD;
}

void sampleDistanceByDistSquare<S : ISampleGenerator>(const float dmin, const float dmax, inout S sg, out float d, out float pdf)
{
    // avoid dmin == dmax
    if (dmax - dmin < 1E-5)
    {
        d = 0;
        pdf = 1;
        return;
    }

    // sample with CDF
    float c = dmin * dmax / (dmax - dmin);
    float x = sampleNext1D(sg);
    d = sqrt(c / x);
    pdf = c / (d * d);
}

uint computeMortonCodeByPos(const float3 pos)
{
    //normalize position to [0,1]
    float3 normPos = (pos - sceneBound.minPoint) / sceneBound.extent();
    uint3 quantPos = min(max(0, uint3(normPos * quantLevels)), quantLevels - 1);
    uint mortonCode = interleave_uint3(quantPos);
    return mortonCode;
}

void sampleRangeUniformly<S : ISampleGenerator>(const uint2 range, inout S sg, out uint idx, out float pdf)
{
    float delta = range.y - range.x + 1;
    float x = sampleNext1D(sg) * 0.999999f; // FIXME: hack to avoid x == 1
    idx = floor(range.x + x * delta);
    pdf = 1 / delta;
}

[numthreads(CHUNK_SIZE, CHUNK_SIZE, 1)]
void selectGridAndLight(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint2 launchIndex = dispatchThreadId.xy;
    uint2 launchDim = dispatchDim;
    if (any(launchIndex >= launchDim)) return;

    float4 result = 0;

    // random generator
    uint frameSeed = frameCount; // TODO: useFixedSeed ? 0 : frameCount;
    SampleGenerator sg = SampleGenerator.create(launchIndex, frameSeed);

    ShadingData sd;
    if (loadShadingData(launchIndex, launchDim, gScene.camera, sd))
    {
        float3 pos = sd.posW;

        // compute direction by bsdf
        // TODO: randomly choose direction
        BSDFSample bsdfSample;
        sampleBSDF(sd, sg, bsdfSample);
        float3 dir = bsdfSample.wi;
        float dirPdf = bsdfSample.pdf;

        // multiple grid selection
        // TODO: make as a struct?
        UniformGrid chosenGrid = {};
        float3 chosenGridBSDF = 0;
        float chosenGridDistance = 0;
        float weightSum = 0;
        for (int dirSampleIdx = 0; dirSampleIdx < samplesPerDirection; dirSampleIdx++)
        {
            // random generate length
            float dmin = minDistance;
            float dmax = intersectWithSceneBound(pos, dir);
            float distance, distancePdf;
            sampleDistanceByDistSquare(dmin, dmax, sg, distance, distancePdf);

            // choose grid
            float3 gridPos = pos + dir * distance; //TODO: rename
            uint gridMortonCode = computeMortonCodeByPos(gridPos);
            float gridPdf = dirPdf * distancePdf;
            UniformGrid grid = traverseTreeForGird(gridMortonCode);

            // recompute BSDF
            float3 bsdf = evalBSDFCosine(sd, normalize(grid.pos - pos));

            // ------------------------ BUG ------------------------ //
            // random choose by weight
            float gridWeight = length(bsdf * grid.intensity) / (distance * distance) / gridPdf;
            weightSum += gridWeight;
            if (sampleNext1D(sg) < gridWeight / weightSum)
            {
                chosenGrid = grid;
                chosenGridBSDF = bsdf;
                chosenGridDistance = distance;
            }
        }

        float chosenGridWeight = length(chosenGridBSDF * chosenGrid.intensity) / (chosenGridDistance * chosenGridDistance);
        float chosenGridPdf = chosenGridWeight * samplesPerDirection / weightSum;
            // ------------------------ BUG ------------------------ //

        // choose light
        uint emissiveIdx; // TODO: rename this to leadNodeIdx
        float idxPdf;
        sampleRangeUniformly(chosenGrid.range, sg, emissiveIdx, idxPdf);
        uint emissiveTriangleIndex = gLeafNodes[emissiveIdx].triangleIdx;
        float emissivePdf = chosenGridPdf * idxPdf;

        result = float4(emissiveTriangleIndex, emissivePdf, emissiveIdx, 1);
    }

    gLightIndex[launchIndex] = result;
}
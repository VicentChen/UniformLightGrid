import Scene.Scene;
import RenderPasses.Shared.PathTracer.LoadShadingData;
import TraverseBVH;
import Samplers;

#ifndef CHUNK_SIZE
    // Compile-time error if GROUP_SIZE is not defined.
    #error CHUNK_SIZE is not defined. Add define in cpp file.
#endif

struct EmissivePower
{
    float           invWeightsSum;
    Buffer<uint2>   triangleAliasTable;
};

cbuffer PerFrameCB
{
    uint2 dispatchDim;
    uint frameCount;

    float minDistance;

    // TODO: rename this for restir
    uint samplesPerDirection;

    uint gridCount; // TODO: better position?
    uint powerGridCount;

    EmissivePower emissivePower;
}

// float4: triangleIndex, pdf, ??, ??
RWTexture2D<float4> gLightIndex;
StructuredBuffer<BVHPowerNode> gPowerNodes;
StructuredBuffer<UniformGrid> gPowerGrids;

// TODO: merge 3 selectGridByX
void selectGridByOctree<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf)
{
    float3 pos = sd.posW;

    chosenGrid = {};
    chosenGridPdf = 1.0f;

    // multiple grid selection
    // TODO: make as a struct?
    float3 chosenGridBSDF = 0;
    float chosenGridDistance = 0;
    float weightSum = 0;
    for (int dirSampleIdx = 0; dirSampleIdx < samplesPerDirection; dirSampleIdx++)
    {
        BSDFSample bsdfSample;
        uint lobeBackup = sd.activeLobes;
        sampleBSDF(sd, sg, bsdfSample);
        float3 dir = bsdfSample.wi;
        float dirPdf = bsdfSample.pdf;
        float gridPdf = 1.f;

        // choose grid

        // UniformGrid grid = traverseOctree(sd, dir, gridPdf, sg);
        // gridPdf *= dirPdf;
        UniformGrid grid = traverseOctree(sd, gridPdf, sg);

        // recompute BSDF & distance
        float3 bsdf = float3(1,1,1);
        // float3 bsdf = evalBSDFCosine(sd, normalize(grid.pos - pos));

        // ------------------------ BUG ------------------------ //
        // random choose by weight
        float distance = length(grid.pos - pos);
        float gridWeight = length(bsdf * grid.intensity) / (distance * distance) / gridPdf;
        weightSum += gridWeight;
        if (sampleNext1D(sg) < gridWeight / weightSum)
        {
            chosenGrid = grid;
            chosenGridBSDF = bsdf;
            chosenGridDistance = distance;
        }
    }

    float chosenGridWeight = length(chosenGridBSDF * chosenGrid.intensity) / max((chosenGridDistance * chosenGridDistance), 0.0001f);
    chosenGridPdf = chosenGridWeight * samplesPerDirection / max(weightSum, 0.0001f);
        // ------------------------ BUG ------------------------ //
}

void selectGridByBVH<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf)
{
    float3 pos = sd.posW;

    // compute direction by bsdf
    // TODO: randomly choose direction
    BSDFSample bsdfSample;
    sampleBSDF(sd, sg, bsdfSample);
    float3 dir = bsdfSample.wi;
    float dirPdf = bsdfSample.pdf;
    chosenGrid = {};
    chosenGridPdf = 1.0f;

    // multiple grid selection
    // TODO: make as a struct?
    float3 chosenGridBSDF = 0;
    float chosenGridDistance = 0;
    float weightSum = 0;
    for (int dirSampleIdx = 0; dirSampleIdx < samplesPerDirection; dirSampleIdx++)
    {
        // choose grid
        float gridPdf = 1.f;
        UniformGrid grid = traverseTreeForGird(sd, dir, gridPdf, sg);
        gridPdf *= dirPdf;

        // recompute BSDF & distance
        float3 bsdf = evalBSDFCosine(sd, normalize(grid.pos - pos));
        float  distance = length(grid.pos - pos);

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

    float chosenGridWeight = length(chosenGridBSDF * chosenGrid.intensity) / max((chosenGridDistance * chosenGridDistance), 0.0001f);
    chosenGridPdf = chosenGridWeight * samplesPerDirection / max(weightSum, 0.0001f);
        // ------------------------ BUG ------------------------ //
}

// NOTE: If there is only one grid, half of the scene (surface "in front of" the grid position) will evaluate bsdf == 0.
//       Therefore half of the scene may become black.
void selectGridByResampling<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf)
{
    float3 pos = sd.posW;
    float gridPdf = 1.f / gridCount;

    float weightSum = 0.0001f;
    float3 chosenGridBSDF = 0;
    float  chosenGridDistance = 0;
    uint   chosenGridIdx = 0;
    for (int i = 0; i < samplesPerDirection; i++)
    {
        uint gridIdx = uint(floor(sampleNext1D(sg) * 0.999999f * gridCount));
        UniformGrid grid = gGrids[gridIdx];
        float3 dir = normalize(grid.pos - pos);
        float distance = length(grid.pos - pos);
        float3 bsdf = evalBSDFCosine(sd, dir);

        float gridWeight = length(bsdf * grid.intensity) / (distance * distance) / gridPdf;
        weightSum += gridWeight;
        if (sampleNext1D(sg) < gridWeight / weightSum)
        {
            chosenGrid = grid;
            chosenGridBSDF = bsdf;
            chosenGridDistance = distance;
        }
    }

    float chosenGridWeight = length(chosenGridBSDF * chosenGrid.intensity) / max((chosenGridDistance * chosenGridDistance), 0.0001f);
    chosenGridPdf = chosenGridWeight * samplesPerDirection / max(weightSum, 0.0001f);
}

void selectGridByPowerResampling<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf)
{
    float3 pos = sd.posW;
    float gridPdf = 1.f;

    float weightSum = 0.0001f;
    float3 chosenGridBSDF = 0;
    float  chosenGridDistance = 0;
    uint   chosenGridIdx = 0;
    for (int i = 0; i < samplesPerDirection; i++)
    {
        uint gridIdx = uint(floor(sampleNext1D(sg) * 0.999999f * gridCount));
        uint2 packed = emissivePower.triangleAliasTable[gridIdx];
        float threshold = f16tof32(packed.x >> 16u);
        uint  selectAbove = ((packed.x & 0xFFFFu) << 8u) | ((packed.y >> 24u) & 0xFFu);
        uint  selectBelow = packed.y & 0xFFFFFFu;
        gridIdx = (sampleNext1D(sg) >= threshold) ? selectAbove : selectBelow;

        UniformGrid grid = gPowerGrids[gridIdx];
        float3 dir = normalize(grid.pos - pos);
        float distance = length(grid.pos - pos);
        float3 bsdf = evalBSDFCosine(sd, dir);
        gridPdf = grid.flux * emissivePower.invWeightsSum;

        float gridWeight = length(bsdf * grid.intensity) / (distance * distance) / gridPdf;
        weightSum += gridWeight;
        if (sampleNext1D(sg) < gridWeight / weightSum)
        {
            chosenGrid = grid;
            chosenGridBSDF = bsdf;
            chosenGridDistance = distance;
        }
    }

    float chosenGridWeight = length(chosenGridBSDF * chosenGrid.intensity) / max((chosenGridDistance * chosenGridDistance), 0.0001f);
    chosenGridPdf = chosenGridWeight * samplesPerDirection / max(weightSum, 0.0001f);
}

// TODO: check this when use
void chooseTriangle<S : ISampleGenerator>(in uint2 range, in const ShadingData sd, inout S sg, out uint triangleIdx, out float pdf)
{
    float weightSum = 1e-10;
    float pHat = 0;
    float choosenPHat = 0;
    pdf = 0;
    triangleIdx = range.x;
    float3 pos = sd.posW;
    uint RISCount = 4; // TODO: add to UI
    for (int i = 0; i < RISCount; i++) // TODO:: make 4 a variable
    {
        uint idx = 0;
        float delta = range.y - range.x + 1;
        float x = sampleNext1D(sg) * 0.999999f; // FIXME: hack to avoid x == 1
        idx = floor(range.x + x * delta);
        BVHLeafNode node = gLeafNodes[idx];
        float distance = length(node.debug.xyz - pos);
        // pHat = length(node.intensity) / distance / distance * delta;
        pHat = 100.f / distance / distance * delta;
        weightSum += pHat;
        if (sampleNext1D(sg) < pHat / weightSum)
        {
            triangleIdx = idx;
            choosenPHat = pHat;
        }
    }
    pdf = choosenPHat * RISCount / weightSum;
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
        UniformGrid chosenGrid = {};
        float chosenGridPdf = 0.0f;
        
        // TODO: add a function for this?
        if (kGridSelectionStrategy == (uint)GridSelectionStrategy::Octree)
            selectGridByOctree(sd, sg, chosenGrid, chosenGridPdf);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::BVH)
            selectGridByBVH(sd, sg, chosenGrid, chosenGridPdf);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::Resampling)
            // selectGridByResampling(sd, sg, chosenGrid, chosenGridPdf);
            selectGridByPowerResampling(sd, sg, chosenGrid, chosenGridPdf);
        else // Use resampling by default
            selectGridByResampling(sd, sg, chosenGrid, chosenGridPdf);

        // choose light
        uint leafNodeIdx;
        float idxPdf;
        sampleRangeUniformly(chosenGrid.range, sg, leafNodeIdx, idxPdf);
        // chooseTriangle(chosenGrid.range, sd, sg, leafNodeIdx, idxPdf);
        // TODO: add if for two triangle index?
        // uint emissiveTriangleIndex = gLeafNodes[leafNodeIdx].triangleIdx;
        uint emissiveTriangleIndex = gPowerNodes[leafNodeIdx].triangleIdx;
        float emissivePdf = chosenGridPdf * idxPdf;

        result = float4(emissiveTriangleIndex, emissivePdf, chosenGrid.gridIndex, 1);
    }

    gLightIndex[launchIndex] = result;
}

// =========== Backup Codes =========== //

// void selectGridByOctree<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf)
// {
//     float3 pos = sd.posW;

//     // compute direction by bsdf
//     // TODO: randomly choose direction
//     chosenGrid = {};
//     chosenGridPdf = 1.0f;

//     // multiple grid selection
//     // TODO: make as a struct?
//     float3 chosenGridBSDF = 0;
//     float chosenGridDistance = 0;
//     float weightSum = 0;
//     for (int dirSampleIdx = 0; dirSampleIdx < samplesPerDirection; dirSampleIdx++)
//     {
//         BSDFSample bsdfSample;
//         uint lobeBackup = sd.activeLobes;
//         sampleBSDF(sd, sg, bsdfSample);
//         float3 dir = bsdfSample.wi;
//         float dirPdf = bsdfSample.pdf;
//         float gridPdf = 1.f;

//         // random generate length
//         float distancePdf = 1.0f;
//         float distance = 0.0f;
//         float dmin = minDistance;
//         float dmax = intersectWithSceneBound(pos, dir);
//         sampleDistanceUniformly(dmin, dmax, sg, distance, distancePdf);
//         // sampleDistanceByDist(dmin, dmax, sg, distance, distancePdf);
//         // sampleDistanceByDistSquare(dmin, dmax, sg, distance, distancePdf);
//         // sampleDistanceByDistQuad(dmin, dmax, sg, distance, distancePdf);

//         // choose grid
//         float3 gridPos = pos + dir * distance; //TODO: rename
//         uint gridMortonCode = computeMortonCodeByPos(gridPos);

//         // UniformGrid grid = traverseTreeForGird(gridMortonCode, gridPdf, sg);
//         // UniformGrid grid = findContainingGird(gridMortonCode);
//         // gridPdf *= dirPdf * distancePdf;
//         // UniformGrid grid = traverseOctree(sd, dir, gridPdf, sg);
//         // gridPdf *= dirPdf;
//         UniformGrid grid = traverseOctree(sd, gridPdf, sg);

//         // recompute BSDF & distance
//         float3 bsdf = float3(1,1,1);
//         // float3 bsdf = evalBSDFCosine( sd, normalize(grid.pos - pos));;
//         distance = length(grid.pos - pos);

//         // ------------------------ BUG ------------------------ //
//         // random choose by weight
//         float gridWeight = length(bsdf * grid.intensity) / (distance * distance) / gridPdf;
//         weightSum += gridWeight;
//         if (sampleNext1D(sg) < gridWeight / weightSum)
//         {
//             chosenGrid = grid;
//             chosenGridBSDF = bsdf;
//             chosenGridDistance = distance;
//         }
//     }

//     float chosenGridWeight = length(chosenGridBSDF * chosenGrid.intensity) / max((chosenGridDistance * chosenGridDistance), 0.0001f);
//     chosenGridPdf = chosenGridWeight * samplesPerDirection / max(weightSum, 0.0001f);
//         // ------------------------ BUG ------------------------ //
// }

// uint search(uint mortonCode)
// {
//     uint2 range = uint2(0, gridCount - 1);
//     uint mid = (range.x + range.y) * 0.5f;
//     while (range.y - range.x >= 1 )
//         range = gGrids[mid].mortonCode < mortonCode ? uint2(mid, range.y) : uint2(range.x, mid);

//     return range.x;
// }


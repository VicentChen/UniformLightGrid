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

    uint RISSampleCount;

    uint gridCount; // TODO: better position?
    uint powerGridCount;

    EmissivePower emissivePower;
}

Texture2D<float4> gAOMap;

// float4: triangleIndex, pdf, ??, ??
RWTexture2D<float4> gLightIndex;
StructuredBuffer<UniformGrid> gPowerGrids;

float computeOctreeNodeWeightForResampling(const OctreeNode node, const ShadingData sd)
{
    float3   dir = normalize(node.pos - sd.posW);
    float    dis = length(node.pos - sd.posW);
    // float3  bsdf = evalBSDFCosine(sd, dir);
    // float weight = length(bsdf * node.intensity) / (dis * dis);
    float weight = length(node.intensity) * max(dot(sd.N, dir), 0) / (dis * dis);
    return weight;
}

void selectGridByOctree<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
{
    // initialize return values
    chosenGrid = {};
    chosenGridPdf = 1.0f;

    float3 pos = sd.posW;

    float3 chosenGridBSDF = 0;
    float chosenGridDistance = 0;
    float weightSum = 0;
    for (int dirSampleIdx = 0; dirSampleIdx < RISSampleCount; dirSampleIdx++)
    {
        BSDFSample bsdfSample;
        uint lobeBackup = sd.activeLobes;
        sampleBSDF(sd, sg, bsdfSample);
        float3 dir = bsdfSample.wi;
        float dirPdf = bsdfSample.pdf;
        float gridPdf = 1.f;

        // choose grid
        UniformGrid grid = traverseOctree(sd, octreeRootIndex, gridPdf, sg);

        // recompute BSDF & distance
        float3 bsdf = float3(1,1,1);
        // float3 bsdf = evalBSDFCosine(sd, normalize(grid.pos - pos));

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
    chosenGridPdf = chosenGridWeight * RISSampleCount / max(weightSum, 0.0001f);
}

void selectGridByOctreeWithResampling<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
{
    // initialize return values
    chosenGrid = {};
    chosenGridPdf = 1.0f;

    // choose node
    float nodePdf = 1.f;
    OctreeNode root = traverseOctreeForNode(sd, octreeRootIndex, 64, nodePdf, sg);
    
    // resampling
    OctreeNode chosenNode = {};
    float chosenNodeWeight = 0.f;
    float  weightSum = 0;
    for (int i = root.leafRange.x; i <= root.leafRange.y; i++)
    {
        OctreeNode node = gOctree[i];
        float weight = computeOctreeNodeWeightForResampling(node, sd);
        weightSum += weight;
        if (sampleNext1D(sg) < weight / weightSum)
        {
            chosenNode = node;
            chosenNodeWeight = weight;
        }
    }

    buildGridByGridNode(chosenNode, chosenGrid);
    chosenGridPdf = nodePdf * (chosenNodeWeight / weightSum);
}

void selectGridByBVH<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
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
    for (int dirSampleIdx = 0; dirSampleIdx < RISSampleCount; dirSampleIdx++)
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
    chosenGridPdf = chosenGridWeight * RISSampleCount / max(weightSum, 0.0001f);
        // ------------------------ BUG ------------------------ //
}

// NOTE: If there is only one grid, half of the scene (surface "in front of" the grid position) will evaluate bsdf == 0.
//       Therefore half of the scene may become black.
void selectGridByResampling<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
{
    // initialize return values
    chosenGrid = {};
    chosenGridPdf = 1.0f;

    float3 pos = sd.posW;
    float gridPdf = 1.f / gridCount;

    float weightSum = 0.0001f;
    float3 chosenGridBSDF = 0;
    float  chosenGridDistance = 0;
    for (int i = 0; i < RISSampleCount; i++)
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
    chosenGridPdf = chosenGridWeight * RISSampleCount / max(weightSum, 0.0001f);
}

void selectGridByKNNWithFixedRadius<S : ISampleGenerator>(float ao, const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
{
    // initialize return values
    chosenGrid = {};
    chosenGridPdf = 1.0f;

    // compute aabb by ao
    float3 pos = sd.posW;
    uint   mortonCode = computeMortonCodeByPos(pos);
    uint size = floor(3 + (1 - ao) * 0.999999f * 7); // size in [3, 9)
    size = size * 3; // size in {9, 12, ..., 24}
    AABB   aabb = computeAABBByMortonCode(mortonCode, size);

    // make pos as AABB center
    float3 aabbCenter = aabb.center();
    float3 offset = pos - aabbCenter;
    aabb.minPoint += offset;
    aabb.maxPoint += offset;

    OctreeNode root = findContainingGrid(aabb);

    float weightSum = 0.0001f;
    OctreeNode chosenNode = root;
    float chosenNodeWeight = 1.f;
    for (int i = root.leafRange.x; i <= root.leafRange.y; i++)
    {
        OctreeNode node = gOctree[i];

        float nodeWeight = computeOctreeNodeWeightForResampling(node, sd);
        weightSum += nodeWeight;
        if (sampleNext1D(sg) < nodeWeight / weightSum)
        {
            chosenNode = node;
            chosenNodeWeight = nodeWeight;
        }
    }
    buildGridByGridNode(chosenNode, chosenGrid);
    chosenGridPdf = chosenNodeWeight / weightSum;
}

/** Scan through all octree leaf nodes and choose one by weight.
*/
void selectGridByBruteForceResampling<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
{
    // initialize return values
    chosenGrid = {};
    chosenGridPdf = 1.f;

    uint2 leafRange = gOctree[octreeRootIndex].leafRange;

    OctreeNode chosenNode = {};
    float chosenNodeWeight = 0.f;
    float  weightSum = 0;
    for (int i = leafRange.x; i <= leafRange.y; i++)
    {
        OctreeNode node = gOctree[i];
        float weight = computeOctreeNodeWeightForResampling(node, sd);
        weightSum += weight;
        if (sampleNext1D(sg) < weight / weightSum)
        {
            chosenNode = node;
            chosenNodeWeight = weight;
        }
    }

    buildGridByGridNode(chosenNode, chosenGrid);
    chosenGridPdf = chosenNodeWeight / weightSum;
}

/** Select 16 grids with maximum weights by brute force.(KNN with K = 16)
*/
void selectGridByKNNWithFixedK<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf, out float4 debugValue)
{
    // initialize return values
    chosenGrid = {};
    chosenGridPdf = 1.f;

    OctreeNode root = gOctree[octreeRootIndex];
    uint rangeCount = root.leafRange.y - root.leafRange.x + 1;

    int        knnCount = 0;
    float      knnWeight[16];
    OctreeNode knn[16];

    if (rangeCount <= 16)
    {
        // use all leaf nodes as KNN
        knnCount = rangeCount;
        for (int i = root.leafRange.x; i <= root.leafRange.y; i++)
        {
            knn[i] = gOctree[i];
            knnWeight[i] = computeOctreeNodeWeightForResampling(knn[i], sd);
        }
    }
    else
    {
        // initialize KNN
        knnCount = 16;
        float initialPdf = 1.0f;
        for (int i = 0; i < knnCount; i++)
        {
            knn[i] = gOctree[root.leafRange.x + i];
            knnWeight[i] = computeOctreeNodeWeightForResampling(knn[i], sd);
        }

        // scan through leaf nodes
        for (int i = root.leafRange.x + knnCount; i <= root.leafRange.y; i++)
        {
            // find entry with minimum weight in knn array
            uint chosenIndex = 0;
            float chosenWeight = 1000000.f;
            for (int k = 0; k < knnCount; k++)
            {
                float weight = knnWeight[k];
                if (chosenWeight > weight)
                {
                    chosenIndex = k;
                    chosenWeight = weight;
                }
            }

            // update knn
            OctreeNode node = gOctree[i];
            float weight = computeOctreeNodeWeightForResampling(node, sd);
            if (weight > chosenWeight)
            {
                knn[chosenIndex] = node;
                knnWeight[chosenIndex] = weight;
            }
        }
    }

    // select 1 node from knn
    OctreeNode chosenNode = root; // initialize as root
    float weightSum = 0.0001f;
    float chosenNodeWeight = 0;
    for (int i = 0; i < knnCount; i++)
    {
        float nodeWeight = knnWeight[i];
        weightSum += nodeWeight;
        if (sampleNext1D(sg) < nodeWeight / weightSum)
        {
            chosenNode = knn[i];
            chosenNodeWeight = nodeWeight;
        }
    }
    buildGridByGridNode(chosenNode, chosenGrid);
    chosenGridPdf = chosenNodeWeight / weightSum;
}

void selectGridByPowerResampling<S : ISampleGenerator>(const ShadingData sd, inout S sg, out UniformGrid chosenGrid, out float chosenGridPdf)
{
    float3 pos = sd.posW;
    float gridPdf = 1.f;

    float weightSum = 0.0001f;
    float3 chosenGridBSDF = 0;
    float  chosenGridDistance = 0;
    uint   chosenGridIdx = 0;
    for (int i = 0; i < RISSampleCount; i++)
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
    chosenGridPdf = chosenGridWeight * RISSampleCount / max(weightSum, 0.0001f);
}

void chooseTriangle<S : ISampleGenerator>(in uint2 range, in const ShadingData sd, inout S sg, out uint triangleIdx, out float pdf)
{
    // initial choice
    pdf = 0;
    triangleIdx = range.x;

    uint RISCount = 4; // TODO: add to UI
    float weightSum = 1e-10;
    float chosenPHat = 0;
    float3 pos = sd.posW;
    for (int i = 0; i < RISCount; i++) // TODO:: make 4 a variable
    {
        uint idx = 0;
        float idxPdf = 1.f;
        sampleRangeUniformly(range, sg, idx, idxPdf);

        LightProxy proxy = gProxys[idx];
        float distance = length(proxy.pos - pos);
        float pHat = length(proxy.intensity) / (distance * distance);
        // float pHat = 100.f / distance / distance;
        float p = pHat / idxPdf;
        weightSum += p;
        if (sampleNext1D(sg) < p / weightSum)
        {
            triangleIdx = idx;
            chosenPHat = pHat;
        }
    }
    pdf = chosenPHat * RISCount / weightSum;
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

    float ao = gAOMap[launchIndex].x;
    ShadingData sd;
    if (loadShadingData(launchIndex, launchDim, gScene.camera, sd))
    {
        UniformGrid chosenGrid = {};
        float chosenGridPdf = 0.0f;
        
        float4 debugValue = 0;

        // choose one grid from all grids
        if (kGridSelectionStrategy == (uint)GridSelectionStrategy::Octree)
            selectGridByOctree(sd, sg, chosenGrid, chosenGridPdf, debugValue);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::OctreeWithResampling)
            selectGridByOctreeWithResampling(sd, sg, chosenGrid, chosenGridPdf, debugValue);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::BVH)
            selectGridByBVH(sd, sg, chosenGrid, chosenGridPdf, debugValue);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::Resampling)
            selectGridByResampling(sd, sg, chosenGrid, chosenGridPdf, debugValue);
            // selectGridByPowerResampling(sd, sg, chosenGrid, chosenGridPdf);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::BruteForceResampling)
            selectGridByBruteForceResampling(sd, sg, chosenGrid, chosenGridPdf, debugValue);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::KNNWithFixedRadius)
            selectGridByKNNWithFixedRadius(ao, sd, sg, chosenGrid, chosenGridPdf, debugValue);
        else if (kGridSelectionStrategy == (uint)GridSelectionStrategy::KNNWithFixedK)
            selectGridByKNNWithFixedK(sd, sg, chosenGrid, chosenGridPdf, debugValue);
        else // Use resampling by default
            selectGridByResampling(sd, sg, chosenGrid, chosenGridPdf, debugValue);

        // choose one proxy(/light) from chosen grid
        uint  proxyIdx = 0;
        float proxyPdf = 1.f;

        if (kTriangleSelectionStrategy == (uint)TriangleSelectionStrategy::Uniform)
            sampleRangeUniformly(chosenGrid.range, sg, proxyIdx, proxyPdf);
        else if (kTriangleSelectionStrategy == (uint)TriangleSelectionStrategy::Resampling)
            chooseTriangle(chosenGrid.range, sd, sg, proxyIdx, proxyPdf);
        else // Use uniform sampling by default
            sampleRangeUniformly(chosenGrid.range, sg, proxyIdx, proxyPdf);

        // TODO: add if for two triangle index?
        uint emissiveTriangleIndex = gProxys[proxyIdx].triangleIdx;
        // uint emissiveTriangleIndex = gPowerNodes[proxyIdx].triangleIdx;
        float emissivePdf = chosenGridPdf * proxyPdf;

        result = float4(emissiveTriangleIndex, emissivePdf, debugValue.x, debugValue.y);
    }

    gLightIndex[launchIndex] = result;
}

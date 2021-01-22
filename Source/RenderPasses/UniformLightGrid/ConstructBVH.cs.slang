// TODO:
//   1. remove `commonPrefix` and get `commonPrefix` by `splitPoint` and `commonPrefixLength` 
//   2. pack `isLChildLeaf`, `isRChildLeaf` and `commonPrefixLength` into a uint
//   3. remove min and max in `setIsLChildLeaf` and `setIsRChildLeaf`
//   4. add temp variable for getters and setters in `generateTree`
#include "BVHNodeData.slangh"

#ifndef GROUP_SIZE
    // Compile-time error if GROUP_SIZE is not defined.
    #error GROUP_SIZE is not defined. Add define in cpp file.
#endif

StructuredBuffer<LeafNode> gLeafNodes;
RWStructuredBuffer<InternalNode> gInternalNodes;

cbuffer PerFrameCB
{
    uint numLeafNodes;
}

int calcCommonPrefix(uint l, uint r)
{
    if (r >= 0 && r < numLeafNodes)
        return 31 - floor(log2(float(gLeafNodes[l].mortonCode ^ gLeafNodes[r].mortonCode)));
    else
        return -1;
}

uint2 determineRange(uint i)
{
    int d = sign(calcCommonPrefix(i, i + 1) - calcCommonPrefix(i, i - 1));
    int minPrefix = calcCommonPrefix(i, i - d);
    int lMax = 2;
    while (calcCommonPrefix(i, i + lMax * d) > minPrefix)
        lMax *= 2;

    int L = 0;
    for (int t = lMax; t >= 1; t = t / 2)
    {
        if (calcCommonPrefix(i, i + (L + t) * d) > minPrefix)
            L += t;
    }
    int edge = i + L * d;
    return uint2(i, edge);
}

uint findSplit(uint first, uint last, uint idx)
{
    // Identical Morton codes => split the range in the middle.

    uint firstCode = gLeafNodes[first].mortonCode;
    uint lastCode = gLeafNodes[last].mortonCode; 

    if (firstCode == lastCode)
        return (first + last) >> 1;

    // Calculate the number of highest bits that are the same
    // for all objects, using the count-leading-zeros intrinsic.

    int commonPrefix = calcCommonPrefix(first, last);

    // Use binary search to find where the next bit differs.
    // Specifically, we are looking for the highest object that
    // shares more than commonPrefix bits with the first one.

    int split = first; // initial guess
    int step = last - first;

    do
    {
        step = (step + 1) >> 1; // exponential decrease
        int newSplit = split + step; // proposed new position

        if (newSplit < last)
        {
            int splitPrefix = calcCommonPrefix(first, newSplit);
            if (splitPrefix > commonPrefix)
                split = newSplit; // accept proposal
        }
    }
    while (step > 1);

    return split;
}

[numthreads(GROUP_SIZE, 1, 1)]
void constructBVH(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint idx = dispatchThreadId.x;
    if (idx >= numLeafNodes - 1) return;
    // Find out which range of objects the node corresponds to.
    // (This is where the magic happens!)

    InternalNode node = {};

    uint2 range = determineRange(idx);
    int first = range.x;
    int last = range.y;
    node.range = first > last ? uint2(last, first) : uint2(first, last);

    node.setCommonPrefixLength(calcCommonPrefix(first, last)); // TODO: common prefix may overflow here
    node.commonPrefix = gLeafNodes[first].mortonCode >> (31 - node.getCommonPrefixLength());

    // Determine where to split the range.
    int split = findSplit(first, last, idx);
    node.splitMortonCode = gLeafNodes[split].mortonCode;

    // Select Lchild.
    node.lChildId = split;
    node.setIsLChildLeaf(split == min(first, last));
    // Select Rchild.
    node.rChildId = split + 1;
    node.setIsRChildLeaf(split + 1 == max(last, first));

    float3 intensity = 0;
    for (int i = node.range.x; i <= node.range.y; i++)
        intensity += gLeafNodes[i].intensity;
    node.intensity = intensity;

    gInternalNodes[idx] = node;
}

// uint findSplit(uint first, uint last, uint idx)
// {
//     int nodePrefix = calcCommonPrefix(first, last);
//     int S = 0;
//     float L = abs(float(first) - float(last));
//     int d = last > first ? 1 : -1;
//     float i = 0.5f;

//     while (true)
//     {
//         int t = ceil(L * i);
//         if (calcCommonPrefix(idx, idx + (S + t) * d) > nodePrefix)
//             S += t;
//         if (t == 1) break;
//         i *= 0.5f;
//     }

//     return idx + S * d + min(d, 0);
// }
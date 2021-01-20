// TODO:
//   1. remove `commonPrefix` and get `commonPrefix` by `splitPoint` and `commonPrefixLength` 
//   2. pack `isLChildLeaf`, `isRChildLeaf` and `commonPrefixLength` into a uint
//   3. remove min and max in `setIsLChildLeaf` and `setIsRChildLeaf`
//   4. add temp variable for getters and setters in `generateTree`
#include "BVHNodeData.slangh"

RWStructuredBuffer<LeafNode> gLeafNodes;
RWStructuredBuffer<InternalNode> gInternalNodes;

cbuffer PerFrameCB
{
    uint leafLength;
}

int calcCommonPrefix(uint l, uint r)
{
    if (r >= 0 && r < leafLength)
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
    int nodePrefix = calcCommonPrefix(first, last);
    int S = 0;
    float L = abs(float(first) - float(last));
    int d = last > first ? 1 : -1;
    float i = 0.5f;

    while (true)
    {
        int t = ceil(L * i);
        if (calcCommonPrefix(idx, idx + (S + t) * d) > nodePrefix)
            S += t;
        if (t == 1) break;
        i *= 0.5f;
    }

    return idx + S * d + min(d, 0);
}

void generateTree(uint numObjects)
{
    for (int idx = 0; idx < numObjects - 1; idx++) // in parallel
    {
        // Find out which range of objects the node corresponds to.
        // (This is where the magic happens!)

        uint2 range = determineRange(idx);
        int first = range.x;
        int last = range.y;
        gInternalNodes[idx].range = first > last ? uint2(last, first) : uint2(first, last);

        gInternalNodes[idx].setCommonPrefixLength(calcCommonPrefix(first, last));
        gInternalNodes[idx].commonPrefix = gLeafNodes[first].first >> (31 - gInternalNodes[idx].getCommonPrefixLength());
        
        // Determine where to split the range.
        int split = findSplit(first, last, idx);
        gInternalNodes[idx].splitPoint = split;

        // Select Lchild.
        gInternalNodes[idx].LChild = split;
        gInternalNodes[idx].setIsLChildLeaf(split == min(first, last));
        // Select Rchild.
        gInternalNodes[idx].RChild = split + 1;
        gInternalNodes[idx].setIsRChildLeaf(split + 1 == max(last, first));
    }
}

// in generateTree
//
// if (split == min(first, last))
// {
//     gInternalNodes[idx].LChild = split;
//     gInternalNodes[idx].isLChildLeaf = true;
// }
// else
// {
//     gInternalNodes[idx].LChild = split;
//     gInternalNodes[idx].isLChildLeaf = false;
// }

// if (split + 1 == max(last, first))
// {
//     gInternalNodes[idx].RChild = split + 1;
//     gInternalNodes[idx].isRChildLeaf = true;
// }
// else
// {
//     gInternalNodes[idx].RChild = split + 1;
//     gInternalNodes[idx].isRChildLeaf = false;
// }

// in findSplit
//
// int i = 2;
// while (true)
// {
//     int t = ceil(L / i);
//     if (calcCommonPrefix(idx, idx + (S + t) * d) > nodePrefix)
//         S += t;
//     if (L / i < 1) break;
//     i *= 2;
// }

// for (int i = 2; t >= 1; i *= 2)
// {
//     t = ceil(L / i);
//     if (calcCommonPrefix(idx, idx + (S + t) * d) > nodePrefix)
//         S += t;
// }


// int sign(int i)
// {
//     return i >= 0 ? 1 : -1;
// }

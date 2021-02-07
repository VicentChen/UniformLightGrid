#pragma once

enum class TreeTraverseWeightType
{
    DistanceIntensity           = 1,
    DirectionDistanceIntensity  = 2,
    BRDFShading                 = 3,
    Debug                       = 4, // >>> For debug usage.
};

enum class GridSelectionStrategy
{
    Octree                      = 1, // >>> This cannot choose far lights.
    OctreeWithResampling        = 2, // >>> Still cannot generate correct for ceiling in cornell box.
    BVH                         = 3, // >>> This cannot build balanced tree.
    Resampling                  = 4, // >>> This cannot sample nearing lights.
    ResamplingByPower           = 5,
    BruteForceResampling        = 6, 
    KNNWithFixedRadius          = 7, // >>> This will lead to blocking.
    KNNWithFixedK               = 8, // >>> This can only choose nearing lights.
    Debug                       = 9, // >>> For debug usage.
};

enum class TriangleSelectionStrategy
{
    Uniform                     = 1,
    Resampling                  = 2,
    Debug                       = 3, // >>> For debug usage.
};

#ifdef ULG_TRACER_PARAM
    static const bool kShadowRayAlwaysVisible  = SHADOW_RAY_ALWAYS_VISIBLE;
    static const bool kUseGroundTruthShadowRay = USE_GROUND_TRUTH_SHADOW_RAY;
    static const bool kUseReflection           = USE_REFLECTION;
#endif

#ifdef ULG_GRID_AND_LIGHT_SELECTOR_PARAM
    static const uint kTreeTraverseWeightType    = TREE_TRAVERSE_WEIGHT_TYPE;
    static const uint kGridSelectionStrategy     = GRID_SELECTION_STRATEGY;
    static const uint kTriangleSelectionStrategy = TRIANGLE_SELECTION_STRATEGY;
#endif

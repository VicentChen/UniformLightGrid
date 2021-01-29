#pragma once

enum class TreeTraverseWeightType
{
    DistanceIntensity           = 1,
    DirectionDistanceIntensity  = 2,
    BRDFShading                 = 3,
};

enum class GridSelectionStrategy
{
    Octree                      = 1, // >>> This will lead to blocking
    BVH                         = 2, // >>> This cannot select nearing lights
    Resampling                  = 3, // >>> BUG: This cannot select light for some walls
};

#ifdef ULG_TRACER_PARAM
    static const bool kShadowRayAlwaysVisible = SHADOW_RAY_ALWAYS_VISIBLE;
    static const bool kUseGroundTruthShadowRay = USE_GROUND_TRUTH_SHADOW_RAY;
    static const bool kUseReflection = USE_REFLECTION;
#endif

#ifdef ULG_GRID_AND_LIGHT_SELECTOR_PARAM
    static const uint kTreeTraverseWeightType = TREE_TRAVERSE_WEIGHT_TYPE;
    static const uint kGridSelectionStrategy  = GRID_SELECTION_STRATEGY;
#endif

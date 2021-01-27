enum class TreeTraverseWeightType
{
    Direction                   = 1,
    DistanceIntensity           = 2,
    DirectionDistanceIntensity  = 3,
    BRDFShading                 = 4,
};

#ifdef ULG_TRACER_PARAM
    static const bool kShadowRayAlwaysVisible = SHADOW_RAY_ALWAYS_VISIBLE;
    static const bool kUseGroundTruthShadowRay = USE_GROUND_TRUTH_SHADOW_RAY;
    static const bool kUseReflection = USE_REFLECTION;
#endif

#ifdef ULG_GRID_AND_LIGHT_SELECTOR_PARAM
    static const uint kTreeTraverseWeightType = TREE_TRAVERSE_WEIGHT_TYPE;
#endif
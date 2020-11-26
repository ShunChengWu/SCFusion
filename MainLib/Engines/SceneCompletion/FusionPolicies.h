#pragma once

namespace SCFUSION{
    namespace Policy {
        enum FuseTwo {
            FuseTwo_OCCUPIED, // Fuse completion result entirely into global map
            FuseTwo_ALL_CONFIDENCE, // treat softmax as confidence value and multiply it with COMPLETION_WEIGHT
            FuseTwo_UNKNOWN,  // Fuse only on unknown area (not observed by camera)
            FuseTwo_UNKNOWN_CONFIDENCE,
            FuseTwo_ALL_OCCUPANCY, // Unlike above, using 1 as a observation. use the softmax value instead.
            FuseTwo_UNKNOWN_OCCUPANCY,
            FuseTwo_ALL_UNWEIGHT, // This takes only depth fusion information as input. This will force integrate policy in fusetwo to Integrate_DIRECT
            FuseTwo_UNKNOWN_UNWEIGHT,
        };
    };


    enum SceneCompletionMethod {
        SceneCompletionMethod_ForkNet, SceneCompletionMethod_SceneInpainting
    };
}
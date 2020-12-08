#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_DATA_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_DATA_HPP

#include "../../Geometry.hpp"

#include <memory>

namespace ldplab
{
    struct RayBuffer
    {
        Ray* data;
        size_t num_rays;
        size_t branching_depth;
    };
}

#endif

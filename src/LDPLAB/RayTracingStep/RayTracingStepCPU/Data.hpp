#ifndef WWU_LDPLAB_RTSCPU_DATA_HPP
#define WWU_LDPLAB_RTSCPU_DATA_HPP

#include "../../Geometry.hpp"

#include <memory>

namespace ldplab
{
    namespace rtscpu
    {
        struct RayBuffer
        {
            Ray* data;
            size_t num_rays;
            size_t branching_depth;
        };

        struct RodeParticle
        {
            double radius;

        };
    }
}

#endif

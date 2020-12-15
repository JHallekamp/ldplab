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

        struct IntersectionBuffer
        {
            Vec3* point;
            Vec3* normal;
            size_t num_intersections;
        };

        /**
         * @brief Structure models the geometries of a rode like particle. The 
         *        particle is cylindric shaped with a spherical cap and a 
         *        spherical indent at the bottom.
         */
        struct RodeParticle
        {
            double cylinder_radius;
            double cylinder_length;
            double sphere_radius;
            Vec3 origin_cap;
            Vec3 origin_indentation;
        };
    }
}

#endif

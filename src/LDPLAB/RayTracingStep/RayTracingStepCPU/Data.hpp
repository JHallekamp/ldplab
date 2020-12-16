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
            size_t index;
            size_t num_rays;
            size_t branching_depth;
        };

        /**
         * @brief Buffer holding intersection points and the corresponding 
         *        normal of the particle surface. 
         */
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
         * @detail The orientation of the cylinder is pointing to the 
         *         z-direction. The origin is of the coordinate system is set 
         *         in the middle point of bottom surface where the indentation 
         *         is.
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

#ifndef WWU_LDPLAB_RTSCPU_DATA_HPP
#define WWU_LDPLAB_RTSCPU_DATA_HPP

#include "../../Geometry.hpp"

#include <cstdint>
#include <memory>

namespace ldplab
{
    namespace rtscpu
    {
        /** @brief Holds the data of a batch of rays. */
        struct RayBuffer
        {
            RayBuffer(size_t index, size_t depth, size_t size)
                :
                index{ index },
                depth{ depth },
                size{ size },
                ray_data{ nullptr },
                index_data{ nullptr },
                active_rays{ 0 }
            { }
            /** @brief Array containing size Ray element. */
            Ray* ray_data;
            /** @brief Array containing size (particle) indices. */
            int32_t* index_data;
            /** @brief Counter for the still active rays inside this. */
            size_t active_rays;
            /** @brief Index of this buffer. */
            const size_t index;
            /** @brief Branching depth (number of times rays split before). */
            const size_t depth;
            /** @brief Number of elements in ray_data and index_data arrays. */
            const size_t size;
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
         * @brief Holds data that is used to transform world space rays into
         *        particle space or the other way around.
         */
        struct ParticleTransformation
        {
            /** 
             * @brief Vector from particle space origin (in world space) to 
             *        world space origin.
             */
            Vec3 w2p_translation;
            /**
             * @brief Matrix that transforms a world space vector (when the
             *        vector is applied to the matrix from the right-hand side) 
             *        by first rotating and then scaling it into particle space
             *        (safe for translation).
             */
            Mat3 w2p_rotation_scale;
            /**
             * @brief Vector to the particle space (in world space)
             */
            Vec3 p2w_translation;
            /**
             * @brief Matrix that transforms a particle space vector (when the
             *        vector is applied to the matrix from the right-hand side)
             *        by first scaling and then rotating it into world space
             *        (safe for translation).
             */
            Mat3 p2w_scale_rotation;
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

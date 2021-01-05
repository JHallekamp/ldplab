#ifndef WWU_LDPLAB_RTSCPU_DATA_HPP
#define WWU_LDPLAB_RTSCPU_DATA_HPP

#include "../../Geometry.hpp"
#include "../../Utils/UID.hpp"

#include <cstdint>
#include <memory>

namespace ldplab
{
    namespace rtscpu
    {
        /** @brief Holds the data of a batch of rays. */
        struct RayBuffer
        {
            RayBuffer(size_t depth, size_t size)
                :
                depth{ depth },
                size{ size },
                inner_particle_rays{ false },
                ray_data{ nullptr },
                index_data{ nullptr },
                active_rays{ 0 },
                world_space_rays{ 0 }
            { }
            /** @brief Array containing size Ray element. */
            Ray* ray_data;
            /** @brief Array containing size (particle) indices. */
            int32_t* index_data;
            /** @brief Array containing the min distance to bounding volumes */
            double* min_bounding_volume_distance_data;
            /** @brief Counter for the still active rays inside this. */
            size_t active_rays;
            /** @brief Counter for the active rays that are in world space. */
            size_t world_space_rays;
            /** @brief States whether buffer rays are inside a particle. */
            bool inner_particle_rays;
            /** @brief Branching depth (number of times rays split before). */
            const size_t depth;
            /** @brief Number of elements in ray_data and index_data arrays. */
            const size_t size;
            /** @brief Unique identifier of the buffer. */
            UID<RayBuffer> uid;
        };

        /**
         * @brief Buffer holding force and torque change of the particle
         */
        struct OutputBuffer
        {
            /** @brief Array containing size force changes per particle. */
            Vec3* force;
            /** @brief Array containing size torque changes per particle. */
            Vec3* torque;
            /** @brief Number of particles. */
            size_t size;
        };

        /**
         * @brief Buffer holding intersection points and the corresponding 
         *        normal of the particle surface. 
         */
        struct IntersectionBuffer
        {
            /** @brief Array containing size intersection points. */
            Vec3* point;
            /** @brief Array containing size intersection normals. */
            Vec3* normal;
            /** @brief Array containing indices of intersected particles. */
            int32_t* particle_index;
            /** @brief Number of elements in point and normal arrays. */
            size_t size;
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
         * @brief Interface for particle data rehashed to be used during ray 
         *        tracing. 
         */
        struct IParticleData
        {
            /** @brief The type of the particle data. */
            enum class Type { rod_particle };
            /** @brief Returns the type of the particle. */
            virtual Type type() const = 0;
        };

        /**
         * @brief Structure models the geometries of a rod like particle. The 
         *        particle is cylindric shaped with a spherical cap and a 
         *        spherical indent at the bottom.
         * @detail The orientation of the cylinder is pointing to the 
         *         z-direction. The origin is of the coordinate system is set 
         *         in the middle point of bottom surface where the indentation 
         *         is.
         */
        struct RodParticle : public IParticleData
        {
            RodParticle(
                double cylinder_radius,
                double cylinder_length,
                double sphere_radius,
                Vec3 origin_cap,
                Vec3 origin_indentation)
                :
                cylinder_radius{ cylinder_radius },
                cylinder_length{ cylinder_length },
                sphere_radius{ sphere_radius },
                origin_cap{ origin_cap },
                origin_indentation{ origin_indentation }
            { }
            Type type() const override { return Type::rod_particle; }
            double cylinder_radius;
            double cylinder_length;
            double sphere_radius;
            Vec3 origin_cap;
            Vec3 origin_indentation;
        };
    }
}

#endif

#ifndef WWU_LDPLAB_RTSCPU_DATA_HPP
#define WWU_LDPLAB_RTSCPU_DATA_HPP

#include <LDPLAB/ExperimentalSetup/BoundingVolume.hpp>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/UID.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /** 
         * @brief Holds mapping between internal indices and outward uids for 
         *        various experimental setup components.
         * @details Components that exist outside of the ray tracing step,
         *          like for example particles, are identified by different
         *          simulation steps - like the ray tracing step - using their
         *          respective UID. 
         *          Internally however the ray tracing step uses an index based
         *          identification to access and identify components directly 
         *          and therefore more efficiently.
         *          Therefore the ray tracing step needs mappings between 
         *          simulation wide UIDs and internally used indices for all
         *          relevant components, which are provided inside this 
         *          structure.
         */
        struct InterfaceMapping
        {
            std::map<UID<Particle>, size_t> particle_uid_to_index;
            std::map<size_t, UID<Particle>> particle_index_to_uid;
        };

        /**
         * @brief Holds additional information for stage dependent memory
         *        allocations.
         */
        struct MemoryInfo
        {
            /** @brief The internal thread index for which data is created. */
            size_t thread_idx;
            /** @brief Number of thread-local ray buffers. */
            size_t num_ray_buffers;
            /** @brief Number of thread-local intersection buffers. */
            size_t num_intersection_buffers;
        };

        /** Contains simulation parameters. */
        struct SimulationParameter
        {
            size_t max_branching_depth;
            size_t num_particles;
            size_t num_rays_per_batch;
            size_t num_surface_interaction_reflection_passes;
            size_t num_surface_interaction_transmission_passes;
            int32_t ray_world_space_index;
            int32_t ray_invalid_index = -1;
        };

        /** @brief Holds the data of a batch of rays. */
        struct RayBuffer
        {
            RayBuffer(size_t buffer_index, size_t depth, size_t size)
                :
                uid{ },
                buffer_index{ buffer_index },
                depth{ depth },
                size{ size },
                ray_data{ nullptr },
                index_data{ nullptr },
                min_bounding_volume_distance_data{ nullptr },
                active_rays{ 0 },
                world_space_rays{ 0 },
                inner_particle_rays{ false }
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
            /** The thread-local ray buffer index. */
            const size_t buffer_index;
            /** @brief Branching depth (number of times rays split before). */
            const size_t depth;
            /** @brief Number of elements in ray_data and index_data arrays. */
            const size_t size;
            /** @brief Unique identifier of the buffer. */
            UID<RayBuffer> uid;
        };

        /**
         * @brief Buffer holding intersection points and the corresponding
         *        normal of the particle surface per ray.
         */
        struct IntersectionBuffer
        {
            IntersectionBuffer(size_t buffer_index, size_t size) 
                :
                buffer_index{ buffer_index },
                point{ nullptr },
                normal{ nullptr },
                particle_index{ nullptr },
                size{ size }
            { }
            /** @brief The thread-local intersection buffer index. */
            const size_t buffer_index;
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
         * @brief Buffer holding force and torque change of the particle per
         *        particle.
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
    }
}

#endif

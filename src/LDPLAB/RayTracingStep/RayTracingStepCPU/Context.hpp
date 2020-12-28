#ifndef WWU_LDPLAB_RTSCPU_CONTEXT_HPP
#define WWU_LDPLAB_RTSCPU_CONTEXT_HPP

#include "RayTracingStepCPU.hpp"
#include "Data.hpp"

#include "..\..\Geometry.hpp"
#include "..\..\ExperimentalSetup\Lightsource.hpp"
#include "..\..\ExperimentalSetup\Particle.hpp"
#include "..\..\Utils\UID.hpp"

#include <mutex>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief Holds context data for a ray tracer run.
         * @details The context data consists of configuration, the state of
         *          the experimental setup and memory container instances for
         *          storing the temporary data during ray tracer execution.
         *          This data is shared between the different stages of the ray
         *          tracing pipeline.
         *          Individual pipeline stages can hold their own data.
         */
        struct Context
        {
            /**
             * @brief Stores the particles present in the experimental setup. 
             */
            std::vector<Particle> particles;
            /**
             * @brief Stores the particle transformations for the current 
             *        state of the experimental setup.
             * @details The index of a particle transformation directly
             *          corresponds to the index of the related particle.
             */
            std::vector<ParticleTransformation> particle_transformations;
            /**
             * @brief Stores the particle geometrical representation of rode 
             *        like particles with analytical representation.
             */
            std::vector<RodeParticle> rode_particle_geometry;
            /**
             * @brief Stores the light sources present in the experimental 
             *        setup.
             */
            std::vector<LightSource> light_sources;
            /**
             * @brief Stores the rays during the processing of the ray tracing
             *        pipeline and is used as the data source for the ray 
             *        buffers.
             * @note This amounts to number_rays_per_buffer^branching_depth
             *       rays.
             */
            std::vector<Ray> batch_rays;
            /** @brief Stores the initial batch rays. */
            RayBuffer batch_buffer;
            /** 
             * @brief Under this cutoff intensity rays are not further traced. 
             */
            double intensity_cutoff;
            /** @brief The ID of the context. */
            UID<Context> uid;
            /** @brief Number of rays per buffer. */
            size_t number_rays_per_buffer;
            /** @brief Number of rays per world space unit. */
            size_t number_rays_per_unit;
            /** @brief Maximum number of times a ray can split. */
            size_t maximum_depth;
        };
    }
}

#endif
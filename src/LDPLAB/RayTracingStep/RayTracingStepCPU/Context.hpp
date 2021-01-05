#ifndef WWU_LDPLAB_RTSCPU_CONTEXT_HPP
#define WWU_LDPLAB_RTSCPU_CONTEXT_HPP

#include "RayTracingStepCPU.hpp"
#include "Pipeline.hpp"
#include "Data.hpp"

#include "..\RayTracingStepOutput.hpp"
#include "..\..\Geometry.hpp"
#include "..\..\ThreadPool.hpp"
#include "..\..\ExperimentalSetup\Lightsource.hpp"
#include "..\..\ExperimentalSetup\Particle.hpp"
#include "..\..\Utils\UID.hpp"

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
            Context(const std::vector<Particle>& particles,
                const std::vector<LightSource>& light_sources)
                :
                uid{ }, 
                particles{ particles },
                light_sources{ light_sources },
                particle_transformations{ },
                transformed_bounding_spheres{ },
                rode_particle_geometry{ },
                pipeline{ nullptr },
                thread_pool{ nullptr },
                parameters{ }
            {}
            /** @brief The ID of the context. */
            UID<Context> uid;
            /**
             * @brief Stores the particles present in the experimental setup. 
             */
            const std::vector<Particle> particles;
            /**
             * @brief Stores the light sources present in the experimental
             *        setup.
             */
            const std::vector<LightSource> light_sources;
            /**
             * @brief Stores the particle transformations for the current 
             *        state of the experimental setup.
             * @details The index of a particle transformation directly
             *          corresponds to the index of the related particle.
             */
            std::vector<ParticleTransformation> particle_transformations;
            std::vector<BoundingVolumeSphere> transformed_bounding_spheres;
            /**
             * @brief Stores the particle geometrical representation of rode 
             *        like particles with analytical representation.
             */
            std::vector<RodeParticle> rode_particle_geometry;
            /**
             * @brief The ray tracing step cpu pipeline.
             */
            std::shared_ptr<Pipeline> pipeline;
            /**
             * @brief The thread pool used by the ray tracing step.
             */
            std::shared_ptr<ThreadPool> thread_pool;

            struct
            {
                /**
                 * @brief Under this cutoff intensity rays are not further
                 *        traced.
                 */
                double intensity_cutoff;
                /** @brief Index of reflection from the medium. */
                double medium_reflection_index;
                /** @brief Maximum number of times a ray can split. */
                size_t maximum_branching_depth;
                /** @brief Number of rays per buffer. */
                size_t number_rays_per_buffer;
                /** @brief Number of rays per world space unit. */
                size_t number_rays_per_unit;
                /** @brief Number of parallel pipeline instances. */
                size_t number_parallel_pipelines;
            } parameters;
        };
    }
}

#endif
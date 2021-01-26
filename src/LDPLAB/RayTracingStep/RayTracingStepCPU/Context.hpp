#ifndef WWU_LDPLAB_RTSCPU_CONTEXT_HPP
#define WWU_LDPLAB_RTSCPU_CONTEXT_HPP

#include <map>
#include <vector>

#include "RayTracingStep.hpp"
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
                pipeline{ nullptr },
                thread_pool{ nullptr },
                particle_data{ nullptr },
                bounding_volume_data{ nullptr },
                particle_uid_to_index_map{ },
                light_source_uid_to_index_map{ },
                particle_index_to_uid_map{ },
                light_source_index_to_uid_map{ },
                parameters{ }
            {
                // Create uid, index maps
                for (size_t i = 0; i < particles.size(); ++i)
                {
                    particle_uid_to_index_map.emplace(std::make_pair(
                        particles[i].uid, i));
                    particle_index_to_uid_map.emplace(std::make_pair(
                        i, particles[i].uid));
                }
                for (size_t i = 0; i < light_sources.size(); ++i)
                {
                    light_source_uid_to_index_map.emplace(std::make_pair(
                        light_sources[i].uid, i));
                    light_source_index_to_uid_map.emplace(std::make_pair(
                        i, light_sources[i].uid));
                }
            }
            /** @brief The ID of the context. */
            UID<Context> uid;
            /**
             * @brief Stores a copy of the particles present in the
             *        experimental setup. 
             */
            const std::vector<Particle> particles;
            /**
             * @brief Stores a copy of the light sources present in the 
             *        experimental setup.
             */
            const std::vector<LightSource> light_sources;
            /**
             * @brief Stores the particle transformations for the current 
             *        state of the experimental setup.
             * @details The index of a particle transformation directly
             *          corresponds to the index of the related particle.
             */
            std::vector<ParticleTransformation> particle_transformations;
            /** @brief The ray tracing step cpu pipeline. */
            std::shared_ptr<Pipeline> pipeline;
            /** @brief The thread pool used by the ray tracing step. */
            std::shared_ptr<ThreadPool> thread_pool;
            /** @brief Holds an array with particle data. */
            std::shared_ptr<IParticleData> particle_data;
            /** @brief Holds an array with bounding volume data. */
            std::shared_ptr<IBoundingVolumeData> bounding_volume_data;
            /** @brief Maps particle UIDs to the internaly used indices. */
            std::map<UID<Particle>, size_t> particle_uid_to_index_map;
            /** @brief Maps light source UIDs to the internaly used indices. */
            std::map<UID<LightSource>, size_t> light_source_uid_to_index_map;
            /** @brief Maps the internaly used particle indices to UIDs. */
            std::map<size_t, UID<Particle>> particle_index_to_uid_map;
            /** @brief Maps the internaly used light source indices to UIDs. */
            std::map<size_t, UID<LightSource>> light_source_index_to_uid_map;
            /** @brief Structure holding simulation parameters. */
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
            /** @brief Structure holding simulation flags. */
            struct
            {
                /** 
                 * @brief Determines whether a warning is emited when active
                 *        rays are not further traced because they would exceed
                 *        the maximum branching depth.
                 */
                bool emit_warning_on_maximum_branching_depth_discardment;
            } flags;
        };
    }
}

#endif
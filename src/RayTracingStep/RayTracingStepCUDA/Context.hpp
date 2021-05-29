#ifndef WWU_LDPLAB_RTSCUDA_CONTEXT_HPP
#define WWU_LDPLAB_RTSCUDA_CONTEXT_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/Lightsource.hpp>
#include <LDPLAB/ExperimentalSetup/Particle.hpp>
#include <LDPLAB/UID.hpp>
#include <map>

#include "Data.hpp"
#include "Pipeline.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /**
         * @brief Holds context data for a ray tracer run.
         * @details The context data consists of configuration, the state of
         *          the experimental setup and memory container instances for
         *          storing the temporary data during ray tracer execution.
         *          This data is shared between the different stages of the ray
         *          tracing pipeline.
         *          It also includes all device resources.
         */
        struct Context
        {
            /** @brief Context uid. */
            UID<Context> uid;
            /** @brief Holds mapping between RTS internals and its interface. */
            struct
            {
                /** @brief Maps internally used light indices to uids. */
                std::map<size_t, UID<LightSource>> light_source_index_to_uid;
                /** @brief Maps light uids to internally used indices. */
                std::map<UID<LightSource>, size_t> light_source_uid_to_index;
                /** @brief Maps internally used particle indices to uids. */
                std::map<size_t, UID<Particle>> particle_index_to_uid;
                /** @brief Maps particle uids to internally used indices. */
                std::map<UID<Particle>, size_t> particle_uid_to_index;
            } interface_mapping;
            /** @brief Structure holding simulation parameters. */
            struct
            {
                /** @brief Ray intensity cutoff. */
                double intensity_cutoff;
                /** @brief Reflexion index of the medium. */
                double medium_reflection_index;
                /** @brief The maximum branching depth. */
                size_t max_branching_depth;
                /** @brief The number of rays per batch. */
                size_t num_rays_per_batch;
                /** @brief The number of threads per block. */
                size_t num_threads_per_block;
                /** @brief The number of particles in the simulation setup. */
                size_t num_particles;
            } parameters;
            /** @brief The pipeline instance. */
            std::unique_ptr<IPipeline> pipeline;
            /** @brief Structure holding device resources. */
            struct
            {
                /** @brief Holding ray buffer resources. */
                RayBufferResources ray_buffer;
                /** @brief Holding intersection buffer resources. */
                IntersectionBufferResources intersection_buffer;
                /** @brief Holding output buffer resources. */
                OutputBufferResources output_buffer;
                /** @brief Holding transformation resources. */
                TransformationResources transformations;
                /** @brief Holding bounding volume resources. */
                BoundingVolumeResources bounding_volumes;
                /** @brief Holding particle data. */
                ParticleResources particles;
                /** @brief Holding other pipeline resources. */
                PipelineResources pipeline;
            } resources;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_CONTEXT_HPP
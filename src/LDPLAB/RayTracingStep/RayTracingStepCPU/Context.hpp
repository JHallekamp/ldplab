#ifndef WWU_LDPLAB_RTSCPU_CONTEXT_HPP
#define WWU_LDPLAB_RTSCPU_CONTEXT_HPP

#include "RayTracingStepCPU.hpp"
#include "Data.hpp"

#include "..\..\Geometry.hpp"
#include "..\..\ExperimentalSetup\Lightsource.hpp"
#include "..\..\ExperimentalSetup\Particle.hpp"

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
             * @brief Stores the particle geometrical representation of rode 
             *        like particles with analytical representation.
             */
            std::vector<RodeParticle> rode_particle_geometry;
            /**
             * @brief Stores the light sources present in the expermental 
             *        setup.
             */
            std::vector<LightSource> light_sources;
            /**
             * @brief Stores the temporary rays of the tracing of a single ray
             *        batch.
             */
            std::vector<Ray> batch_rays;
        };
    }
}

#endif
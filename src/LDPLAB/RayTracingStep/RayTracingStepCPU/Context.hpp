#ifndef WWU_LDPLAB_RTSCPU_CONTEXT_HPP
#define WWU_LDPLAB_RTSCPU_CONTEXT_HPP

#include "RayTracingStepCPU.hpp"
#include "Data.hpp"

#include "..\..\Geometry.hpp"
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
            std::vector<Particle> m_particles;
            /**
             * @brief Stores the temporary rays of the tracing of a single ray
             *        batch.
             */
            std::vector<Ray> m_batch_rays;
        };
    }
}

#endif
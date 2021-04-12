#ifndef WWU_LDPLAB_RAY_TRACING_STEP_OUTPUT_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_OUTPUT_HPP

#include "../ExperimentalSetup/Particle.hpp"
#include "../Geometry.hpp"
#include "../UID.hpp"

#include <map>

namespace ldplab
{
    /** @brief Contains the output of a ray tracing step. */
    struct RayTracingStepOutput
    {
        /** @brief Force vector per particle. */
        std::map<UID<Particle>, Vec3> force_per_particle;
        /** @brief Torque vector per particle. */
        std::map<UID<Particle>, Vec3> torque_per_particle;
    };
}

#endif
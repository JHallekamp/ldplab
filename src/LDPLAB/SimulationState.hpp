#ifndef WWU_LDPLAB_SIMULATION_STATE_HPP
#define WWU_LDPLAB_SIMULATION_STATE_HPP

#include "Geometry.hpp"

#include <vector>

namespace ldplab
{
    struct ParticleInstance
    {
        Vec3 position;
        Vec3 orientation;
    };

    struct SimulationState
    {
        std::vector<ParticleInstance> particles;
    };
}

#endif

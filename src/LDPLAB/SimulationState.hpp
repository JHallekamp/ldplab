#ifndef LDPLAB_SIMULATION_STATE_HPP
#define LDPLAB_SIMULATION_STATE_HPP

#include "Geometry.hpp"

#include <vector>

namespace ldplab
{
    struct ParticleInstance
    {
        Vec3 position;
        Vec3 orientation;
        size_t type;
    };
    struct SimulationState
    {
        std::vector<ParticleInstance> particles;
    };
}

#endif

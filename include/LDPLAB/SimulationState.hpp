#ifndef WWU_LDPLAB_SIMULATION_STATE_HPP
#define WWU_LDPLAB_SIMULATION_STATE_HPP

#include "ExperimentalSetup/ExperimentalSetup.hpp"
#include "Geometry.hpp"
#include "UID.hpp"

#include <map>

namespace ldplab
{
    /**
     * @brief Stores information about particles that may change over the 
     *        course of the simulation.
     */
    struct ParticleInstance
    {
        ParticleInstance()
            :
            position{ 0, 0, 0 },
            orientation{ 0, 0, 0 },
            rotation_order{ RotationOrder::xyz }
        { }
        ParticleInstance(const Particle& particle)
            :
            position{ particle.position },
            orientation{ particle.orientation },
            rotation_order{ particle.rotation_order }
        { }
        /** @brief Position of the particle in world space. */
        Vec3 position;
        /** @brief Rotation around each respective axis in particle space. */
        Vec3 orientation;
        /** @brief Defines the order of rotation around respective axis. */
        RotationOrder rotation_order;
    };

    /**
     * @brief Stores the current state of simulated objects.
     * @details The simulation state encapsulates the information that may
     *          change during simulation and therefore has to be distinguished
     *          from the experimental setup, which only describes the initial
     *          setup.
     */
    struct SimulationState
    {
        SimulationState(const ExperimentalSetup& setup)
        {
            for (size_t i = 0; i < setup.particles.size(); ++i)
            {
                particle_instances.emplace(std::make_pair(
                    setup.particles[i].uid, 
                    ParticleInstance{ setup.particles[i] }));
            }
        }
        /**
         * @brief Stores the particle instances.
         * @note Particle instances are accessible via their respective 
         *       particle uid.
         */
        std::map<UID<Particle>, ParticleInstance> particle_instances;
    };
}

#endif

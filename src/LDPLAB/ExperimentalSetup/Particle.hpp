#ifndef WWU_LDPLAB_PARTICLE_HPP
#define WWU_LDPLAB_PARTICLE_HPP

#include "../Geometry.hpp"
#include "IParticleGeometry.hpp"
#include "IParticleMaterial.hpp"

#include <memory>
#include <string>

namespace ldplab
{
    /**
     * @brief Class modeling a particle.
     */
    struct Particle
    {
        std::string particle_id;
        BoundingBox bouding_volume;
        Vec3 position;
        Vec3 orientation;
        std::shared_ptr<IParticleGeometry> geometry;
        std::shared_ptr<IParticleMaterial> material;
    };
}

#endif
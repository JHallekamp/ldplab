#ifndef WWU_LDPLAB_PARTICLE_HPP
#define WWU_LDPLAB_PARTICLE_HPP

#include "../Geometry.hpp"
#include "BoundingVolume.hpp"
#include "ParticleGeometry.hpp"
#include "ParticleMaterial.hpp"

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
        std::shared_ptr<IBoundingVolume> bounding_volume;
        Vec3 position;
        Vec3 orientation;
        Vec3 centre_of_mass;
        std::shared_ptr<IParticleGeometry> geometry;
        std::shared_ptr<IParticleMaterial> material;
    };
}

#endif
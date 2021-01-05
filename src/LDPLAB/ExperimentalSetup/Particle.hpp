#ifndef WWU_LDPLAB_PARTICLE_HPP
#define WWU_LDPLAB_PARTICLE_HPP

#include "../Geometry.hpp"
#include "../Utils/UID.hpp"
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
        Particle()
            :
            uid{ },
            bounding_volume{ nullptr },
            geometry{ nullptr },
            material{ nullptr },
            position{ 0, 0, 0 },
            orientation{ 0, 0, 0 },
            centre_of_mass{ 0, 0, 0 }
        { }
        UID<Particle> uid;
        std::shared_ptr<IBoundingVolume> bounding_volume;
        std::shared_ptr<IParticleGeometry> geometry;
        std::shared_ptr<IParticleMaterial> material;
        Vec3 position;
        Vec3 orientation;
        Vec3 centre_of_mass;
    }; 
}

#endif
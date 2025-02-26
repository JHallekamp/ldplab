#ifndef WWU_LDPLAB_PARTICLE_HPP
#define WWU_LDPLAB_PARTICLE_HPP

#include "../Geometry.hpp"
#include "../UID.hpp"
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
            centre_of_mass{ 0, 0, 0 },
            rotation_order{ RotationOrder::xyz }
        { }
        UID<Particle> uid;
        std::shared_ptr<IBoundingVolume> bounding_volume;
        std::shared_ptr<IParticleGeometry> geometry;
        std::shared_ptr<IParticleMaterial> material;
        Vec3 position;
        /**
         * @brief Rotation angels in Euler angle representation.
         *        The vector entries x, y and z corresponding to the first, 
         *        second and last rotation while the corresponding axes 
         *        are defined by the rotation_order.
         */
        Vec3 orientation;
        /** @brief Defines the order of rotation around respective axis. */
        RotationOrder rotation_order;
        Vec3 centre_of_mass;
    }; 
}

#endif
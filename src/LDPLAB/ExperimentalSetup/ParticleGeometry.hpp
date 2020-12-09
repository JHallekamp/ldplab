#ifndef WWU_LDPLAB_PARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_PARTICLE_GEOMETRY_HPP

#include "IParticleGeometry.hpp"

namespace ldplab
{
    /** 
     *  @brief Geometry of an rod like particle. The particle combines a 
     *         cylinder with a half sphere as a cap. At the bottom the same 
     *         half is subtracted.
     */
    struct RodParticleGeometry : IParticleGeometry
    {
        /** @brief Radius of the cylinder. */
        double cylinder_radius;
        /** @brief Hight of the cylinder. */
        double cylinder_hight;
        /** @brief Volume of the particle.*/
        double volume;
        /** @brief Hight of the cap in units of the cylinder radius. */
        double kappa;
        Type type() { return IParticleGeometry::Type::sphere_capped_cylinder; }
    };
}

#endif
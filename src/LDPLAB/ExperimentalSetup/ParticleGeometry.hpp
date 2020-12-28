#ifndef WWU_LDPLAB_PARTICLE_GEOMETRY_HPP
#define WWU_LDPLAB_PARTICLE_GEOMETRY_HPP

namespace ldplab
{
    struct IParticleGeometry
    {
        enum class Type { shpere, sphere_capped_cylinder };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IParticleGeometry.
         */
        virtual ~IParticleGeometry() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
    };

    /** 
     *  @brief Geometry of an rod like particle. The particle combines a 
     *         cylinder with a half sphere as a cap. At the bottom the same 
     *         half is subtracted.
     */
    struct RodeParticleGeometry : IParticleGeometry
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